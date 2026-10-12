#include "navi_multi_goals_pub_rviz_plugin/virtual_wall_editor.hpp"
#include "behavior_tree/plugins/action/dataReceiver/virtual_wall_ros.h"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/yaml_config_writer.hpp>

namespace navi_multi_goals_pub_rviz_plugin {
namespace {
Eigen::Vector3d vector(const Ogre::Vector3 &p) { return {p.x, p.y, p.z}; }
Ogre::Vector3 ogre(const Eigen::Vector3d &p) { return {float(p.x()), float(p.y()), float(p.z())}; }
void setVector(rviz_common::Config config, const QString &key, const Eigen::Vector3d &point) {
    auto child = config.mapMakeChild(key);
    child.mapSetValue("X", point.x());
    child.mapSetValue("Y", point.y());
    child.mapSetValue("Z", point.z());
}
Eigen::Vector3d getVector(const rviz_common::Config &config, const QString &key) {
    const auto child = config.mapGetChild(key);
    Eigen::Vector3d point;
    for (int i = 0; i < 3; ++i) {
        const QString axis = QString("XYZ").mid(i, 1);
        bool ok = false;
        point[i] = child.mapGetChild(axis).getValue().toDouble(&ok);
        if (!ok || !std::isfinite(point[i]))
            throw std::invalid_argument("Missing or invalid wall coordinate");
    }
    return point;
}
} // namespace

WallTool::WallTool() {
    shortcut_key_ = 'w';
    access_all_keys_ = true;
    cloud_topic_ = new rviz_common::properties::StringProperty(
        "Cloud topic", "/tomogram/traversability_cloud", "Point cloud used for endpoint snapping",
        getPropertyContainer(), SLOT(configureTopics()), this);
    wall_topic_ = new rviz_common::properties::StringProperty(
        "Wall topic", "/virtual_walls", "Complete three-dimensional wall snapshots",
        getPropertyContainer(), SLOT(configureTopics()), this);
    height_ = new rviz_common::properties::FloatProperty(
        "Wall height", 1.0, "Height above the selected bottom edge", getPropertyContainer());
    thickness_ = new rviz_common::properties::FloatProperty(
        "Wall thickness", 0.1, "Wall thickness in meters", getPropertyContainer());
    snap_distance_ = new rviz_common::properties::FloatProperty(
        "Snap distance", 0.25, "Maximum distance from rendered depth to a cloud point",
        getPropertyContainer());
    for (auto *property : {height_, thickness_, snap_distance_}) {
        property->setMin(0.01);
        property->setMax(100.0);
    }
}

WallTool::~WallTool() {
    cloud_subscription_.reset();
    destroyObjects(preview_);
    destroyObjects(shapes_);
    if (bool(material_))
        Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

void WallTool::onInitialize() {
    const auto abstraction = context_->getRosNodeAbstraction().lock();
    if (!abstraction)
        throw std::runtime_error("RViz ROS node is unavailable");
    node_ = abstraction->get_raw_node();
    setName("3D Virtual Wall");
    material_ = Ogre::MaterialManager::getSingleton().create(
        "VirtualWallMaterial" + std::to_string(reinterpret_cast<uintptr_t>(this)),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    auto *pass = material_->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setCullingMode(Ogre::CULL_NONE);
    configureTopics();
    rebuildScene();
}

void WallTool::configureTopics() {
    if (!node_)
        return;
    try {
        const auto qos = rclcpp::QoS(1).reliable().transient_local();
        auto publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            wall_topic_->getStdString(), qos);
        auto subscription = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_->getStdString(), qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                incoming_cloud_ = std::move(message);
            });
        publisher_ = std::move(publisher);
        cloud_subscription_ = std::move(subscription);
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            incoming_cloud_.reset();
        }
        parsed_message_.reset();
        cloud_.reset();
        cloud_tree_.reset();
        if (restored_ || !snapshot_.walls.empty())
            publish();
    } catch (const std::exception &error) {
        setStatus(QString("Invalid topic: %1").arg(error.what()));
    }
}

void WallTool::destroyObjects(std::vector<Ogre::ManualObject *> &objects) {
    if (scene_manager_)
        for (auto *object : objects)
            scene_manager_->destroyManualObject(object);
    objects.clear();
}

void WallTool::drawWall(const virtual_walls::Wall &wall, bool preview) {
    if (!wall.valid() || !context_ || !material_)
        return;
    Ogre::Vector3 translation;
    Ogre::Quaternion rotation;
    const std::string frame = preview ? anchor_frame_ : snapshot_.frame;
    if (frame == context_->getFixedFrame().toStdString()) {
        translation = Ogre::Vector3::ZERO;
        rotation = Ogre::Quaternion::IDENTITY;
    } else if (!context_->getFrameManager()->getTransform(frame, translation, rotation)) {
        return;
    }
    auto *object = scene_manager_->createManualObject();
    object->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (const auto &corner : wall.corners()) {
        object->position(rotation * ogre(corner) + translation);
        object->colour(preview ? Ogre::ColourValue(1, 0.7f, 0, 0.5f)
                               : Ogre::ColourValue(0, 1, 0, 0.4f));
    }
    constexpr unsigned indices[] = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
                                    1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7};
    for (auto index : indices)
        object->index(index);
    object->end();
    scene_manager_->getRootSceneNode()->attachObject(object);
    (preview ? preview_ : shapes_).push_back(object);
}

void WallTool::rebuildScene() {
    if (!context_)
        return;
    destroyObjects(shapes_);
    for (const auto &wall : snapshot_.walls)
        drawWall(wall, false);
    rendered_fixed_frame_ = context_->getFixedFrame().toStdString();
    rendered_transform_valid_ = true;
    if (snapshot_.frame == rendered_fixed_frame_ || snapshot_.frame.empty()) {
        rendered_translation_ = Ogre::Vector3::ZERO;
        rendered_rotation_ = Ogre::Quaternion::IDENTITY;
    } else {
        rendered_transform_valid_ = context_->getFrameManager()->getTransform(
            snapshot_.frame, rendered_translation_, rendered_rotation_);
    }
    context_->queueRender();
}

void WallTool::update(float, float) {
    if (snapshot_.walls.empty())
        return;
    const auto fixed = context_->getFixedFrame().toStdString();
    Ogre::Vector3 translation = Ogre::Vector3::ZERO;
    Ogre::Quaternion rotation = Ogre::Quaternion::IDENTITY;
    const bool valid = snapshot_.frame == fixed || context_->getFrameManager()->getTransform(
                                                       snapshot_.frame, translation, rotation);
    if (fixed != rendered_fixed_frame_ || valid != rendered_transform_valid_ ||
        (valid && (translation != rendered_translation_ || rotation != rendered_rotation_)))
        rebuildScene();
}

void WallTool::activate() {
    deactivate();
    setStatus("Click two cloud points to add a wall. Right-click/Escape: cancel. "
              "U: undo.");
}

void WallTool::deactivate() {
    drawing_ = false;
    destroyObjects(preview_);
}

bool WallTool::pick(rviz_common::ViewportMouseEvent &event, Eigen::Vector3d &point) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        message = incoming_cloud_;
    }
    if (!message)
        return false;
    if (message != parsed_message_) {
        auto parsed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const char *name : {"x", "y", "z"})
            if (std::none_of(message->fields.begin(), message->fields.end(),
                             [name](const auto &field) { return field.name == name; }))
                return false;
        try {
            pcl::fromROSMsg(*message, *parsed);
        } catch (const std::exception &) {
            return false;
        }
        parsed->points.erase(std::remove_if(parsed->points.begin(), parsed->points.end(),
                                            [](const auto &point) {
                                                return !std::isfinite(point.x) ||
                                                       !std::isfinite(point.y) ||
                                                       !std::isfinite(point.z);
                                            }),
                             parsed->points.end());
        parsed->width = uint32_t(parsed->size());
        parsed->height = 1;
        parsed->is_dense = true;
        auto tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
        if (!parsed->empty())
            tree->setInputCloud(parsed);
        cloud_ = std::move(parsed);
        cloud_tree_ = std::move(tree);
        parsed_message_ = message;
    }
    if (cloud_->empty() || message->header.frame_id.empty())
        return false;
    Ogre::Vector3 translation;
    Ogre::Quaternion rotation;
    if (!context_->getFrameManager()->getTransform(message->header.frame_id, translation, rotation))
        return false;
    // Hide editor meshes so their surfaces cannot intercept depth picking.
    for (auto *object : shapes_)
        object->setVisible(false);
    for (auto *object : preview_)
        object->setVisible(false);
    Ogre::Vector3 hit;
    const bool found = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, hit);
    for (auto *object : shapes_)
        object->setVisible(true);
    for (auto *object : preview_)
        object->setVisible(true);
    if (!found)
        return false;
    const Eigen::Vector3d source = vector(rotation.Inverse() * (hit - translation));
    if (!source.allFinite())
        return false;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const pcl::PointXYZ query(float(source.x()), float(source.y()), float(source.z()));
    if (cloud_tree_->nearestKSearch(query, 1, indices, distances) != 1 ||
        distances[0] > std::pow(snap_distance_->getFloat(), 2))
        return false;
    const auto &nearest = (*cloud_)[indices[0]];
    point = vector(rotation * Ogre::Vector3(nearest.x, nearest.y, nearest.z) + translation);
    return point.allFinite();
}

int WallTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
    if (event.rightDown()) {
        deactivate();
        return Render;
    }
    if (drawing_ && anchor_frame_ != context_->getFixedFrame().toStdString()) {
        deactivate();
        setStatus("Fixed frame changed; cancelled the unfinished wall.");
        return Render;
    }
    if (!event.leftDown() && (!drawing_ || event.type != QEvent::MouseMove))
        return 0;
    Eigen::Vector3d point;
    if (!pick(event, point)) {
        destroyObjects(preview_);
        setStatus("No cloud point under cursor. Check the cloud display, topic and TF.");
        return Render;
    }
    if (!drawing_) {
        const auto frame = context_->getFixedFrame().toStdString();
        if (!snapshot_.walls.empty() && snapshot_.frame != frame) {
            setStatus("Set the fixed frame to the existing wall frame before adding "
                      "walls.");
            return Render;
        }
        anchor_ = point;
        anchor_frame_ = frame;
        drawing_ = true;
        setStatus("Choose the second cloud point; height and thickness are in the "
                  "panel/tool properties.");
        return Render;
    }
    virtual_walls::Wall wall;
    wall.id = next_id_;
    wall.start = anchor_;
    wall.end = point;
    wall.height = height_->getFloat();
    wall.thickness = thickness_->getFloat();
    destroyObjects(preview_);
    drawWall(wall, true);
    if (event.leftDown() && wall.valid()) {
        if (snapshot_.walls.size() >= 1000) {
            setStatus("Wall limit reached (1000).");
            return Render;
        }
        if (next_id_ >= 1000000000) {
            setStatus("Wall ID limit reached; save and reload the wall file.");
            return Render;
        }
        remember();
        snapshot_.frame = anchor_frame_;
        snapshot_.walls.push_back(wall);
        ++next_id_;
        deactivate();
        commit();
    }
    return Render;
}

int WallTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *) {
    if (event->key() == Qt::Key_Escape) {
        deactivate();
        return Render | Finished;
    }
    if (event->key() == Qt::Key_U) {
        undo();
        return Render;
    }
    return 0;
}

void WallTool::remember() {
    history_.push_back(snapshot_);
    if (history_.size() > 100)
        history_.erase(history_.begin());
}
void WallTool::commit() {
    restored_ = true;
    rebuildScene();
    publish();
    Q_EMIT wallsChanged();
}
void WallTool::publish() {
    if (!publisher_)
        return;
    if (snapshot_.frame.empty())
        snapshot_.frame = context_->getFixedFrame().toStdString();
    restored_ = true;
    publisher_->publish(pp_behavior_tree::virtual_wall_ros::encode(snapshot_, node_->now()));
}
void WallTool::setDimensions(double height, double thickness) {
    height_->setFloat(float(height));
    thickness_->setFloat(float(thickness));
}
bool WallTool::replaceWall(size_t index, const virtual_walls::Wall &wall) {
    if (index >= snapshot_.walls.size() || !wall.valid())
        return false;
    deactivate();
    remember();
    auto changed = wall;
    changed.id = snapshot_.walls[index].id;
    snapshot_.walls[index] = changed;
    commit();
    return true;
}
void WallTool::removeWall(size_t index) {
    if (index >= snapshot_.walls.size())
        return;
    deactivate();
    remember();
    snapshot_.walls.erase(snapshot_.walls.begin() + index);
    commit();
}
void WallTool::clear() {
    deactivate();
    remember();
    snapshot_.walls.clear();
    commit();
}
void WallTool::undo() {
    if (history_.empty())
        return;
    deactivate();
    snapshot_ = history_.back();
    history_.pop_back();
    for (const auto &wall : snapshot_.walls)
        next_id_ = std::max(next_id_, wall.id + 1);
    commit();
}

void WallTool::writeConfig(rviz_common::Config config) const {
    config.mapSetValue("Version", 1);
    const auto frame = snapshot_.frame.empty() && context_
                           ? context_->getFixedFrame()
                           : QString::fromStdString(snapshot_.frame);
    config.mapSetValue("Frame", frame);
    auto list = config.mapMakeChild("Walls");
    list.setType(rviz_common::Config::List);
    for (const auto &wall : snapshot_.walls) {
        auto item = list.listAppendNew();
        item.mapSetValue("Id", wall.id);
        item.mapSetValue("Height", wall.height);
        item.mapSetValue("Thickness", wall.thickness);
        setVector(item, "Start", wall.start);
        setVector(item, "End", wall.end);
        setVector(item, "Up", wall.up);
    }
}
void WallTool::readConfig(const rviz_common::Config &config) {
    int version = 0;
    QString frame;
    if (!config.mapGetInt("Version", &version) || version != 1 ||
        !config.mapGetString("Frame", &frame) || frame.isEmpty())
        throw std::invalid_argument("Unsupported or unframed wall file");
    virtual_walls::Snapshot loaded;
    loaded.frame = frame.toStdString();
    const auto list = config.mapGetChild("Walls");
    if (list.getType() != rviz_common::Config::List || list.listLength() > 1000)
        throw std::invalid_argument("Invalid wall list");
    std::set<int> ids;
    int next = 1;
    for (int i = 0; i < list.listLength(); ++i) {
        const auto item = list.listChildAt(i);
        virtual_walls::Wall wall;
        if (!item.mapGetInt("Id", &wall.id) || wall.id < 0 || wall.id >= 1000000000 ||
            !ids.insert(wall.id).second)
            throw std::invalid_argument("Invalid or duplicate wall ID");
        wall.start = getVector(item, "Start");
        wall.end = getVector(item, "End");
        wall.up = getVector(item, "Up");
        bool ok_height = false, ok_thickness = false;
        wall.height = item.mapGetChild("Height").getValue().toDouble(&ok_height);
        wall.thickness = item.mapGetChild("Thickness").getValue().toDouble(&ok_thickness);
        if (!ok_height || !ok_thickness || !wall.valid())
            throw std::invalid_argument("Invalid wall dimensions");
        loaded.walls.push_back(wall);
        next = std::max(next, wall.id + 1);
    }
    deactivate();
    remember();
    snapshot_ = std::move(loaded);
    next_id_ = next;
    commit();
}
void WallTool::save(rviz_common::Config config) const {
    rviz_common::Tool::save(config);
    if (restored_ || !snapshot_.walls.empty())
        writeConfig(config.mapMakeChild("Wall geometry"));
}
void WallTool::load(const rviz_common::Config &config) {
    rviz_common::Tool::load(config);
    const auto geometry = config.mapGetChild("Wall geometry");
    if (geometry.isValid()) {
        try {
            readConfig(geometry);
        } catch (const std::exception &error) {
            setStatus(QString("Wall restore failed: %1").arg(error.what()));
        }
    }
}
void WallTool::saveFile(const QString &path) const {
    rviz_common::Config config;
    writeConfig(config);
    rviz_common::YamlConfigWriter writer;
    const QString yaml = writer.writeString(config);
    if (writer.error())
        throw std::runtime_error(writer.errorMessage().toStdString());
    const auto bytes = yaml.toUtf8();
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error("Could not save wall file");
}
void WallTool::loadFile(const QString &path) {
    rviz_common::Config config;
    rviz_common::YamlConfigReader reader;
    reader.readFile(config, path);
    if (reader.error())
        throw std::runtime_error(reader.errorMessage().toStdString());
    readConfig(config);
}

WallPanel::WallPanel(QWidget *parent) : Panel(parent) {
    auto *layout = new QVBoxLayout(this);
    status_ = new QLabel("Add the 3D Virtual Wall tool to begin.");
    status_->setWordWrap(true);
    layout->addWidget(status_);
    list_ = new QListWidget;
    layout->addWidget(list_);
    auto *form = new QFormLayout;
    for (int i = 0; i < 6; ++i) {
        coordinates_[i] = new QDoubleSpinBox;
        coordinates_[i]->setRange(-1000000, 1000000);
        coordinates_[i]->setDecimals(3);
        form->addRow(QString(i < 3 ? "Start %1" : "End %1").arg(QString("XYZ").mid(i % 3, 1)),
                     coordinates_[i]);
    }
    height_ = new QDoubleSpinBox;
    thickness_ = new QDoubleSpinBox;
    for (auto *spin : {height_, thickness_}) {
        spin->setRange(0.01, 100);
        spin->setDecimals(3);
        spin->setSingleStep(0.05);
    }
    height_->setValue(1.0);
    thickness_->setValue(0.1);
    form->addRow("Height (m)", height_);
    form->addRow("Thickness (m)", thickness_);
    layout->addLayout(form);
    auto *buttons = new QHBoxLayout;
    auto button = [&](const QString &text, auto callback) {
        auto *b = new QPushButton(text);
        buttons->addWidget(b);
        connect(b, &QPushButton::clicked, this, callback);
    };
    button("Draw", [this]() {
        attach();
        if (!tool_ && getDisplayContext()) {
            getDisplayContext()->getToolManager()->addTool(
                "navi_multi_goals_pub_rviz_plugin/VirtualWallTool");
            attach();
        }
        if (tool_) {
            tool_->setDimensions(height_->value(), thickness_->value());
            getDisplayContext()->getToolManager()->setCurrentTool(tool_);
        }
    });
    button("Apply", [this]() { apply(); });
    button("Delete", [this]() {
        if (tool_ && list_->currentRow() >= 0)
            tool_->removeWall(size_t(list_->currentRow()));
    });
    button("Undo", [this]() {
        if (tool_)
            tool_->undo();
    });
    layout->addLayout(buttons);
    buttons = new QHBoxLayout;
    button("Clear", [this]() {
        if (tool_)
            tool_->clear();
    });
    button("Publish", [this]() {
        if (tool_)
            tool_->publish();
    });
    button("Save", [this]() {
        if (!tool_)
            return;
        const auto path =
            QFileDialog::getSaveFileName(this, "Save virtual walls", "", "YAML (*.yaml)");
        if (path.isEmpty())
            return;
        try {
            tool_->saveFile(path);
        } catch (const std::exception &e) {
            QMessageBox::warning(this, "Virtual walls", e.what());
        }
    });
    button("Load", [this]() {
        if (!tool_)
            return;
        const auto path =
            QFileDialog::getOpenFileName(this, "Load virtual walls", "", "YAML (*.yaml)");
        if (path.isEmpty())
            return;
        try {
            tool_->loadFile(path);
        } catch (const std::exception &e) {
            QMessageBox::warning(this, "Virtual walls", e.what());
        }
    });
    layout->addLayout(buttons);
    connect(list_, &QListWidget::currentRowChanged, this, [this](int) { showSelection(); });
    connect(height_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (tool_)
            tool_->setDimensions(height_->value(), thickness_->value());
    });
    connect(thickness_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (tool_)
            tool_->setDimensions(height_->value(), thickness_->value());
    });
}
void WallPanel::onInitialize() {
    auto *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() { attach(); });
    timer->start(500);
    attach();
}
void WallPanel::attach() {
    if (!getDisplayContext() || tool_)
        return;
    auto *manager = getDisplayContext()->getToolManager();
    for (int i = 0; i < manager->numTools(); ++i) {
        if (auto *tool = qobject_cast<WallTool *>(manager->getTool(i))) {
            tool_ = tool;
            connect(tool, &WallTool::wallsChanged, this, [this]() {
                refresh();
                Q_EMIT configChanged();
            });
            connect(tool, &QObject::destroyed, this, [this]() {
                list_->clear();
                status_->setText("Add the 3D Virtual Wall tool to begin.");
            });
            refresh();
            return;
        }
    }
}
void WallPanel::refresh() {
    if (!tool_)
        return;
    const int previous = list_->currentRow();
    list_->clear();
    for (const auto &wall : tool_->snapshot().walls)
        list_->addItem(
            QString("Wall %1 | h=%2 m | t=%3 m").arg(wall.id).arg(wall.height).arg(wall.thickness));
    status_->setText(QString("Frame: %1 | %2 walls")
                         .arg(QString::fromStdString(tool_->snapshot().frame))
                         .arg(list_->count()));
    if (list_->count())
        list_->setCurrentRow(std::clamp(previous, 0, list_->count() - 1));
}
void WallPanel::showSelection() {
    if (!tool_ || list_->currentRow() < 0)
        return;
    const auto &wall = tool_->snapshot().walls[size_t(list_->currentRow())];
    for (int i = 0; i < 3; ++i) {
        coordinates_[i]->setValue(wall.start[i]);
        coordinates_[i + 3]->setValue(wall.end[i]);
    }
    const QSignalBlocker height_blocker(height_), thickness_blocker(thickness_);
    height_->setValue(wall.height);
    thickness_->setValue(wall.thickness);
}
void WallPanel::apply() {
    if (!tool_ || list_->currentRow() < 0)
        return;
    const size_t index = size_t(list_->currentRow());
    auto wall = tool_->snapshot().walls[index];
    for (int i = 0; i < 3; ++i) {
        wall.start[i] = coordinates_[i]->value();
        wall.end[i] = coordinates_[i + 3]->value();
    }
    wall.height = height_->value();
    wall.thickness = thickness_->value();
    if (!tool_->replaceWall(index, wall))
        QMessageBox::warning(this, "Virtual walls",
                             "Choose distinct endpoints and a valid height/thickness.");
}

} // namespace navi_multi_goals_pub_rviz_plugin

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::WallTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::WallPanel, rviz_common::Panel)
