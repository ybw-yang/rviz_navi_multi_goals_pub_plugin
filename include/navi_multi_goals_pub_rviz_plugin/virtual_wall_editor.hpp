#pragma once

#include "virtual_wall_geometry.h"
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <QPointer>
#include <QString>
#include <memory>
#include <mutex>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/tool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class QDoubleSpinBox;
class QLabel;
class QListWidget;
namespace Ogre {
class ManualObject;
}
namespace rviz_common::properties {
class StringProperty;
class FloatProperty;
} // namespace rviz_common::properties

namespace navi_multi_goals_pub_rviz_plugin {

class WallTool : public rviz_common::Tool {
    Q_OBJECT
  public:
    WallTool();
    ~WallTool() override;
    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;
    int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel) override;
    void update(float, float) override;
    const virtual_walls::Snapshot &snapshot() const { return snapshot_; }
    bool replaceWall(size_t index, const virtual_walls::Wall &wall);
    void removeWall(size_t index);
    void undo();
    void clear();
    void setDimensions(double height, double thickness);
    void saveFile(const QString &path) const;
    void loadFile(const QString &path);
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config &config) override;
    void publish();
  Q_SIGNALS:
    void wallsChanged();
  private Q_SLOTS:
    void configureTopics();

  private:
    bool pick(rviz_common::ViewportMouseEvent &event, Eigen::Vector3d &point);
    void rebuildScene();
    void remember();
    void commit();
    void writeConfig(rviz_common::Config config) const;
    void readConfig(const rviz_common::Config &config);
    virtual_walls::Snapshot snapshot_;
    std::vector<virtual_walls::Snapshot> history_;
    std::vector<Ogre::ManualObject *> shapes_;
    std::vector<Ogre::ManualObject *> preview_;
    Ogre::MaterialPtr material_;
    void destroyObjects(std::vector<Ogre::ManualObject *> &objects);
    void drawWall(const virtual_walls::Wall &wall, bool preview);
    Eigen::Vector3d anchor_ = Eigen::Vector3d::Zero();
    std::string anchor_frame_;
    bool drawing_ = false, restored_ = false;
    int next_id_ = 1;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    std::mutex cloud_mutex_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr incoming_cloud_, parsed_message_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> cloud_tree_;
    std::string rendered_fixed_frame_;
    Ogre::Vector3 rendered_translation_ = Ogre::Vector3::ZERO;
    Ogre::Quaternion rendered_rotation_ = Ogre::Quaternion::IDENTITY;
    bool rendered_transform_valid_ = false;
    rviz_common::properties::StringProperty *cloud_topic_, *wall_topic_;
    rviz_common::properties::FloatProperty *height_, *thickness_, *snap_distance_;
};

class WallPanel : public rviz_common::Panel {
    Q_OBJECT
  public:
    explicit WallPanel(QWidget *parent = nullptr);
    void onInitialize() override;

  private:
    void attach();
    void refresh();
    void showSelection();
    void apply();
    QPointer<WallTool> tool_;
    QListWidget *list_;
    QLabel *status_;
    QDoubleSpinBox *coordinates_[6];
    QDoubleSpinBox *height_, *thickness_;
};

} // namespace navi_multi_goals_pub_rviz_plugin
