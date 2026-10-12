#include "navi_multi_goals_pub_rviz_plugin/height_display.hpp"
#include "navi_multi_goals_pub_rviz_plugin/height_filter.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace navi_multi_goals_pub_rviz_plugin {

HeightCloudDisplay::HeightCloudDisplay() {
    enabled_ = new rviz_common::properties::BoolProperty(
        "Height filter", false, "Show only points within the selected source-frame Z interval",
        this, SLOT(rangeChanged()));
    lower_ = new rviz_common::properties::FloatProperty(
        "Minimum height", 0, "Inclusive lower Z bound in meters, in the cloud frame", this,
        SLOT(rangeChanged()));
    upper_ = new rviz_common::properties::FloatProperty(
        "Maximum height", 3, "Inclusive upper Z bound in meters, in the cloud frame", this,
        SLOT(rangeChanged()));
}

bool HeightCloudDisplay::filtering() const { return enabled_->getBool(); }
double HeightCloudDisplay::lower() const { return lower_->getFloat(); }
double HeightCloudDisplay::upper() const { return upper_->getFloat(); }
QString HeightCloudDisplay::heightFrame() const {
    return latest_ ? QString::fromStdString(latest_->header.frame_id) : QString();
}

void HeightCloudDisplay::setHeightRange(bool enabled, double lower, double upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper))
        return;
    setting_range_ = true;
    enabled_->setBool(enabled);
    lower_->setFloat(float(std::min(lower, upper)));
    upper_->setFloat(float(std::max(lower, upper)));
    setting_range_ = false;
    rangeChanged();
}

void HeightCloudDisplay::rangeChanged() {
    if (setting_range_)
        return;
    dirty_ = bool(latest_);
    Q_EMIT heightChanged();
}

void HeightCloudDisplay::processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    latest_ = std::move(cloud);
    dirty_ = true;
}

void HeightCloudDisplay::reset() {
    latest_.reset();
    dirty_ = false;
    has_bounds_ = false;
    PointCloud2Display::reset();
    Q_EMIT heightChanged();
}

void HeightCloudDisplay::update(float wall_dt, float ros_dt) {
    if (dirty_ && latest_ && isEnabled()) {
        dirty_ = false;
        try {
            const auto result = filterHeight(*latest_, filtering(), lower(), upper());
            const bool first = !has_bounds_;
            has_bounds_ = result.finite_count > 0;
            if (has_bounds_) {
                minimum_ = result.minimum;
                maximum_ = result.maximum;
                if (first && !filtering()) {
                    setting_range_ = true;
                    lower_->setFloat(float(minimum_));
                    upper_->setFloat(float(maximum_));
                    setting_range_ = false;
                }
            }
            // An empty height interval must also clear the previous cloud.
            PointCloud2Display::reset();
            if (result.cloud->width)
                PointCloud2Display::processMessage(result.cloud);
            setStatus(rviz_common::properties::StatusProperty::Ok, "Height interval",
                      QString("%1 / %2 finite points | Z in %3")
                          .arg(result.cloud->width)
                          .arg(qulonglong(result.finite_count))
                          .arg(heightFrame()));
            Q_EMIT heightChanged();
            context_->queueRender();
        } catch (const std::exception &error) {
            PointCloud2Display::reset();
            has_bounds_ = false;
            setStatus(rviz_common::properties::StatusProperty::Error, "Height interval",
                      error.what());
            Q_EMIT heightChanged();
        }
    }
    PointCloud2Display::update(wall_dt, ros_dt);
}

HeightRangePanel::HeightRangePanel(QWidget *parent) : Panel(parent) {
    auto *layout = new QVBoxLayout(this);
    displays_ = new QComboBox;
    layout->addWidget(displays_);
    enabled_ = new QCheckBox("Filter by height");
    layout->addWidget(enabled_);
    auto *form = new QFormLayout;
    lower_slider_ = new QSlider(Qt::Horizontal);
    upper_slider_ = new QSlider(Qt::Horizontal);
    for (auto *slider : {lower_slider_, upper_slider_}) {
        slider->setRange(0, 10000);
        slider->setTracking(true);
    }
    lower_ = new QDoubleSpinBox;
    upper_ = new QDoubleSpinBox;
    for (auto *spin : {lower_, upper_}) {
        spin->setRange(-1000000, 1000000);
        spin->setDecimals(3);
        spin->setSingleStep(0.05);
    }
    form->addRow("Minimum Z (m)", lower_);
    form->addRow("Drag minimum", lower_slider_);
    form->addRow("Maximum Z (m)", upper_);
    form->addRow("Drag maximum", upper_slider_);
    layout->addLayout(form);
    auto *all = new QPushButton("Show all heights");
    layout->addWidget(all);
    status_ = new QLabel("Add a HeightCloud display to begin.");
    status_->setWordWrap(true);
    layout->addWidget(status_);
    layout->addStretch();
    connect(enabled_, &QCheckBox::toggled, this, [this]() { apply(); });
    connect(lower_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (value > upper_->value()) {
            const QSignalBlocker blocker(upper_);
            upper_->setValue(value);
        }
        apply();
    });
    connect(upper_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (value < lower_->value()) {
            const QSignalBlocker blocker(lower_);
            lower_->setValue(value);
        }
        apply();
    });
    connect(lower_slider_, &QSlider::valueChanged, this, [this](int value) { slide(true, value); });
    connect(upper_slider_, &QSlider::valueChanged, this,
            [this](int value) { slide(false, value); });
    connect(all, &QPushButton::clicked, this, [this]() {
        if (display_)
            display_->setHeightRange(false, display_->minimum(), display_->maximum());
    });
    connect(displays_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (display_)
            disconnect(display_, nullptr, this, nullptr);
        display_ = index < 0 ? QPointer<HeightCloudDisplay>()
                             : displays_->itemData(index).value<QPointer<HeightCloudDisplay>>();
        if (display_)
            connect(display_, &HeightCloudDisplay::heightChanged, this, [this]() { refresh(); });
        refresh();
    });
    refresh();
}

void HeightRangePanel::onInitialize() {
    auto *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() { discover(); });
    timer->start(500);
    discover();
}

void HeightRangePanel::discover() {
    if (!getDisplayContext())
        return;
    QList<HeightCloudDisplay *> found;
    auto scan = [&](auto &&self, rviz_common::DisplayGroup *group) -> void {
        for (int i = 0; i < group->numDisplays(); ++i) {
            auto *display = group->getDisplayAt(i);
            if (auto *height = qobject_cast<HeightCloudDisplay *>(display))
                found.push_back(height);
            if (auto *child = qobject_cast<rviz_common::DisplayGroup *>(display))
                self(self, child);
        }
    };
    scan(scan, getDisplayContext()->getRootDisplayGroup());
    bool same = found.size() == displays_->count();
    for (int i = 0; same && i < found.size(); ++i)
        same = displays_->itemData(i).value<QPointer<HeightCloudDisplay>>().data() == found[i] &&
               displays_->itemText(i) == found[i]->getName();
    if (same)
        return;
    const auto selected = display_;
    if (display_)
        disconnect(display_, nullptr, this, nullptr);
    const QSignalBlocker blocker(displays_);
    displays_->clear();
    for (auto *display : found)
        displays_->addItem(display->getName(),
                           QVariant::fromValue(QPointer<HeightCloudDisplay>(display)));
    int index = found.indexOf(selected.data());
    if (index < 0 && !found.empty())
        index = 0;
    displays_->setCurrentIndex(index);
    display_ = index < 0 ? nullptr : found[index];
    if (display_)
        connect(display_, &HeightCloudDisplay::heightChanged, this, [this]() { refresh(); });
    refresh();
}

void HeightRangePanel::refresh() {
    const bool available = bool(display_);
    const std::array<QWidget *, 5> widgets = {enabled_, lower_, upper_, lower_slider_,
                                              upper_slider_};
    for (auto *widget : widgets)
        widget->setEnabled(available);
    if (!display_) {
        status_->setText("Add a navi_multi_goals_pub_rviz_plugin/HeightCloud display to begin.");
        return;
    }
    const QSignalBlocker b1(enabled_), b2(lower_), b3(upper_), b4(lower_slider_), b5(upper_slider_);
    enabled_->setChecked(display_->filtering());
    lower_->setValue(display_->lower());
    upper_->setValue(display_->upper());
    slider_min_ = display_->hasBounds() ? display_->minimum() : std::min(display_->lower(), 0.0);
    slider_max_ = display_->hasBounds() ? display_->maximum() : std::max(display_->upper(), 1.0);
    slider_max_ = std::max(slider_max_, slider_min_ + 0.001);
    auto position = [&](double z) {
        return int(std::clamp((z - slider_min_) / (slider_max_ - slider_min_), 0.0, 1.0) * 10000);
    };
    lower_slider_->setValue(position(display_->lower()));
    upper_slider_->setValue(position(display_->upper()));
    status_->setText(display_->hasBounds()
                         ? QString("Cloud Z: %1 to %2 m | Frame: %3")
                               .arg(display_->minimum(), 0, 'f', 3)
                               .arg(display_->maximum(), 0, 'f', 3)
                               .arg(display_->heightFrame())
                         : "Waiting for cloud data. Heights use the cloud frame's Z axis.");
}

void HeightRangePanel::apply() {
    if (!display_)
        return;
    display_->setHeightRange(enabled_->isChecked(), lower_->value(), upper_->value());
    Q_EMIT configChanged();
}

void HeightRangePanel::slide(bool lower, int position) {
    if (!display_)
        return;
    const double value = slider_min_ + (slider_max_ - slider_min_) * position / 10000.0;
    const QSignalBlocker b1(enabled_), b2(lower_), b3(upper_);
    enabled_->setChecked(true);
    if (lower) {
        lower_->setValue(value);
        if (value > upper_->value())
            upper_->setValue(value);
    } else {
        upper_->setValue(value);
        if (value < lower_->value())
            lower_->setValue(value);
    }
    apply();
}

} // namespace navi_multi_goals_pub_rviz_plugin

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::HeightCloudDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::HeightRangePanel, rviz_common::Panel)
