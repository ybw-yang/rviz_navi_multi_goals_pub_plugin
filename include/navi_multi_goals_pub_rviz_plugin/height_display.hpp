#pragma once

#include <QPointer>
#include <rviz_common/panel.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QSlider;
namespace rviz_common::properties {
class BoolProperty;
class FloatProperty;
} // namespace rviz_common::properties

namespace navi_multi_goals_pub_rviz_plugin {

class HeightCloudDisplay : public rviz_default_plugins::displays::PointCloud2Display {
    Q_OBJECT
  public:
    HeightCloudDisplay();
    void reset() override;
    void update(float wall_dt, float ros_dt) override;
    void setHeightRange(bool enabled, double lower, double upper);
    bool filtering() const;
    double lower() const;
    double upper() const;
    double minimum() const { return minimum_; }
    double maximum() const { return maximum_; }
    bool hasBounds() const { return has_bounds_; }
    QString heightFrame() const;
  Q_SIGNALS:
    void heightChanged();
  private Q_SLOTS:
    void rangeChanged();

  protected:
    void processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) override;

  private:
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_;
    rviz_common::properties::BoolProperty *enabled_;
    rviz_common::properties::FloatProperty *lower_, *upper_;
    double minimum_ = 0, maximum_ = 1;
    bool has_bounds_ = false, dirty_ = false, setting_range_ = false;
};

class HeightRangePanel : public rviz_common::Panel {
    Q_OBJECT
  public:
    explicit HeightRangePanel(QWidget *parent = nullptr);
    void onInitialize() override;

  private:
    void discover();
    void refresh();
    void apply();
    void slide(bool lower, int position);
    QPointer<HeightCloudDisplay> display_;
    QComboBox *displays_;
    QCheckBox *enabled_;
    QSlider *lower_slider_, *upper_slider_;
    QDoubleSpinBox *lower_, *upper_;
    QLabel *status_;
    double slider_min_ = 0, slider_max_ = 1;
};

} // namespace navi_multi_goals_pub_rviz_plugin

Q_DECLARE_METATYPE(QPointer<navi_multi_goals_pub_rviz_plugin::HeightCloudDisplay>)
