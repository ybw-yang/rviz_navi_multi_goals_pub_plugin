#ifndef MULTI_NAVI_GOAL_PANEL_HPP
#define MULTI_NAVI_GOAL_PANEL_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/transform_datatypes.h>




namespace navi_multi_goals_pub_rviz_plugin {

class MultiNaviGoalsPanel : public rviz_common::Panel {
Q_OBJECT
public:
    explicit MultiNaviGoalsPanel(QWidget *parent = 0);
    ~MultiNaviGoalsPanel(){};

    void onInitialize() override;

public Q_SLOTS:
    void writePose(geometry_msgs::msg::Pose pose);
    void markPose(const geometry_msgs::msg::PoseStamped::SharedPtr &pose);
    void deleteMark();

protected Q_SLOTS:
    void initPoseTable();
    void startNavi();
    void cancelNavi();
    void checkCycle();
    void startSpin();

    void goalCntCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void statusCB(const std_msgs::msg::Empty::SharedPtr statuses);


private:
    void addGoal(const geometry_msgs::msg::PoseStamped &ps);
    // Frame the markers are published in; must never be empty (see deleteMark).
    std::string markerFrame() const;

protected:
    // initPoseTable() runs from the constructor, before these are constructed, and
    // checks them for null — so they must start null rather than indeterminate.
    QPushButton *output_reset_button_ = nullptr;
    QPushButton *output_cancelNavi_button_ = nullptr;
    QPushButton *output_startNavi_button_ = nullptr;
    QTableWidget *poseArray_table_ = nullptr;
    QCheckBox    *cycle_checkbox_ = nullptr;

    rclcpp::Node::SharedPtr nh_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             cancel_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr             status_sub_;


    int  curGoalIdx_ = 0, cycleCnt_ = 0;
    bool permit_ = false, cycle_ = false, arrived_ = false;
    // rviz_common::Panel does not initialise its context_ member in the constructor;
    // it is only set by initialize(), which runs after ours. Gate every access on this.
    bool context_ready_ = false;

    geometry_msgs::msg::PoseArray pose_array_;

};

} // namespace navi_multi_goals_pub_rviz_plugin
#endif // MULTI_NAVI_GOAL_PANEL_HPP
