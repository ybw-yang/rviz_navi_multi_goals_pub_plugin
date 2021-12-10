#ifndef MULTI_NAVI_GOAL_PANEL_HPP
#define MULTI_NAVI_GOAL_PANEL_HPP


#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <actionlib_msgs/msg/goal_status_array.hpp>
#include <tf2/transform_datatypes.h>

// #include "pp_msgs/action/trajectory_control.hpp"

namespace navi_multi_goals_pub_rviz_plugin {



    class MultiNaviGoalsPanel : public rviz_common::Panel {
    Q_OBJECT
    public:
        explicit MultiNaviGoalsPanel(QWidget *parent = 0);
        ~MultiNaviGoalsPanel(){};

        void onInitialize() override;

    public Q_SLOTS:

        void setMaxNumGoal(const QString &maxNumGoal);

        void writePose(geometry_msgs::msg::Pose pose);
        void markPose(const geometry_msgs::msg::PoseStamped::SharedPtr &pose);
        void deleteMark();

    protected Q_SLOTS:

        void updateMaxNumGoal();             // update max number of goal
        void initPoseTable();               // initialize the pose table

        void updatePoseTable();             // update the pose table
        void startNavi();                   // start navigate for the first pose
        void cancelNavi();
        void cycleNavi();

        void goalCntCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose);  //goal count sub callback function
        void statusCB(const action_msgs::msg::GoalStatusArray::SharedPtr statuses); //status sub callback function
        void missionCompleteCB(const std_msgs::msg::Empty::SharedPtr success); //mission complete sub callback function

        void checkCycle();

        bool checkGoal(std::vector<action_msgs::msg::GoalStatus> status_list);  // check whether arrived the goal

        void startSpin(); // spin for sub
    protected:
        QLineEdit *output_maxNumGoal_editor_;
        QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_cancel_button_;
        QTableWidget *poseArray_table_;
        QCheckBox *cycle_checkbox_;

        QString output_maxNumGoal_;

        // The ROS node handle.
        rclcpp::Node::SharedPtr nh_;
        // publisher
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
        // subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mission_complete_sub_;

        // client
        // rclcpp::Client<pp_msgs::action::TrajectoryControl::Impl::CancelGoalService>::SharedPtr cancel_client_;
        int maxNumGoal_;
        int curGoalIdx_ = 0, cycleCnt_ = 0;
        bool permit_ = false, cycle_ = false, arrived_ = false;
        geometry_msgs::msg::PoseArray pose_array_;

        action_msgs::msg::GoalInfo cur_goalid_;


    };

} // end namespace navi_multi_goals_pub_rviz_plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
