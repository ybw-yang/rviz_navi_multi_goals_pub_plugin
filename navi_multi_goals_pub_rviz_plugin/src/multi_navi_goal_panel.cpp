#include <cstdio>

#include <fstream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/impl/utils.h"
#include "rviz_common/display_context.hpp"

#include "multi_navi_goal_panel.hpp"

namespace navi_multi_goals_pub_rviz_plugin {


    MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
            : rviz_common::Panel(parent), maxNumGoal_(1) 
    {
        // nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        nh_ = std::make_shared<rclcpp::Node>("navi_multi_goals");
        waypoints_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>("/wapoints_pose", 1);
        marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("/waypoint_model/visualization_marker", 1);
        cancel_pub_ = nh_->create_publisher<std_msgs::msg::Empty>("/mission_cancel", 1);
        // cancel_client_ = nh_->create_client<pp_msgs::action::TrajectoryControl::Impl::CancelGoalService>("/tracking_action/_action/cancel_goal");

        QVBoxLayout *root_layout = new QVBoxLayout;
        // create a panel about "maxNumGoal"
        QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
        maxNumGoal_layout->addWidget(new QLabel("目标最大数量"));
        output_maxNumGoal_editor_ = new QLineEdit;
        maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
        output_maxNumGoal_button_ = new QPushButton("确定");
        maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
        root_layout->addLayout(maxNumGoal_layout);

        cycle_checkbox_ = new QCheckBox("循环");
        root_layout->addWidget(cycle_checkbox_);
        // creat a QTable to contain the poseArray
        poseArray_table_ = new QTableWidget;
        initPoseTable();
        root_layout->addWidget(poseArray_table_);
        //creat a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        output_reset_button_ = new QPushButton("重置");
        manipulate_layout->addWidget(output_reset_button_);
        output_cancel_button_ = new QPushButton("取消");
        manipulate_layout->addWidget(output_cancel_button_);
        output_startNavi_button_ = new QPushButton("开始导航!");
        manipulate_layout->addWidget(output_startNavi_button_);
        root_layout->addLayout(manipulate_layout);

        setLayout(root_layout);
        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

        // 设置信号与槽的连接
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updateMaxNumGoal()));
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updatePoseTable()));
        connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
        connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
        connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));

        goal_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/waypoint_model/goal_temp", 1,
                                                              std::bind(&MultiNaviGoalsPanel::goalCntCB, this, std::placeholders::_1));

        status_sub_ = nh_->create_subscription<action_msgs::msg::GoalStatusArray>("/tracking_action/_action/status", 1,
                                                                     std::bind(&MultiNaviGoalsPanel::statusCB, this,
                                                                                 std::placeholders::_1));
        mission_complete_sub_ = nh_->create_subscription<std_msgs::msg::Empty>("/mission_complete", 1,
                                                                     std::bind(&MultiNaviGoalsPanel::missionCompleteCB, this,
                                                                                 std::placeholders::_1));                                                                
    }

    void MultiNaviGoalsPanel::onInitialize()
    {
    }

// 更新maxNumGoal命名
    void MultiNaviGoalsPanel::updateMaxNumGoal() {
        setMaxNumGoal(output_maxNumGoal_editor_->text());
    }

// set up the maximum number of goals
    void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal) {
        // 检查maxNumGoal是否发生改变.
        if (new_maxNumGoal != output_maxNumGoal_) {
            output_maxNumGoal_ = new_maxNumGoal;

            // 如果命名为空，不发布任何信息
            if (output_maxNumGoal_ == "") {
                // nh_.setParam("maxNumGoal_", 1);
                maxNumGoal_ = 1;
            } else {
//                velocity_publisher_ = nh_->create_publisher<geometry_msgs::Twist>(output_maxNumGoal_.toStdString(), 1);
                // nh_.setParam("maxNumGoal_", output_maxNumGoal_.toInt());
                maxNumGoal_ = output_maxNumGoal_.toInt();
            }
            Q_EMIT configChanged();
        }
    }

    // initialize the table of pose
    void MultiNaviGoalsPanel::initPoseTable() {
        RCLCPP_INFO(nh_->get_logger(), "Initialize");
        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false, cycle_ = false;
        poseArray_table_->clear();
        pose_array_.poses.clear();
        deleteMark();
        poseArray_table_->setRowCount(maxNumGoal_);
        poseArray_table_->setColumnCount(3);
        poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_->setCheckState(Qt::Unchecked);

    }

    // delete marks in the map
    void MultiNaviGoalsPanel::deleteMark() {
        visualization_msgs::msg::Marker marker_delete;
        marker_delete.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_pub_->publish(marker_delete);
    }

    //update the table of pose
    void MultiNaviGoalsPanel::updatePoseTable() {
        poseArray_table_->setRowCount(maxNumGoal_);
//        pose_array_.poses.resize(maxNumGoal_);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        poseArray_table_->show();
    }

    // call back function for counting goals
    void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        if (int(pose_array_.poses.size()) < maxNumGoal_) {
            pose_array_.poses.push_back(pose->pose);
            pose_array_.header.frame_id = pose->header.frame_id;
            writePose(pose->pose);
            markPose(pose);
        } else {
            RCLCPP_ERROR(nh_->get_logger(), "Beyond the maximum number of goals: %d", maxNumGoal_);
        }
    }

    // write the poses into the table
    void MultiNaviGoalsPanel::writePose(geometry_msgs::msg::Pose pose) {

        poseArray_table_->setItem(pose_array_.poses.size() - 1, 0,
                                  new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 1,
                                  new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        
        tf2::Quaternion tf_quaternion;
        tf2::fromMsg(pose.orientation, tf_quaternion);
        double yaw = tf2::impl::getYaw(tf_quaternion);
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 2,
                                  new QTableWidgetItem(
                                          QString::number(yaw * 180.0 / 3.14, 'f', 2)));

    }

    // when setting a Navi Goal, it will set a mark on the map
    void MultiNaviGoalsPanel::markPose(const geometry_msgs::msg::PoseStamped::SharedPtr &pose) {
        if (rclcpp::ok()) {
            visualization_msgs::msg::Marker arrow;
            visualization_msgs::msg::Marker number;
            arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
            arrow.ns = "navi_point_arrow";
            number.ns = "navi_point_number";
            arrow.action = number.action = visualization_msgs::msg::Marker::ADD;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            number.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            arrow.pose = number.pose = pose->pose;
            number.pose.position.z += 1.0;
            arrow.scale.x = 1.0;
            arrow.scale.y = 0.2;
            number.scale.z = 1.0;
            arrow.color.r = number.color.r = 1.0f;
            arrow.color.g = number.color.g = 0.98f;
            arrow.color.b = number.color.b = 0.80f;
            arrow.color.a = number.color.a = 1.0;
            arrow.id = number.id = pose_array_.poses.size();
            number.text = std::to_string(pose_array_.poses.size());
            marker_pub_->publish(arrow);
            marker_pub_->publish(number);
        }
    }

    // check whether it is in the cycling situation
    void MultiNaviGoalsPanel::checkCycle() {
        cycle_ = cycle_checkbox_->isChecked();
    }

    // start to navigate, and only command the first goal
    void MultiNaviGoalsPanel::startNavi() {
        if (!pose_array_.poses.empty()) {
            waypoints_pub_->publish(pose_array_);
            for(int curGoalIdx=0;curGoalIdx < maxNumGoal_;curGoalIdx++){
                poseArray_table_->item(curGoalIdx, 0)->setBackgroundColor(QColor(255, 69, 0));
                poseArray_table_->item(curGoalIdx, 1)->setBackgroundColor(QColor(255, 69, 0));
                poseArray_table_->item(curGoalIdx, 2)->setBackgroundColor(QColor(255, 69, 0));
            }
            permit_ = true;
        } else {
            RCLCPP_ERROR(nh_->get_logger(), "Something Wrong");
        }
    }

    // cancel the current command
    void MultiNaviGoalsPanel::cancelNavi() {
        if (!cur_goalid_.goal_id.uuid.empty()) {
            cancel_pub_->publish(std_msgs::msg::Empty());
            // auto cancel_request = std::make_shared<pp_msgs::action::TrajectoryControl::Impl::CancelGoalService::Request>();
            // cancel_request->goal_info = cur_goalid_;
            // cancel_client_->async_send_request(cancel_request);

            RCLCPP_ERROR(nh_->get_logger(), "Navigation have been canceled");
        }
    }
    // command the goals cyclically
    void MultiNaviGoalsPanel::cycleNavi() {
        if (permit_) {
            waypoints_pub_->publish(pose_array_);
            static bool even = 0;
            QColor color_table;
            if (even){
                color_table = QColor(255, 69, 0); 
            } else{
                color_table = QColor(100, 149, 237);
            }
            even = !even;
            for(int curGoalIdx=0;curGoalIdx < maxNumGoal_;curGoalIdx++){
                poseArray_table_->item(curGoalIdx, 0)->setBackgroundColor(color_table);
                poseArray_table_->item(curGoalIdx, 1)->setBackgroundColor(color_table);
                poseArray_table_->item(curGoalIdx, 2)->setBackgroundColor(color_table);
            }
        }
    }
    // call back for listening current state
    void MultiNaviGoalsPanel::statusCB(const action_msgs::msg::GoalStatusArray::SharedPtr statuses) {
        arrived_ = checkGoal(statuses->status_list);
        // if (arrived_ && rclcpp::ok() && permit_) {
        //     if (cycle_) cycleNavi();
        //     else completeNavi();
        // }
    }

    void MultiNaviGoalsPanel::missionCompleteCB(const std_msgs::msg::Empty::SharedPtr ) {
        if (cycle_) cycleNavi();
    }

    //check the current state of goal
    bool MultiNaviGoalsPanel::checkGoal(std::vector<action_msgs::msg::GoalStatus> status_list) {
        bool done;
        if (!status_list.empty()) {
            auto statu_i = status_list.back();
            // for (auto &i : status_list) {
                if (statu_i.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                    done = true;
                    RCLCPP_INFO(nh_->get_logger(), "completed Goal %d", curGoalIdx_);
                } else if (statu_i.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
                    RCLCPP_ERROR(nh_->get_logger(), "Goal %d is Invalid, Navi to Next Goal %d",curGoalIdx_, curGoalIdx_ + 1);
                    return true;
                } else if (statu_i.status == action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
                    done = true;
                } else if (statu_i.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                    cur_goalid_ = statu_i.goal_info;
                    done = false;
                } else done = false;
            // }
        } else {
            RCLCPP_INFO(nh_->get_logger(), "Please input the Navi Goal");
            done = false;
        }
        return done;
    }

// spin for subscribing
    void MultiNaviGoalsPanel::startSpin() {
        if (rclcpp::ok()) {
            rclcpp::spin_some(nh_);
        }
    }

} // end namespace navi-multi-goals-pub-rviz-plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz_common::Panel)

