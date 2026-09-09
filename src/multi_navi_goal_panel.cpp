#include <cstdio>
#include <fstream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/impl/utils.h"
#include "rviz_common/display_context.hpp"

#include "multi_navi_goal_panel.hpp"

namespace navi_multi_goals_pub_rviz_plugin {

MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
    : rviz_common::Panel(parent)
{
    nh_ = std::make_shared<rclcpp::Node>("navi_multi_goals");
    waypoints_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>("/wapoints_pose", 1);
    // Depth must cover a burst: every goal publishes 2 markers (arrow + number),
    // and a reset publishes a DELETEALL. A depth-1 reliable queue can drop
    // samples of such a burst before the display ever sees them.
    marker_pub_    = nh_->create_publisher<visualization_msgs::msg::Marker>(
        "/waypoint_model/visualization_marker", rclcpp::QoS(100));
    cancel_pub_    = nh_->create_publisher<std_msgs::msg::Empty>("/mission_cancel", 1);

    QVBoxLayout *root_layout = new QVBoxLayout;

    // ── Goal table ────────────────────────────────────────────────
    poseArray_table_ = new QTableWidget;
    initPoseTable();
    root_layout->addWidget(poseArray_table_);

    cycle_checkbox_ = new QCheckBox("循环");
    root_layout->addWidget(cycle_checkbox_);

    // ── Control row ───────────────────────────────────────────────
    QHBoxLayout *ctrl_layout = new QHBoxLayout;
    output_reset_button_     = new QPushButton("重置");
    output_cancelNavi_button_ = new QPushButton("结束导航");
    output_startNavi_button_ = new QPushButton("开始导航!");
    // Cancel only makes sense once a mission is running.
    output_cancelNavi_button_->setEnabled(false);
    ctrl_layout->addWidget(output_reset_button_);
    ctrl_layout->addWidget(output_cancelNavi_button_);
    ctrl_layout->addWidget(output_startNavi_button_);
    root_layout->addLayout(ctrl_layout);

    setLayout(root_layout);

    // ── Signals ───────────────────────────────────────────────────
    connect(output_reset_button_, SIGNAL(clicked()),       this, SLOT(initPoseTable()));
    connect(output_cancelNavi_button_, SIGNAL(clicked()),  this, SLOT(cancelNavi()));
    connect(output_startNavi_button_, SIGNAL(clicked()),   this, SLOT(startNavi()));
    connect(cycle_checkbox_,      SIGNAL(clicked(bool)),   this, SLOT(checkCycle()));

    QTimer *spin_timer = new QTimer(this);
    spin_timer->start(200);
    connect(spin_timer, SIGNAL(timeout()), this, SLOT(startSpin()));

    // backward-compat: accept goals from external goal_temp topic
    goal_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/waypoint_model/goal_temp", 1,
        std::bind(&MultiNaviGoalsPanel::goalCntCB, this, std::placeholders::_1));

    // "2D Goal Pose" tool in RViz — gives position + orientation in one interaction
    goal_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&MultiNaviGoalsPanel::goalPoseCB, this, std::placeholders::_1));

    status_sub_ = nh_->create_subscription<std_msgs::msg::Empty>(
        "/mission_complete", 1,
        std::bind(&MultiNaviGoalsPanel::statusCB, this, std::placeholders::_1));
}

void MultiNaviGoalsPanel::onInitialize() {
    // initialize() has stored the DisplayContext by the time this runs.
    context_ready_ = true;
}

// ── Table ─────────────────────────────────────────────────────────

void MultiNaviGoalsPanel::initPoseTable() {
    RCLCPP_INFO(nh_->get_logger(), "Initialize");
    // Reset drops the goals the robot is driving to, so it must also stop the
    // mission — otherwise the robot keeps following a route the panel forgot.
    if (permit_) cancelNavi();
    curGoalIdx_ = 0; cycleCnt_ = 0;
    permit_ = false; cycle_ = false;
    poseArray_table_->clear();
    pose_array_.poses.clear();
    deleteMark();
    poseArray_table_->setRowCount(0);
    poseArray_table_->setColumnCount(3);
    poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    poseArray_table_->setHorizontalHeaderLabels(QStringList() << "x" << "y" << "yaw");
    if (cycle_checkbox_) cycle_checkbox_->setCheckState(Qt::Unchecked);
}

// ── Marker ────────────────────────────────────────────────────────

std::string MultiNaviGoalsPanel::markerFrame() const {
    // The constructor calls initPoseTable() -> deleteMark() -> here, long before RViz
    // calls initialize(). Panel::context_ is raw and uninitialised until then, so
    // getDisplayContext() would hand back garbage and the virtual call would segfault.
    if (context_ready_) {
        if (const auto *ctx = getDisplayContext()) {
            std::string fixed = ctx->getFixedFrame().toStdString();
            if (!fixed.empty()) return fixed;
        }
    }
    if (!pose_array_.header.frame_id.empty()) return pose_array_.header.frame_id;
    return "map";
}

void MultiNaviGoalsPanel::deleteMark() {
    visualization_msgs::msg::Marker m;
    // RViz's Marker display is a MessageFilterDisplay: every marker, DELETEALL
    // included, passes through a tf2 MessageFilter that silently drops messages
    // with an empty frame_id. Without a frame the clear never arrives and the
    // previous round's markers stay on screen.
    m.header.frame_id = markerFrame();
    m.header.stamp    = nh_->get_clock()->now();
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(m);
}

void MultiNaviGoalsPanel::writePose(geometry_msgs::msg::Pose pose) {
    int row = static_cast<int>(pose_array_.poses.size()) - 1;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double yaw = tf2::impl::getYaw(q);
    poseArray_table_->setItem(row, 0, new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
    poseArray_table_->setItem(row, 1, new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
    poseArray_table_->setItem(row, 2, new QTableWidgetItem(QString::number(yaw * 180.0 / M_PI, 'f', 1)));
}

void MultiNaviGoalsPanel::markPose(const geometry_msgs::msg::PoseStamped::SharedPtr &pose) {
    if (!rclcpp::ok()) return;
    visualization_msgs::msg::Marker arrow, number;
    arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
    arrow.ns  = "navi_point_arrow";
    number.ns = "navi_point_number";
    arrow.action  = number.action = visualization_msgs::msg::Marker::ADD;
    arrow.type  = visualization_msgs::msg::Marker::ARROW;
    number.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    arrow.pose = number.pose = pose->pose;
    number.pose.position.z += 1.0;
    arrow.scale.x  = 1.0;  arrow.scale.y = 0.2;
    number.scale.z = 1.0;
    arrow.color.r = number.color.r = 1.0f;
    arrow.color.g = number.color.g = 0.98f;
    arrow.color.b = number.color.b = 0.80f;
    arrow.color.a = number.color.a = 1.0;
    arrow.id = number.id = static_cast<int>(pose_array_.poses.size());
    number.text = std::to_string(pose_array_.poses.size());
    marker_pub_->publish(arrow);
    marker_pub_->publish(number);
}

// ── Add goal (shared path) ────────────────────────────────────────

void MultiNaviGoalsPanel::addGoal(const geometry_msgs::msg::PoseStamped &ps) {
    pose_array_.poses.push_back(ps.pose);
    pose_array_.header.frame_id = ps.header.frame_id;
    // auto-expand table
    poseArray_table_->setRowCount(static_cast<int>(pose_array_.poses.size()));
    writePose(ps.pose);
    auto ps_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
    markPose(ps_ptr);
}

// ── Subscriptions ─────────────────────────────────────────────────

void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    addGoal(*pose);
}

void MultiNaviGoalsPanel::goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    addGoal(*pose);
}

void MultiNaviGoalsPanel::statusCB(const std_msgs::msg::Empty::SharedPtr) {
    if (rclcpp::ok() && permit_ && cycle_) startNavi();
}

// ── Navigation ────────────────────────────────────────────────────

void MultiNaviGoalsPanel::startNavi() {
    if (pose_array_.poses.empty()) {
        RCLCPP_ERROR(nh_->get_logger(), "No goals set");
        return;
    }
    curGoalIdx_ = curGoalIdx_ % static_cast<int>(pose_array_.poses.size());
    pose_array_.header.stamp = nh_->get_clock()->now();
    waypoints_pub_->publish(pose_array_);
    for (int col = 0; col < 3; ++col)
        if (poseArray_table_->item(curGoalIdx_, col))
            poseArray_table_->item(curGoalIdx_, col)->setBackground(QColor(255, 69, 0));
    permit_ = true;
    if (output_cancelNavi_button_) output_cancelNavi_button_->setEnabled(true);
}

void MultiNaviGoalsPanel::cancelNavi() {
    if (!rclcpp::ok()) return;
    // pp_TrajectoryCore consumes this as a one-shot flag off the blackboard and
    // halts its running children; the goal list itself is left intact so the
    // same route can be restarted with "开始导航!".
    cancel_pub_->publish(std_msgs::msg::Empty());
    RCLCPP_WARN(nh_->get_logger(), "mission cancel requested");
    permit_ = false;
    cycle_  = false;
    if (cycle_checkbox_) cycle_checkbox_->setCheckState(Qt::Unchecked);
    // Drop the "current goal" highlight — nothing is being driven to any more.
    for (int row = 0; row < poseArray_table_->rowCount(); ++row)
        for (int col = 0; col < 3; ++col)
            if (poseArray_table_->item(row, col))
                poseArray_table_->item(row, col)->setBackground(QBrush());
    if (output_cancelNavi_button_) output_cancelNavi_button_->setEnabled(false);
}

void MultiNaviGoalsPanel::checkCycle() {
    cycle_ = cycle_checkbox_->isChecked();
}

void MultiNaviGoalsPanel::startSpin() {
    if (rclcpp::ok()) rclcpp::spin_some(nh_);
}

} // namespace navi_multi_goals_pub_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz_common::Panel)
