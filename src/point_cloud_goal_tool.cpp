#include "point_cloud_goal_tool.hpp"

#include <cmath>

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>

#include <QCursor>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace navi_multi_goals_pub_rviz_plugin {

PointCloudGoalTool::PointCloudGoalTool()
{
    shortcut_key_ = 'g';
}

PointCloudGoalTool::~PointCloudGoalTool() = default;

void PointCloudGoalTool::onInitialize()
{
    arrow_ = std::make_shared<rviz_rendering::Arrow>(
        scene_manager_, nullptr,
        /*shaft_length=*/0.8f, /*shaft_diameter=*/0.1f,
        /*head_length=*/0.2f,  /*head_diameter=*/0.2f);
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);

    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
}

void PointCloudGoalTool::activate()
{
    state_ = kPosition;
    arrow_->getSceneNode()->setVisible(false);
}

void PointCloudGoalTool::deactivate()
{
    arrow_->getSceneNode()->setVisible(false);
}

bool PointCloudGoalTool::get3DPosition(
    rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & out)
{
    // Primary: depth-buffer pick — returns the real rendered 3-D point
    // (point-cloud voxels are rendered geometry, so their depth is correct).
    if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, out)) {
        return true;
    }
    // Fallback: intersect the camera ray with the z=0 XY plane so the tool
    // still works when the cursor is over empty space.
    rviz_rendering::ViewportProjectionFinder finder;
    auto [hit, pos] = finder.getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
    if (hit) {
        out = pos;
        return true;
    }
    return false;
}

int PointCloudGoalTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
    if (state_ == kPosition) {
        if (event.leftDown()) {
            Ogre::Vector3 pos;
            if (!get3DPosition(event, pos)) {
                return Render;
            }
            anchor_pos_ = pos;
            state_ = kOrientation;

            arrow_->setPosition(anchor_pos_);
            arrow_->getSceneNode()->setVisible(true);
            return Render;
        }
        return 0;
    }

    // kOrientation state: mouse is being dragged
    if (state_ == kOrientation) {
        Ogre::Vector3 cur;
        if (get3DPosition(event, cur)) {
            // Compute yaw from the XY displacement between anchor and current cursor
            double dx = cur.x - anchor_pos_.x;
            double dy = cur.y - anchor_pos_.y;
            double angle = std::atan2(dy, dx);

            // Orient the arrow: PoseTool convention — arrow points along +X in model
            // space, so we rotate from -Z (Ogre Arrow default forward) by (angle + π/2).
            Ogre::Quaternion q;
            q.FromAngleAxis(Ogre::Radian(angle + M_PI_2), Ogre::Vector3::UNIT_Z);
            // Arrow default up is +Y; tilt it to lie flat on XY plane:
            Ogre::Quaternion tilt;
            tilt.FromAngleAxis(Ogre::Radian(-M_PI_2), Ogre::Vector3::UNIT_X);
            arrow_->setOrientation(q * tilt);
        }

        if (event.leftUp()) {
            // Compute final yaw from anchor to current position
            Ogre::Vector3 cur2;
            if (get3DPosition(event, cur2)) {
                double dx = cur2.x - anchor_pos_.x;
                double dy = cur2.y - anchor_pos_.y;
                double yaw = std::atan2(dy, dx);
                publishGoal(anchor_pos_, yaw);
            }
            arrow_->getSceneNode()->setVisible(false);
            state_ = kPosition;
            return Render | Finished;
        }
        return Render;
    }
    return 0;
}

void PointCloudGoalTool::publishGoal(const Ogre::Vector3 & pos, double yaw_rad)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = node_->get_clock()->now();
    ps.header.frame_id = context_->getFixedFrame().toStdString();

    ps.pose.position.x = static_cast<double>(pos.x);
    ps.pose.position.y = static_cast<double>(pos.y);
    ps.pose.position.z = static_cast<double>(pos.z);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    ps.pose.orientation = tf2::toMsg(q);

    pub_->publish(ps);
    RCLCPP_INFO(node_->get_logger(),
        "PointCloudGoalTool: (%.2f, %.2f, %.2f) yaw=%.1f°",
        pos.x, pos.y, pos.z, yaw_rad * 180.0 / M_PI);
}

}  // namespace navi_multi_goals_pub_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
    navi_multi_goals_pub_rviz_plugin::PointCloudGoalTool,
    rviz_common::Tool)
