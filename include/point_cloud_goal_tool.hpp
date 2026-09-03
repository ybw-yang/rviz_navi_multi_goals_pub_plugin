#ifndef NAVI_MULTI_GOALS_PUB_RVIZ_PLUGIN__POINT_CLOUD_GOAL_TOOL_HPP_
#define NAVI_MULTI_GOALS_PUB_RVIZ_PLUGIN__POINT_CLOUD_GOAL_TOOL_HPP_

#include <memory>

#include <OgreVector3.h>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/objects/arrow.hpp>

namespace navi_multi_goals_pub_rviz_plugin {

/// A custom RViz tool that picks a 3-D point directly off the depth buffer
/// (i.e. the real geometry rendered on screen, including point-cloud voxels)
/// and lets the user drag to specify yaw, then publishes the result as a
/// geometry_msgs/PoseStamped on /goal_pose.
///
/// Interaction is identical to the built-in "2D Goal Pose" tool:
///   • left-click and hold  → anchor position (snapped to rendered geometry)
///   • drag                 → preview arrow shows chosen heading
///   • release              → publish PoseStamped and deactivate arrow
///
/// Falls back to the XY-plane intersection when the depth buffer returns no
/// hit at the cursor position (e.g. the cursor is over empty space).
class PointCloudGoalTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PointCloudGoalTool();
  ~PointCloudGoalTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int  processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  // return the 3-D world position under the cursor:
  //   1st choice – depth-buffer hit (real rendered geometry)
  //   2nd choice – XY-plane intersection (fallback)
  bool get3DPosition(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & out);

  void publishGoal(const Ogre::Vector3 & pos, double yaw_rad);

  enum State { kPosition, kOrientation };
  State state_{kPosition};

  Ogre::Vector3 anchor_pos_{0, 0, 0};

  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

}  // namespace navi_multi_goals_pub_rviz_plugin
#endif  // NAVI_MULTI_GOALS_PUB_RVIZ_PLUGIN__POINT_CLOUD_GOAL_TOOL_HPP_
