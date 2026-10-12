# RViz2 navigation and map editing plugins

The source directory is `rviz_navi_multi_goals_pub_plugin`; the ROS package name is `navi_multi_goals_pub_rviz_plugin`. This package contains all navigation, wall editing and cloud height controls in one shared library.

| Plugin class | Function |
| --- | --- |
| `navi_multi_goals_pub_rviz_plugin/MultiNaviGoalsPanel` | Collect goal poses and start, cancel or repeat a waypoint mission. |
| `navi_multi_goals_pub_rviz_plugin/PointCloudGoalTool` | Pick a 3D goal from rendered cloud geometry and drag to set heading. |
| `navi_multi_goals_pub_rviz_plugin/VirtualWallTool` | Select two cloud endpoints and preview a 3D virtual wall. |
| `navi_multi_goals_pub_rviz_plugin/VirtualWallPanel` | Edit wall geometry, undo, delete, clear, save and load. |
| `navi_multi_goals_pub_rviz_plugin/HeightCloud` | Display a PointCloud2 within a chosen source-frame Z interval. |
| `navi_multi_goals_pub_rviz_plugin/HeightRangePanel` | Drag the lower and upper cloud height bounds. |

## Build and open

```bash
cd /home/ybw/navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select navi_multi_goals_pub_rviz_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
rviz2 -d src/general_algorithm/behavior_tree_test/rviz/pathplanning.rviz
```

The navigation project remains a separate CMake project. Wall geometry and its ROS adapter are shared as headers from the sibling `general_algorithm` source tree; the plugin links no navigation library. See [virtual wall and height filtering usage](VIRTUAL_WALL.md) for configuration, editing, persistence and tests.

## Multi-goal navigation

Add `MultiNaviGoalsPanel` through **Panels > Add New Panel**. Select goals with RViz's goal tool on `/goal_pose` or the package's `PointCloudGoalTool`; both position and heading are retained. The panel also accepts `/waypoint_model/goal_temp`.

The panel publishes waypoint missions on `/wapoints_pose`, cancellation on `/mission_cancel`, and goal markers on `/waypoint_model/visualization_marker`. `/mission_complete` advances a running mission. Enable cycling before starting to repeat the route. Reset clears goals and cancels a running mission. The goal tool uses rendered depth and falls back to the XY plane when no geometry is under the cursor.

## Migration from the separate wall package

`rviz_virtual_wall` has been merged into this package. Its source package is no longer needed. The supplied RViz configuration uses the unified class IDs above. For saved configurations, replace the `rviz_virtual_wall/` class prefix with `navi_multi_goals_pub_rviz_plugin/`; wall geometry and height properties keep the same format. `/virtual_walls` and `/tomogram/traversability_cloud` keep their existing message protocols.
