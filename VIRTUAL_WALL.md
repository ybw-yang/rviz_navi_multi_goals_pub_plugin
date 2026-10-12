# RViz2 3D virtual wall editor

The wall editor and cloud height display are part of `navi_multi_goals_pub_rviz_plugin`, alongside its multi-goal panel and point-cloud goal tool. It lives next to `general_algorithm` and builds separately with colcon. Navigation continues to use its top-level CMake workflow. No launch file is required.

## Open the editor

Build and install the plugin in the workspace, then start RViz directly:

```bash
cd /home/ybw/navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select navi_multi_goals_pub_rviz_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
rviz2 -d src/general_algorithm/behavior_tree_test/rviz/pathplanning.rviz
```

The supplied RViz configuration contains the tool, panel, input cloud display, and wall marker display. In an existing RViz session, use **Panels > Add New Panel > navi_multi_goals_pub_rviz_plugin/VirtualWallPanel**, then **Draw**; the panel adds the tool if needed. The tool is also available through **Add Tool > navi_multi_goals_pub_rviz_plugin/VirtualWallTool**.

Set the fixed frame to the stable wall frame, normally `map`. Display `/tomogram/traversability_cloud` as selectable XYZ points with Reliable / Transient Local QoS. The editor requires rendered depth and snaps it to an actual input cloud point within the configured snap distance. Missing cloud data, depth, or TF prevents placement.

## Select displayed cloud heights

The supplied RViz configuration uses `navi_multi_goals_pub_rviz_plugin/HeightCloud` for `/tomogram/traversability_cloud` and includes the **Cloud Height Range** panel. Drag **Drag minimum** and **Drag maximum** to select the inclusive Z interval. Dragging automatically enables filtering; the numeric fields allow precise bounds. **Show all heights** restores the full cloud.

The slider span follows the original cloud's minimum and maximum finite Z values. Heights are in meters in the cloud's source frame, shown in the panel, normally `map`. Filtering applies immediately to the cached snapshot even if no new message arrives. Empty intervals clear the rendered cloud. Intensity/color fields and normal point size, style and selection properties are preserved. The input topic and navigation data are unchanged.

For an existing RViz configuration, replace the relevant ordinary PointCloud2 display with **Add > navi_multi_goals_pub_rviz_plugin/HeightCloud**, set its topic and Reliable / Transient Local QoS, and add **Panels > Add New Panel > navi_multi_goals_pub_rviz_plugin/HeightRangePanel**. Disable the previous full-cloud display so it does not cover the filtered view. The panel's dropdown selects among multiple HeightCloud displays. Save the RViz configuration to retain filter settings.

## Edit walls

1. Set **Height (m)** and **Thickness (m)**, then select **Draw** or press `W`.
2. Click two cloud points to place the bottom edge. Their original Z coordinates are retained. Move the cursor between clicks to preview the wall. Right-click or Escape cancels placement.
3. Select a wall in the panel, change start/end XYZ, height, or thickness, and press **Apply**. Walls are filled three-dimensional prisms; the bottom edge can slope between floors or along a ramp.
4. **Delete**, **Clear**, and **Undo** update navigation immediately. `U` also undoes while the drawing tool is active. **Publish** republishes the current complete snapshot, including an explicit clear for an empty list.
5. **Save** and **Load** persist the wall list in YAML. Saving the RViz configuration also stores its walls and restores/publishes them when opened again.

New walls extend along the fixed frame's positive Z direction. A loaded wall also retains its `Up` vector, allowing arbitrary orientations. Numeric coordinates are expressed in the wall frame shown in the panel. To add walls after changing RViz's fixed frame, return to the existing wall frame or clear the wall list first.

Changes publish automatically. Clear removes virtual wall costs while preserving underlying map obstacles. A snapshot supports at most 1000 walls and undo retains the last 100 edits. Run one editor publisher per wall topic; saving/loading is required to retain walls across process shutdown, because transient-local history belongs to the publisher.

## Navigation data flow

```text
/tomogram/traversability_cloud -> RViz depth + cloud snapping -> WallTool / WallPanel
WallTool -> /virtual_walls -> sensorInput -> immutable blackboard snapshot
snapshot -> map/odom TF -> globalMap + localMap -> cost / ESDF / obstacle cloud -> planners
```

`/virtual_walls` uses `visualization_msgs/msg/MarkerArray`, Reliable / Transient Local, depth 1. Every message is a complete versioned snapshot: a framed `DELETEALL` marker in namespace `navigation_virtual_wall/v1`, followed by one `LINE_STRIP` marker per wall. Its five points describe the closed wall parallelogram; `scale.x` carries wall thickness. Frames and identities must be consistent. Invalid or partial snapshots are rejected without replacing the previous navigation snapshot. A standard MarkerArray display shows wall outlines; the tool renders filled wall volumes and its preview.

This protocol replaces `/virtualwall_points`; old PCD point walls are not imported automatically. Robot YAML files use `data_receiver.virtual_wall_topic: /virtual_walls`. If you change the topic, update both the RViz tool property and the robot configuration.

The navigation overlay intersects each wall volume with each surface's robot body height interval from `robot_height_`, then applies horizontal footprint clearance and inflation. Overhead walls outside the body interval do not block that surface. Distinct floors remain independent; duplicate tomogram slices representing the same surface receive consistent wall costs. The overlay is restored/reapplied during map rebuilds and local buffer exchanges.

The plugin XML and ament index resource belong to `navi_multi_goals_pub_rviz_plugin`; `libnavi_multi_goals_pub_rviz_plugin.so` installs into that package's `lib` directory. Plugin class IDs are `navi_multi_goals_pub_rviz_plugin/VirtualWallTool` and `navi_multi_goals_pub_rviz_plugin/VirtualWallPanel`.

The plugin shares the pure geometry header from `general_algorithm/map_system/include` and the header-only ROS adapter from `general_algorithm/pathplanning_behavior_tree/include/behavior_tree/plugins/action/dataReceiver`. It does not link navigation libraries or require a colcon build of `general_algorithm`. `map_system` contains no wall-related ROS message encoding/decoding. For another directory layout, pass `-DVIRTUAL_WALL_GEOMETRY_INCLUDE_DIR=<geometry-include-root>` and `-DVIRTUAL_WALL_RECEIVER_INCLUDE_DIR=<receiver-include-root>` to colcon's CMake arguments.

## Validation targets

The colcon package uses `ament_cmake_gtest` for `cloud_height_filter_test` (inclusive ranges, empty selection, row padding, field retention, endianness, double coordinates and malformed input) and `virtual_wall_protocol_test`, covering the shared receiver adapter and pure geometry, including malformed snapshots, tilted/sloped walls and explicit clear messages:

```bash
colcon test --packages-select navi_multi_goals_pub_rviz_plugin --event-handlers console_direct+
colcon test-result --test-result-base build/navi_multi_goals_pub_rviz_plugin --verbose
```

The navigation CMake project also provides `virtual_wall_overlay_test` when `BUILD_TESTING=ON`, covering overhead walls, separate floors, duplicate surfaces, replacement, map rebuilds and local planning snapshots.
