# 4.1: pixel to map to Nav2 goal

A ROS 2 (Jazzy) node that takes one image pixel, turns it into a point in the map frame, and sends the robot to a standoff pose in front of it. In the full stack the pixel would come from a VLM pointing output; here it is a parameter.

```
(u, v) + depth -> camera optical frame -> map frame (tf2, image timestamp)
               -> RViz marker -> standoff pose -> Nav2 goToPose
```

## Files

- `vlm_nav_bridge/geometry.py`: `deproject` and `standoff_pose`. Pure Python, no ROS.
- `vlm_nav_bridge/pixel_to_goal.py`: the node (depth, tf2, marker, Nav2).
- `test/test_geometry.py`: unit tests for the geometry.

## Run it

Terminal 1, simulation:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch ~/ros2_ws/custom_launch/turtlebot3_world_headless.launch.py
```

Use the normal Gazebo launch instead if you want the window. Check that `/camera/depth/image_raw` and `/camera/depth/camera_info` exist with `ros2 topic list | grep depth`.

Terminal 2, build and check the marker first (no motion):

```bash
cd ~/ros2_ws && colcon build --packages-select vlm_nav_bridge && source install/setup.bash
ros2 run vlm_nav_bridge pixel_to_goal --ros-args -p send_goal:=false -p map_frame:=odom
```

Open RViz, add a Marker display on `/pixel_to_goal/target_marker`, and confirm the sphere sits where the pixel points. `map_frame:=odom` is for a simulation without SLAM or AMCL. With Nav2 and AMCL running, use `map`.

Terminal 3, with Nav2 and AMCL running, send the robot:

```bash
ros2 run vlm_nav_bridge pixel_to_goal --ros-args -p pixel_u:=600 -p pixel_v:=400 -p standoff_radius:=0.6
```

## Unit tests (no ROS needed)

```bash
cd ~/ros2_ws/src/aries-vlm-nav/src/vlm_nav_bridge && python3 -m pytest -q
```

## Status

- `deproject` and `standoff_pose` are unit-tested (10 tests: axis directions, depth scaling, radius, facing direction, robot side, already-inside cases).
- Depth, deprojection and the tf2 transform to the marker were checked in the sim (marker against the lidar range of about 4.7 m).
- The standoff pose and the Nav2 `goToPose` path have **not yet been run end to end in the sim**. Run the three commands above and note the result here before presenting it as working.

## Design notes

- The point is stamped with the image timestamp, not `now()`, so tf2 uses the robot's pose when the frame was captured.
- `depth_at` takes a median over a small patch and ignores zero and NaN pixels, so one hole in the depth image does not kill the goal.
- The goal is stored in the callback and sent from `main()`. `BasicNavigator` spins its own futures, which should not happen inside a subscription callback.
- The standoff goal sits `radius` metres short of the target on the robot's side. A goal on the surface itself would land inside an obstacle in the costmap.
