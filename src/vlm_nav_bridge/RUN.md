# pixel_to_goal — run notes

Mini-project 4.1. Two functions are stubbed for you in
`vlm_nav_bridge/pixel_to_goal.py`: `deproject()` and `standoff_pose()`.
Everything else runs.

## 0. Place the package

Colcon only discovers packages under `~/ros2_ws/src/`.

```bash
mkdir -p ~/ros2_ws/src/aries-vlm-nav
cp -r vlm_nav_bridge ~/ros2_ws/src/aries-vlm-nav/
```

## 1. Find the real topic and frame names first

These differ between TB3 models and between sim and hardware. Do not guess.

```bash
# terminal 1 — sim
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# terminal 2
ros2 topic list | grep -i depth
ros2 run tf2_tools view_frames        # writes frames.pdf — open it
```

From `view_frames` confirm three things:

1. the exact spelling of the optical frame
2. that `map` is connected to the robot's subtree — if it is not, localization
   is not running and you will get `ConnectivityException`
3. the base frame name (`base_link` vs `base_footprint`)

## 2. Bring up Nav2 with a map

```bash
# terminal 3
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
     use_sim_time:=True map:=$HOME/map.yaml
```

Then set the initial pose in RViz (2D Pose Estimate) — AMCL publishes nothing
until it is initialised, so `map -> odom` will not exist before you do this.

## 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select vlm_nav_bridge --symlink-install
source install/setup.bash
```

`--symlink-install` means you can edit the Python and re-run without rebuilding.

## 4. Marker only, first

Do not send a goal until the marker lands on the object.

```bash
ros2 run vlm_nav_bridge pixel_to_goal --ros-args \
  -p use_sim_time:=true \
  -p send_goal:=false \
  -p pixel_u:=320 -p pixel_v:=240 \
  -p depth_topic:=/camera/depth/image_raw \
  -p info_topic:=/camera/depth/camera_info \
  -p optical_frame:=camera_depth_optical_frame
```

In RViz add a **Marker** display on `/pixel_to_goal/target_marker`, fixed frame
`map`.

`use_sim_time:=true` is not optional. Nav2 and Gazebo are on sim time; if this
node is on wall time, every lookup fails with `ExtrapolationException` and the
cause is not obvious from the message.

### Reading the result

| what you see | what it means |
|---|---|
| marker on the object | steps 1–3 correct — go to step 5 |
| marker rotated ~90° off | wrong `frame_id`. You want the **optical** frame, not `camera_depth_frame` |
| marker 1000× too far | depth units — `16UC1` is millimetres, `32FC1` is metres |
| `no valid depth near (u,v)` | pixel is on a hole. Aim at something matte and solid |
| `ConnectivityException` | no `map -> odom`. AMCL not running, or initial pose not set |
| `ExtrapolationException` | usually `use_sim_time` mismatch |

## 5. Send the goal

```bash
ros2 run vlm_nav_bridge pixel_to_goal --ros-args \
  -p use_sim_time:=true -p send_goal:=true -p standoff_radius:=0.6
```

**Done when:** the marker sits on the object and the robot drives to a sensible
pose in front of it, facing it.

## Notes

- The node is one-shot by design (`self.done`) — it sends a single goal per run
  rather than re-goaling on every depth frame.
- `depth_at()` takes the median over a small patch rather than a single pixel,
  because a single depth pixel is often a hole. `patch:=0` for one pixel.
- Stretch goal, once this works: replace the hardcoded pixel with a call to
  Molmo or Qwen3-VL on the RGB frame. Watch the coordinate conventions —
  Molmo returns 0–100 percent, Qwen3-VL returns 0–1000.
