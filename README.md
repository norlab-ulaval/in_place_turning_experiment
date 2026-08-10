# In-place Turning Experiment

Records the transient motion of a skid-steering robot turning in place.

`in_place_turning_experiment_node` integrates the IMU gyroscope to track the robot's heading, then runs a
sweep of _runs_. One run = command a constant angular velocity until the robot has rotated a target angle,
then cut the command to zero and wait while the robot coasts and settles. The angular velocity is then
incremented and the next run starts. The sweep ends when the next increment would exceed the maximum
velocity.

## Installation

1. Clone the repo in a ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/norlab-ulaval/in_place_turning_experiment.git
```

2. Install rosdeps

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev --packages-select micro_drive
source install/setup.bash
```

## Interfaces

Remap the in/out topics inside the launch file [micro_drive.launch.py](micro_drive/launch/micro_drive.launch.py). (Internal topics can stay as is)

| Topic                   | Type                           | Direction | Description                                           |
| ----------------------- | ------------------------------ | --------- | ----------------------------------------------------- |
| `/imu/data_raw`         | `sensor_msgs/Imu`              | in        | Raw IMU, feeds the gyro bias observer and compensator |
| `/teleop/lock_autonomy` | `std_msgs/Bool`                | in        | `true` = deadman released → pause the experiment      |
| `/controller/cmd_vel`   | `geometry_msgs/TwistStamped`   | out       | Angular velocity command sent to the robot            |
| `/imu/bias`             | `geometry_msgs/Vector3Stamped` | internal  | Estimated gyro bias, published once                   |
| `/imu/data_unbiased`    | `sensor_msgs/Imu`              | internal  | Bias-compensated IMU consumed by the experiment node  |

The experiment node also requires a valid tf between `imu_frame` and `base_frame`. So a robot_state_publisher must be publishing static TFs.

## IMU Bias Observer/Compensator

Gyroscope integration accumulates the gyro's bias, so the bias has to be removed before the experiment starts. Two nodes launched
alongside the experiment node will take care of it.

`imu_bias_observer` averages the first 4000 samples of `/imu/data_raw` (~20 s at 200 Hz, ~40 s at 100 Hz) and
publishes the mean angular velocity once on `/imu/bias`. **The robot must stay stationary for the whole
observation**, otherwise the motion is baked into the bias estimate.

`imu_bias_compensator_node` subscribes to `/imu/bias` and `/imu/data_raw`, subtracts the bias from the angular
velocity and republishes on `/imu/data_unbiased`, which is what the experiment node consumes. It publishes
nothing until the bias arrives, so `/imu/data_unbiased` staying silent means the observation is not finished. 

## Configuration

Parameters live in [in_place_turning_experiment_node.yaml](micro_drive/config/in_place_turning_experiment_node.yaml). You need to edit them so that the velocities and timing match what the robot can do.

> ⚠️ **Warning: Low-level controller acceleration limits**
> If the robot's low-level controller enforces acceleration limits, commanded accelerations must always stay within them. Exceeding these limits causes the controller to clamp or smooth commands.

### Velocity sweep

| Parameter                        | Default | Description                                                           |
| -------------------------------- | ------- | --------------------------------------------------------------------- |
| `start_angular_velocity_rad`     | `0.6`   | Angular velocity of the first run, rad/s. Must be positive            |
| `angular_velocity_increment_rad` | `0.1`   | Increment added after each run, rad/s. Must be positive               |
| `target_angular_velocity_rad`    | `2.0`   | Sweep stops once the next step would exceed this, rad/s               |
| `invert_rotation`                | `false` | Publish the negative of the commanded velocity, to turn the other way |

### Timing

| Parameter              | Default | Description                                                                                                                                                                        |
| ---------------------- | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `min_run_time_seconds` | `2.0`   | Time the robot needs to reach steady-state rotation. Sets the run's target angle with `angular_velocity * min_run_time_seconds`, so every run spends roughly the same time turning |
| `wait_time_seconds`    | `2.5`   | Pause after the command is cut. The robot should not be wiggling after that time. Better to make it longer than shorter                                                            |
| `publish_frequency_hz` | `20.0`  | Rate of the `/controller/cmd_vel` and `~/state` publishers                                                                                                                         |

### IMU

| Parameter    | Default     | Description      |
| ------------ | ----------- | ---------------- |
| `imu_frame`  | `imu_link`  | IMU frame        |
| `base_frame` | `base_link` | Robot body frame |

The node looks up the static `imu_frame` → `base_frame` transform at startup and uses it to project the gyro
onto `base_link`'s z axis, so the IMU can be mounted in any orientation. **This transform must be available**
(robot description or a `static_transform_publisher`): the node retries for 10 s and then exits with an error if it can't find it.

## Running an experiment

1. Place the robot in open space with room to rotate. It should be on hard flat terrain (Asphalt, concrete, tiles, etc.)

2. Launch a rosbag recording

```bash
ros2 bag record -a
```

3. Launch the experiment and keep the robot **stationary**.

```bash
ros2 launch micro_drive micro_drive.launch.py
```

4. Wait for the bias estimation. Once you see `Bias acquired` in the console, you can continue

5. Teleoperate the robot around a bit just so we have a few lidar scans for offline mapping afterwards.

6. Call this service to start the experiment:

```bash
ros2 service call /in_place_turning_experiment_node/start_experiment std_srvs/srv/Trigger
```

The call is rejected if the experiment is already running or if no IMU sample has been received yet.

7. Press the deadman switch, the robot should start spinning. You can release it at any point during the experiment if needed.

8. Watch the logs. Each run reports the rotation achieved and the velocity used; the node prints
   `Experiment complete.` when the sweep is done.

9. Calling the service again restarts the whole sweep from `start_angular_velocity_rad`.

```bash
ros2 service call /in_place_turning_experiment_node/start_experiment std_srvs/srv/Trigger
```

10. You can stop after 3-5 sweeps.
