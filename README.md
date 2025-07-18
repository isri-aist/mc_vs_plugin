# mc_vs_plugin

This mc_rtc plugin adds and updates VisualServoing BodySensor(s) to your robot.

## Dependencies

* mc_rtc
* rclcpp
* geometry_msgs

## Install

```bash
git clone https://github.com/isri-aist/mc_vs_plugin
cd mc_vs_plugin
mkdir build && cd build
# Please edit the INSTALL_PREFIX depending on your installation
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${HOME}/workspace/install
make
make install
```

## Usage

In order to use this plugin, please consider adding the following line to your configuration file (`~/.config/mc_rtc/mc_rtc.yaml`).

```yaml
Plugins: [VisualServoingPlugin]
```

In order to configure your plugin, you have to edit the config file `etc/VisualServoingPlugin.yaml` or mc_rtc configuration file.
VisualServoingPlugin configuration is expressed as follow :

```yaml
---
VisualServoingPlugin:
  - name: "hand_visual_servoing"
    twist_topic: "/twist_msg"
    body: "tool0" # example for UR robots

  - name: "head_visual_servoing"
    twist_topic: "/twist_msg_head"
    body: "tool0"
```

When running your controller, the plugin will start to subscribe to the given topics.
If the topics cannot be reached or the connection is being dropped, the velocity will be set to a zero vector.

>
> For information : \
> A new tab `VisualServoingPlugin` should appear in `mc_rtc_panel`. A log entry is added for each sensor.

To get the BodySensor values in your controller, please consider using the following lines :

```cpp
robot().bodySensor("hand_visual_servoing").linearVelocity();
robot().bodySensor("hand_visual_servoing").angularVelocity();
```

## Test

To test the plugin you can run the following command to publish a `geometry_msgs::msg::Twist` message :

```bash
ros2 topic pub -r 10 /twist_msg geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The velocity information can then be retrieved in your controller by using the lines mentionned above.
