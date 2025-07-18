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

> A new tab `VisualServoingPlugin` should appear in `mc_rtc_panel`.
