#include "VisualServoingPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/logging.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <chrono>
#include <functional>
#include <mc_rtc_ros/ros.h>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <thread>

namespace mc_plugin
{

VisualServoingPlugin::~VisualServoingPlugin()
{
  executor_->cancel();
  if(spin_thread.joinable())
  {
    spin_thread.join();
  }
};

void VisualServoingPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  controller.controller().config().load(config);
  reset(controller);

  addToGui(*controller.controller().gui());
  addToLogger(controller.controller().logger());

  mc_rtc::log::info("[VisualServoingPlugin] Initialisation done");
}

void VisualServoingPlugin::reset(mc_control::MCGlobalController & controller)
{
  node_ = rclcpp::Node::make_shared("VisualServoingPlugin");
  executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  executor_->add_node(node_);

  spin_thread = std::thread(
      [&]
      {
        std::cout << "[VisualServoingPlugin] Executor spinning..." << std::endl;
        executor_->spin();
        std::cout << "[VisualServoingPlugin] Executor stopped (unexpected)" << std::endl;
      });

  loadConfig(controller.controller().config());

  for(auto & vs_sensor : vs_bodysensors_)
  {
    vs_sensor.second->init(node_);
    controller.robot().addBodySensor(
        mc_rbdyn::BodySensor(vs_sensor.first, vs_sensor.second->bodyname_, sva::PTransformd::Identity()));
  }
}

void VisualServoingPlugin::loadConfig(const mc_rtc::Configuration & config)
{
  const auto & vs_plugin_config_ = config.find("VisualServoingPlugin");

  if(vs_plugin_config_)
  {
    for(auto it = vs_plugin_config_->begin(); it != vs_plugin_config_->end(); ++it)
    {
      const auto & vs_cfg = *it;
      auto vs_sensor = std::make_shared<VisualServoingBodySensor>(vs_cfg("body"), vs_cfg("twist_topic"));

      vs_bodysensors_.emplace(vs_cfg("name"), std::move(vs_sensor));
    }
  }
  else
  {
    mc_rtc::log::error("[VisualServoingPlugin] Cannot find VisualServoingPlugin configuration");
  }
}

void VisualServoingPlugin::before(mc_control::MCGlobalController & controller)
{
  for(const auto & sensor : vs_bodysensors_)
  {
    auto & s = const_cast<mc_rbdyn::BodySensor &>(controller.robot().bodySensor(sensor.first));
    {
      const std::lock_guard<std::mutex> lock(sensor.second->vel_mutex_);
      s.angularVelocity(sensor.second->vel_bodysensor_.angular());
      s.linearVelocity(sensor.second->vel_bodysensor_.linear());
    }
  }
}

void VisualServoingPlugin::after(mc_control::MCGlobalController & controller) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration VisualServoingPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

void VisualServoingPlugin::addToGui(mc_rtc::gui::StateBuilder & gui)
{
  for(const auto & sensor : vs_bodysensors_)
  {
    gui.addElement({"VisualServoingPlugin"},
                   mc_rtc::gui::ArrayLabel(fmt::format("Vel [{}] :", sensor.first),
                                           [&]
                                           {
                                             const std::lock_guard<std::mutex> lock(sensor.second->vel_mutex_);
                                             return sensor.second->vel_bodysensor_;
                                           }));
  }
}

void VisualServoingPlugin::addToLogger(mc_rtc::Logger & log)
{
  for(const auto & sensor : vs_bodysensors_)
  {
    log.addLogEntry(fmt::format("VisualServoingPlugin:{}", sensor.first), this,
                    [&]
                    {
                      const std::lock_guard<std::mutex> lock(sensor.second->vel_mutex_);
                      return sensor.second->vel_bodysensor_;
                    });
  }
}

/** VisualServoingBodySensor **/

VisualServoingPlugin::VisualServoingBodySensor::VisualServoingBodySensor(const std::string & bodyname,
                                                                         const std::string & topic)
: topic_(topic), bodyname_(bodyname)
{
}

void VisualServoingPlugin::VisualServoingBodySensor::init(rclcpp::Node::SharedPtr & node)
{
  if(node != nullptr)
  {
    sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        topic_, 1, std::bind(&VisualServoingPlugin::VisualServoingBodySensor::callback, this, std::placeholders::_1));
    mc_rtc::log::info("[VisualServoingPlugin] BodySensor attached to {} is connected to {}", bodyname_, topic_);

    // Create a 1ms timer to check if the publisher is still reachable, if not set vel to 0
    topic_disconnection_timer_ = node->create_wall_timer(std::chrono::milliseconds(0),
                                                         [&]
                                                         {
                                                           if(sub_->get_publisher_count() == 0)
                                                           {
                                                             const std::lock_guard<std::mutex> lock(vel_mutex_);
                                                             vel_bodysensor_ = sva::MotionVecd::Zero();
                                                           }
                                                         });
  }
  else
  {
    mc_rtc::log::error("[VisualServoingPlugin] mc_rtc ros node is not initialized");
  }
}

void VisualServoingPlugin::VisualServoingBodySensor::callback(const geometry_msgs::msg::Twist & vel)
{
  const std::lock_guard<std::mutex> lock(vel_mutex_);
  vel_bodysensor_ = {{vel.angular.x, vel.angular.y, vel.angular.z}, {vel.linear.x, vel.linear.y, vel.linear.z}};
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("VisualServoingPlugin", mc_plugin::VisualServoingPlugin)
