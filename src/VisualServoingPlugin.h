/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rtc_ros/ros.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#ifdef __linux__
#  include <sched.h>

void reset_affinity()
{
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  for(unsigned int i = 0; i < std::thread::hardware_concurrency(); i++)
  {
    CPU_SET(i, &cpu_set);
  }
  int result = sched_setaffinity(0, sizeof(cpu_set_t), &cpu_set);
  if(result != 0) std::perror("sched_setaffinity");
}
#else

void reset_affinity() {}
#endif

namespace mc_plugin
{

struct VisualServoingPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~VisualServoingPlugin() override;

private:
  struct VisualServoingBodySensor
  {
    VisualServoingBodySensor(const std::string & bodyname, const std::string & topic);
    void init(rclcpp::Node::SharedPtr & node);

    void callback(const geometry_msgs::msg::Twist & vel);

    std::mutex vel_mutex_;
    sva::MotionVecd vel_bodysensor_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr topic_disconnection_timer_;
    std::string topic_;
    std::string bodyname_;
  };

  std::unordered_map<std::string, std::shared_ptr<VisualServoingBodySensor>> vs_bodysensors_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread;

  void loadConfig(const mc_rtc::Configuration & config);
  void addToGui(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & log);
};

} // namespace mc_plugin
