/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_control/MCController.h>
#include <mc_rtc/Schema.h>
#include <mc_rtc/gui/StateBuilder.h>
#include "Schemas.h"

namespace mc_plugin
{

struct ExtraRobot
{
  ExtraRobot(mc_rbdyn::Robot * robot, std::function<void()> callback) : robot(robot), callback(callback) {}

  mc_rbdyn::Robot * robot;
  std::function<void()> callback;

  bool operator==(const ExtraRobot & other) const
  {
    return robot == other.robot;
  }
};

} // namespace mc_plugin

namespace std
{
template<>
struct hash<mc_plugin::ExtraRobot>
{
  std::size_t operator()(const mc_plugin::ExtraRobot & er) const
  {
    // Hash the robot pointer
    return std::hash<mc_rbdyn::Robot *>{}(er.robot);
  }
};
} // namespace std

namespace mc_plugin
{

struct RobotModelUpdate : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~RobotModelUpdate() override;

protected:
  void registerRobot(mc_control::MCController & ctl, ExtraRobot && extraRobot);
  void unregisterRobot(mc_control::MCController & ctl, mc_rbdyn::Robot & extRobot);
  void addRobotToGUI(mc_rtc::gui::StateBuilder & gui, mc_rbdyn::Robot & robot);
  void removeRobotFromGUI(mc_rtc::gui::StateBuilder & gui, mc_rbdyn::Robot & robot);
  void updateRobotModel(mc_rbdyn::Robot & robot);
  void updateRobotModel(mc_control::MCController & ctl);
  void resetToDefault(mc_control::MCController & ctl);
  void resetRobotToDefault(mc_rbdyn::Robot & robot);

  void addFrames(mc_control::MCController & ctl, mc_rbdyn::Robot & robot, bool show = true);

  /**
   * Estimate the configuration from Xsens sensors
   */
  void configFromXsens(mc_control::MCController & ctl);

  /**
   * Load from a manually provided human measurement configuration
   * It is located in
   * ```yaml
   * human:
   *  <human_name>:
   *    ...
   * ```
   */
  void configFromHumanMeasurements(const std::string & humanName);

protected:
  bool firstScale_ = true;
  RobotUpdate robotUpdate;
  RobotUpdate defaultRobotUpdate_;
  std::string humanName_ = "";
  std::string pluginName_ = "";

private:
  mc_rtc::Configuration config_;
  PluginConfigSchema pluginConfig_;

  std::unordered_set<ExtraRobot> extraRobots_;
};

} // namespace mc_plugin
