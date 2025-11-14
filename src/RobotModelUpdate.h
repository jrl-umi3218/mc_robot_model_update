/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_control/MCController.h>
#include <mc_rtc/Schema.h>
#include <mc_rtc/gui/StateBuilder.h>

namespace mc_plugin
{

struct RobotUpdateJoint
{
  MC_RTC_NEW_SCHEMA(RobotUpdateJoint)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(RobotUpdateJoint, __VA_ARGS__))
  MEMBER(std::string, name, "Name of the joint", mc_rtc::schema::None, "");
  MEMBER(Eigen::Vector3d, relTranslation, "relTranslation", mc_rtc::schema::None, Eigen::Vector3d::Zero());
#undef MEMBER
};

struct RobotUpdateBody
{
  MC_RTC_NEW_SCHEMA(RobotUpdateBody)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(RobotUpdateBody, __VA_ARGS__))
  MEMBER(std::string, name, "Name of the body", mc_rtc::schema::None, "");
  MEMBER(Eigen::Vector3d, scale, "bodyScale", mc_rtc::schema::None, Eigen::Vector3d::Zero());
#undef MEMBER
};

struct RobotUpdate
{
  MC_RTC_NEW_SCHEMA(RobotUpdate)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(RobotUpdate, __VA_ARGS__))
  MEMBER(std::vector<RobotUpdateJoint>, joints, "joints to update")
  MEMBER(std::vector<RobotUpdateBody>, bodies, "bodies to update")
#undef MEMBER
};

struct JointsMeasurementsSchema
{
  MC_RTC_NEW_SCHEMA(JointsMeasurementsSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(JointsMeasurementsSchema, double, __VA_ARGS__))
  MEMBER(BodyHeight, "BodyHeight", mc_rtc::schema::Interactive, 1.64)
  MEMBER(FootLength, "FootLength", mc_rtc::schema::Interactive, 0.25)
  MEMBER(ShoulderHeight, "ShoulderHeight", mc_rtc::schema::Interactive, 1.395)
  MEMBER(ShoulderWidth, "ShoulderWidth", mc_rtc::schema::Interactive, 0.36)
  MEMBER(ElbowSpan, "ElbowSpan", mc_rtc::schema::Interactive, 0.785)
  MEMBER(WristSpan, "WristSpan", mc_rtc::schema::Interactive, 1.235)
  MEMBER(ArmSpan, "ArmSpan", mc_rtc::schema::Interactive, 1.61)
  MEMBER(HipHeight, "HipHeight", mc_rtc::schema::Interactive, 0.905)
  MEMBER(HipWidth, "HipWidth", mc_rtc::schema::Interactive, 0.23)
  MEMBER(KneeHeight, "KneeHeight", mc_rtc::schema::Interactive, 0.5)
  MEMBER(AnkleHeight, "AnkleHeight", mc_rtc::schema::Interactive, 0.135)
  MEMBER(ExtraShoe, "ExtraShoe", mc_rtc::schema::Interactive, 0.0)
#undef MEMBER
};

struct HumanMeasurementsSchema
{
  MC_RTC_NEW_SCHEMA(HumanMeasurementsSchema)
  MC_RTC_SCHEMA_MEMBER(HumanMeasurementsSchema,
                       JointsMeasurementsSchema,
                       joints,
                       "joints",
                       mc_rtc::schema::None,
                       JointsMeasurementsSchema{})
  using RobotUpdateBodiesVector = std::vector<RobotUpdateBody>;
  MC_RTC_SCHEMA_MEMBER(HumanMeasurementsSchema,
                       RobotUpdateBodiesVector,
                       bodies,
                       "bodies",
                       mc_rtc::schema::None,
                       RobotUpdateBodiesVector{})
};

struct FrameDescriptionSchema
{
  MC_RTC_NEW_SCHEMA(FrameDescriptionSchema)
  MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(FrameDescriptionSchema, std::string, name, "name")
  MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(FrameDescriptionSchema, std::string, parent, "name")
  MC_RTC_SCHEMA_MEMBER(FrameDescriptionSchema,
                       sva::PTransformd,
                       X_p_f,
                       "X_p_f",
                       mc_rtc::schema::None,
                       sva::PTransformd::Identity())
  MC_RTC_SCHEMA_MEMBER(FrameDescriptionSchema, bool, baked, "baked", mc_rtc::schema::None, false)
};

struct PluginConfigSchema
{
  MC_RTC_NEW_SCHEMA(PluginConfigSchema)
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema, std::string, robot, "robot", mc_rtc::schema::None, "")
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema,
                       std::string,
                       pluginName,
                       "pluginName",
                       mc_rtc::schema::None,
                       "RobotModelUpdate")
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema, bool, publishVisual, "publishVisual", mc_rtc::schema::None, false)
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema, bool, publishConvex, "publishConvex", mc_rtc::schema::None, false)
  using HumanMeasurementsMap = std::map<std::string, HumanMeasurementsSchema>;
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema,
                       HumanMeasurementsMap,
                       human,
                       "human",
                       mc_rtc::schema::None,
                       HumanMeasurementsMap{})
  using FramesDescriptionVector = std::vector<FrameDescriptionSchema>;
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema,
                       FramesDescriptionVector,
                       frames,
                       "frames",
                       mc_rtc::schema::None,
                       FramesDescriptionVector{})
  using JointsVector = std::vector<RobotUpdateJoint>;
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema, JointsVector, joints, "joints", mc_rtc::schema::None, JointsVector{})
  using BodiesVector = std::vector<RobotUpdateBody>;
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema, BodiesVector, bodies, "bodies", mc_rtc::schema::None, BodiesVector{})
};

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
  void resetToDefault(mc_control::MCController & robot);

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
