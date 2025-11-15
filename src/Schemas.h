#pragma once
#include <mc_rtc/Schema.h>

// TODO:
// - Uniify use of RobotUpdate and HumanMeasurementSchema

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

struct DefaultSchema
{
  MC_RTC_NEW_SCHEMA(DefaultSchema)
  MC_RTC_SCHEMA_MEMBER(DefaultSchema, std::string, human, "human", mc_rtc::schema::None, "")
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
  MC_RTC_SCHEMA_MEMBER(PluginConfigSchema,
                       DefaultSchema,
                       defaultConfig,
                       "defaultConfig",
                       mc_rtc::schema::None,
                       DefaultSchema{})
};

} // namespace mc_plugin
