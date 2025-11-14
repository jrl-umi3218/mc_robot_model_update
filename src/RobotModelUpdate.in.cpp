#include "RobotModelUpdate.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>
#include <sch-core/S_Polyhedron.h>
#include "Convex.h"
#include <algorithm>

namespace mc_plugin
{

RobotModelUpdate::~RobotModelUpdate() = default;

void RobotModelUpdate::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();

  config_ = config;
  mc_rtc::log::info("[RobotModelUpdate] init called with configuration:\n{}", config.dump(true, true));
  if(auto ctlConfig = ctl.config().find("@RobotModelUpdatePluginName@"))
  {
    mc_rtc::log::info("[RobotModelUpdate] Merging with controller configuration:\n{}", ctlConfig->dump(true, true));
    config_.load(*ctlConfig);
  }

  mc_rtc::log::info("[RobotModelUpdate] Loading plugin configuration schema from config");
  pluginConfig_.load(config);
  auto & robotName = pluginConfig_.robot;
  if(robotName.empty())
  {
    mc_rtc::log::warning("[RobotModelUpdate] No robot specified in the configuration.\nIf this is intended you can "
                         "disregard this warning.\nOtherwise please set the 'robot' property, or you may manually "
                         "register robots to be updated with the RobotModelUpdate::registerRobot datastore call");
  }
  else
  {
    if(!ctl.hasRobot(robotName))
    {
      mc_rtc::log::error_and_throw("[RobotModelUpdate] No robot named '{}' in the controller", robotName);
    }
    registerRobot(ctl, ExtraRobot{&ctl.robot(robotName), []() {}});
    registerRobot(ctl, ExtraRobot{&ctl.outputRobot(robotName), []() {}});

    // remove automatically added gui robot model (not updated)
    ctl.gui()->removeElement({"Robots"}, "human");
  }

  ctl.datastore().make_call(pluginConfig_.pluginName + "::LoadConfig", [this, &ctl]() { configFromXsens(ctl); });
  ctl.datastore().make_call(pluginConfig_.pluginName + "::UpdateModel", [this, &ctl]() { updateRobotModel(ctl); });
  ctl.datastore().make_call(pluginConfig_.pluginName + "::registerRobot",
                            [this, &ctl](mc_rbdyn::Robot & extRobot, std::function<void()> callback) {
                              registerRobot(ctl, ExtraRobot{&extRobot, callback});
                            });
  ctl.datastore().make_call(pluginConfig_.pluginName + "::unregisterRobot",
                            [this, &ctl](mc_rbdyn::Robot & extRobot) { unregisterRobot(ctl, extRobot); });
  ctl.datastore().make_call(pluginConfig_.pluginName + "::updateRobotModel",
                            [this, &ctl](mc_rbdyn::Robot & extRobot)
                            {
                              mc_rtc::log::info("[RobotModelUpdate::updateRobotModel] updating robot '{}'",
                                                extRobot.name());
                              updateRobotModel(extRobot);
                            });
  reset(controller);
}

void RobotModelUpdate::addFrames(mc_control::MCController & ctl, mc_rbdyn::Robot & robot, bool show)
{
  for(const auto & f : pluginConfig_.frames)
  {
    mc_rtc::log::info("Adding frame {} to robot {}", f.name, robot.name());
    robot.makeFrame(f.name, robot.frame(f.parent), f.X_p_f);
    auto & frame = f.name;
    if(show)
    {
      ctl.gui()->addElement(
          {"Plugins", pluginConfig_.pluginName, "Frames"},
          mc_rtc::gui::Transform(f.name, [&robot, frame]() { return robot.frame(frame).position(); }));
    }
  }
}

void RobotModelUpdate::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RobotModelUpdate::reset called");

  auto & ctl = controller.controller();
  auto & gui = *ctl.gui();

  std::vector<RobotUpdateJoint> joints;

  mc_rtc::Configuration conf;
  conf.add("joints", joints);
  mc_rtc::log::info("Conf is:\n{}", conf.dump(true, true));

  robotUpdate.load(conf);
  defaultRobotUpdate_ = robotUpdate;

  mc_rtc::log::info("Robot Update is:\n{}", robotUpdate.dump(true, true));

  // TODO: load as a schema
  std::vector<std::string> humanNames;
  for(const auto & [humanName, _] :
      static_cast<std::map<std::string, mc_rtc::Configuration>>(config_("human", mc_rtc::Configuration{})))
  {
    humanNames.push_back(humanName);
  }

  gui.removeElements(this);
  gui.addElement(this, {"Plugins", pluginConfig_.pluginName},
                 mc_rtc::gui::Button("Reset to default", [this, &ctl]() { resetToDefault(ctl); }),
                 mc_rtc::gui::Button("Load Xsens config", [this, &ctl]() { configFromXsens(ctl); }),
                 mc_rtc::gui::ComboInput(
                     "Load Human Measurement config", humanNames, [this]() { return humanName_; },
                     [this](const std::string & humanName) { configFromHumanMeasurements(humanName); }));

  // gui.addElement(this, {},
  //                mc_rtc::gui::Button("Rescale human model",
  //                                    [this, &ctl]()
  //                                    {
  //                                      configFromXsens(ctl);
  //                                      updateRobotModel(ctl);
  //                                    }));

  robotUpdate.addToGUI(gui, {"Plugins", pluginConfig_.pluginName}, "Update robot model from loaded config",
                       [this, &ctl]()
                       {
                         mc_rtc::log::info("Updated robot schema:\n{}", robotUpdate.dump(true, true));
                         updateRobotModel(ctl);
                       });
}

void RobotModelUpdate::registerRobot(mc_control::MCController & ctl, ExtraRobot && extraRobot)
{
  mc_rtc::log::info("RobotModelUpdate::registerRobot: adding robot {}", extraRobot.robot->name());
  extraRobots_.emplace(extraRobot);
  addFrames(ctl, *extraRobot.robot, true);
  addRobotToGUI(*ctl.gui(), *extraRobot.robot);
}

void RobotModelUpdate::unregisterRobot(mc_control::MCController & ctl, mc_rbdyn::Robot & extRobot)
{
  mc_rtc::log::info("RobotModelUpdate::unregisterRobot: removing robot {}", extRobot.name());
  removeRobotFromGUI(*ctl.gui(), extRobot);
  auto it = std::find_if(extraRobots_.begin(), extraRobots_.end(),
                         [&extRobot](const ExtraRobot & er) { return er.robot == &extRobot; });
  if(it != extraRobots_.end())
  {
    extraRobots_.erase(it);
  }
}

void RobotModelUpdate::addRobotToGUI(mc_rtc::gui::StateBuilder & gui, mc_rbdyn::Robot & robot)
{
  if(pluginConfig_.publishVisual)
  {
    for(const auto & visual : robot.module()._visual)
    {
      const auto & bodyName = visual.first;
      for(size_t i = 0; i < visual.second.size(); i++)
      {
        // TODO: allow to customize visual display (transparency, etc)
        auto & v = const_cast<rbd::parsers::Visual &>(visual.second[i]);
        auto visualName = fmt::format("{}_{}", visual.first, i);
        ;
        if(v.name.empty())
        {
          v.name = visualName;
        }
        else
        {
          visualName = v.name;
        }

        gui.addElement(this, {"Plugins", pluginConfig_.pluginName, "Human", "Visuals"},
                       mc_rtc::gui::Visual(
                           visualName, [&v]() -> const auto & { return v; },
                           [&robot, bodyName]()
                           { return robot.collisionTransform(bodyName) * robot.frame(bodyName).position(); }));
      }
    }
  }

  if(pluginConfig_.publishConvex)
  {
    // TODO: load polyhedron configuration
    mc_rtc::gui::PolyhedronConfig polyConfig;
    polyConfig.triangle_color = {0, 0.9, 0, 0.5};
    polyConfig.vertices_config.color = {0, 1, 0, 0.5};
    polyConfig.show_vertices = false;
    polyConfig.edge_config.color = {0, 1, 0, 0.5};
    polyConfig.show_edges = false;

    for(const auto & [convexName, convexPair] : robot.convexes())
    {
      const auto & bodyName = convexPair.first;
      const auto & object = convexPair.second;

      if(auto poly = std::dynamic_pointer_cast<sch::S_Polyhedron>(object))
      {
        const auto & convexName_ = convexName;
        gui.addElement({"Plugins", pluginConfig_.pluginName, "Human", "Convex"},
                       mc_rtc::gui::Convex(
                           convexName, polyConfig, [poly]() -> const auto & { return poly; },
                           [bodyName, convexName_, &robot]()
                           {
                             const sva::PTransformd X_0_b = robot.frame(bodyName).position();
                             const sva::PTransformd & X_b_c = robot.collisionTransform(convexName_);
                             sva::PTransformd X_0_c = X_b_c * X_0_b;
                             return X_0_c;
                           }));
      }
    }
  }
}

void RobotModelUpdate::removeRobotFromGUI(mc_rtc::gui::StateBuilder & gui, mc_rbdyn::Robot & robot) {}

void RobotModelUpdate::configFromHumanMeasurements(const std::string & humanName)
{
  humanName_ = humanName;
  if(pluginConfig_.human.count(humanName) == 0)
  {
    mc_rtc::log::error(
        "[RobotModelUpdate] Failed to load human from measurements config, no object human->{} in the configuration",
        humanName);
    return;
  }
  const auto & humanConfig = pluginConfig_.human.at(humanName_);
  const auto & bodyDim = humanConfig.joints;
  double BodyHeight = bodyDim.BodyHeight;
  double HipHeight = bodyDim.HipHeight;
  double LegHeight = HipHeight - 0.1;
  double HipWidth = bodyDim.HipWidth;
  double LegsWidth = HipWidth - 0.03;
  double ShoulderHeight = bodyDim.ShoulderHeight;
  double ArmHeight = ShoulderHeight - 0.15;
  double ShoulderWidth = bodyDim.ShoulderWidth;
  double ArmsWidth = ShoulderWidth - 0.06;
  double ElbowSpan = bodyDim.ElbowSpan;
  double WristSpan = bodyDim.WristSpan;
  double KneeHeight = bodyDim.KneeHeight;
  double AnkleHeight = bodyDim.AnkleHeight;

  auto & joints = robotUpdate.joints;
  joints.clear();
  joints.push_back(RobotUpdateJoint{"Head_0", Eigen::Vector3d{-0.03, 0, ShoulderHeight - HipHeight}});
  joints.push_back(RobotUpdateJoint{"Torso_0", Eigen::Vector3d{0, 0, LegHeight - HipHeight}});
  joints.push_back(RobotUpdateJoint{"LArm_0", Eigen::Vector3d{-0.03, ArmsWidth / 2, ArmHeight - HipHeight}});
  joints.push_back(RobotUpdateJoint{"LElbow", Eigen::Vector3d{0, ((ElbowSpan - ArmsWidth) / 2) * 0.66, 0}});
  joints.push_back(RobotUpdateJoint{"LForearm", Eigen::Vector3d{0, ((ElbowSpan - ArmsWidth) / 2) * 0.33, 0}});
  joints.push_back(RobotUpdateJoint{"LWrist_0", Eigen::Vector3d{0, (WristSpan - ElbowSpan) / 2, 0}});
  joints.push_back(RobotUpdateJoint{"RArm_0", Eigen::Vector3d{-0.03, -ArmsWidth / 2, ArmHeight - HipHeight}});
  joints.push_back(RobotUpdateJoint{"RElbow", Eigen::Vector3d{0, -((ElbowSpan - ArmsWidth) / 2) * 0.66, 0}});
  joints.push_back(RobotUpdateJoint{"RForearm", Eigen::Vector3d{0, -((ElbowSpan - ArmsWidth) / 2) * 0.33, 0}});
  joints.push_back(RobotUpdateJoint{"RWrist_0", Eigen::Vector3d{0, -(WristSpan - ElbowSpan) / 2, 0}});
  joints.push_back(RobotUpdateJoint{"LLeg_0", Eigen::Vector3d{0, LegsWidth / 2, LegHeight - HipHeight}});
  joints.push_back(RobotUpdateJoint{"LShin_0", Eigen::Vector3d{0, 0, KneeHeight - LegHeight}});
  joints.push_back(RobotUpdateJoint{"LAnkle_0", Eigen::Vector3d{0, 0, AnkleHeight - KneeHeight}});
  joints.push_back(RobotUpdateJoint{"RLeg_0", Eigen::Vector3d{0, -LegsWidth / 2, LegHeight - HipHeight}});
  joints.push_back(RobotUpdateJoint{"RShin_0", Eigen::Vector3d{0, 0, KneeHeight - LegHeight}});
  joints.push_back(RobotUpdateJoint{"RAnkle_0", Eigen::Vector3d{0, 0, AnkleHeight - KneeHeight}});

  robotUpdate.bodies = humanConfig.bodies;
}

void RobotModelUpdate::configFromXsens(mc_control::MCController & ctl)
{
  auto & robotName = pluginConfig_.robot;

  // New version: using data directly from xsens to scale
  // step 1: get poses (world frame)
  auto X_Hips_0 = ctl.datastore()
                      .call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Pelvis"))
                      .inv();
  auto X_0_Head =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Head"));
  auto X_0_Torso =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("T8"));

  auto X_0_Neck =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Neck"));

  auto X_0_LArm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Left Upper Arm"));
  auto X_0_LForearm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                             static_cast<const std::string &>("Left Forearm"));
  auto X_0_LWrist = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Hand"));

  auto X_0_RArm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Right Upper Arm"));
  auto X_0_RForearm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                             static_cast<const std::string &>("Right Forearm"));
  auto X_0_RWrist = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Hand"));

  auto X_0_LLeg = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Left Upper Leg"));
  auto X_0_LShin = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                          static_cast<const std::string &>("Left Lower Leg"));
  auto X_0_LAnkle = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Foot"));

  auto X_0_RLeg = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Right Upper Leg"));
  auto X_0_RShin = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                          static_cast<const std::string &>("Right Lower Leg"));
  auto X_0_RAnkle = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Foot"));

  // make xsens origins coherent with our human model
  X_Hips_0.translation().z() -= 0.15; // origin of hips is at the top of our human model, 15cm higher than legs origin
  mc_rtc::log::info("X_Hips_0 rot with offsets is {}", X_Hips_0.rotation());
  mc_rtc::log::info("trans is {}", X_Hips_0.translation());

  // step 2: convert world poses to local frames (kinematic tree from hips link)
  // auto X_Hips_Neck = X_0_Neck * X_Hips_0;
  // X_Hips_Neck.translation().y() = 0; // center
  // X_Hips_Neck.translation().x() = -0.025;

  auto X_Hips_Head = X_0_Head * X_Hips_0;
  X_Hips_Head.translation().z() -= 0.05; // xsens head is at ears height but head model joint is lower than that
  X_Hips_Head.translation().y() = 0;
  X_Hips_Head.translation().x() = -0.025;

  auto X_Hips_LArm = X_0_LArm * X_Hips_0;
  X_Hips_LArm.translation().x() = -0.05;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto X_LArm_LForearm = X_0_LForearm * X_0_LArm.inv();
  Eigen::Vector3d X_LArm_LElbow = 0.66 * X_LArm_LForearm.translation();
  Eigen::Vector3d X_LElbow_LForearm = 0.34 * X_LArm_LForearm.translation();
  auto X_LForearm_LWrist = X_0_LWrist * X_0_LForearm.inv();
  X_LForearm_LWrist.translation().x() += 0.01;
  X_LForearm_LWrist.translation().z() -= 0.005;

  auto X_Hips_RArm = X_0_RArm * X_Hips_0;
  X_Hips_RArm.translation().x() = -0.05;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto X_RArm_RForearm = X_0_RForearm * X_0_RArm.inv();
  Eigen::Vector3d X_RArm_RElbow = 0.66 * X_RArm_RForearm.translation();
  Eigen::Vector3d X_RElbow_RForearm = 0.34 * X_RArm_RForearm.translation();
  auto X_RForearm_RWrist = X_0_RWrist * X_0_RForearm.inv();
  X_RForearm_RWrist.translation().x() += 0.01;
  X_RForearm_RWrist.translation().z() -= 0.005;

  auto X_Hips_LLeg = X_0_LLeg * X_Hips_0;
  auto X_LLeg_LShin = X_0_LShin * X_0_LLeg.inv();
  auto X_LShin_LAnkle = X_0_LAnkle * X_0_LShin.inv();

  auto X_Hips_RLeg = X_0_RLeg * X_Hips_0;
  auto X_RLeg_RShin = X_0_RShin * X_0_RLeg.inv();
  auto X_RShin_RAnkle = X_0_RAnkle * X_0_RShin.inv();

  // step 3: update joint configs using local translations

  std::vector<RobotUpdateJoint> joints;

  // joints.push_back(RobotUpdateJoint{"Head_0", X_Hips_Neck.translation()});
  joints.push_back(RobotUpdateJoint{"Head_0", X_Hips_Head.translation()});

  // torso link doesn't need an update as it has the same origin as the hips
  // joints.push_back(RobotUpdateJoint{"Torso_0", Hips_W_0.inv().translation()});

  joints.push_back(RobotUpdateJoint{"LArm_0", X_Hips_LArm.translation()});
  joints.push_back(RobotUpdateJoint{"LElbow", X_LArm_LElbow});
  joints.push_back(RobotUpdateJoint{"LForearm", X_LElbow_LForearm});
  joints.push_back(RobotUpdateJoint{"LWrist_0", X_LForearm_LWrist.translation()});
  joints.push_back(RobotUpdateJoint{"RArm_0", X_Hips_RArm.translation()});
  joints.push_back(RobotUpdateJoint{"RElbow", X_RArm_RElbow});
  joints.push_back(RobotUpdateJoint{"RForearm", X_RElbow_RForearm});
  joints.push_back(RobotUpdateJoint{"RWrist_0", X_RForearm_RWrist.translation()});

  joints.push_back(RobotUpdateJoint{"LLeg_0", X_Hips_LLeg.translation()});
  joints.push_back(RobotUpdateJoint{"LShin_0", X_LLeg_LShin.translation()});
  joints.push_back(RobotUpdateJoint{"LAnkle_0", X_LShin_LAnkle.translation()});
  joints.push_back(RobotUpdateJoint{"RLeg_0", X_Hips_RLeg.translation()});
  joints.push_back(RobotUpdateJoint{"RShin_0", X_RLeg_RShin.translation()});
  joints.push_back(RobotUpdateJoint{"RAnkle_0", X_RShin_RAnkle.translation()});

  // step 4: save joint configurations
  robotUpdate.joints = joints;

  // step 5: update body scales
  // logic: scale ref is distance between original body origin and original next body origin
  // new scale is new distance/ old distance

  // torso and hips must be scaled in Z AND Y using arms transforms and legs transforms

  std::vector<RobotUpdateBody> bodies;
  Eigen::Vector3d scaleTorso = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRArm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLArm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRForearm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLForearm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleHips = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRUpperLeg = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLUpperLeg = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRLowerLeg = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLLowerLeg = Eigen::Vector3d{1, 1, 1};

  if(firstScale_)
  {
    scaleTorso = Eigen::Vector3d{0.9, 1, 1};
    scaleRArm = Eigen::Vector3d{0.85, 1, 0.85};
    scaleRForearm = Eigen::Vector3d{0.9, 1, 0.9};
    scaleLArm = Eigen::Vector3d{0.85, 1, 0.85};
    scaleLForearm = Eigen::Vector3d{0.9, 1, 0.9};
    scaleHips = Eigen::Vector3d{0.9, 1, 1};
  }

  auto & robot = ctl.robot(robotName);

  // get original relative transform of joints and compare to new transforms

  // torso
  auto originalTransform1 = robot.mb().transform(robot.jointIndexByName("Head_0")).translation();
  // auto newTransform1 = X_Hips_Neck.translation();
  auto newTransform1 = X_Hips_Head.translation();
  scaleTorso.z() *= newTransform1.z() / originalTransform1.z();

  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LArm_0")).translation();
  auto originalTransform2 = robot.mb().transform(robot.jointIndexByName("RArm_0")).translation();
  newTransform1 = X_Hips_LArm.translation();
  auto newTransform2 = X_Hips_RArm.translation();
  // scaleTorso.y() *= (newTransform1.y() - newTransform2.y()) / (originalTransform1.y() - originalTransform2.y());

  // hips
  originalTransform1 = sva::interpolate(robot.mb().transform(robot.jointIndexByName("LLeg_0")),
                                        robot.mb().transform(robot.jointIndexByName("RLeg_0")), 0.5)
                           .translation();
  newTransform1 = sva::interpolate(X_Hips_LLeg, X_Hips_RLeg, 0.5).translation();
  scaleHips.z() *= newTransform1.z() / originalTransform1.z();

  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LLeg_0")).translation();
  originalTransform2 = robot.mb().transform(robot.jointIndexByName("RLeg_0")).translation();
  newTransform1 = X_Hips_LLeg.translation();
  newTransform2 = X_Hips_RLeg.translation();
  // scaleHips.y() *= (newTransform1.y() - newTransform2.y()) / (originalTransform1.y() - originalTransform2.y());
  // scaleHips.z() = scaleHips.y();

  // r upper arm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RElbow")).translation();
  newTransform1 = X_RArm_RElbow;
  scaleRArm.y() *= newTransform1.y() / originalTransform1.y();

  // r forearm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RWrist_0")).translation();
  newTransform1 = X_RForearm_RWrist.translation();
  scaleRForearm.y() *= newTransform1.y() / originalTransform1.y();

  // l upper arm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LElbow")).translation();
  newTransform1 = X_LArm_LElbow;
  scaleLArm.y() *= newTransform1.y() / originalTransform1.y();

  // l forearm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LWrist_0")).translation();
  newTransform1 = X_LForearm_LWrist.translation();
  scaleLForearm.y() *= newTransform1.y() / originalTransform1.y();

  // l upper leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LShin_0")).translation();
  newTransform1 = X_LLeg_LShin.translation();
  scaleLUpperLeg.z() *= newTransform1.z() / originalTransform1.z();

  // r upper leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RShin_0")).translation();
  newTransform1 = X_RLeg_RShin.translation();
  scaleRUpperLeg.z() *= newTransform1.z() / originalTransform1.z();

  // l lower leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LAnkle_0")).translation();
  newTransform1 = X_LShin_LAnkle.translation();
  scaleLLowerLeg.z() *= newTransform1.z() / originalTransform1.z();

  // r lower leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RAnkle_0")).translation();
  newTransform1 = X_RShin_RAnkle.translation();
  scaleRLowerLeg.z() *= newTransform1.z() / originalTransform1.z();

  bodies.push_back(RobotUpdateBody{"TorsoLink", scaleTorso});
  bodies.push_back(RobotUpdateBody{"RArmLink", scaleRArm});
  bodies.push_back(RobotUpdateBody{"RElbowLink", scaleRArm});
  bodies.push_back(RobotUpdateBody{"RForearmLink", scaleRForearm});
  bodies.push_back(RobotUpdateBody{"LArmLink", scaleLArm});
  bodies.push_back(RobotUpdateBody{"LElbowLink", scaleLArm});
  bodies.push_back(RobotUpdateBody{"LForearmLink", scaleLForearm});
  bodies.push_back(RobotUpdateBody{"HipsLink", scaleHips});
  bodies.push_back(RobotUpdateBody{"RLegLink", scaleRUpperLeg});
  bodies.push_back(RobotUpdateBody{"RShinLink", scaleRLowerLeg});
  bodies.push_back(RobotUpdateBody{"LLegLink", scaleLUpperLeg});
  bodies.push_back(RobotUpdateBody{"LShinLink", scaleLLowerLeg});

  // step 6: save scale configurations
  robotUpdate.bodies = bodies;
  robotUpdate.toConfiguration().save("/tmp/robotUpdate.json");
  if(firstScale_)
  {
    firstScale_ = false;
  }
}

void RobotModelUpdate::resetToDefault(mc_control::MCController & ctl)
{
  auto resetRobot = [this](mc_rbdyn::Robot & robot)
  {
    auto robots = mc_rbdyn::loadRobot(robot.module());
    auto & defaultRobot = robots->robot();
    robot.mb() = defaultRobot.mb();
    robot.mbg() = defaultRobot.mbg();

    // Restore scaleV of all visuals
    auto & visuals = const_cast<mc_rbdyn::RobotModule &>(robot.module())._visual;
    const auto & originalVisuals = defaultRobot.module()._visual;
    for(const auto & body : robot.mb().bodies())
    {
      auto bName = body.name();
      mc_rtc::log::info("Processing visuals for body {}", bName);
      if(visuals.count(bName) == 0) continue;

      for(size_t i = 0; i < visuals[bName].size(); i++)
      {
        auto & v = visuals[bName][i];
        const auto & ov = originalVisuals.at(bName)[i];
        if(v.geometry.type == rbd::parsers::Geometry::MESH && ov.geometry.type == rbd::parsers::Geometry::MESH)
        {
          auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(v.geometry.data);
          const auto & oMesh = boost::get<rbd::parsers::Geometry::Mesh>(ov.geometry.data);
          mesh.scaleV = mesh.scaleV.cwiseQuotient(oMesh.scaleV);
          mc_rtc::log::info("Resetting visual mesh scale for body {} visual {} to {}", bName, i,
                            mesh.scaleV.transpose());
        }
      }
    }

    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();
  };

  resetRobot(ctl.robot(pluginConfig_.robot));
  resetRobot(ctl.outputRobot(pluginConfig_.robot));

  // Apply default update from config
  robotUpdate = defaultRobotUpdate_;
  updateRobotModel(ctl);
}

void RobotModelUpdate::updateRobotModel(mc_rbdyn::Robot & robot)
{
  mc_rtc::log::info("Update robot model: {}", robot.name());
  for(const auto & joint : robotUpdate.joints)
  {
    if(robot.hasJoint(joint.name))
    {
      auto jIdx = robot.jointIndexByName(joint.name);
      // getting original relative transform of joint
      auto originalTransform = robot.mb().transform(jIdx);
      auto newTransform = originalTransform;
      // updating with new translation while keeping same orientation
      newTransform.translation() = joint.relTranslation;
      robot.mb().transform(jIdx, newTransform);
      mc_rtc::log::info("Updated transform for joint {} from {} to {}", joint.name,
                        originalTransform.translation().transpose(), newTransform.translation().transpose());
    }
  }

  for(const auto & surface : robot.availableSurfaces())
  {
    // for all surfaces on the robot, find the parent body
    auto parentName = robot.surface(surface).bodyName();
    auto newScale = Eigen::Vector3d(1, 1, 1);
    for(const auto & body : robotUpdate.bodies)
    {
      // take new scale of the parent body of the surface
      if(body.name == parentName)
      {
        newScale = body.scale;
        mc_rtc::log::info("Updated scale for body {} to {}", body.name, newScale.transpose());
      }
    }
    // apply the new scale of the body to the translation of the surface transform
    auto originalTransform = robot.surface(surface).X_b_s();
    auto newTransform = originalTransform;
    newTransform.translation() = newScale.array() * newTransform.translation().array();
    robot.surface(surface).X_b_s(newTransform);
    mc_rtc::log::info("Updated transform for surface {} from {} to {}", surface,
                      originalTransform.translation().transpose(), newTransform.translation().transpose());
    // XXX update associated frame as well ! after instanciation frames and surfaces are dissociated
    robot.frame(surface).X_p_f(newTransform);
  }

  for(const auto & convex : robot.convexes())
  {
    for(const auto & body : robotUpdate.bodies)
    {
      if(convex.first == body.name)
      {
        auto originalConvex = convex.second.second;
        // check we can actually cast the convex to a polyhedron
        if(auto poly = std::dynamic_pointer_cast<sch::S_Polyhedron>(originalConvex))
        {
          auto vertices = poly->getPolyhedronAlgorithm()->vertexes_;
          for(auto vertex : vertices)
          {
            auto origVertexCoord = vertex->getCoordinates();
            vertex->setCoordinates(origVertexCoord.m_x * body.scale.x(), origVertexCoord.m_y * body.scale.y(),
                                   origVertexCoord.m_z * body.scale.z());
          }
        }
        // careful: if there were existing collision constraints on the convex, they need to be removed and readded
        mc_rtc::log::info("Updated convex {} with scale {}", convex.first, body.scale.transpose());
      }
    }
  }

  auto & visuals = const_cast<mc_rbdyn::RobotModule &>(robot.module())._visual;
  for(const auto & body : robotUpdate.bodies)
  {
    mc_rtc::log::info("Processing visuals for body {}", body.name);
    if(visuals.find(body.name) == visuals.end())
    {
      mc_rtc::log::info("No visuals found for body {}", body.name);
      continue;
    }
    for(auto & visual : visuals[body.name])
    {
      if(visual.geometry.type == rbd::parsers::Geometry::MESH)
      {
        auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(visual.geometry.data);
        mesh.scaleV = mesh.scaleV.cwiseProduct(body.scale);
      }
    }
  }

  robot.forwardKinematics();
  robot.forwardVelocity();
  robot.forwardAcceleration();
}

void RobotModelUpdate::updateRobotModel(mc_control::MCController & ctl)
{
  // Update all robot models and call their callback to notify the controller that the robot has changed
  std::for_each(extraRobots_.begin(), extraRobots_.end(),
                [this](auto & extraRobot)
                {
                  updateRobotModel(*extraRobot.robot);
                  extraRobot.callback();
                });
}

void RobotModelUpdate::before(mc_control::MCGlobalController & /* controller */) {}

void RobotModelUpdate::after(mc_control::MCGlobalController & /* controller */) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration RobotModelUpdate::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("@RobotModelUpdatePluginName@", mc_plugin::RobotModelUpdate)
