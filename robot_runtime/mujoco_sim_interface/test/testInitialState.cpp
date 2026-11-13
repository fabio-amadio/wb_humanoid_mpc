#include <gtest/gtest.h>
#include <mujoco_sim_interface/MujocoSimInterface.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <limits>

using namespace robot;
using namespace robot::mujoco_sim_interface;

TEST(MujocoInitialState, PreservesRequestedJointStateAndResetWithoutGroundPenetration) {
  const auto modelPath = ament_index_cpp::get_package_share_directory("g1_description") + "/urdf/g1_29dof";
  model::RobotDescription description(modelPath + ".urdf");
  auto initial = std::make_shared<model::RobotState>(description, 2);
  initial->setRootPositionInWorldFrame(vector3_t(0.0, 0.0, 0.76));
  const quaternion_t rotation(Eigen::AngleAxisd(0.2, vector3_t::UnitZ()));
  initial->setRootRotationLocalToWorldFrame(rotation);
  initial->setRootLinearVelocityInLocalFrame(vector3_t(0.01, 0.02, 0.03));
  initial->setRootAngularVelocityInLocalFrame(vector3_t(0.02, 0.01, 0.03));
  for (const std::string side : {"left", "right"}) {
    initial->setJointPosition(description.getJointIndex(side + "_hip_pitch_joint"), -0.349);
    initial->setJointPosition(description.getJointIndex(side + "_knee_joint"), 0.723);
    initial->setJointPosition(description.getJointIndex(side + "_ankle_pitch_joint"), -0.369);
  }
  initial->setJointPosition(description.getJointIndex("left_wrist_yaw_joint"), 0.1);
  initial->setJointPosition(description.getJointIndex("right_wrist_yaw_joint"), -0.1);
  for (auto id : description.getJointIndices()) {
    initial->setJointVelocity(id, 0.01 * (id + 1));
  }
  MujocoSimConfig config;
  config.scenePath = modelPath + ".xml";
  config.headless = true;
  config.initStatePtr_ = initial;
  MujocoSimInterface simulator(config, modelPath + ".urdf");
  const auto* model = simulator.getModel();
  MjState snapshot(model);
  const auto check = [&] {
    simulator.copyMjState(snapshot);
    EXPECT_NEAR(snapshot.data->qpos[2], 0.76, 1e-12);
    EXPECT_NEAR(snapshot.data->qpos[3], rotation.w(), 1e-12);
    EXPECT_NEAR(snapshot.data->qpos[6], rotation.z(), 1e-12);
    for (int joint = 1; joint < model->njnt; ++joint) {
      const auto* name = mj_id2name(model, mjOBJ_JOINT, joint);
      const auto id = description.getJointIndex(name);
      EXPECT_DOUBLE_EQ(snapshot.data->qpos[model->jnt_qposadr[joint]], initial->getJointPosition(id)) << name;
      EXPECT_DOUBLE_EQ(snapshot.data->qvel[model->jnt_dofadr[joint]], initial->getJointVelocity(id)) << name;
      EXPECT_DOUBLE_EQ(simulator.getRobotState().getJointPosition(id), initial->getJointPosition(id)) << name;
    }
  };
  check();
  mj_forward(model, snapshot.data);
  double minimumFootHeight = std::numeric_limits<double>::infinity();
  int spheres = 0;
  for (int geom = 0; geom < model->ngeom; ++geom) {
    const std::string body = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom]);
    if (body.find("ankle_roll_link") != std::string::npos && model->geom_contype[geom] && model->geom_type[geom] == mjGEOM_SPHERE) {
      minimumFootHeight = std::min(minimumFootHeight, snapshot.data->geom_xpos[3 * geom + 2] - model->geom_size[3 * geom]);
      ++spheres;
    }
  }
  EXPECT_EQ(spheres, 8);
  EXPECT_GE(minimumFootHeight, 0.0);
  EXPECT_LT(minimumFootHeight, 0.02);
  simulator.simulationStep();
  simulator.reset();
  // Reset restores the exact configured qpos/qvel before any further physics integration.
  simulator.copyMjState(snapshot);
  for (int joint = 1; joint < model->njnt; ++joint) {
    const auto id = description.getJointIndex(mj_id2name(model, mjOBJ_JOINT, joint));
    EXPECT_DOUBLE_EQ(snapshot.data->qpos[model->jnt_qposadr[joint]], initial->getJointPosition(id));
    EXPECT_DOUBLE_EQ(snapshot.data->qvel[model->jnt_dofadr[joint]], initial->getJointVelocity(id));
  }
  mj_deleteData(snapshot.data);
}
