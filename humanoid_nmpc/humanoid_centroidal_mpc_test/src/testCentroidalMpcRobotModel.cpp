#include <gtest/gtest.h>

#include <Eigen/Core>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h"
#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "humanoid_centroidal_mpc_test/CentroidalTestingModelInterface.h"

using namespace ocs2;
using namespace ocs2::humanoid;

class CentroidalMpcRobotModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pinocchioInterface = std::make_shared<PinocchioInterface>(testingModelInterface.getPinocchioInterface());

    // Setup test state and input vectors
    setupTestVectors();
  }

  void setupTestVectors() {
    // State vector: [centroidal_momentum(6), base_pose(6), joint_angles(12)]
    testState = vector_t::Zero(testingModelInterface.getMpcRobotModel().getStateDim());

    // Centroidal momentum [vcom_x, vcom_y, vcom_z, L_x/m, L_y/m, L_z/m]
    testState.segment(0, 6) << 0.1, 0.2, 0.0, 0.05, -0.03, 0.01;

    // Base pose [px, py, pz, rx, ry, rz]
    testState.segment(6, 6) << 0.0, 0.0, 0.85, 0.0, 0.0, 0.1;

    // Joint angles (12 joints)
    testState.segment(12, testingModelInterface.getMpcRobotModel().getJointDim()) =
        vector_t::Random(testingModelInterface.getMpcRobotModel().getJointDim());

    testState.segment(12, 2) = vector2_t(0.1, -0.2);

    // Input vector: [contact_wrenches(12), joint_velocities(12)]
    testInput = vector_t::Zero(testingModelInterface.getMpcRobotModel().getInputDim());

    // Left foot wrench [fx, fy, fz, mx, my, mz]
    testInput.segment(0, 6) << 0.0, 0.0, 400.0, 5.0, -2.0, 1.0;

    // Right foot wrench [fx, fy, fz, mx, my, mz]
    testInput.segment(6, 6) << 0.0, 0.0, 400.0, -5.0, 2.0, -1.0;

    // Joint velocities (12 joints)
    testInput.segment(12, testingModelInterface.getMpcRobotModel().getJointDim()) =
        vector_t::Random(testingModelInterface.getMpcRobotModel().getJointDim());
    testInput.segment(12, 2) = vector2_t(0.1, 0.2);
  }

  CentroidalTestingModelInterface testingModelInterface;
  std::shared_ptr<PinocchioInterface> pinocchioInterface;
  vector_t testState;
  vector_t testInput;
};

// Test constructor and basic properties
TEST_F(CentroidalMpcRobotModelTest, ConstructorAndDimensions) {
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getStateDim(), 12 + testingModelInterface.getMpcRobotModel().getJointDim());
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getInputDim(), 12 + testingModelInterface.getMpcRobotModel().getJointDim());
}

TEST_F(CentroidalMpcRobotModelTest, ActiveJointMappingMatchesJointNames) {
  const auto& settings = testingModelInterface.getModelSettings();
  ASSERT_EQ(settings.mpcModelToFullJointsIndices.size(), settings.mpcModelJointNames.size());
  for (size_t i = 0; i < settings.mpcModelJointNames.size(); ++i) {
    const auto fullIndex = settings.mpcModelToFullJointsIndices.at(i);
    ASSERT_LT(fullIndex, settings.fullJointNames.size());
    EXPECT_EQ(settings.fullJointNames.at(fullIndex), settings.mpcModelJointNames.at(i));
  }
}

TEST_F(CentroidalMpcRobotModelTest, JointConversionPreservesFixedJointsAndRoundTrips) {
  const auto& settings = testingModelInterface.getModelSettings();
  const auto& model = testingModelInterface.getMpcRobotModel();
  ASSERT_EQ(settings.mpcModelToFullJointsIndices.size(), settings.mpcModelJointNames.size());
  ASSERT_FALSE(settings.fixedJointNames.empty());
  const vector_t active = vector_t::LinSpaced(settings.mpc_joint_dim, 0.1, 0.9);
  const vector_t defaults = vector_t::Constant(settings.full_joint_dim, -0.25);
  const vector_t full = model.getFullModelJointAngles(active, defaults);
  for (size_t fullIndex = 0; fullIndex < settings.fullJointNames.size(); ++fullIndex) {
    const auto& name = settings.fullJointNames[fullIndex];
    const auto activeIt = settings.jointIndexMap.find(name);
    EXPECT_DOUBLE_EQ(full[fullIndex], activeIt == settings.jointIndexMap.end() ? defaults[fullIndex] : active[activeIt->second]);
  }
  EXPECT_TRUE(model.getMpcModelJointAngles(full).isApprox(active));
}

TEST_F(CentroidalMpcRobotModelTest, ResetTargetPreservesConfigurationAndClearsMomentum) {
  SystemObservation observation;
  observation.time = 3.5;
  observation.state = testState;
  observation.state.segment<6>(6) << 1.0, -2.0, 0.76, 0.7, 0.2, -0.1;
  observation.input = testInput;
  const auto target = CentroidalMpcMrtJointController::currentObservationToResetTrajectory(observation);
  ASSERT_EQ(target.stateTrajectory.size(), 1);
  EXPECT_DOUBLE_EQ(target.timeTrajectory.front(), observation.time);
  const auto& state = target.stateTrajectory.front();
  EXPECT_TRUE(state.head<6>().isZero());
  EXPECT_TRUE(state.segment<4>(6).isApprox(observation.state.segment<4>(6)));
  EXPECT_TRUE(state.segment<2>(10).isZero());
  EXPECT_TRUE(state.tail(state.size() - 12).isApprox(observation.state.tail(state.size() - 12)));
  EXPECT_EQ(target.inputTrajectory.front().size(), observation.input.size());
  EXPECT_TRUE(target.inputTrajectory.front().isZero());
}

TEST_F(CentroidalMpcRobotModelTest, VelocityTargetsStartAtConfiguredHeightAndSmoothHeightChanges) {
  scalar_t height = 0.0;
  loadData::loadCppDataType(testingModelInterface.referenceFile, "defaultBaseHeight", height);
  CentroidalMpcTargetTrajectoriesCalculator calculator(testingModelInterface.referenceFile, testingModelInterface.getMpcRobotModel(),
                                                       testingModelInterface.getPinocchioInterface(),
                                                       testingModelInterface.getCentroidalModelInfo(), 1.0);
  vector_t state = vector_t::Zero(testingModelInterface.getMpcRobotModel().getStateDim());
  state[8] = height;
  WalkingVelocityCommand command(0.0, 0.0, height, 0.0);
  const auto stationary = calculator.commandedVelocityToTargetTrajectories(command, 0.0, state);
  for (const auto& target : stationary.stateTrajectory) {
    EXPECT_TRUE(target.allFinite());
    EXPECT_NEAR(target[8], height, 1e-12);
    EXPECT_TRUE(target.head<6>().isZero(1e-12));
  }
  command.desired_pelvis_height = height - 0.1;
  const auto lower = calculator.commandedVelocityToTargetTrajectories(command, 0.02, state);
  EXPECT_NEAR(lower.stateTrajectory.front()[8], height, 1e-12);
  EXPECT_NEAR(lower.stateTrajectory.back()[8], height - 0.02, 1e-12);
  for (const auto& target : lower.stateTrajectory) {
    EXPECT_TRUE(target.allFinite());
    EXPECT_GE(target[8], command.desired_pelvis_height);
    EXPECT_LE(target[8], height + 1e-12);
  }
}

TEST_F(CentroidalMpcRobotModelTest, FirstMotionManagerTargetUsesConfiguredHeightWithoutCommands) {
  CentroidalMpcInterface interface(testingModelInterface.taskFile, testingModelInterface.urdfFile, testingModelInterface.referenceFile,
                                   false);
  scalar_t height = 0.0;
  loadData::loadCppDataType(testingModelInterface.referenceFile, "defaultBaseHeight", height);
  CentroidalMpcTargetTrajectoriesCalculator calculator(testingModelInterface.referenceFile, interface.getMpcRobotModel(),
                                                       interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), 1.0);
  TargetTrajectories captured;
  scalar_t receivedHeight = 0.0;
  const auto gaitFile = ament_index_cpp::get_package_share_directory("humanoid_common_mpc") + "/config/command/gait.info";
  Ros2ProceduralMpcMotionManager manager(
      gaitFile, testingModelInterface.referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(),
      [&](const WalkingVelocityCommand& command, scalar_t start, scalar_t, const vector_t& state) {
        receivedHeight = command.desired_pelvis_height;
        captured = calculator.commandedVelocityToTargetTrajectories(command, start, state);
        return captured;
      },
      "g1");
  vector_t state = interface.getInitialState();
  state[8] = height;
  manager.preSolverRun(0.0, 1.0, state, *interface.getReferenceManagerPtr());
  EXPECT_NEAR(receivedHeight, height, 1e-12);
  ASSERT_FALSE(captured.stateTrajectory.empty());
  for (const auto& target : captured.stateTrajectory) {
    EXPECT_TRUE(target.allFinite());
    EXPECT_NEAR(target[8], height, 1e-12);
  }
}

TEST_F(CentroidalMpcRobotModelTest, ExpiredWalkingCommandStopsMotionAndPreservesPosture) {
  CentroidalMpcInterface interface(testingModelInterface.taskFile, testingModelInterface.urdfFile, testingModelInterface.referenceFile,
                                   false);
  WalkingVelocityCommand captured;
  const auto gaitFile = ament_index_cpp::get_package_share_directory("humanoid_common_mpc") + "/config/command/gait.info";
  Ros2ProceduralMpcMotionManager manager(
      gaitFile, testingModelInterface.referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(),
      [&](const WalkingVelocityCommand& command, scalar_t start, scalar_t, const vector_t& state) {
        captured = command;
        return TargetTrajectories({start}, {state}, {testInput});
      },
      "g1");
  WalkingVelocityCommand command(0.8, 0.5, 0.76, 0.5, 0.2);
  manager.setAndScaleVelocityCommand(command);
  manager.preSolverRun(0.0, 1.0, testState, *interface.getReferenceManagerPtr());
  EXPECT_GT(captured.linear_velocity_x, 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  manager.preSolverRun(0.0, 1.0, testState, *interface.getReferenceManagerPtr());
  EXPECT_DOUBLE_EQ(captured.linear_velocity_x, 0.0);
  EXPECT_DOUBLE_EQ(captured.linear_velocity_y, 0.0);
  EXPECT_DOUBLE_EQ(captured.angular_velocity_z, 0.0);
  EXPECT_GT(captured.desired_pelvis_height, 0.7);
  EXPECT_DOUBLE_EQ(captured.desired_waist_yaw, 0.2);
  manager.setAndScaleVelocityCommand(command);
  manager.preSolverRun(0.0, 1.0, testState, *interface.getReferenceManagerPtr());
  EXPECT_GT(captured.linear_velocity_x, 0.0);
}

// Test clone functionality
TEST_F(CentroidalMpcRobotModelTest, CloneTest) {
  auto clonedModel = std::unique_ptr<CentroidalMpcRobotModel<scalar_t>>(testingModelInterface.getMpcRobotModel().clone());
  ASSERT_NE(clonedModel, nullptr);
  EXPECT_EQ(clonedModel->getStateDim(), testingModelInterface.getMpcRobotModel().getStateDim());
  EXPECT_EQ(clonedModel->getInputDim(), testingModelInterface.getMpcRobotModel().getInputDim());
}

// Test start indices
TEST_F(CentroidalMpcRobotModelTest, StartIndices) {
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getBaseStartindex(), 6);
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getJointStartindex(), 12);
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getJointVelocitiesStartindex(), 12);  // 6 * N_CONTACTS

  // Contact indices
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getContactWrenchStartIndices(0), 0);
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getContactWrenchStartIndices(1), 6);
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getContactForceStartIndices(0), 0);
  EXPECT_EQ(testingModelInterface.getMpcRobotModel().getContactMomentStartIndices(0), 3);
}

// Test generalized coordinates extraction
TEST_F(CentroidalMpcRobotModelTest, GeneralizedCoordinates) {
  vector_t genCoords = testingModelInterface.getMpcRobotModel().getGeneralizedCoordinates(testState);
  EXPECT_EQ(genCoords.size(), 6 + testingModelInterface.getMpcRobotModel().getJointDim());  // 6 (base) + 12 (joints)

  // Should contain base pose and joint angles
  EXPECT_DOUBLE_EQ(genCoords[2], 0.85);  // base z position
  EXPECT_DOUBLE_EQ(genCoords[6], 0.1);   // first joint angle
}

// Test base pose extraction and setting
TEST_F(CentroidalMpcRobotModelTest, BasePose) {
  vector6_t basePose = testingModelInterface.getMpcRobotModel().getBasePose(testState);
  EXPECT_EQ(basePose.size(), 6);
  EXPECT_DOUBLE_EQ(basePose[2], 0.85);  // z position
  EXPECT_DOUBLE_EQ(basePose[5], 0.1);   // z rotation

  // Test setting base pose
  vector_t modifiedState = testState;
  vector6_t newBasePose;
  newBasePose << 1.0, 2.0, 1.0, 0.1, 0.2, 0.3;
  testingModelInterface.getMpcRobotModel().setBasePose(modifiedState, newBasePose);

  vector6_t retrievedPose = testingModelInterface.getMpcRobotModel().getBasePose(modifiedState);
  EXPECT_TRUE(retrievedPose.isApprox(newBasePose, 1e-10));
}

// Test base position extraction and setting
TEST_F(CentroidalMpcRobotModelTest, BasePosition) {
  vector3_t basePos = testingModelInterface.getMpcRobotModel().getBasePosition(testState);
  EXPECT_EQ(basePos.size(), 3);
  EXPECT_DOUBLE_EQ(basePos[2], 0.85);

  // Test setting position
  vector_t modifiedState = testState;
  vector3_t newPos(1.5, 2.5, 1.2);
  testingModelInterface.getMpcRobotModel().setBasePosition(modifiedState, newPos);

  vector3_t retrievedPos = testingModelInterface.getMpcRobotModel().getBasePosition(modifiedState);
  EXPECT_TRUE(retrievedPos.isApprox(newPos, 1e-10));
}

// Test base orientation extraction and setting
TEST_F(CentroidalMpcRobotModelTest, BaseOrientation) {
  vector3_t baseOri = testingModelInterface.getMpcRobotModel().getBaseOrientationEulerZYX(testState);
  EXPECT_EQ(baseOri.size(), 3);
  EXPECT_DOUBLE_EQ(baseOri[2], 0.1);  // z rotation

  // Test setting orientation
  vector_t modifiedState = testState;
  vector3_t newOri(0.05, 0.1, 0.15);
  testingModelInterface.getMpcRobotModel().setBaseOrientationEulerZYX(modifiedState, newOri);

  vector3_t retrievedOri = testingModelInterface.getMpcRobotModel().getBaseOrientationEulerZYX(modifiedState);
  EXPECT_TRUE(retrievedOri.isApprox(newOri, 1e-10));
}

// Test COM velocity extraction
TEST_F(CentroidalMpcRobotModelTest, BaseComVelocity) {
  vector3_t comLinVel = testingModelInterface.getMpcRobotModel().getBaseComLinearVelocity(testState);
  EXPECT_EQ(comLinVel.size(), 3);
  EXPECT_DOUBLE_EQ(comLinVel[0], 0.1);
  EXPECT_DOUBLE_EQ(comLinVel[1], 0.2);
  EXPECT_DOUBLE_EQ(comLinVel[2], 0.0);

  vector6_t comVel = testingModelInterface.getMpcRobotModel().getBaseComVelocity(testState);
  EXPECT_EQ(comVel.size(), 6);
  EXPECT_DOUBLE_EQ(comVel[3], 0.05);  // Angular momentum component
}

// Test joint angles extraction and setting
TEST_F(CentroidalMpcRobotModelTest, JointAngles) {
  vector_t jointAngles = testingModelInterface.getMpcRobotModel().getJointAngles(testState);
  EXPECT_DOUBLE_EQ(jointAngles[0], 0.1);
  EXPECT_DOUBLE_EQ(jointAngles[1], -0.2);

  // Test setting joint angles
  vector_t modifiedState = testState;
  vector_t newJointAngles = vector_t::Random(testingModelInterface.getMpcRobotModel().getJointDim());
  testingModelInterface.getMpcRobotModel().setJointAngles(modifiedState, newJointAngles);

  vector_t retrievedAngles = testingModelInterface.getMpcRobotModel().getJointAngles(modifiedState);
  EXPECT_TRUE(retrievedAngles.isApprox(newJointAngles, 1e-10));
}

// Test joint velocities extraction and setting
TEST_F(CentroidalMpcRobotModelTest, JointVelocities) {
  vector_t jointVels = testingModelInterface.getMpcRobotModel().getJointVelocities(testState, testInput);
  EXPECT_EQ(jointVels.size(), testingModelInterface.getMpcRobotModel().getJointDim());
  EXPECT_DOUBLE_EQ(jointVels[0], 0.1);
  EXPECT_DOUBLE_EQ(jointVels[1], 0.2);

  // Test setting joint velocities
  vector_t modifiedState = testState;
  vector_t modifiedInput = testInput;
  vector_t newJointVels = vector_t::Random(testingModelInterface.getMpcRobotModel().getJointDim());
  testingModelInterface.getMpcRobotModel().setJointVelocities(modifiedState, modifiedInput, newJointVels);

  vector_t retrievedVels = testingModelInterface.getMpcRobotModel().getJointVelocities(modifiedState, modifiedInput);
  EXPECT_TRUE(retrievedVels.isApprox(newJointVels, 1e-10));
}

// Test contact wrench extraction and setting
TEST_F(CentroidalMpcRobotModelTest, ContactWrench) {
  // Test left foot (contact 0)
  vector6_t leftWrench = testingModelInterface.getMpcRobotModel().getContactWrench(testInput, 0);
  EXPECT_EQ(leftWrench.size(), 6);
  EXPECT_DOUBLE_EQ(leftWrench[2], 400.0);  // Force in z
  EXPECT_DOUBLE_EQ(leftWrench[3], 5.0);    // Moment about x

  // Test right foot (contact 1)
  vector6_t rightWrench = testingModelInterface.getMpcRobotModel().getContactWrench(testInput, 1);
  EXPECT_EQ(rightWrench.size(), 6);
  EXPECT_DOUBLE_EQ(rightWrench[2], 400.0);  // Force in z
  EXPECT_DOUBLE_EQ(rightWrench[3], -5.0);   // Moment about x

  // Test setting contact wrench
  vector_t modifiedInput = testInput;
  vector6_t newWrench;
  newWrench << 10.0, 20.0, 500.0, 1.0, 2.0, 3.0;
  testingModelInterface.getMpcRobotModel().setContactWrench(modifiedInput, newWrench, 0);

  vector6_t retrievedWrench = testingModelInterface.getMpcRobotModel().getContactWrench(modifiedInput, 0);
  EXPECT_TRUE(retrievedWrench.isApprox(newWrench, 1e-10));
}

// Test contact force extraction and setting
TEST_F(CentroidalMpcRobotModelTest, ContactForce) {
  vector3_t leftForce = testingModelInterface.getMpcRobotModel().getContactForce(testInput, 0);
  EXPECT_EQ(leftForce.size(), 3);
  EXPECT_DOUBLE_EQ(leftForce[2], 400.0);

  // Test setting contact force
  vector_t modifiedInput = testInput;
  vector3_t newForce(50.0, 60.0, 700.0);
  testingModelInterface.getMpcRobotModel().setContactForce(modifiedInput, newForce, 1);

  vector3_t retrievedForce = testingModelInterface.getMpcRobotModel().getContactForce(modifiedInput, 1);
  EXPECT_TRUE(retrievedForce.isApprox(newForce, 1e-10));
}

// Test contact moment extraction and setting
TEST_F(CentroidalMpcRobotModelTest, ContactMoment) {
  vector3_t leftMoment = testingModelInterface.getMpcRobotModel().getContactMoment(testInput, 0);
  EXPECT_EQ(leftMoment.size(), 3);
  EXPECT_DOUBLE_EQ(leftMoment[0], 5.0);
  EXPECT_DOUBLE_EQ(leftMoment[1], -2.0);

  // Test setting contact moment
  vector_t modifiedInput = testInput;
  vector3_t newMoment(8.0, 9.0, 10.0);
  testingModelInterface.getMpcRobotModel().setContactMoment(modifiedInput, newMoment, 1);

  vector3_t retrievedMoment = testingModelInterface.getMpcRobotModel().getContactMoment(modifiedInput, 1);
  EXPECT_TRUE(retrievedMoment.isApprox(newMoment, 1e-10));
}

// Test centroidal momentum extraction
TEST_F(CentroidalMpcRobotModelTest, CentroidalMomentum) {
  vector_t centroidalMomentum = testingModelInterface.getMpcRobotModel().getCentroidalMomentum(testState);
  EXPECT_EQ(centroidalMomentum.size(), 6);
  EXPECT_DOUBLE_EQ(centroidalMomentum[0], 0.1);   // vcom_x
  EXPECT_DOUBLE_EQ(centroidalMomentum[1], 0.2);   // vcom_y
  EXPECT_DOUBLE_EQ(centroidalMomentum[3], 0.05);  // L_x/m
}

// Test height adaptation
TEST_F(CentroidalMpcRobotModelTest, AdaptBasePoseHeight) {
  vector_t modifiedState = testState;
  scalar_t heightChange = 0.1;
  scalar_t originalHeight = modifiedState[6 + 2];  // Base z position

  testingModelInterface.getMpcRobotModel().adaptBasePoseHeight(modifiedState, heightChange);

  EXPECT_DOUBLE_EQ(modifiedState[6 + 2], originalHeight + heightChange);
}

// Test generalized coordinates setting
TEST_F(CentroidalMpcRobotModelTest, SetGeneralizedCoordinates) {
  vector_t modifiedState = testState;
  vector_t newGenCoords = vector_t::Random(testingModelInterface.getMpcRobotModel().getGenCoordinatesDim());  // 6 base + 12 joints

  testingModelInterface.getMpcRobotModel().setGeneralizedCoordinates(modifiedState, newGenCoords);
  vector_t retrievedGenCoords = testingModelInterface.getMpcRobotModel().getGeneralizedCoordinates(modifiedState);

  EXPECT_TRUE(retrievedGenCoords.isApprox(newGenCoords, 1e-10));
}

// Test generalized coordinates setting
TEST_F(CentroidalMpcRobotModelTest, testGeneralizedVelocities) {
  vector_t qd = testingModelInterface.getMpcRobotModel().getGeneralizedVelocities(testState, testInput);
  vector_t qd_j = qd.tail(testingModelInterface.getMpcRobotModel().getJointDim());
  EXPECT_TRUE(qd_j.isApprox(testInput.tail(qd_j.size()), 1e-10));
}

// Test boundary conditions and error handling
TEST_F(CentroidalMpcRobotModelTest, BoundaryConditions) {
  // Test with zero vectors
  vector_t zeroState = vector_t::Zero(testingModelInterface.getMpcRobotModel().getStateDim());
  vector_t zeroInput = vector_t::Zero(testingModelInterface.getMpcRobotModel().getInputDim());

  EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getBasePose(zeroState));
  EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getJointAngles(zeroState));
  EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getContactWrench(zeroInput, 0));
  EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getContactWrench(zeroInput, 1));

  // Test contact indices
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getContactWrench(testInput, i));
    EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getContactForce(testInput, i));
    EXPECT_NO_THROW(testingModelInterface.getMpcRobotModel().getContactMoment(testInput, i));
  }
}

TEST_F(CentroidalMpcRobotModelTest, HandTargetsRejectMalformedPacketsAndRecover) {
  const auto config = ament_index_cpp::get_package_share_directory("g1_centroidal_mpc");
  CentroidalMpcInterface interface(config + "/config/mpc/task_hand_pose.info", testingModelInterface.urdfFile,
                                   testingModelInterface.referenceFile, false);
  const auto references = interface.getSwitchedModelReferenceManagerPtr()->getHandPoseReferenceManagerPtr();
  const auto gaitFile = ament_index_cpp::get_package_share_directory("humanoid_common_mpc") + "/config/command/gait.info";
  Ros2ProceduralMpcMotionManager manager(
      gaitFile, testingModelInterface.referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(),
      [](const WalkingVelocityCommand&, scalar_t, scalar_t, const vector_t&) { return TargetTrajectories(); }, "g1");
  for (const std::string name : {"left_hand", "right_hand"}) {
    // Lightweight interfaces skip OCP construction, which normally registers task reference frames.
    references->setReferenceFrame(name, "torso_link");
    ASSERT_EQ(references->getReferenceFrame(name), std::optional<std::string>("torso_link"));
    geometry_msgs::msg::PoseStamped valid;
    valid.header.frame_id = "torso_link";
    valid.pose.position.x = 0.3;
    valid.pose.position.y = name == "left_hand" ? 0.2 : -0.2;
    valid.pose.orientation.w = 2.0;
    ASSERT_TRUE(manager.setHandPoseReference(name, valid));
    const auto saved = references->getReference(name);
    ASSERT_TRUE(saved);
    EXPECT_NEAR(saved->orientationReferenceToHand.norm(), 1.0, 1e-12);
    const auto rejected = [&](const geometry_msgs::msg::PoseStamped& packet) {
      EXPECT_FALSE(manager.setHandPoseReference(name, packet));
      const auto current = references->getReference(name);
      ASSERT_TRUE(current);
      EXPECT_TRUE(current->positionInReferenceFrame.isApprox(saved->positionInReferenceFrame));
      EXPECT_TRUE(current->orientationReferenceToHand.coeffs().isApprox(saved->orientationReferenceToHand.coeffs()));
    };
    for (const auto& frame : {"", "world", "pelvis", "mpc/world"}) {
      auto packet = valid;
      packet.header.frame_id = frame;
      rejected(packet);
    }
    for (const auto bad :
         {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      for (int component = 0; component < 7; ++component) {
        auto packet = valid;
        double* fields[] = {&packet.pose.position.x,    &packet.pose.position.y,    &packet.pose.position.z,   &packet.pose.orientation.x,
                            &packet.pose.orientation.y, &packet.pose.orientation.z, &packet.pose.orientation.w};
        *fields[component] = bad;
        rejected(packet);
      }
    }
    for (double norm : {0.0, 1e-15}) {
      auto packet = valid;
      packet.pose.orientation.w = norm;
      rejected(packet);
    }
    // Recover using the visualizer frame and a large, non-unit quaternion.
    auto next = valid;
    next.header.frame_id = "mpc/torso_link";
    next.pose.position.x = 0.35;
    next.pose.orientation.w = 1e200;
    next.pose.orientation.z = 1e200;
    ASSERT_TRUE(manager.setHandPoseReference(name, next));
    const auto recovered = references->getReference(name);
    ASSERT_TRUE(recovered);
    EXPECT_NEAR(recovered->positionInReferenceFrame.x(), 0.35, 1e-12);
    EXPECT_NEAR(recovered->orientationReferenceToHand.norm(), 1.0, 1e-12);
    EXPECT_NEAR(recovered->orientationReferenceToHand.w(), std::sqrt(0.5), 1e-12);
    EXPECT_NEAR(recovered->orientationReferenceToHand.z(), std::sqrt(0.5), 1e-12);
    next.pose.position.x = 1000.0;
    ASSERT_TRUE(manager.setHandPoseReference(name, next));
    EXPECT_NEAR(references->getReference(name)->positionInReferenceFrame.x(), 0.45, 1e-12);
    references->setReferenceFrame(name, "custom_reference_link");
    EXPECT_FALSE(manager.setHandPoseReference(name, next));
    next.header.frame_id = "custom_reference_link";
    ASSERT_TRUE(manager.setHandPoseReference(name, next));
  }
  geometry_msgs::msg::PoseStamped unconfigured;
  unconfigured.header.frame_id = "torso_link";
  unconfigured.pose.orientation.w = 1.0;
  EXPECT_FALSE(manager.setHandPoseReference("unknown_hand", unconfigured));
  EXPECT_FALSE(references->getReference("unknown_hand"));
}
