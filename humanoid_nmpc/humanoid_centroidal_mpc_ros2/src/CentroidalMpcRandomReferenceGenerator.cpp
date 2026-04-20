/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sys/wait.h>
#include <unistd.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_sqp/SqpMpc.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_common_mpc/gait/GaitScheduleUpdater.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <zlib.h>

using namespace ocs2;
using namespace ocs2::humanoid;

namespace {

constexpr scalar_t kPi = 3.141592653589793238462643383279502884;

constexpr std::array<const char*, 30> kClampBodyNames = {
    "pelvis",
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
    "waist_yaw_link",
    "waist_roll_link",
    "torso_link",
    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
};

struct Options {
  std::string taskFile;
  std::string referenceFile;
  std::string urdfFile;
  std::string gaitFile;
  std::string output = "/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_random_mpc_reference.npz";
  scalar_t duration = 20.0;
  scalar_t fps = 30.0;
  uint32_t seed = 1;
  size_t numMotions = 1;
  size_t maxAttemptsPerMotion = 20;
  scalar_t vxMin = -0.5;
  scalar_t vxMax = 1.0;
  scalar_t vyMin = -0.5;
  scalar_t vyMax = 0.5;
  scalar_t yawRateMin = -0.5;
  scalar_t yawRateMax = 0.5;
  scalar_t heightMin = 0.4;
  scalar_t heightMax = 0.8;
  scalar_t baseCommandDeadband = 0.1;
  scalar_t cmdVelSegmentMin = 3.0;
  scalar_t cmdVelSegmentMax = 9.0;
  scalar_t waistSegmentMin = 1.5;
  scalar_t waistSegmentMax = 4.5;
  scalar_t armSegmentMin = 2.0;
  scalar_t armSegmentMax = 6.0;
  scalar_t wristSegmentMin = 1.5;
  scalar_t wristSegmentMax = 4.5;
  scalar_t armLimitMargin = 0.05;
  scalar_t stanceProbability = 0.2;
  scalar_t headingProbability = 0.8;
  scalar_t headingMin = -kPi;
  scalar_t headingMax = kPi;
  scalar_t headingControlStiffness = 1.0;
  scalar_t upperBodyFixedProbability = 0.2;
};

struct SmoothScalarTrajectory {
  std::vector<scalar_t> times;
  std::vector<scalar_t> values;

  scalar_t value(scalar_t time) const {
    if (times.empty() || values.empty()) {
      throw std::runtime_error("SmoothScalarTrajectory is empty.");
    }
    if (time <= times.front()) {
      return values.front();
    }
    if (time >= times.back()) {
      return values.back();
    }

    const auto upper = std::upper_bound(times.begin(), times.end(), time);
    const size_t i1 = static_cast<size_t>(std::distance(times.begin(), upper));
    const size_t i0 = i1 - 1;
    const scalar_t alpha = (time - times[i0]) / (times[i1] - times[i0]);
    const scalar_t smoothAlpha = 0.5 - 0.5 * std::cos(kPi * alpha);
    return values[i0] + smoothAlpha * (values[i1] - values[i0]);
  }
};

struct StanceSchedule {
  std::vector<scalar_t> times;
  std::vector<bool> values;

  bool value(scalar_t time) const {
    if (times.empty()) {
      return false;
    }
    const auto upper = std::upper_bound(times.begin(), times.end(), time);
    if (upper == times.begin()) {
      return values.front();
    }
    return values[static_cast<size_t>(std::distance(times.begin(), upper) - 1)];
  }
};

struct HeadingSchedule {
  std::vector<scalar_t> times;
  std::vector<bool> active;
  std::vector<scalar_t> targets;

  std::pair<bool, scalar_t> value(scalar_t time) const {
    if (times.empty()) {
      return {false, 0.0};
    }
    const auto upper = std::upper_bound(times.begin(), times.end(), time);
    size_t index = 0;
    if (upper != times.begin()) {
      index = static_cast<size_t>(std::distance(times.begin(), upper) - 1);
    }
    return {active[index], targets[index]};
  }
};

struct MotionBuffers {
  std::vector<float> jointPos;
  std::vector<float> jointVel;
  std::vector<float> bodyPos;
  std::vector<float> bodyQuat;
  std::vector<float> bodyLinVel;
  std::vector<float> bodyAngVel;
  std::vector<float> contactWrench;
  std::vector<uint8_t> contactFlags;
  std::vector<double> fps;
  size_t numFrames = 0;
  size_t numJoints = 0;
  size_t numBodies = 0;
  size_t numContacts = 0;
};

struct ZipEntry {
  std::string filename;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> compressedPayload;
  uint32_t crc = 0;
  uint32_t offset = 0;
};

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1u) ^ (0xedb88320u & mask);
    }
  }
  return ~crc;
}

std::vector<uint8_t> deflateRaw(const std::vector<uint8_t>& input) {
  if (input.empty()) {
    return {};
  }

  z_stream stream{};
  const int initResult = deflateInit2(&stream, Z_BEST_SPEED, Z_DEFLATED, -MAX_WBITS, 8, Z_DEFAULT_STRATEGY);
  if (initResult != Z_OK) {
    throw std::runtime_error("Failed to initialize zlib deflate stream.");
  }

  std::vector<uint8_t> output(deflateBound(&stream, static_cast<uLong>(input.size())));
  stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(input.data()));
  stream.avail_in = static_cast<uInt>(input.size());
  stream.next_out = reinterpret_cast<Bytef*>(output.data());
  stream.avail_out = static_cast<uInt>(output.size());

  const int deflateResult = deflate(&stream, Z_FINISH);
  if (deflateResult != Z_STREAM_END) {
    deflateEnd(&stream);
    throw std::runtime_error("Failed to deflate NPZ entry.");
  }

  output.resize(stream.total_out);
  deflateEnd(&stream);
  return output;
}

void appendU16(std::vector<uint8_t>& out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 8u) & 0xffu));
}

void appendU32(std::vector<uint8_t>& out, uint32_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 8u) & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 16u) & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 24u) & 0xffu));
}

std::string shapeString(const std::vector<size_t>& shape) {
  std::ostringstream ss;
  ss << "(";
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i > 0) {
      ss << ", ";
    }
    ss << shape[i];
  }
  if (shape.size() == 1) {
    ss << ",";
  }
  ss << ")";
  return ss.str();
}

std::vector<uint8_t> makeNpyHeader(const std::string& descr, const std::vector<size_t>& shape) {
  std::string header = "{'descr': '" + descr + "', 'fortran_order': False, 'shape': " + shapeString(shape) + ", }";
  const size_t prefixLength = 10;
  const size_t padding = 16 - ((prefixLength + header.size() + 1) % 16);
  header.append(padding, ' ');
  header.push_back('\n');

  if (header.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::runtime_error("NPY header is too large for v1.0 format.");
  }

  std::vector<uint8_t> out;
  out.reserve(prefixLength + header.size());
  out.push_back(0x93);
  const std::string magic = "NUMPY";
  out.insert(out.end(), magic.begin(), magic.end());
  out.push_back(1);
  out.push_back(0);
  appendU16(out, static_cast<uint16_t>(header.size()));
  out.insert(out.end(), header.begin(), header.end());
  return out;
}

template <typename T>
std::vector<uint8_t> makeNumericNpy(const std::vector<T>& data, const std::string& descr, const std::vector<size_t>& shape) {
  std::vector<uint8_t> out = makeNpyHeader(descr, shape);
  const auto* bytes = reinterpret_cast<const uint8_t*>(data.data());
  out.insert(out.end(), bytes, bytes + data.size() * sizeof(T));
  return out;
}

std::vector<uint8_t> makeStringNpy(const std::vector<std::string>& strings, size_t width) {
  std::vector<uint8_t> out = makeNpyHeader("|S" + std::to_string(width), {strings.size()});
  const size_t payloadOffset = out.size();
  out.resize(payloadOffset + strings.size() * width, 0);
  for (size_t i = 0; i < strings.size(); ++i) {
    const size_t n = std::min(width, strings[i].size());
    std::copy_n(strings[i].data(), n, out.data() + payloadOffset + i * width);
  }
  return out;
}

void writeZip(const std::filesystem::path& path, std::vector<ZipEntry> entries) {
  std::vector<uint8_t> out;
  for (auto& entry : entries) {
    entry.crc = crc32(entry.payload.data(), entry.payload.size());
    entry.compressedPayload = deflateRaw(entry.payload);
    entry.offset = static_cast<uint32_t>(out.size());

    appendU32(out, 0x04034b50u);
    appendU16(out, 20);
    appendU16(out, 0);
    appendU16(out, 8);
    appendU16(out, 0);
    appendU16(out, 0);
    appendU32(out, entry.crc);
    appendU32(out, static_cast<uint32_t>(entry.compressedPayload.size()));
    appendU32(out, static_cast<uint32_t>(entry.payload.size()));
    appendU16(out, static_cast<uint16_t>(entry.filename.size()));
    appendU16(out, 0);
    out.insert(out.end(), entry.filename.begin(), entry.filename.end());
    out.insert(out.end(), entry.compressedPayload.begin(), entry.compressedPayload.end());
  }

  const uint32_t centralDirectoryOffset = static_cast<uint32_t>(out.size());
  for (const auto& entry : entries) {
    appendU32(out, 0x02014b50u);
    appendU16(out, 20);
    appendU16(out, 20);
    appendU16(out, 0);
    appendU16(out, 8);
    appendU16(out, 0);
    appendU16(out, 0);
    appendU32(out, entry.crc);
    appendU32(out, static_cast<uint32_t>(entry.compressedPayload.size()));
    appendU32(out, static_cast<uint32_t>(entry.payload.size()));
    appendU16(out, static_cast<uint16_t>(entry.filename.size()));
    appendU16(out, 0);
    appendU16(out, 0);
    appendU16(out, 0);
    appendU16(out, 0);
    appendU32(out, 0);
    appendU32(out, entry.offset);
    out.insert(out.end(), entry.filename.begin(), entry.filename.end());
  }
  const uint32_t centralDirectorySize = static_cast<uint32_t>(out.size()) - centralDirectoryOffset;

  appendU32(out, 0x06054b50u);
  appendU16(out, 0);
  appendU16(out, 0);
  appendU16(out, static_cast<uint16_t>(entries.size()));
  appendU16(out, static_cast<uint16_t>(entries.size()));
  appendU32(out, centralDirectorySize);
  appendU32(out, centralDirectoryOffset);
  appendU16(out, 0);

  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open output NPZ path: " + path.string());
  }
  file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
}

void saveClampNpz(const std::filesystem::path& path,
                  const MotionBuffers& buffers,
                  const std::vector<std::string>& jointNames,
                  const std::vector<std::string>& bodyNames,
                  const std::vector<std::string>& contactNames) {
  const size_t expectedJointScalars = buffers.numFrames * buffers.numJoints;
  const size_t expectedBodyPositionScalars = buffers.numFrames * buffers.numBodies * 3;
  const size_t expectedBodyQuaternionScalars = buffers.numFrames * buffers.numBodies * 4;
  const size_t expectedContactWrenchScalars = buffers.numFrames * buffers.numContacts * 6;
  const size_t expectedContactFlagScalars = buffers.numFrames * buffers.numContacts;
  if (buffers.jointPos.size() != expectedJointScalars || buffers.jointVel.size() != expectedJointScalars ||
      buffers.bodyPos.size() != expectedBodyPositionScalars || buffers.bodyLinVel.size() != expectedBodyPositionScalars ||
      buffers.bodyAngVel.size() != expectedBodyPositionScalars || buffers.bodyQuat.size() != expectedBodyQuaternionScalars ||
      buffers.contactWrench.size() != expectedContactWrenchScalars || buffers.contactFlags.size() != expectedContactFlagScalars) {
    throw std::runtime_error("Motion buffer sizes do not match the requested NPZ shapes.");
  }

  std::vector<ZipEntry> entries;
  entries.push_back({"joint_pos.npy", makeNumericNpy(buffers.jointPos, "<f4", {buffers.numFrames, buffers.numJoints})});
  entries.push_back({"joint_vel.npy", makeNumericNpy(buffers.jointVel, "<f4", {buffers.numFrames, buffers.numJoints})});
  entries.push_back({"body_pos_w.npy", makeNumericNpy(buffers.bodyPos, "<f4", {buffers.numFrames, buffers.numBodies, 3})});
  entries.push_back({"body_quat_w.npy", makeNumericNpy(buffers.bodyQuat, "<f4", {buffers.numFrames, buffers.numBodies, 4})});
  entries.push_back({"body_lin_vel_w.npy", makeNumericNpy(buffers.bodyLinVel, "<f4", {buffers.numFrames, buffers.numBodies, 3})});
  entries.push_back({"body_ang_vel_w.npy", makeNumericNpy(buffers.bodyAngVel, "<f4", {buffers.numFrames, buffers.numBodies, 3})});
  entries.push_back({"contact_wrench_w.npy", makeNumericNpy(buffers.contactWrench, "<f4", {buffers.numFrames, buffers.numContacts, 6})});
  entries.push_back({"contact_flags.npy", makeNumericNpy(buffers.contactFlags, "|u1", {buffers.numFrames, buffers.numContacts})});
  entries.push_back({"body_names.npy", makeStringNpy(bodyNames, 32)});
  entries.push_back({"body_link_names.npy", makeStringNpy(bodyNames, 32)});
  entries.push_back({"contact_names.npy", makeStringNpy(contactNames, 32)});
  entries.push_back({"joint_names.npy", makeStringNpy(jointNames, 40)});
  entries.push_back({"fps.npy", makeNumericNpy(buffers.fps, "<f8", {1})});
  writeZip(path, std::move(entries));
}

std::string requireValue(int& index, int argc, char** argv) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("Missing value for option ") + argv[index]);
  }
  ++index;
  return argv[index];
}

scalar_t parseScalar(const std::string& text, const std::string& option) {
  try {
    return std::stod(text);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid scalar value for " + option + ": " + text);
  }
}

void requireOrderedRange(scalar_t lower, scalar_t upper, const std::string& name) {
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
    throw std::runtime_error(name + " must be finite with min <= max.");
  }
}

void requirePositiveSegmentRange(scalar_t lower, scalar_t upper, const std::string& name) {
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower <= 0.0 || upper <= 0.0 || lower > upper) {
    throw std::runtime_error(name + " must be finite, positive, and min <= max.");
  }
}

Options parseOptions(int argc, char** argv) {
  Options options;

  const auto g1MpcShare = ament_index_cpp::get_package_share_directory("g1_centroidal_mpc");
  const auto g1DescriptionShare = ament_index_cpp::get_package_share_directory("g1_description");
  const auto commonMpcShare = ament_index_cpp::get_package_share_directory("humanoid_common_mpc");
  options.taskFile = g1MpcShare + "/config/mpc/task_random_reference.info";
  options.referenceFile = g1MpcShare + "/config/command/reference_random_reference.info";
  options.urdfFile = g1DescriptionShare + "/urdf/g1_29dof.urdf";
  options.gaitFile = commonMpcShare + "/config/command/gait.info";

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--task-file") {
      options.taskFile = requireValue(i, argc, argv);
    } else if (arg == "--reference-file") {
      options.referenceFile = requireValue(i, argc, argv);
    } else if (arg == "--urdf-file") {
      options.urdfFile = requireValue(i, argc, argv);
    } else if (arg == "--gait-file") {
      options.gaitFile = requireValue(i, argc, argv);
    } else if (arg == "--output") {
      options.output = requireValue(i, argc, argv);
    } else if (arg == "--duration") {
      options.duration = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--fps") {
      options.fps = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--seed") {
      options.seed = static_cast<uint32_t>(std::stoul(requireValue(i, argc, argv)));
    } else if (arg == "--num-motions" || arg == "--motions") {
      options.numMotions = static_cast<size_t>(std::stoul(requireValue(i, argc, argv)));
    } else if (arg == "--max-attempts-per-motion") {
      options.maxAttemptsPerMotion = static_cast<size_t>(std::stoul(requireValue(i, argc, argv)));
    } else if (arg == "--vx-min") {
      options.vxMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--vx-max") {
      options.vxMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--vy-min") {
      options.vyMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--vy-max") {
      options.vyMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--yaw-rate-min") {
      options.yawRateMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--yaw-rate-max") {
      options.yawRateMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--height-min") {
      options.heightMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--height-max") {
      options.heightMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--base-command-deadband") {
      options.baseCommandDeadband = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--cmd-vel-segment-min") {
      options.cmdVelSegmentMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--cmd-vel-segment-max") {
      options.cmdVelSegmentMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--stance-probability") {
      options.stanceProbability = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--heading-prob" || arg == "--heading-probability" || arg == "--heading_prob") {
      options.headingProbability = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--heading-min") {
      options.headingMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--heading-max") {
      options.headingMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--heading-control-stiffness") {
      options.headingControlStiffness = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--upper-body-fixed-probability") {
      options.upperBodyFixedProbability = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--wrist-segment-min") {
      options.wristSegmentMin = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--wrist-segment-max") {
      options.wristSegmentMax = parseScalar(requireValue(i, argc, argv), arg);
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [--output PATH] [--duration SEC] [--fps HZ] [--seed N] [--num-motions N]\n"
                << "       [--max-attempts-per-motion N]\n"
                << "       [--vx-min V] [--vx-max V] [--vy-min V] [--vy-max V]\n"
                << "       [--yaw-rate-min V] [--yaw-rate-max V] [--height-min H] [--height-max H]\n"
                << "       [--base-command-deadband V]\n"
                << "       [--cmd-vel-segment-min SEC] [--cmd-vel-segment-max SEC]\n"
                << "       [--stance-probability P] [--heading-prob P] [--upper-body-fixed-probability P]\n"
                << "       [--heading-min RAD] [--heading-max RAD] [--heading-control-stiffness K]\n"
                << "       [--wrist-segment-min SEC] [--wrist-segment-max SEC]\n"
                << "       [--task-file PATH] [--reference-file PATH] [--urdf-file PATH] [--gait-file PATH]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  if (options.duration <= 0.0) {
    throw std::runtime_error("--duration must be positive.");
  }
  if (options.fps <= 0.0) {
    throw std::runtime_error("--fps must be positive.");
  }
  if (options.numMotions == 0) {
    throw std::runtime_error("--num-motions must be positive.");
  }
  if (options.maxAttemptsPerMotion == 0) {
    throw std::runtime_error("--max-attempts-per-motion must be positive.");
  }
  if (options.upperBodyFixedProbability < 0.0 || options.upperBodyFixedProbability > 1.0) {
    throw std::runtime_error("--upper-body-fixed-probability must be in [0, 1].");
  }
  if (options.stanceProbability < 0.0 || options.stanceProbability > 1.0) {
    throw std::runtime_error("--stance-probability must be in [0, 1].");
  }
  if (options.headingProbability < 0.0 || options.headingProbability > 1.0) {
    throw std::runtime_error("--heading-prob must be in [0, 1].");
  }
  if (options.headingControlStiffness < 0.0) {
    throw std::runtime_error("--heading-control-stiffness must be non-negative.");
  }
  if (options.baseCommandDeadband < 0.0) {
    throw std::runtime_error("--base-command-deadband must be non-negative.");
  }
  requireOrderedRange(options.vxMin, options.vxMax, "--vx-min/max");
  requireOrderedRange(options.vyMin, options.vyMax, "--vy-min/max");
  requireOrderedRange(options.yawRateMin, options.yawRateMax, "--yaw-rate-min/max");
  requireOrderedRange(options.headingMin, options.headingMax, "--heading-min/max");
  requireOrderedRange(options.heightMin, options.heightMax, "--height-min/max");
  requirePositiveSegmentRange(options.cmdVelSegmentMin, options.cmdVelSegmentMax, "--cmd-vel-segment-min/max");
  requirePositiveSegmentRange(options.waistSegmentMin, options.waistSegmentMax, "--waist-segment-min/max");
  requirePositiveSegmentRange(options.armSegmentMin, options.armSegmentMax, "--arm-segment-min/max");
  requirePositiveSegmentRange(options.wristSegmentMin, options.wristSegmentMax, "--wrist-segment-min/max");
  return options;
}

std::filesystem::path outputPathForMotion(const std::filesystem::path& requestedOutput, size_t motionIndex, size_t numMotions) {
  if (numMotions == 1) {
    return requestedOutput;
  }

  std::ostringstream suffix;
  suffix << "_" << std::setw(4) << std::setfill('0') << motionIndex;

  if (requestedOutput.has_extension()) {
    return requestedOutput.parent_path() / (requestedOutput.stem().string() + suffix.str() + requestedOutput.extension().string());
  }

  return requestedOutput / ("g1_random_mpc_reference" + suffix.str() + ".npz");
}

std::filesystem::path temporaryOutputPathForAttempt(const std::filesystem::path& outputPath, size_t attemptIndex) {
  std::ostringstream suffix;
  suffix << ".attempt_" << std::setw(3) << std::setfill('0') << attemptIndex << ".tmp";
  return outputPath.parent_path() / (outputPath.filename().string() + suffix.str());
}

void applyBaseCommandDeadband(vector4_t& command, scalar_t deadband) {
  if (deadband <= 0.0) {
    return;
  }
  for (const int index : {0, 1, 3}) {
    if (std::abs(command[index]) < deadband) {
      command[index] = 0.0;
    }
  }
}

SmoothScalarTrajectory makeRandomSmoothTrajectory(scalar_t duration,
                                                  scalar_t segmentMin,
                                                  scalar_t segmentMax,
                                                  scalar_t valueMin,
                                                  scalar_t valueMax,
                                                  scalar_t initialValue,
                                                  scalar_t holdPreviousValueProbability,
                                                  std::mt19937& rng) {
  std::uniform_real_distribution<scalar_t> segmentDist(segmentMin, segmentMax);
  std::uniform_real_distribution<scalar_t> valueDist(valueMin, valueMax);
  std::bernoulli_distribution holdDist(std::clamp(holdPreviousValueProbability, scalar_t(0.0), scalar_t(1.0)));

  SmoothScalarTrajectory trajectory;
  trajectory.times.push_back(0.0);
  trajectory.values.push_back(std::clamp(initialValue, valueMin, valueMax));

  scalar_t time = 0.0;
  while (time < duration) {
    time = std::min(duration, time + segmentDist(rng));
    trajectory.times.push_back(time);
    trajectory.values.push_back(holdDist(rng) ? trajectory.values.back() : valueDist(rng));
  }
  return trajectory;
}

StanceSchedule makeRandomStanceSchedule(scalar_t duration,
                                         scalar_t segmentMin,
                                         scalar_t segmentMax,
                                         scalar_t stanceProbability,
                                         std::mt19937& rng) {
  std::uniform_real_distribution<scalar_t> segmentDist(segmentMin, segmentMax);
  std::bernoulli_distribution stanceDist(std::clamp(stanceProbability, scalar_t(0.0), scalar_t(1.0)));

  StanceSchedule schedule;
  schedule.times.push_back(0.0);
  schedule.values.push_back(stanceDist(rng));

  scalar_t time = 0.0;
  while (time < duration) {
    time = std::min(duration, time + segmentDist(rng));
    schedule.times.push_back(time);
    schedule.values.push_back(stanceDist(rng));
  }
  return schedule;
}

HeadingSchedule makeRandomHeadingSchedule(scalar_t duration,
                                          scalar_t segmentMin,
                                          scalar_t segmentMax,
                                          scalar_t headingProbability,
                                          scalar_t headingMin,
                                          scalar_t headingMax,
                                          std::mt19937& rng) {
  std::uniform_real_distribution<scalar_t> segmentDist(segmentMin, segmentMax);
  std::uniform_real_distribution<scalar_t> targetDist(headingMin, headingMax);
  std::bernoulli_distribution headingDist(std::clamp(headingProbability, scalar_t(0.0), scalar_t(1.0)));

  HeadingSchedule schedule;
  schedule.times.push_back(0.0);
  schedule.active.push_back(headingDist(rng));
  schedule.targets.push_back(targetDist(rng));

  scalar_t time = 0.0;
  while (time < duration) {
    time = std::min(duration, time + segmentDist(rng));
    schedule.times.push_back(time);
    schedule.active.push_back(headingDist(rng));
    schedule.targets.push_back(targetDist(rng));
  }
  return schedule;
}

scalar_t wrapToPi(scalar_t angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void throwIfNotFinite(const vector_t& vector, const std::string& name, size_t motionIndex, size_t frame, scalar_t time) {
  if (!vector.allFinite()) {
    std::ostringstream ss;
    ss << "Non-finite " << name << " at motion " << motionIndex << ", frame " << frame << ", t=" << time;
    throw std::runtime_error(ss.str());
  }
}

void throwIfUnexpectedSize(const vector_t& vector,
                           Eigen::Index expectedSize,
                           const std::string& name,
                           size_t motionIndex,
                           size_t frame,
                           scalar_t time) {
  if (vector.size() != expectedSize) {
    std::ostringstream ss;
    ss << "Unexpected " << name << " size at motion " << motionIndex << ", frame " << frame << ", t=" << time << ": got "
       << vector.size() << ", expected " << expectedSize;
    throw std::runtime_error(ss.str());
  }
}

bool isWaistJoint(const std::string& jointName) {
  return jointName.find("waist") != std::string::npos;
}

bool isArmJoint(const std::string& jointName) {
  return jointName.find("shoulder") != std::string::npos || jointName.find("elbow") != std::string::npos ||
         jointName.find("wrist") != std::string::npos;
}

bool isWristJoint(const std::string& jointName) {
  return jointName.find("wrist") != std::string::npos;
}

std::vector<size_t> getRandomizedMpcJointIndices(const ModelSettings& modelSettings) {
  std::vector<size_t> indices;
  for (size_t i = 0; i < modelSettings.mpcModelJointNames.size(); ++i) {
    const auto& name = modelSettings.mpcModelJointNames[i];
    if (isWaistJoint(name) || isArmJoint(name)) {
      indices.push_back(i);
    }
  }
  return indices;
}

std::pair<scalar_t, scalar_t> getJointLimits(const pinocchio::Model& model, const std::string& jointName) {
  const pinocchio::JointIndex jointId = model.getJointId(jointName);
  if (jointId == static_cast<pinocchio::JointIndex>(model.njoints)) {
    throw std::runtime_error("Joint not found in Pinocchio model: " + jointName);
  }
  const int qIndex = model.idx_qs[jointId];
  scalar_t lower = model.lowerPositionLimit[qIndex];
  scalar_t upper = model.upperPositionLimit[qIndex];
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
    lower = -1.0;
    upper = 1.0;
  }
  return {lower, upper};
}

std::vector<std::string> toStringVector(const std::vector<std::string>& names) {
  return names;
}

std::vector<std::string> clampBodyNames() {
  std::vector<std::string> names;
  names.reserve(kClampBodyNames.size());
  for (const char* name : kClampBodyNames) {
    names.emplace_back(name);
  }
  return names;
}

void appendFullJointState(MotionBuffers& buffers, const vector_t& fullJointPos, const vector_t& fullJointVel) {
  for (Eigen::Index i = 0; i < fullJointPos.size(); ++i) {
    buffers.jointPos.push_back(static_cast<float>(fullJointPos[i]));
  }
  for (Eigen::Index i = 0; i < fullJointVel.size(); ++i) {
    buffers.jointVel.push_back(static_cast<float>(fullJointVel[i]));
  }
}

void appendBodyPoses(MotionBuffers& buffers,
                     pinocchio::Model& model,
                     pinocchio::Data& data,
                     const std::vector<pinocchio::FrameIndex>& frameIds,
                     const vector6_t& basePose,
                     const vector_t& fullJointPos) {
  vector_t q(model.nq);
  q.setZero();
  q.head<6>() = basePose;
  q.tail(fullJointPos.size()) = fullJointPos;

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  for (const auto frameId : frameIds) {
    const auto& placement = data.oMf[frameId];
    const Eigen::Quaternion<scalar_t> quat(placement.rotation());
    buffers.bodyPos.push_back(static_cast<float>(placement.translation().x()));
    buffers.bodyPos.push_back(static_cast<float>(placement.translation().y()));
    buffers.bodyPos.push_back(static_cast<float>(placement.translation().z()));
    buffers.bodyQuat.push_back(static_cast<float>(quat.w()));
    buffers.bodyQuat.push_back(static_cast<float>(quat.x()));
    buffers.bodyQuat.push_back(static_cast<float>(quat.y()));
    buffers.bodyQuat.push_back(static_cast<float>(quat.z()));
  }
}

void appendContactData(MotionBuffers& buffers,
                       const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                       const vector_t& input,
                       const contact_flag_t& contactFlags) {
  if (contactFlags.size() != buffers.numContacts) {
    throw std::runtime_error("Contact flag dimension does not match configured contacts.");
  }

  for (size_t contactIndex = 0; contactIndex < buffers.numContacts; ++contactIndex) {
    const vector6_t wrench = mpcRobotModel.getContactWrench(input, contactIndex);
    for (Eigen::Index i = 0; i < wrench.size(); ++i) {
      buffers.contactWrench.push_back(static_cast<float>(wrench[i]));
    }
  }

  for (size_t contactIndex = 0; contactIndex < buffers.numContacts; ++contactIndex) {
    buffers.contactFlags.push_back(contactFlags[contactIndex] ? uint8_t{1} : uint8_t{0});
  }
}

void fillFiniteDifferenceBodyVelocities(MotionBuffers& buffers, scalar_t dt) {
  const size_t nFrames = buffers.numFrames;
  const size_t nBodies = buffers.numBodies;
  buffers.bodyLinVel.assign(nFrames * nBodies * 3, 0.0f);
  buffers.bodyAngVel.assign(nFrames * nBodies * 3, 0.0f);

  if (nFrames < 2) {
    return;
  }

  for (size_t t = 0; t + 1 < nFrames; ++t) {
    for (size_t b = 0; b < nBodies; ++b) {
      const size_t p0 = (t * nBodies + b) * 3;
      const size_t p1 = ((t + 1) * nBodies + b) * 3;
      for (size_t k = 0; k < 3; ++k) {
        buffers.bodyLinVel[p0 + k] = (buffers.bodyPos[p1 + k] - buffers.bodyPos[p0 + k]) / static_cast<float>(dt);
      }

      const size_t q0 = (t * nBodies + b) * 4;
      const size_t q1 = ((t + 1) * nBodies + b) * 4;
      Eigen::Quaternion<scalar_t> quat0(buffers.bodyQuat[q0], buffers.bodyQuat[q0 + 1], buffers.bodyQuat[q0 + 2],
                                        buffers.bodyQuat[q0 + 3]);
      Eigen::Quaternion<scalar_t> quat1(buffers.bodyQuat[q1], buffers.bodyQuat[q1 + 1], buffers.bodyQuat[q1 + 2],
                                        buffers.bodyQuat[q1 + 3]);
      quat0.normalize();
      quat1.normalize();
      Eigen::Quaternion<scalar_t> delta = quat1 * quat0.conjugate();
      if (delta.w() < 0.0) {
        delta.coeffs() *= -1.0;
      }
      Eigen::AngleAxis<scalar_t> angleAxis(delta);
      const vector3_t omega = angleAxis.axis() * angleAxis.angle() / dt;
      const size_t w0 = (t * nBodies + b) * 3;
      buffers.bodyAngVel[w0] = static_cast<float>(omega.x());
      buffers.bodyAngVel[w0 + 1] = static_cast<float>(omega.y());
      buffers.bodyAngVel[w0 + 2] = static_cast<float>(omega.z());
    }
  }

  const size_t lastOffset = (nFrames - 1) * nBodies * 3;
  const size_t prevOffset = (nFrames - 2) * nBodies * 3;
  std::copy_n(buffers.bodyLinVel.begin() + prevOffset, nBodies * 3, buffers.bodyLinVel.begin() + lastOffset);
  std::copy_n(buffers.bodyAngVel.begin() + prevOffset, nBodies * 3, buffers.bodyAngVel.begin() + lastOffset);
}

void generateMotion(const Options& options, const std::filesystem::path& outputPath, uint32_t seed, size_t motionIndex) {
  std::cout << "[random_mpc_generator] motion " << motionIndex << " seed: " << seed << "\n";
  std::cout << "[random_mpc_generator] motion " << motionIndex << " output: " << outputPath.string() << "\n";

  std::mt19937 rng(seed);

  CentroidalMpcInterface interface(options.taskFile, options.urdfFile, options.referenceFile, true);
  interface.getSwitchedModelReferenceManagerPtr()->setArmSwingReferenceActive(false);

  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  MPC_MRT_Interface mrt(mpc);
  mrt.initRollout(&interface.getRollout());

  const auto& mpcRobotModel = interface.getMpcRobotModel();
  const auto& modelSettings = interface.modelSettings();

  scalar_t defaultBaseHeight = 0.0;
  vector_t defaultMpcJointState(mpcRobotModel.getJointDim());
  loadData::loadCppDataType(options.referenceFile, "defaultBaseHeight", defaultBaseHeight);
  loadData::loadEigenMatrix(options.referenceFile, "defaultJointState", defaultMpcJointState);

  PinocchioInterface fullPinocchioInterface = createDefaultPinocchioInterface(options.urdfFile);
  pinocchio::Model fullModel = fullPinocchioInterface.getModel();
  pinocchio::Data fullData(fullModel);

  const auto randomizedMpcJointIndices = getRandomizedMpcJointIndices(modelSettings);
  std::unordered_map<size_t, SmoothScalarTrajectory> jointTrajectories;
  for (const size_t mpcJointIndex : randomizedMpcJointIndices) {
    const std::string& jointName = modelSettings.mpcModelJointNames[mpcJointIndex];
    const auto [lowerRaw, upperRaw] = getJointLimits(fullModel, jointName);
    scalar_t lower = lowerRaw + options.armLimitMargin * (upperRaw - lowerRaw);
    scalar_t upper = upperRaw - options.armLimitMargin * (upperRaw - lowerRaw);
    scalar_t segmentMin = options.armSegmentMin;
    scalar_t segmentMax = options.armSegmentMax;
    if (isWaistJoint(jointName)) {
      lower = std::max(lower, -M_PI / 2.0);
      upper = std::min(upper, M_PI / 2.0);
      segmentMin = options.waistSegmentMin;
      segmentMax = options.waistSegmentMax;
    } else if (isWristJoint(jointName)) {
      segmentMin = options.wristSegmentMin;
      segmentMax = options.wristSegmentMax;
    }
    jointTrajectories.emplace(mpcJointIndex,
                              makeRandomSmoothTrajectory(options.duration, segmentMin, segmentMax, lower, upper,
                                                         defaultMpcJointState[mpcJointIndex],
                                                         options.upperBodyFixedProbability, rng));
    std::cout << "[random_mpc_generator] sampling " << jointName << " in [" << lower << ", " << upper << "] every [" << segmentMin
              << ", " << segmentMax << "] s\n";
  }

  const auto vxTrajectory =
      makeRandomSmoothTrajectory(options.duration, options.cmdVelSegmentMin, options.cmdVelSegmentMax, options.vxMin, options.vxMax, 0.0, 0.0,
                                 rng);
  const auto vyTrajectory =
      makeRandomSmoothTrajectory(options.duration, options.cmdVelSegmentMin, options.cmdVelSegmentMax, options.vyMin, options.vyMax, 0.0, 0.0,
                                 rng);
  const auto yawRateTrajectory = makeRandomSmoothTrajectory(options.duration, options.cmdVelSegmentMin, options.cmdVelSegmentMax,
                                                           options.yawRateMin, options.yawRateMax, 0.0, 0.0, rng);
  const auto heightTrajectory = makeRandomSmoothTrajectory(options.duration, options.cmdVelSegmentMin, options.cmdVelSegmentMax,
                                                          options.heightMin, options.heightMax, defaultBaseHeight, 0.0, rng);
  const auto stanceSchedule = makeRandomStanceSchedule(options.duration, options.cmdVelSegmentMin, options.cmdVelSegmentMax,
                                                       options.stanceProbability, rng);
  const auto headingSchedule = makeRandomHeadingSchedule(options.duration, options.cmdVelSegmentMin, options.cmdVelSegmentMax,
                                                         options.headingProbability, options.headingMin, options.headingMax, rng);

  const auto gaitMap = getGaitMap(options.gaitFile);
  const std::vector<ProceduralMpcMotionManager::GaitModeStateConfig> gaitModeStates{
      {"stance", -0.1, 0.1, -0.1, 0.1, 10.0, 10.0},
      {"slow_walk", 0.05, 0.3, 0.05, 0.2, 0.05, 0.05},
      {"walk", 0.25, 0.5, 0.15, 0.35, 0.05, 0.05},
      {"slower_trot", 0.45, 0.7, 0.3, 0.55, 0.1, 0.1},
      {"slow_trot", 0.65, 0.9, 0.5, 0.7, 0.2, 0.2},
      {"trot", 0.8, 1.3, 0.65, 10.0, 0.2, 0.2},
      {"run", 1.2, 10.0, 0.65, 10.0, 0.2, 0.2},
  };
  for (const auto& gaitModeState : gaitModeStates) {
    if (gaitMap.find(gaitModeState.gaitCommand) == gaitMap.end()) {
      throw std::runtime_error("Requested gait `" + gaitModeState.gaitCommand + "` is not present in " + options.gaitFile);
    }
  }
  auto gaitSchedulePtr = interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule();
  BreakFrequencyAlphaFilter velocityCommandFilter(5, vector4_t::Zero());
  size_t currentGaitMode = 0;
  std::string currentGaitCommand = gaitModeStates[currentGaitMode].gaitCommand;
  std::string lastGaitCommand = currentGaitCommand;
  scalar_t lastGaitChangeTime = 0.0;

  CentroidalMpcTargetTrajectoriesCalculator targetCalculator(options.referenceFile, mpcRobotModel, interface.getPinocchioInterface(),
                                                             interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);

  SystemObservation observation;
  observation.time = 0.0;
  observation.state = interface.getInitialState();
  observation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  observation.mode = ModeNumber::STANCE;

  vector4_t initialVelocityTarget;
  initialVelocityTarget << 0.0, 0.0, defaultBaseHeight, 0.0;
  for (int i = 0; i < 20; ++i) {
    targetCalculator.commandedVelocityToTargetTrajectories(initialVelocityTarget, observation.time, observation.state);
  }

  TargetTrajectories initTargetTrajectories({0.0}, {observation.state}, {observation.input});
  mrt.resetMpcNode(initTargetTrajectories);
  mrt.setCurrentObservation(observation);

  size_t attempts = 0;
  while (!mrt.initialPolicyReceived()) {
    mrt.advanceMpc();
    mrt.updatePolicy();
    if (++attempts > 20) {
      throw std::runtime_error("Failed to receive the initial MPC policy.");
    }
  }

  const std::vector<std::string> bodyNames = clampBodyNames();
  std::vector<pinocchio::FrameIndex> bodyFrameIds;
  bodyFrameIds.reserve(bodyNames.size());
  for (const auto& bodyName : bodyNames) {
    const auto frameId = fullModel.getFrameId(bodyName);
    if (frameId == static_cast<pinocchio::FrameIndex>(fullModel.nframes)) {
      throw std::runtime_error("Body frame not found in full Pinocchio model: " + bodyName);
    }
    bodyFrameIds.push_back(frameId);
  }

  vector_t defaultFullJointState = vector_t::Zero(modelSettings.full_joint_dim);
  defaultFullJointState = mpcRobotModel.getFullModelJointAngles(defaultMpcJointState, defaultFullJointState);

  MotionBuffers buffers;
  buffers.numJoints = modelSettings.full_joint_dim;
  buffers.numBodies = bodyNames.size();
  buffers.numContacts = modelSettings.contactNames6DoF.size();
  buffers.fps = {options.fps};

  const scalar_t dt = 1.0 / options.fps;
  const size_t numFrames = static_cast<size_t>(std::floor(options.duration * options.fps)) + 1;

  for (size_t frame = 0; frame < numFrames; ++frame) {
    const scalar_t time = observation.time;

    vector_t sampledMpcJointState = defaultMpcJointState;
    for (const auto& [jointIndex, trajectory] : jointTrajectories) {
      sampledMpcJointState[jointIndex] = trajectory.value(time);
    }

    const bool forceStance = stanceSchedule.value(time);
    vector4_t velocityTarget;
    velocityTarget << vxTrajectory.value(time), vyTrajectory.value(time), heightTrajectory.value(time), yawRateTrajectory.value(time);
    if (forceStance) {
      velocityTarget[0] = 0.0;
      velocityTarget[1] = 0.0;
      velocityTarget[3] = 0.0;
    }
    vector4_t filteredVelCommand = velocityCommandFilter.getFilteredVector(velocityTarget);
    const auto [useHeading, headingTarget] = headingSchedule.value(time);
    if (!forceStance && useHeading) {
      // Base pose layout is [x, y, z, yaw, pitch, roll].
      const scalar_t baseYaw = mpcRobotModel.getBasePose(observation.state)[3];
      const scalar_t yawRate = options.headingControlStiffness * wrapToPi(headingTarget - baseYaw);
      filteredVelCommand[3] = std::clamp(yawRate, options.yawRateMin, options.yawRateMax);
    }
    if (forceStance) {
      filteredVelCommand[0] = 0.0;
      filteredVelCommand[1] = 0.0;
      filteredVelCommand[3] = 0.0;
    }
    applyBaseCommandDeadband(filteredVelCommand, options.baseCommandDeadband);
    const vector6_t baseVelocity = mpcRobotModel.getBaseComVelocity(observation.state);
    auto currentCfg = gaitModeStates[currentGaitMode];

    if (forceStance && currentGaitCommand != "stance") {
      currentGaitMode = 0;
      currentCfg = gaitModeStates[currentGaitMode];
      currentGaitCommand = currentCfg.gaitCommand;
      lastGaitChangeTime = time;
      std::cout << "[random_mpc_generator] forcing stance at t=" << std::fixed << std::setprecision(2) << time << "\n";
    } else if (!forceStance && time > lastGaitChangeTime + 0.2) {
      if (currentGaitMode + 1 < gaitModeStates.size() &&
          ProceduralMpcMotionManager::transitionToFasterGait(filteredVelCommand, baseVelocity, currentCfg)) {
        ++currentGaitMode;
        currentCfg = gaitModeStates[currentGaitMode];
        currentGaitCommand = currentCfg.gaitCommand;
        lastGaitChangeTime = time;
        std::cout << "[random_mpc_generator] increasing to gait `" << currentGaitCommand << "` at t=" << std::fixed
                  << std::setprecision(2) << time << "\n";
      } else if (currentGaitMode > 0 &&
                 ProceduralMpcMotionManager::transitionToSlowerGait(filteredVelCommand, baseVelocity, currentCfg)) {
        --currentGaitMode;
        currentCfg = gaitModeStates[currentGaitMode];
        currentGaitCommand = currentCfg.gaitCommand;
        lastGaitChangeTime = time;
        std::cout << "[random_mpc_generator] decreasing to gait `" << currentGaitCommand << "` at t=" << std::fixed
                  << std::setprecision(2) << time << "\n";
      }
    }

    if (currentGaitCommand != lastGaitCommand) {
      GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr, gaitMap.at(currentGaitCommand), time,
                                              time + interface.mpcSettings().timeHorizon_);
      lastGaitCommand = currentGaitCommand;
    }

    targetCalculator.setTargetJointState(sampledMpcJointState);
    TargetTrajectories targetTrajectories =
        targetCalculator.commandedVelocityToTargetTrajectories(filteredVelCommand, time, observation.state);
    mrt.getReferenceManager().setTargetTrajectories(targetTrajectories);

    mrt.setCurrentObservation(observation);
    mrt.advanceMpc();
    mrt.updatePolicy();

    vector_t nextState;
    vector_t nextInput;
    size_t nextMode = observation.mode;
    mrt.rolloutPolicy(time, observation.state, dt, nextState, nextInput, nextMode);
    throwIfUnexpectedSize(nextState, observation.state.size(), "rollout state", motionIndex, frame, time);
    throwIfUnexpectedSize(nextInput, observation.input.size(), "rollout input", motionIndex, frame, time);
    throwIfNotFinite(nextState, "rollout state", motionIndex, frame, time);
    throwIfNotFinite(nextInput, "rollout input", motionIndex, frame, time);

    observation.time = time + dt;
    observation.state = std::move(nextState);
    observation.input = std::move(nextInput);
    observation.mode = nextMode;

    const vector_t fullJointPos = mpcRobotModel.getFullModelJointAngles(mpcRobotModel.getJointAngles(observation.state), defaultFullJointState);
    vector_t fullJointVel = vector_t::Zero(modelSettings.full_joint_dim);
    fullJointVel = mpcRobotModel.getFullModelJointAngles(mpcRobotModel.getJointVelocities(observation.state, observation.input), fullJointVel);

    appendFullJointState(buffers, fullJointPos, fullJointVel);
    appendBodyPoses(buffers, fullModel, fullData, bodyFrameIds, mpcRobotModel.getBasePose(observation.state), fullJointPos);
    appendContactData(buffers, mpcRobotModel, observation.input,
                      interface.getSwitchedModelReferenceManagerPtr()->getContactFlags(observation.time));
    ++buffers.numFrames;

    if (frame % 25 == 0) {
      std::cout << "[random_mpc_generator] frame " << frame << "/" << (numFrames - 1) << " time=" << std::fixed
                << std::setprecision(2) << observation.time << "\n";
    }
  }

  fillFiniteDifferenceBodyVelocities(buffers, dt);
  saveClampNpz(outputPath, buffers, toStringVector(modelSettings.fullJointNames), bodyNames, modelSettings.contactNames6DoF);

  std::cout << "[random_mpc_generator] saved " << buffers.numFrames << " frames to " << outputPath.string() << "\n";
}

int runGenerationAttempt(const Options& options, const std::filesystem::path& outputPath, uint32_t seed, size_t motionIndex) {
  std::cout.flush();
  std::cerr.flush();

  const pid_t pid = fork();
  if (pid < 0) {
    throw std::runtime_error("Failed to fork a random MPC generation attempt.");
  }

  if (pid == 0) {
    try {
      generateMotion(options, outputPath, seed, motionIndex);
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(EXIT_SUCCESS);
    } catch (const std::exception& e) {
      std::cerr << "[random_mpc_generator] child attempt failed with seed " << seed << ": " << e.what() << "\n";
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(EXIT_FAILURE);
    } catch (...) {
      std::cerr << "[random_mpc_generator] child attempt failed with seed " << seed << ": unknown exception\n";
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(EXIT_FAILURE);
    }
  }

  int status = 0;
  while (waitpid(pid, &status, 0) == -1) {
    if (errno == EINTR) {
      continue;
    }
    throw std::runtime_error("Failed while waiting for a random MPC generation attempt.");
  }
  return status;
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);

  std::cout << "[random_mpc_generator] task file: " << options.taskFile << "\n";
  std::cout << "[random_mpc_generator] reference file: " << options.referenceFile << "\n";
  std::cout << "[random_mpc_generator] urdf file: " << options.urdfFile << "\n";
  std::cout << "[random_mpc_generator] gait file: " << options.gaitFile << "\n";
  std::cout << "[random_mpc_generator] output: " << options.output << "\n";
  std::cout << "[random_mpc_generator] num motions: " << options.numMotions << "\n";
  std::cout << "[random_mpc_generator] max attempts per motion: " << options.maxAttemptsPerMotion << "\n";

  for (size_t motionIndex = 0; motionIndex < options.numMotions; ++motionIndex) {
    const auto outputPath = outputPathForMotion(options.output, motionIndex, options.numMotions);
    bool generated = false;
    for (size_t attemptIndex = 0; attemptIndex < options.maxAttemptsPerMotion; ++attemptIndex) {
      const uint32_t attemptSeed =
          options.seed + static_cast<uint32_t>(motionIndex * options.maxAttemptsPerMotion + attemptIndex);
      const auto temporaryOutputPath = temporaryOutputPathForAttempt(outputPath, attemptIndex);
      std::filesystem::remove(temporaryOutputPath);

      const int attemptStatus = runGenerationAttempt(options, temporaryOutputPath, attemptSeed, motionIndex);
      if (WIFEXITED(attemptStatus) && WEXITSTATUS(attemptStatus) == EXIT_SUCCESS) {
        std::filesystem::remove(outputPath);
        std::filesystem::rename(temporaryOutputPath, outputPath);
        std::cout << "[random_mpc_generator] motion " << motionIndex << " accepted after attempt " << (attemptIndex + 1)
                  << "/" << options.maxAttemptsPerMotion << ": " << outputPath.string() << "\n";
        generated = true;
        break;
      } else {
        std::filesystem::remove(temporaryOutputPath);
        std::cerr << "[random_mpc_generator] motion " << motionIndex << " attempt " << (attemptIndex + 1) << "/"
                  << options.maxAttemptsPerMotion << " failed with seed " << attemptSeed;
        if (WIFSIGNALED(attemptStatus)) {
          std::cerr << " due to signal " << WTERMSIG(attemptStatus);
        } else if (WIFEXITED(attemptStatus)) {
          std::cerr << " with exit code " << WEXITSTATUS(attemptStatus);
        }
        std::cerr << "\n";
      }
    }

    if (!generated) {
      std::ostringstream ss;
      ss << "Failed to generate motion " << motionIndex << " after " << options.maxAttemptsPerMotion << " attempts.";
      throw std::runtime_error(ss.str());
    }
  }
  return 0;
}
