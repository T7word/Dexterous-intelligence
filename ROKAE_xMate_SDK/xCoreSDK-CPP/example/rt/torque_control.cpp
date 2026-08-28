/**
 * @file torque_control.cpp
 * @brief 实时模式 - 直接力矩控制
 * 此示例需要使用xMateModel模型库，请设置编译选项XCORE_USE_XMATE_MODEL=ON
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include "rokae/utility.h"
#include "../function_helper.hpp"

using namespace rokae;

/**
 * @brief 力矩控制. 注意:
 * 1) 力矩值不要超过机型的限制条件(见手册);
 * 2) 初次运行时请手握急停开关, 避免机械臂非预期运动造成碰撞
 */
void torqueControl(xMateErProRobot &robot) {
  using namespace RtSupportedFields;
  auto rtCon = robot.getRtMotionController().lock();
  auto model = robot.model();
  error_code ec;

  robot.stopReceiveRobotState();
  robot.startReceiveRobotState(std::chrono::milliseconds(1),
                               {jointPos_m, jointVel_m, jointAcc_c, tcpPose_m});

  // 控制模式为力矩控制
  rtCon->startMove(RtControllerMode::torque);

  // Compliance parameters
  const double translational_stiffness{200.0};
  const double rotational_stiffness{5.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 0.0 * sqrt(translational_stiffness) *
    Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 0.0 * sqrt(rotational_stiffness) *
    Eigen::MatrixXd::Identity(3, 3);

  std::array<double, 16> init_position {};
  Eigen::Matrix<double, 6, 7> jacobian;
  Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);

  std::function<Torque(void)> callback = [&]{
    using namespace RtSupportedFields;
    static double time=0;
    time += 0.001;
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(init_position.data()).transpose());
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    std::array<double, 7> q{}, dq_m{}, ddq_c{};
    std::array<double, 16> pos_m {};

    // 接收设置为true, 回调函数中可以直接读取
    robot.getStateData(jointPos_m, q);
    robot.getStateData(jointVel_m, dq_m);
    robot.getStateData(jointAcc_c, ddq_c);

    std::array<double, 42> jacobian_array = model.jacobian(q);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 6>> jacobian_(jacobian_array.data());
    jacobian = jacobian_.transpose();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_mat(q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_mat(dq_m.data());
    robot.getStateData(tcpPose_m, pos_m);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pos_m.data()).transpose());
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired equilibrium pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    Eigen::VectorXd tau_d(7);

    // cartesian space impedance calculate && map to joint space
    tau_d << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq_mat));

    Torque cmd(7);
    Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;

    if(time > 30){
      cmd.setFinished();
    }
    return cmd;
  };

  // 由于需要在callback里读取状态数据, 这里useStateDataInLoop = true
  // 并且调用startReceiveRobotState()时, 设定的发送周期是1ms
  rtCon->setControlLoop(callback, 0, true);
  rtCon->startLoop(true);
}

/**
 * @brief 发送0力矩. 力控模型准确的情况下, 机械臂应保持静止不动
 */
template <unsigned short DoF>
void zeroTorque(Cobot<DoF> &robot) {
  error_code ec;
  auto rtCon = robot.getRtMotionController().lock();

  // 控制模式为力矩控制
  rtCon->startMove(RtControllerMode::torque);
  Torque cmd {};
  cmd.tau.resize(DoF);

  std::function<Torque(void)> callback = [&]() {
    static double time=0;
    time += 0.001;
    if(time > 30){
      cmd.setFinished();
    }
    return cmd;
  };

  rtCon->setControlLoop(callback);
  rtCon->startLoop();
  print(std::cout, "力矩控制结束");
}

/**
 * @brief main program
 */
int main() {
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.100"); // ****   xMate 7-axis

    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if(ec) {
      std::cerr << "Set motion control mode failed " << ec.message() << std::endl;
      return 0;
    }
    // 上电
    robot.setOperateMode(rokae::OperateMode::automatic,  ec);
    robot.setPowerState(true, ec);
    // 先运动到起始位置, xMate Pro机型的拖拽位姿
    MoveAbsJCommand start_joint({0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0}, 200, 0);
    std::string id;
    robot.moveAppend(start_joint, id, ec);
    robot.moveStart(ec);
    helper::waitRobot(robot);

    // 切换到实时模式控制
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    try {
      torqueControl(robot);
    } catch (const rokae::RealtimeMotionException &e) {
      // 发生错误
      print(std::cerr, e.what());
    }

    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);
    robot.setPowerState(false, ec);

  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}