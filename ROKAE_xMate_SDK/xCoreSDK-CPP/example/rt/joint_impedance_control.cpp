/**
 * @file joint_impedance_control.cpp
 * @brief 实时模式 - 轴空间阻抗控制。程序适用机型xMateER7 Pro
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"
#include "../function_helper.hpp"
#include "rt_funtion_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;
  rokae::xMateErProRobot robot;
  std::string robot_ip = "192.168.0.160"; // 机器人地址
  std::string local_ip = "192.168.0.100"; // 本机地址
  std::error_code ec;
  try {
    robot.connectToRobot(robot_ip, local_ip);
  } catch (const std::exception &e) {
    std::cout << "Connection error: " << e.what();
    return 0;
  }

  // 走MoveAbsJ指令到起点
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Switch MotionControlMode error: " << ec << std::endl;
    return 0;
  }
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::vector<double> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
  std::string id;
  // 速度100mm/s, 转弯区为0
  MoveAbsJCommand absj (q_drag_xm7p, 100, 0);
  robot.moveAppend(absj, id, ec);
  robot.moveStart(ec);
  if( ec) {
    std::cerr << "MoveAbsJ error: " << ec << std::endl;
    return 0;
  }
  helper::waitRobot(robot); // 等待运动结束

  // 然后切换到实时控制
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
    // 设置要接收数据
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
  } catch (const std::exception &e) {
    std::cout << e.what();
    return 0;
  }

  double time = 0; // 周期计数 [秒]
  std::array<double, 7> jntPos {};

  // 回调函数
  std::function<JointPosition(void)> callback = [&jntPos, rtCon, &time] {
    time += 0.001;
    double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

    JointPosition cmd(7);
    for(unsigned i = 0; i < cmd.joints.size(); ++i) {
      cmd.joints[i] = jntPos[i] + delta_angle;
    }

    if(time > 60) {
      cmd.setFinished(); // 60秒后结束
    }
    return cmd;
  };

  // 设置轴空间阻抗系数，
  rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
  // 设置回调函数
  rtCon->setControlLoop(callback);
  // 更新起始位置为当前位置
  jntPos = helper::getCurrentJointPos(robot);

  try {
    // 开始轴空间阻抗运动
    rtCon->startMove(RtControllerMode::jointImpedance);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");
  } catch (const std::exception &e) {
    std::cout << "运动中报错: " << e.what();
  }

  // 下电，切换到非实时模式
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(false, ec);
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);

  return 0;
}
