/**
 * @file joint_s_line.cpp
 * @brief 实时模式 - 轴空间S规划。程序适用机型xMateER7 Pro
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

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  rokae::xMateErProRobot robot;
  try {
    robot.connectToRobot("192.168.0.160", "192.168.0.100"); // 本机地址192.168.0.100
  } catch(const std::exception &e) {
    std::cerr << "连接失败 " << e.what();
    return -1;
  }

  std::error_code ec;

  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setPowerState(true, ec);
  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
    // 设置要接收数据
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
  } catch(const std::exception &e) {
    std::cerr << "初始化实时控制失败" << e.what();
    return -1;
  }

  std::array<double,7> jntPos{}, delta{};
  JointPosition cmd(7);

  double time = 0;

  // 6个目标点
  std::vector<std::array<double, 7>> jntTargets = {
    {0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0},
    {0, -0.078, 0, 1.836, 0, 1.003, 0},
    {0, -0.054, 0, 1.331, 0, 1.485, 0},
    {0, 0.330, 0, 0.887, 0, 1.544, 0},
    {0, 0.150, 0, 1.201, 0, 1.156, 0},
    {0, 0.354, 0, 1.328, 0, 0.824, 0}
  };
  auto it = jntTargets.begin();

  std::function<JointPosition(void)> callback = [&, rtCon]() {
    time += 0.001; // 按1ms为周期规划

    JointMotionGenerator joint_s(0.8, *it);
    joint_s.calculateSynchronizedValues(jntPos);

    // 获取每个周期计算的角度偏移
    if (!joint_s.calculateDesiredValues(time, delta)) {
      for(unsigned i = 0; i < cmd.joints.size(); ++i) {
        cmd.joints[i] = jntPos[i] + delta[i];
      }
    } else {
      // 已到达一个目标点，开始运动到下一个目标点
      if (++it == jntTargets.end()) {
        cmd.setFinished();
      }
      time = 0;
      // 最后的角度值作为下一个规划的起始点
      std::copy(cmd.joints.begin(), cmd.joints.end(), jntPos.begin());
    }
    return cmd;
  };

  try {
    rtCon->setControlLoop(callback);
    // 更新回调的起始位置
    jntPos = robot.jointPos(ec);
    // 开始运动前先设置为轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);
    rtCon->startLoop(true);
    print(std::cout, "控制结束");

  } catch (const std::exception &e) {
    print(std::cerr, "实时运动错误", e.what());
  }

  return 0;
}
