/**
 * @file cartesian_impedance_control.cpp
 * @brief 实时模式 - 笛卡尔阻抗控制
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "../print_helper.hpp"
#include "rt_funtion_helper.hpp"
#include "../function_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;
  rokae::xMateErProRobot robot; // ****   xMate 7-axis
  std::string robot_ip = "192.168.0.160"; // 机器人地址
  std::string local_ip = "192.168.0.100"; // 本机地址
  std::error_code ec;

  try {
    robot.connectToRobot(robot_ip, local_ip);
  } catch (const std::exception &e) {
    std::cerr << "Connect to robot failed " << e.what() << std::endl;
    return 0;
  }

  // 关闭实时模式
  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Set motion control mode failed " << ec.message() << std::endl;
    return 0;
  }
  // 自动模式，上电
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  // 先运动到起始位置, xMate Pro机型的拖拽位姿
  MoveAbsJCommand start_joint({0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0}, 200, 0);
  std::string id;
  robot.moveAppend(start_joint, id, ec);
  robot.moveStart(ec);
  if(ec) {
    std::cerr << "Move failed " << ec.message() << std::endl;
    return 0;
  }
  // 等待运动结束
  helper::waitRobot(robot);

  // 设置实时模式网络阈值50%。需要在启动实时模式之前设置
  robot.setRtNetworkTolerance(50, ec);
  // 切换到实时模式, 上电
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;

  try {
    rtCon = robot.getRtMotionController().lock();
  } catch (const std::exception &e) {
    std::cerr << "Get rt motion controller failed " << e.what() << std::endl;
    return 0;
  }

  // 设置力控坐标系为工具坐标系, 末端相对法兰的坐标系
  std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
  rtCon->setFcCoor(toolToFlange, FrameType::tool, ec);
  // 设置笛卡尔阻抗系数
  rtCon->setCartesianImpedance({1200, 1200, 0, 100, 100, 0}, ec);
  // 设置X和Z方向3N的期望力
  rtCon->setCartesianImpedanceDesiredTorque({3, 0, 3, 0, 0, 0}, ec);

  try {
    // 切换到实时模式控制之后，再开始接收状态数据，确保读到的数据是实时模式下的
    robot.startReceiveRobotState(std::chrono::milliseconds(8), {RtSupportedFields::tcpPose_m});
  } catch (const std::exception &e) {
    std::cerr << "Start receive robot state failed " << e.what() << std::endl;
    return 0;
  }
  // 获取实时模式用的当前笛卡尔位姿，作为起点
  std::array<double, 16> init_position = helper::getCurrentPose_matrix(robot);
  std::cout << "初始位置: " << init_position << std::endl;

  // 记录规划到第几个周期了（也就是第几个毫秒）
  double time = 0;

  std::atomic<bool> stopManually {true};
  // 定义回调函数，内容是每周期要执行的计算，返回计算出的笛卡尔指令
  std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
    time += 0.001; // 由于控制周期固定是1kHz,所以这个时间每次固定增加0.001s

    // 示例: 基于余弦函数的平滑 S 曲线位移规划
    constexpr double kRadius = 0.2; // 最大位移幅度是0.2m

    // 初始速度为 0, 最终速度为 0, 中间加速-减速过程平滑，符合余弦加加速度（jerk）连续的轨迹规划。
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time));
    // 随 angle 的变化走一个半周期余弦波形，形成光滑的上下运动。
    double delta_z = kRadius * (std::cos(angle) - 1);

    CartesianPosition output{};
    output.pos = init_position;
    // 把每个周期变化量叠加在起始位姿上
    output.pos[7] += delta_z;

    // 持续运行40秒
    if(time > 40){
      std::cout << "运动结束" <<std::endl;
      output.setFinished(); // 表明返回的cmd是最后一条指令
      stopManually.store(false); // loop为非阻塞，和主线程同步停止状态
    }
    return output;
  };

  try {
    rtCon->setControlLoop(callback);
    // 开始笛卡尔阻抗控制
    rtCon->startMove(RtControllerMode::cartesianImpedance);
    rtCon->startLoop(false);
    while (stopManually.load());
    // 非阻塞loop, 需要调用一次停止循环
    rtCon->stopLoop();
  } catch (const std::exception &e) {
    std::cerr << "RT move error occur " << e.what() << std::endl;
  }

  // 下电，关闭实时模式
  robot.setPowerState(false, ec);
  robot.setOperateMode(OperateMode::manual, ec);
  robot.setMotionControlMode(MotionControlMode::Idle, ec);

  return 0;
}
