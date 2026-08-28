/**
 * @file force_control_commands.cpp
 * @brief 力控指令
 * @note 力控指令参数设置通常和机型相关。
 * 每个示例写了测试机型，若机型不同，请参考《xCore控制系统手册》，确认参数合适后再运行示例。
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
void waitRobot(BaseRobot *robot);

/**
 * @brief 示例 - 笛卡尔控制力控。适用机型：xMateCR
 * Example - cartesian space force control
 */
void fcCartesianControl(xMateRobot &robot) {
  error_code ec;

  // 设置手持工具坐标系，Ry旋转90°
  Toolset toolset1;
  toolset1.end.rpy[1] = M_PI_2;
  robot.setToolset(toolset1, ec);

  auto fc = robot.forceControl();

  // 力控初始化，使用工具坐标系
  // force control initialization, use tool frame
  fc.fcInit(FrameType::tool, ec);
  // 笛卡尔控制模式
  // set force control type
  fc.setControlType(1, ec);
  // 设置笛卡尔刚度。本示例用的是工具坐标系，所以工具坐标系的x方向0阻抗，其余方向阻抗较大
  fc.setCartesianStiffness({0, 1000, 1000, 500, 500, 500}, ec);
  // 开始力控
  print(std::cout , "开始笛卡尔模式力控");
  fc.fcStart(ec);

#if 0
  // 设置负载, 请根据实际情况设置，确保安全
  Load load;
  load.mass = 1;
  fc.setLoad(load, ec);
#endif

  // 设置期望力
  // set desired force along Z axis
  fc.setCartesianDesiredForce({0, 0, 1, 0, 0, 0}, ec);

  // 按回车结束力控
  while(getchar() != '\n');
  fc.fcStop(ec);
}

/**
 * @brief 示例 - 关节模式力控。适用机型：xMateCR
 */
 template <unsigned short DoF>
void fcJointControl(ForceControl_T<DoF> &fc) {
  error_code ec;

  fc.fcInit(FrameType::base, ec);
  fc.setControlType(0, ec);
  // 设置各轴刚度。2轴4轴小阻抗，其余轴阻抗较大
  fc.setJointStiffness({1000, 10, 1000, 5, 50, 50}, ec);

  print(std::cout, "开始关节模式力控");
  fc.fcStart(ec);
  // 设置期望力
  fc.setJointDesiredTorque({1,1,3,0,0,0}, ec);

  // 按回车结束力控
  while(getchar() != '\n');
  fc.fcStop(ec);
}

/**
 * @brief 示例 - 搜索运动 & 力控监控。测试机型：xMateER3
 */
void fcOverlay(xMateRobot &robot){
  error_code ec;

  // 设置手持工具坐标系，Ry旋转90°
  Toolset toolset1;
  toolset1.end.rpy[1] = M_PI_2;
  robot.setToolset(toolset1, ec);

  auto fc = robot.forceControl();

  // 可选：设置力控监控参数，示例里用的参数是xMateER3机型默认阈值
  // 设置最大轴速度
  fc.setJointMaxVel({3.0, 3.0, 3.5, 3.5, 3.5, 4.0}, ec);
  // 设置最大轴动量
  fc.setJointMaxMomentum({0.1, 0.1, 0.1, 0.055, 0.055, 0.055}, ec);
  // 设置最大轴动能
  fc.setJointMaxEnergy({250, 250, 250, 150, 150, 100}, ec);
  // 设置笛卡尔空间最大速度
  fc.setCartesianMaxVel({1.0, 1.0, 1.0, 2.5, 2.5, 2.5}, ec);
  // 开始监控
  fc.fcMonitor(true, ec);

  // 力控初始化
  fc.fcInit(FrameType::tool, ec);
  // 搜索运动必须为笛卡尔阻抗控制
  fc.setControlType(1, ec);

  // 设置绕Z轴(因为前面指定了力控坐标系为工具坐标系，所有这里是工具Z轴)的正弦搜索运动
  fc.setSineOverlay(2, 6, 1, M_PI, 1, ec);
  // 开始力控
  fc.fcStart(ec);
  // 叠加XZ平面莉萨如搜索运动
  fc.setLissajousOverlay(1, 5, 1, 10, 5, 0, ec);
  // 开始搜索运动
  print(std::cout, "开始搜索运动");
  fc.startOverlay(ec);

#if 0
  // 暂停和重新开始搜索运动
  fc.pauseOverlay(ec);
  fc.restartOverlay(ec);
#endif

  // 按回车结束力控
  while(getchar() != '\n');
  fc.stopOverlay(ec);

  // 监控参数恢复到默认值
  fc.fcMonitor(false, ec);
  // 停止力控
  fc.fcStop(ec);
}

/**
 * @brief 示例 - 设置力控终止条件。测试机型：xMateER3
 */
void fcCondition(xMateRobot &robot) {
  auto fc = robot.forceControl();
  error_code ec;
  Toolset toolset;
  toolset.ref.trans[2] = 0.1;
  robot.setToolset(toolset, ec);

  fc.fcInit(FrameType::world, ec);
  fc.setControlType(1, ec);
  fc.fcStart(ec);
  // 设置力限制
  fc.setForceCondition({-20, 20, -15, 15, -15, 15}, true, 20, ec);
  // 设置长方体区域限制, isInside=false代表在这个区域内时终止等待
  // 长方体所在的坐标系，会叠加外部工件坐标系
  Frame supvFrame;
  supvFrame.trans[2] = -0.1;
  fc.setPoseBoxCondition(supvFrame, {-0.6, 0.6, -0.6, 0.6, 0.2, 0.3}, false, 20, ec);

  // 阻塞等待满足终止条件
  print(std::cout, "开始等待");
  fc.waitCondition(ec);

  print(std::cout, "等待结束，停止力控");
  fc.fcStop(ec);
}

/**
 * @brief 读取末端力矩信息
 */
template <unsigned short DoF>
void readTorqueInfo(ForceControl_T<DoF> &fc) {
  error_code ec;
  std::array<double, DoF> joint_torque{}, external_torque{};
  std::array<double, 3> cart_force{}, cart_torque{};

  // 读取当前力矩信息
  fc.getEndTorque(FrameType::flange, joint_torque, external_torque, cart_torque, cart_force, ec);
  print(std::cout, "末端力矩");
  print(std::cout, "各轴测量力 -", joint_torque);
  print(std::cout, "各轴外部力 -", external_torque);
  print(std::cout, "笛卡尔力矩 -", cart_torque);
  print(std::cout, "笛卡尔力   -", cart_force);
}

/**
 * @brief 示例-力矩传感器标定
 */
void example_CalibrateForceSensor(xMateRobot &robot) {
  error_code ec;
  // 标定全部轴
  robot.calibrateForceSensor(true, 0, ec);
  // 单轴(4轴)标定
  robot.calibrateForceSensor(false, 3, ec);
}

/**
 * @brief main program
 */
int main() {
  using namespace rokae;
  xMateRobot robot;
  try {
    robot.connectToRobot("192.168.0.160");
  } catch(const Exception &e) {
    std::cerr << e.what();
    return 1;
  }
  error_code ec;

  // 力控类
  auto fc = robot.forceControl();
  readTorqueInfo(fc);

  // 上电
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  // 先运动到拖拽位姿, 注意选择正确的机型
  std::vector<double> drag_cr = {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0},
  drag_er = {0, M_PI/6, M_PI/3, 0, M_PI_2, 0};
  robot.executeCommand({MoveAbsJCommand(drag_cr)}, ec);
  waitRobot(&robot);

  // 运行示例程序
  fcCartesianControl(robot);
  fcJointControl(fc);
//  fcCondition(robot);

  robot.setPowerState(false, ec);
  robot.setOperateMode(OperateMode::manual, ec);

  return 0;
}

/**
 * @brief 等待机器人静止
 * @param robot robot pointer
 */
void waitRobot(BaseRobot *robot) {
  bool running = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }}