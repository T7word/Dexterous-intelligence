/**
 * @file read_robot_state.cpp
 * @brief 读取机器人状态数据示例
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <thread>
#include <atomic>
#include <fstream>
#include "rokae/utility.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;

void WaitRobot(BaseRobot *robot);

/**
 * @brief main program
 */
int main() {
  try {
    using namespace RtSupportedFields;
    xMateRobot robot("192.168.0.160", "192.168.0.100");
    error_code ec;
    std::ostream &os = std::cout;
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);

    // 设置数据发送间隔为1s, 接收机器人末端位姿、关节力矩和关节角度
    robot.startReceiveRobotState(chrono::seconds(1), {tcpPoseAbc_m, tau_m, jointPos_m});
    std::array<double, 6> tcpPose{};
    std::array<double, 6> arr6{};

    std::atomic_bool running{true};

    // 接收状态数据的队列不会自动覆盖旧数据，可以通过循环读取的方法清除旧数据
    while (robot.updateRobotState(chrono::steady_clock::duration::zero()));
    // 输出到文件
    std::ofstream ofs;
    ofs.open(("read_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count()) + ".csv"), std::ios::out);

    // 打印末端位姿和关节角度到控制台
    std::thread readState([&] {
      while (running) {
        // 周期性获取当前状态数据，参数timeout最好和设置的数据发送间隔保持一致
        // 或者按照发送频率读取
        robot.updateRobotState(chrono::seconds(1));
        robot.getStateData(tcpPoseAbc_m, tcpPose);
        robot.getStateData(jointPos_m, arr6);
        ofs << tcpPose[0] << "," << tcpPose[1] << "," << tcpPose[2] << ","
          << tcpPose[3] << "," << tcpPose[4] << "," << tcpPose[5] << ",,"
         << arr6[0] << ","<< arr6[1] << ","<< arr6[2] << ","
          << arr6[3] << ","<< arr6[4] << ","<< arr6[5] <<std::endl;
//        print(os, "Ts:", ts, "TCP pose:", pose, "\nJoint:", Utils::radToDeg(jnt));
      }
    });

    // 开始一个运动线程
    std::thread moveThread([&]{
      robot.setOperateMode(rokae::OperateMode::automatic, ec);
      robot.setPowerState(true, ec);
      MoveAbsJCommand p1({0,0,0,0,0,0}), p2({0, M_PI/6, M_PI/3, 0, M_PI_2, 0});
      std::string id;
      robot.moveAppend({p1, p2}, id, ec);
      robot.moveStart(ec);
      WaitRobot(&robot);
    });

    // 等待运动结束
    moveThread.join();
    running = false;
    readState.join();

    // 控制器停止发送
    robot.stopReceiveRobotState();

  } catch(const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}

/**
 * @brief 等待机器人停止
 */
void WaitRobot(BaseRobot *robot) {
  bool checking = true;
  while (checking) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      checking = false;
    }
  }
}