/**
 * @file get_keypad_state.cpp
 * @brief 读取末端按键状态
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;

namespace {
 xMateRobot g_robot; ///< 机器人对象
}

/**
 * @brief 示例 - 读取末端按键状态
 */
void example_ReadKeyPadValue() {
  error_code ec;
  KeyPadState state = g_robot.getKeypadState(ec);
  std::cout << "当前末端按键的状态,key1: " << state.key1_state << ",key2:"<<state.key2_state
            << ",key3:" << state.key3_state << ",key4:" << state.key4_state << ",key5:" << state.key5_state
            << ",key6:" << state.key6_state << ",key7:" << state.key7_state << std::endl;


  // 设置要接收数据。其中keypads是本示例程序会用到的
  g_robot.startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::keypads });

  std::array<bool, 7> keypad{};
  g_robot.getStateData(RtSupportedFields::keypads, keypad);

  // 运行50次
  int count = 50;

  std::thread readKeyPad([&] {
    while (count--) {
      // 每隔1秒读取一次末端按键状态
      g_robot.updateRobotState(std::chrono::milliseconds(1));
      g_robot.getStateData(RtSupportedFields::keypads, keypad);
      std::cout << "当前末端按键的状态,key1: " << keypad[0] << ",key2:" << keypad[1]
                << ",key3:" << keypad[2] << ",key4:" << keypad[3] << ",key5:" << keypad[4]
                << ",key6:" << keypad[5] << ",key7:" << keypad[6] << std::endl;
    }
  });

  readKeyPad.join();
}

/**
 * @brief 示例 - 读写IO, 寄存器
 */
void example_io_register(BaseRobot *robot) {
  error_code ec;
  print(std::cout, "DO1_0当前信号值为:", robot->getDO(1,0,ec));
  robot->setSimulationMode(true, ec); // 只有在打开输入仿真模式下才可以设置DI
  robot->setDI(0, 2, true, ec);
  print(std::cout, "DI0_2当前信号值:", robot->getDI(0, 2, ec));
  robot->setSimulationMode(false, ec); // 关闭仿真模式

  // 读取单个寄存器，类型为float
  // 假设"register0"是个寄存器数组, 长度是10
  float val_f;
  std::vector<float> val_af;
  // 读第1个，即状态监控里的register0[1], 读取结果赋值给val_f
  robot->readRegister("register0", 0, val_f, ec);
  // 读第10个，即状态监控里的register0[10], 读取结果赋值给val_f
  robot->readRegister("register0", 9, val_f, ec);
  // 读整个数组，赋值给val_af, val_af的长度也变为10。此时index参数是多少都无所谓
  robot->readRegister("register0", 9, val_af, ec);

  // 读取int类型寄存器数组
  std::vector<int> val_ai;
  robot->readRegister("register1", 1, val_ai, ec);
  // 写入bool/bit类型寄存器
  robot->writeRegister("register0", 0, true, ec);
  // 写入bool类型寄存器数组
  std::vector<bool> val_bool_array = { false,true,false,true,false,true,false };
  robot->writeRegister("register2", 0, val_bool_array, ec);
}

/**
 * @brief main program
 */
int main() {
  std::string remote_ip = "192.168.0.160";
  std::string local_ip = "192.168.0.100";
  try {
    // 本示例用到了实时状态数据，所以需要设置本机地址
    g_robot.connectToRobot(remote_ip, local_ip);
  }
  catch (const std::exception& e) {
    std::cout << "Connection error: " << e.what();
    return -1;
  }


  return 0;
}