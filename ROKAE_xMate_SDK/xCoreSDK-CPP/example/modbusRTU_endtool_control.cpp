/**
 * @file modbusRTU_endtool_control.cpp
 * @brief 透传协议读写控制末端工具
 * @note  不同商家末端工具发送数据不同，请参照demo更改数据结构
 * 
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;

namespace DhGripParams {
 std::vector<uint8_t> send_data = { 0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D };//初始化的裸传数据
 std::vector<uint8_t> rev_data = { 0,0,0,0,0,0,0,0 };//接收裸传的返回数据
 std::vector<int>  init_set = { 0xA5 };//初始化数据
 std::vector<int>  init_get = { 100 };//获取初始化状态  0-2
 std::vector<int>  grip_status_get = { 0 };//获取夹持状态  0-3
 std::vector<int>  trq_set = { 100 };//设置力 20-100，可以改
 std::vector<int>  vel_set = { 2 };//设置速度 1-100，可以改
 std::vector<int>  pos_set = { 100 };//设置位置 0-1000，可以改
 std::vector<int>  trq_get = { 0 };//存获取到的力
 std::vector<int>  vel_get = { 0 };//存获取到的速度
 std::vector<int>  pos_get = { 0 };//存获取到的位置
 std::vector<int>  pos_now_get = { 0 };//存获取到的实时位置
}

/**
 * @brief DH电爪初始化
 */
void DHGripInit(xMateRobot &robot) {
  error_code ec;
  // 裸传的初始化
  std::vector<int>  init_set = { 0xA5 };//初始化数据
  robot.XPRWModbusRTUReg(1, 0x06, 0x0100, "int16", 1, init_set, false, ec);
  std::cout << "查询DH初始化执行结果: " << ec << std::endl;
}

/**
 * @brief DH电爪运动至
 */
void DHGripMove(xMateRobot &robot, int& trq_set, int& vel_set, int& pos_set) {
  error_code ec;
  //设置力
  std::vector<int>  trq_set_vec;//设置力 20-100，可以改
  trq_set_vec.push_back(trq_set);
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0101, "int16", 0x01, trq_set_vec, false, ec);
  std::cout << "设置力执行结果: " << ec << std::endl;
  //设置速度
  std::vector<int>  vel_set_vec;//设置速度 1-100，可以改
  vel_set_vec.push_back(vel_set);
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0104, "int16", 0x01, vel_set_vec, false, ec);
  std::cout << "设置速度执行结果: " << ec << std::endl;
  //设置位置
  std::vector<int>  pos_set_vec;//设置位置 0-1000，可以改
  pos_set_vec.push_back(pos_set);
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0103, "int16", 0x01, pos_set_vec, false, ec);
  std::cout << "设置位置执行结果: " << ec << std::endl;
}

/**
 * @brief DH电爪获取初始化状态
 */
void DHGripGetInitStatus(xMateRobot& robot,int& init_get) {
  error_code ec;
  std::vector<int>  init_get_vec = { -1 };//获取初始化状态  0-2
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0200, "int16", 0x01, init_get_vec, false, ec);
  init_get = init_get_vec[0];
  std::cout << "获取初始化状态执行结果: " << ec << std::endl;
}

/**
 * @brief DH电爪获取夹爪夹持状态
 */
void DHGripGetStatus(xMateRobot& robot, int& grip_status_get) {
  error_code ec;
  std::vector<int>  grip_status_get_vec = { -1 };//获取夹持状态  0-3
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0201, "int16", 0x01, grip_status_get_vec, false, ec);
  grip_status_get = grip_status_get_vec[0];
  std::cout << "获取初始化状态执行结果 " << ec << std::endl;
}

/*
 * @brief DH电爪获取力矩、速度、位置信息
 */
void DHGripGetInfo(xMateRobot& robot, int& trq_get, int& vel_get, int& pos_get) {
  error_code ec;
  //获取力
  std::vector<int>  trq_get_vec = { 0 };//获取力的缓冲
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0101, "int16", 0x01, trq_get_vec, false, ec);
  trq_get = trq_get_vec[0];
  std::cout << "获取力执行结果: " << ec << std::endl;
  //获取速度
  std::vector<int>  vel_get_vec = { 0 };//获取速度的缓冲
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0104, "int16", 0x01, vel_get_vec, false, ec);
  vel_get = vel_get_vec[0];
  std::cout << "获取速度执行结果: " << ec << std::endl;
  //获取位置
  std::vector<int>  pos_get_vec = { 0 };//获取位置的缓冲
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0103, "int16", 0x01, pos_get_vec, false, ec);
  pos_get = pos_get_vec[0];
  std::cout << "获取位置执行结果: " << ec << std::endl;
}

/**
 * @brief DH电爪获取实时位置
 */
void DHGripGetNewPos(xMateRobot& robot, int& pos_now_get) {
  error_code ec;
  std::vector<int> pos_now_get_vec = { 0 };
  robot.XPRWModbusRTUReg(1, 0x03, 0x0202, "int16", 1, pos_now_get_vec, false, ec);
  pos_now_get = pos_now_get_vec[0];
  std::cout << "获取实时位置执行状态: " << ec << std::endl;
}

/**
 * @brief 线圈测试接口，针对吸盘
 */
void CoilTest(xMateRobot& robot) {
  error_code ec;
  std::vector<bool> bool_data_len1 = { 0 };
  std::vector<bool> bool_data_len4 = { 0,0,0,1 };
  std::vector<bool> bool_data_len10 = { 1,1,1,1,1,1,1,1,1,1 };

  //0x01和0x02类似
  robot.XPRWModbusRTUCoil(0x01, 0x01, 0x0001, 1, bool_data_len1, false, ec);
  std::cout << "coiltest 0x01 code:" << ec << std::endl;

  robot.XPRWModbusRTUCoil(0x01, 0x02, 0x0001, 1, bool_data_len1, false, ec);
  std::cout << "coiltest 0x02 code:" << ec << std::endl;

  //0x05写单个线圈,参数3长度只能为1
  robot.XPRWModbusRTUCoil(0x01, 0x05, 0x0001, 1, bool_data_len4, false, ec);
  std::cout << "coiltest 0x05 code:" << ec << std::endl;

  //0x10写多个线圈
  robot.XPRWModbusRTUCoil(0x01, 0x0F, 0x0001, 4, bool_data_len4, false, ec);
  std::cout << "coiltest 0x10 code:" << ec << std::endl;

  robot.XPRWModbusRTUCoil(0x01, 0x0F, 0x0001, 10, bool_data_len10, false, ec);
  std::cout << "coiltest 0x10 code:" << ec << std::endl;
}

/**
 * @brief func_code寄存器测试接口
 */
void RegFuncCodeTest(xMateRobot& robot) {
  error_code ec;
  std::vector<int> int_data_len1 = { 0 };
  std::vector<int> int_data_len3 = { 0,0,1 };
  std::vector<int> int_data_len4 = { 0,0,0,1 };
  int ret = 0;

  //0x03和0x04类似
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0001, "int16", 1, int_data_len1, false, ec);
  std::cout << "regtest 0x03 code:" << ec << std::endl;

  robot.XPRWModbusRTUReg(0x01, 0x04, 0x0001, "int16", 1, int_data_len1, false, ec);
  std::cout << "regtest 0x04 code:" << ec << std::endl;

  //0x06写单个线圈,参数3长度只能为1
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0001, "int16", 1, int_data_len3, false, ec);
  std::cout << "regtest 0x06 code:" << ec << std::endl;


  //0x10写多个线圈
  robot.XPRWModbusRTUReg(0x01, 0x10, 0x0001, "int16", 1, int_data_len3, false, ec);
  std::cout << "regtest 0x10 code:" << ec << std::endl;

  robot.XPRWModbusRTUReg(0x01, 0x10, 0x0001, "int32", 1, int_data_len1, false, ec);
  std::cout << "regtest 0x10 code:" << ec << std::endl;

}

/**
 * @brief data_type寄存器测试接口,针对rm电爪
 */
void RegDataTypeTest(xMateRobot& robot) {
  error_code ec;
  std::vector<int> int16_data_len1 = { 0 };
  std::vector<int> int16_data_len2 = { 0 };
  std::vector<int> int32_data_len1_1 = { 65535 };
  std::vector<int> int32_data_len1_2 = { 255 };

  //发送：01 03 02 00 00 01 85 B2
  robot.XPRWModbusRTUReg(1, 0x03, 0x0200, "int16", 1, int16_data_len1, false, ec);
  std::cout << "regdate type test int16:" << ec << std::endl;

  //发送：01 03 02 00 00 01 85 B2
  robot.XPRWModbusRTUReg(1, 0x03, 0x0200, "uint16", 1, int16_data_len1, false, ec);
  std::cout << "regdate type test uint16:" << ec << std::endl;

  //发送：01 10 02 00 00 02 04 FF FF 00 00 EA EB
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "int32", 1, int32_data_len1_1, false, ec);
  std::cout << "regdate type test int32:" << ec << std::endl;

  //发送：01 10 02 00 00 02 04 00 FF 00 00 DA FF
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "int32", 1, int32_data_len1_2, false, ec);
  std::cout << "regdate type test int32:" << ec << std::endl;

  //发送：01 10 02 00 00 02 04 FF FF 00 00 EA EB
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "uint32", 1, int32_data_len1_1, false, ec);
  std::cout << "regdate type test uint32:" << ec << std::endl;

  //发送：01 10 02 00 00 02 04 00 FF 00 00 DA FF
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "uint32", 1, int32_data_len1_2, false, ec);
  std::cout << "regdate type test uint32:" << ec << std::endl;
}

/**
 * @brief 裸传数据接口
 */
void RWData_Test(xMateRobot& robot) {
  error_code ec;
  //裸传的初始化,用裸传数据直接进行大寰电爪的初始化，
  std::vector<uint8_t> send_data = { 0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D };//初始化的裸传数据
  std::vector<uint8_t> rev_data = {};//接收裸传数据
  robot.XPRS485SendData((int)send_data.size(), (int)rev_data.size(), send_data, DhGripParams::rev_data,ec);
  //std::vector<int>  init_set = { 0xA5 };//初始化数据，非裸传
  //robot.XPRWModbusRTUReg(1, 0x06, 0x0100, "int16", 1, init_set, false, ec);
  std::cout << "裸传方式DH电爪初始化执行结果: " << ec << std::endl;
}

/**
 * @brief main program
 */
int main() {

  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateRobot robot(ip);

    std::cout << "建立与Robot的连接" << std::endl;

    //线圈接口测试
    //CoilTest(robot);
    //寄存器功能码测试
    //RegFuncCodeTest(robot);

    //寄存器数据类型测试测试
    //RegDataTypeTest(robot);


    ////------------------------------------------DH电爪demo-----------------------------------------------
    //
    //1、打开末端xpanel通电24v和rs485
    std::cout << "设置xpanel中......" << std::endl;
    robot.setxPanelRS485(xPanelOpt::Vout::supply24v, true, ec);
    std::cout << "设置xpanel的结果: " << ec << std::endl;
    //2、---------------------------------运行示例程序--------------------------------------------
    //①初始化
    DHGripInit(robot);

    int i = 14;
    while (i--) {
      //②获取初始化状态
      int init_status = -1;
      DHGripGetInitStatus(robot, init_status);
      std::cout << "获取初始化状态: "<< init_status << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));//5s用于检查初始化，dh电爪初始化需要7s左右
    }
    //③设置力、速度、位置
    int trq_set = 100;
    int vel_set = 2;
    int pos_set = 0;
    //④移动
    DHGripMove(robot, trq_set, vel_set, pos_set);
    //⑤获取力、速度、位置
    int trq_get = 0;
    int vel_get = 0;
    int pos_get = 0;
    DHGripGetInfo(robot, trq_get, vel_get, pos_get);
    std::cout << "获取到的力: " << trq_get << std::endl;
    std::cout << "获取到的速度: " << vel_get << std::endl;
    std::cout << "获取到的位置: " << pos_get << std::endl;
    while (true) {
      //⑥获取夹爪夹持状态
      int grip_status_get = -1;
      DHGripGetStatus(robot, grip_status_get);
      std::cout << "获取夹爪夹持状态: "<< grip_status_get << std::endl;
      //⑦获取实时位置
      int pos_now_get = -1;
      DHGripGetNewPos(robot, pos_now_get);
      std::cout << "获取实时位置: " <<pos_now_get << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500)); //每0.5s获取一次当前电爪的位置状态
    }
  }
  catch (const std::exception& e) {
    std::cout << e.what();
  }
  return 0;
}