/**
 * @file servoj_demo.cpp
 * @brief 实时模式 - servoj功能demo
 * 本示例仅作调用方法展示
 * 
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <ctime>
#include <cerrno>
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

// 用户指令下发周期(s)
constexpr double planPeriod = 0.02; 

// 延迟发送函数
void busy_wait(int milliseconds) {
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + std::chrono::milliseconds(milliseconds);
    while (std::chrono::high_resolution_clock::now() < end) {
    }
}

int main() {
    using namespace std;
    rokae::xMateRobot robot;
    std::error_code ec;
    try {
        robot.connectToRobot("192.168.2.160", "192.168.2.161");//机器人ip、上位机ip
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 0;
    }
    
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    // 必做：启用实时模式
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    try {
        auto rtCon = robot.getRtMotionController().lock();
        // 设置要接收数据
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});

        std::array<double, 6> jntPos{};
        std::array<double, 6> q_drag_xm3 = { M_PI/3, M_PI/6,  M_PI/6,  M_PI/6, M_PI/2,  M_PI/2};
        std::array<double, 6> q2_drag_xm3 ={0, 0,  0, 0, 0, 0};

        while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
        
        jntPos = robot.jointPos(ec);
        // 运行至拖拽位
        rtCon->MoveJ(0.3, robot.jointPos(ec), q_drag_xm3);
        // 必做：启用servoj功能
        rtCon->setServoJoint(planPeriod,planPeriod*3,1,ec);
        jntPos = robot.jointPos(ec);
        // 必做：设置运动模式，注意在启用servoj功能后
        rtCon->startMove(RtControllerMode::jointPosition);
        
        auto start = std::chrono::steady_clock::now();
        
        while(true) {
            robot.updateRobotState(std::chrono::milliseconds(1));
            // 获取当前时间
            auto now = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(now - start).count();
            // 计算目标位置
            double delta_angle = M_PI / 30.0 * (1 - std::cos(M_PI / 2.5 * elapsed_seconds));
            JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                                  jntPos[2] - delta_angle,
                                  jntPos[3] + delta_angle, jntPos[4] - delta_angle,
                                  jntPos[5] + delta_angle}};
            // 必做：发送计算出的关节位置
            rtCon->sendCommand(cmd);
            // 必做：间隔指令周期发送
            busy_wait((int)(planPeriod*1000));
            
            // 检查是否结束
            if ( elapsed_seconds > 30) {
                cmd.setFinished();
                rtCon->sendCommand(cmd);
                // 必做：关闭servoj功能
                rtCon-> stopServoJoint();
                break;
            }
        }

    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
    rtCon->MoveJ(0.3, robot.jointPos(ec), q2_drag_xm3);
    std::cout << "控制结束" << std::endl;

    // 关闭实时模式
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
