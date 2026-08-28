/**
 * @file cartesian_s_line.cpp
 * @brief 实时模式 - 笛卡尔空间S规划。程序适用机型：xMateER7 Pro
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

  try {
    // 机器人地址192.168.0.160，本机地址192.168.0.100
    robot.connectToRobot("192.168.0.160", "192.168.0.100");
  } catch(const rokae::Exception &e) {
    std::cerr << "连接失败 " << e.what();
    return 0;
  }

  std::error_code ec;

  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Set motion control mode failed " << ec.message() << std::endl;
    return 0;
  }
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);
  // 先运动到起始位置, xMate Pro机型的拖拽位姿
  MoveAbsJCommand start_joint({0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0}, 200, 0);
  std::string id;
  robot.moveAppend(start_joint, id, ec);
  robot.moveStart(ec);
  helper::waitRobot(robot);

  // 切换到实时模式, 上电
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
  } catch (const std::exception &e) {
    std::cerr << "获取实时控制器失败 " << e.what();
    return 0;
  }

  try {
    // 切换到实时模式之后，再开始接收状态数据
    robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                 {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});
  } catch (const std::exception &e) {
    std::cerr << "接收实时状态数据失败" << e.what();
    return 0;
  }

  std::array<double, 16> init_pos{}, end_pos{};
  Eigen::Quaterniond rot_cur;
  Eigen::Matrix3d mat_cur;
  double delta_s;

  static bool init = true;
  double time = 0;

  std::function<CartesianPosition()> callback = [&, rtCon]() {
    time += 0.001; // 按1ms为周期规划
    if(init) {
      // 读取当前位置
      // 注意: 只有第一个周期可以发送读取到的位置，目的是让机器人从当前位置开始运动。后续周期不能发送读取到的位置作为指令。
      robot.getStateData(RtSupportedFields::tcpPose_m, init_pos);
      end_pos = init_pos;
      end_pos[11] -= 0.2; // 设置这一段运动的目标点是从起始位置，沿Z轴负方向移动0.2米
      init = false;
    }

    std::array<double, 16> pose_start = init_pos;

    // 提取起点位置 pos_1 和目标位置 pos_2
    Eigen::Vector3d pos_1(pose_start[3], pose_start[7], pose_start[11]);
    Eigen::Vector3d pos_2(end_pos[3], end_pos[7], end_pos[11]);

    // 计算总路径向量 pos_delta 和路径长度 s。
    Eigen::Vector3d pos_delta = pos_2 - pos_1;
    double s = pos_delta.norm();
    Eigen::Vector3d pos_delta_vector = pos_delta.normalized();

    // s → 总路径长度
    CartMotionGenerator cart_s(0.05, s);
    // 同步已经运动的弧长, 也就是0
    cart_s.calculateSynchronizedValues(0);

    // 从起始和目标的齐次矩阵里提取旋转部分，转换为 Quaternion。
    // 后续会用四元数 slerp 做平滑旋转插值
    Eigen::Matrix3d mat_start, mat_end;
    mat_start << pose_start[0], pose_start[1], pose_start[2], pose_start[4], pose_start[5], pose_start[6],
      pose_start[8], pose_start[9], pose_start[10];

    mat_end << end_pos[0], end_pos[1], end_pos[2], end_pos[4], end_pos[5], end_pos[6], end_pos[8],
      end_pos[9], end_pos[10];

    Eigen::Quaterniond rot_start(mat_start);
    Eigen::Quaterniond rot_end(mat_end);

    Eigen::Vector3d pos_cur;
    CartesianPosition cmd;

    // 根据时间 time 计算已经走过的路程长度 delta_s
    // 如果未到终点，返回 false
    // 如果到达终点，返回 true，设置指令为最后一条指令
    if (!cart_s.calculateDesiredValues(time, &delta_s)) {
      // 位置插值：沿着起点和终点的直线，走 delta_s 的比例。
      pos_cur = pos_1 + pos_delta * delta_s / s;
      // 姿态插值：使用四元数 slerp，实现平滑旋转过渡
      Eigen::Quaterniond rot_cur = rot_start.slerp(delta_s / s, rot_end);
      mat_cur = rot_cur.normalized().toRotationMatrix();

      // 最终生成 4x4 齐次矩阵，写入 cmd.pos
      std::array<double, 16> new_pose = {
        {mat_cur(0, 0), mat_cur(0, 1), mat_cur(0, 2), pos_cur(0), mat_cur(1, 0), mat_cur(1, 1),
         mat_cur(1, 2), pos_cur(1), mat_cur(2, 0), mat_cur(2, 1), mat_cur(2, 2), pos_cur(2), 0, 0, 0, 1}};
      cmd.pos = new_pose;
    } else {
      cmd.setFinished();
    }
    return cmd;
  };

  try {
    // 因为callback中用到了getStateData(), 所以参数useStateDataInLoop=true
    rtCon->setControlLoop(callback, 0, true);
    // 开始运动前先设置为笛卡尔空间位置控制
    rtCon->startMove(RtControllerMode::cartesianPosition);
    rtCon->startLoop(true);
    print(std::cout, "控制结束");
  } catch (const std::exception &e) {
    std::cerr << "运动中出错 " << e.what();
  }

  // 下电，关闭实时模式
  robot.setPowerState(false, ec);
  robot.setOperateMode(OperateMode::manual, ec);
  robot.setMotionControlMode(MotionControlMode::Idle, ec);

  return 0;
}
