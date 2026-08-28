/**
 * @file sdk_example.cpp
 * @brief SDK各接口使用示例
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include "rokae/model.h"

using namespace rokae;
std::ostream &os = std::cout; ///< print to console

namespace Workflow {

 /**
  * @brief 示例 - 工具/工件/基坐标系标定
  */
 template<WorkType Wt, unsigned short DoF>
 class CalibrateFrame {
  public:
   /**
    * @brief Constructor
    * @param robot 已创建好的机器人类
    * @param type 标定坐标系
    * @param point_num 传入的位置数量，对应N点法
    * @param is_held 是否机器人手持
    * @param base_aux 基坐标系标定辅助点
    */
   CalibrateFrame(Robot_T<Wt,DoF> &robot, FrameType type, int point_num, bool is_held, const std::array<double, 3> &base_aux = {})
     : robot_(&robot), type_(type), point_list_(point_num), is_held_(is_held), base_aux_(base_aux) {}

   /**
    * @brief 设置标定点
    */
   void setPoint(unsigned point_index) {
     if(point_index >= point_list_.size()) {
       // 自行添加异常处理
       print(std::cerr, "标定点下标超出范围");
       return;
     }
     error_code ec;
     // please guarantee robot is valid before calling
     point_list_[point_index] = robot_->jointPos(ec);
   }

   /**
    * @brief 所有标定位置已确认，得到标定结果
    * @param ec 标定结果错误码
    * @return 标定结果
    */
   FrameCalibrationResult confirm(error_code &ec) {
     return robot_->calibrateFrame(type_, point_list_, is_held_, ec, base_aux_);
   }

  private:
   Robot_T<Wt, DoF> *robot_;
   FrameType type_;
   std::vector<std::array<double, DoF>> point_list_;
   bool is_held_;
   std::array<double, 3> base_aux_;
 };

 /**
  * @brief 示例 - 机械臂超出软限位后Jog回到限位内
  */
 template<WorkType Wt, unsigned short DoF>
 void recoveryFromOverJointRange(Robot_T<Wt, DoF> *robot) {
   error_code ec;
   auto curr_joint = robot->jointPos(ec);
   std::array<double[2], DoF> soft_limits {};
   // 读取当前软限位设置
   robot->getSoftLimit(soft_limits, ec);

   std::array<double, DoF> jog_steps {};
   bool outofRange = false;
   // 将超出限位的轴Jog到软限位内±0.08rad (约5度)
   double margin = 0.08;
   for(unsigned i = 0; i < DoF; ++i) {
     if(curr_joint[i] > soft_limits[i][1]) {
       jog_steps[i] = soft_limits[i][1] - curr_joint[i] - margin;
       outofRange = true;
     }
     if(curr_joint[i] < soft_limits[i][0]) {
       jog_steps[i] = soft_limits[i][0] - curr_joint[i] + margin;
       outofRange = true;
     }
   }
   if(!outofRange) {
     print(std::cout, "当前轴角度处于软限位内，无需恢复");
     return;
   }

   // 下电后, 关闭软限位
   robot->setPowerState(false, ec);
   robot->setOperateMode(OperateMode::manual, ec);
   robot->setSoftLimit(false, ec);
   robot->setPowerState(true, ec);

   // 依次Jog各轴
   double rate = 0.2; // Jog速率
   for(unsigned i = 0; i < DoF; ++i) {
     if(jog_steps[i] != 0) {
       robot->startJog(JogOpt::Space::jointSpace, rate, Utils::radToDeg(abs(jog_steps[i])), i,
                       jog_steps[i] > 0, ec);
       bool running = true;
       while (running) {
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
         auto st = robot->operationState(ec);
         print(std::cout, st);
         if(st == OperationState::jog){
           running = false;
         }
       }
     }
   }
   robot->stop(ec);
   robot->setPowerState(false, ec);
   // 重新打开软限位
   robot->setSoftLimit(true, ec);
 }
}

/**
 * @brief 示例 - 标定工具/工件坐标系
 */
template<WorkType Wt, unsigned short DoF>
void example_calibrateFrame(Robot_T<Wt, DoF> *robot) {
  int point_count = 4;
  Workflow::CalibrateFrame calibrate_frame(*robot, FrameType::tool, point_count, true);
  for(int i = 0; i < point_count; i++) {
    print(std::cout, "将机器人Jog到标定点，按回车确认");
    while(getchar() != '\n');
    calibrate_frame.setPoint(i);
  }
  error_code ec;
  FrameCalibrationResult calibrate_result = calibrate_frame.confirm(ec);
  if(ec) {
    print(std::cerr, "标定失败:", ec);
  } else {
    print(std::cout, "标定成功，结果 -", calibrate_result.frame, "\n偏差:", calibrate_result.errors);
  }
}

/**
 * @brief 示例 - 计算正逆解
 */
template <WorkType wt, unsigned short dof>
void example_coordinateCalculation(Robot_T<wt, dof> *robot){
  error_code ec;
  auto tcp_xyzabc = robot->posture(CoordinateType::endInRef, ec);
  // *** 计算逆解 & 正解 ***
  //设置工具坐标
  Toolset toolset1;
  toolset1 = robot->toolset(ec);
  print(os, "从控制器读取的工具工件坐标系:", toolset1);
  auto model = robot->model();

  // 当前设置的工具工件坐标系下计算逆解
  model.calcIk(tcp_xyzabc, ec);
  // toolset1下计算逆解
  auto ik = model.calcIk(tcp_xyzabc, toolset1, ec);
  // 当前设置的工具工件坐标系下计算正解
  model.calcFk(ik, ec);
  // toolset1下计算正解
  auto fk_ret = model.calcFk(ik, toolset1, ec);
  print(os, "目前的运动学逆解：", ik);
  print(os, "运动学正解：", fk_ret);

  //*** 坐标系转换： 末端相对于外部参考 & 法兰相对于基坐标 ***
  //查询基坐标设置
  auto base_in_world = robot->baseFrame(ec);
  auto flan_in_base =Utils::EndInRefToFlanInBase(base_in_world, toolset1, tcp_xyzabc);
  auto flan_pos = robot->posture(CoordinateType::flangeInBase, ec);
  auto end_in_ref = Utils::FlanInBaseToEndInRef(base_in_world, toolset1, flan_pos);

  print(os, "输入末端相对外部参考坐标系位姿", tcp_xyzabc);
  print(os, "计算得到的末端相对外部参考坐标系位姿", end_in_ref);
  print(os, "输入的法兰相对基坐标系位姿", flan_pos);
  print(os, "计算得到法兰相对基坐标系位姿", flan_in_base);

  // 计算所有逆解示例
  // 对当前的法兰相对基坐标系进行偏移，计算偏移后位姿的所有逆解
  auto cart_pos = robot->cartPosture(CoordinateType::flangeInBase, ec);
  cart_pos.trans[1] += 0.05;
  cart_pos.rpy[0] += Utils::degToRad(20);
  std::vector<std::vector<int>> calc_confs;
  auto ik_solutions = model.calcAllIkSolutions(cart_pos, calc_confs, ec);
  if(ec) {
    print(os, "计算逆解失败:", ec);
  } else {
    print(os, "计算得到的逆解数量", ik_solutions.size());
    for(size_t i = 0; i < ik_solutions.size(); ++i) {
      print(os, " ->", ik_solutions[i], "| conf:", calc_confs[i]);
    }
  }
}

/**
 * @brief 示例 - 基础的信息查询
 */
template <WorkType wt, unsigned short dof>
void example_basicOperation(Robot_T<wt, dof> *robot){
  error_code ec;
  // *** 查询信息 ***
  auto robotinfo = robot->robotInfo(ec);
  print(os, "控制器版本号:", robotinfo.version, "机型:", robotinfo.type);
  print(os, "xCore-SDK版本:", robot->sdkVersion());

  // *** 获取机器人当前位姿，轴角度，基坐标系等信息 ***
  auto joint_pos = robot->jointPos(ec); // 轴角度 [rad]
  auto joint_vel = robot->jointVel(ec); // 轴速度 [rad/s]
  auto joint_torque = robot->jointTorque(ec); // 轴力矩 [Nm]
  auto tcp_xyzabc = robot->posture(CoordinateType::endInRef, ec);
  auto flan_cart = robot->cartPosture(CoordinateType::flangeInBase, ec);
  robot->baseFrame(ec); // 基坐标系
  print(os, "末端相对外部参考坐标系位姿", tcp_xyzabc);
  print(os, "法兰相对基坐标系 -", flan_cart);

#if 0
  // 设置基坐标系。设置后需要重启工控机生效
  Frame base_frame_headstand = {0, 0, 0, M_PI, 0, 0}; // 倒装, A = 180°
  robot->setBaseFrame(base_frame_headstand, ec);
#endif

  // 查询最近5条错误级别控制器日志
  print(os, "查询最近5条错误级别控制器日志");
  auto controller_logs = robot->queryControllerLog(5, {LogInfo::error}, ec);
  for(const auto &log: controller_logs) {
    print(os, log.content);
  }

  // 查询第10-15条所有级别控制器日志
  print(os, "查询第10-15条所有级别控制器日志");
  controller_logs = robot->queryControllerLog(5, {LogInfo::error}, ec, 10);
  for(const auto &log: controller_logs) {
    print(os, log.content);
  }
}

/**
 * @brief 示例 - 打开关闭拖动
 */
void example_drag(BaseCobot *robot) {
  error_code ec;
  robot->setOperateMode(rokae::OperateMode::manual, ec);
  robot->setPowerState(false, ec); // 打开拖动之前，需要机械臂处于手动模式下电状态
  // 笛卡尔空间，自由拖动
  robot->enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
  print(os, "打开拖动", ec, "按回车继续");
  std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
  while(getchar() != '\n');
  robot->disableDrag(ec);
  std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
}

/**
 * @brief 示例 - Jog机器人
 * @param robot
 */
void example_jog(BaseRobot *robot) {
  error_code ec;
  robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  robot->setOperateMode(rokae::OperateMode::manual, ec); // 手动模式下jog
  print(os, "准备Jog机器人, 需手动模式上电, 请确认已上电后按回车键");
  // 对于有外接使能开关的情况，需要按住开关手动上电
  robot->setPowerState(true, ec);

  print(os, "-- 开始Jog机器人-- \n世界坐标系下, 沿Z+方向运动50mm, 速率50%，等待机器人停止运动后按回车继续");
  robot->startJog(JogOpt::world, 0.5, 50, 2, true, ec);
  while(getchar() != '\n');
  print(os, "轴空间，6轴负向连续转动，速率5%，按回车停止Jog");
  robot->startJog(JogOpt::jointSpace, 0.05, 5000, 5, false, ec);
  while(getchar() != '\n'); // 按回车停止
  robot->stop(ec); // jog结束必须调用stop()停止
}

/**
 * @brief 示例 - 奇异点规避Jog，适用于xMateSR、xMateCR系列机型
 */
void example_avoidSingularityJog(xMateRobot &robot) {
  error_code ec;
  robot.setOperateMode(rokae::OperateMode::manual, ec); // 手动模式下jog
  print(os, "准备Jog机器人, 需手动模式上电, 请确认已上电后按回车键");
  // 对于有外接使能开关的情况，需要按住开关手动上电
  robot.setPowerState(true, ec);
  while(getchar() != '\n');

  print(os, "-- 开始Jog机器人-- \n奇异规避模式, 沿Y+方向运动50mm, 速率20%，等待机器人停止运动后按回车继续");
  robot.startJog(JogOpt::singularityAvoidMode, 0.2, 50, 1, true, ec);
  while(getchar() != '\n'); // 按回车停止
  robot.stop(ec); // jog结束必须调用stop()停止
}

/**
 * @brief 示例 - NTP设置。注意: NTP功能非标配，需要对机器人进行额外升级
 */
void example_ConfigNtp(BaseRobot *robot) {
  error_code ec;
  // 设置NTP服务端地址
  robot->configNtp("192.168.0.170", ec);
  if(ec) {
    print(os, "设置NTP服务器地址失败:", ec);
  }
  // 同步一次时间
  robot->syncTimeWithServer(ec);
  if(ec) {
    print(os, "同步时间失败:", ec);
  }
}

/**
 * @brief 示例 - 打开和关闭碰撞检测
 */
template <unsigned short dof>
void example_setCollisionDetection(Cobot<dof> *robot) {
  error_code ec;
  // 设置各轴灵敏度，范围0.01 ~ 2.0，相当于RobotAssist上设置的1% ~ 200%
  // 触发行为：安全停止；回退距离0.01m
  robot->enableCollisionDetection({1.0, 1.0, 0.01, 2.0, 1.0, 1.0, 1.0}, StopLevel::stop1, 0.01, ec);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // 关闭碰撞检测
  robot->disableCollisionDetection(ec);
}

/**
 * @brief 急停复位
 */
void example_emergencyStopReset(BaseRobot *robot) {
  error_code ec;
  print(os, "急停复位");
  robot->recoverState(1, ec);
  if (ec) {
    print(os, "复位失败:", ec);
  } else {
    print(os, "复位成功");
  }
}

/**
 * @brief 是否使能平行基座模式（协作5轴）
 */
void example_CompletePostureLerp(xMateCr5Robot* robot) {
    error_code ec;
    print(os, "使能平行基座模式");
    robot->enableCompletePostureLerp(true, ec); // true: 开启，false: 失败
    if (ec) {
        print(os, "开启平行基座模式失败:", ec);
    }
    else {
        print(os, "开启平行基座模式成功");
    }
}

/**
 * @brief 示例 - 设置示教器模式
 */
void example_setTpMode(BaseRobot *robot) {
  error_code ec;
  // 不带示教器使用
  robot->setTeachPendantMode(false, ec);
  if(ec) {
    print(os, "设置失败:", ec);
    return;
  }
  print(os, "设置不连接示教器成功");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // 不连接示教器时，可以手动模式下上电
  robot->setOperateMode(rokae::OperateMode::manual, ec);
  robot->setPowerState(true, ec); // could be powered on without teach pendant
}

/**
 * @brief 重启/关闭工控机
 */
void example_reboot(BaseRobot* robot) {
  error_code ec;
  robot->rebootSystem(ec);
  print(os, "重启工控机");
  if (ec) print(os, "重启失败:", ec);
  else print(os, "重启成功");

  // 关闭工控机
  robot->shutdownSystem(ec);
  if (ec) print(os, "关机失败:", ec);
  else print(os, "关机成功");
}

/**
 * @brief 示例 - 设置连接断开回调函数
 */
void example_setConnectionHandler(BaseRobot *robot) {
  auto handler = [](bool connected){
    print(os, "Detect", connected ? "connection" : "disconnection");
  };
  robot->setConnectionHandler(handler);
}

/**
 * @brief main program
 */
int main() {
  try {
    // *** 1. 连接机器人 ***
    std::string ip = "192.168.0.160";
    xMateRobot robot(ip);  // 此处连接的是协作6轴机型

    // 其它机型
//    xMateErProRobot robot; // 协作7轴机型
//    StandardRobot robot; // 连接工业6轴机型
//    PCB4Robot robot; // 连接PCB4轴机型
//    PCB3Robot robot; // 连接PCB3轴机型
//    xMateCr5Robot; // 协作5轴机型

    example_basicOperation(&robot);

  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }

  return 0;
}