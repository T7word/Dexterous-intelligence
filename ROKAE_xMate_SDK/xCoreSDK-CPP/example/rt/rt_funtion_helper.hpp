//
// Created by arcia on 2025/7/9.
//

#ifndef XCORESDK_EXAMPLE_RELEASE_RT_RT_FUNTION_HELPER_HPP_
#define XCORESDK_EXAMPLE_RELEASE_RT_RT_FUNTION_HELPER_HPP_

#include "rokae/robot.h"
#include "rokae/utility.h"

namespace rokae::helper {
 /**
  * @brief 获取实时状态数据 - 实时模式下的笛卡尔位姿
  * @param robot instance
  * @return 笛卡尔位姿 [行优先]
  */
 template<WorkType Wt, unsigned short DoF>
 std::array<double, 16> getCurrentPose_matrix(rokae::Robot_T<Wt, DoF> &robot) {
   std::array<double, 16> pose{};
   try {
     // 接收状态数据的队列不会自动覆盖旧数据，可以通过循环读取的方法清除旧数据
     while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
     if (robot.getStateData(RtSupportedFields::tcpPose_m, pose) == 0) {
       return pose;
     }
     throw std::runtime_error("getCurrentPose_matrix: getStateData failed");
   } catch (const std::exception &e) {
     std::cerr << e.what();
   }
   error_code ec;
   // 没有接收实时状态数据，用非实时接口获取
   Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), pose);
   return pose;
 }

 /**
  * @brief 获取实时状态数据 - 实时模式下的关节角度
  * @param robot instance
  * @return 关节角度 [弧度]
  */
 template<WorkType Wt, unsigned short DoF>
 std::array<double, DoF> getCurrentJointPos(rokae::Robot_T<Wt, DoF> &robot) {
   std::array<double, DoF> joint{};
   try {
     // 接收状态数据的队列不会自动覆盖旧数据，可以通过循环读取的方法清除旧数据
     while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()));

     if (robot.getStateData(RtSupportedFields::jointPos_m, joint) == 0) {
       return joint;
     }
     throw std::runtime_error("getCurrentJointPos: getStateData failed");
   } catch (const std::exception &e) {
     std::cerr << e.what();
   }
   error_code ec;
   // 没有接收实时状态数据，用非实时接口获取
   return robot.jointPos(ec);
 }
}
#endif //XCORESDK_EXAMPLE_RELEASE_RT_RT_FUNTION_HELPER_HPP_
