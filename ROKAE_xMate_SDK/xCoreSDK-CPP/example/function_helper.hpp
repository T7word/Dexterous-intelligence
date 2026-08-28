//
// Created by arcia on 2025/7/9.
//

#ifndef XCORESDK_EXAMPLE_RELEASE_FUNCTION_HELPER_HPP_
#define XCORESDK_EXAMPLE_RELEASE_FUNCTION_HELPER_HPP_

#include <thread>
#include "rokae/robot.h"

namespace rokae::helper {

 /**
  * @brief 等待机器人空闲
  */
 void waitRobot(rokae::BaseRobot &robot) {
   using namespace rokae;
   while (true) {
     std::this_thread::sleep_for(std::chrono::milliseconds(100));
     error_code ec;
     auto st = robot.operationState(ec);
     if (st == OperationState::idle || st == OperationState::unknown) {
       return;
     }
   }
 }

}

#endif //XCORESDK_EXAMPLE_RELEASE_FUNCTION_HELPER_HPP_
