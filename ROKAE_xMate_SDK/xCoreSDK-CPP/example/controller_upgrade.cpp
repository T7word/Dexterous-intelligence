/**
 * @file controller_upgrade.cpp
 * @brief 控制器升级, 备份导出示例
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <chrono>
#include <iomanip>
#if defined(_WIN32) || defined(_WIN64)
#include <filesystem>
#endif
#include "rokae/robot.h"
#include "rokae/upgrade.h"

using namespace rokae;

namespace {
 BaseUpgrade upgrader; ///< 升级程序实例
}

/**
 * @brief 输出格式化当前时间日期
 */
std::string getCurrentDateTime();

/**
 * @brief 示例 - 控制器固件版本升级/恢复控制器备份
 */
void upgradeController() {
  error_code ec;

  std::cout << "开始升级控制器固件" << std::endl;
  // 升级该路径下控制器版本
  upgrader.upgrade(R"(C:\Users\rokae\v3.1.2.rpa)", ec);
  if(ec) {
    std::cerr << "升级失败: " << ec.message() << std::endl;
  } else {
    std::cout << "升级成功" << std::endl;
  }
}

/**
 * @brief 示例 - 导出控制器备份
 */
void exportControllerBackup() {
  error_code ec;
  // 保存的文件名 export_[date_time].rpa
  std::string export_file_name = "export_" + getCurrentDateTime() + ".rpa";

#if defined(_WIN32) || defined(_WIN64)
  // 文件保存在当前运行目录下
  std::string file_save_path = (std::filesystem::current_path() / export_file_name).string();
#else
  // 文件保存在当前运行目录下
  std::string file_save_path = "./" + export_file_name;
 #endif

  std::cout << "开始导出控制器备份, 保存到 " << file_save_path << std::endl;

  // 导出控制器日志和RL工程文件
  upgrader.exportBackup(file_save_path, {
    BackupItem::controllerLog, BackupItem::rlProgram}, ec);
  if(ec) {
    std::cerr << "导出失败: " << ec.message() << std::endl;
  } else {
    std::cout << "导出成功" << std::endl;
  }
}

/**
 * @brief Main function
 */
int main() {

  std::string remote_ip = "192.168.0.160"; // 机器人地址

  // 连接到控制器的升级程序
  // 可以单独连接，不要求同时连接机器人控制器。不允许多个连接，不允许和示教器同时连接
  try {
    upgrader.connect(remote_ip);
  } catch (std::exception &e) {
    std::cout << "Failed to connect to UpdateManager: " << e.what() << std::endl;
    return -1;
  }

  // 运行升级示例
//  upgradeController();

  // 运行备份导出示例
  exportControllerBackup();

  return 0;
}

std::string getCurrentDateTime() {
  // 获取当前时间点
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);

  // 转换为本地时间
  std::tm tm_buf;
#ifdef _WIN32
  localtime_s(&tm_buf, &t); // Windows
#else
  localtime_r(&t, &tm_buf); // Linux / Unix
#endif

  // 格式化输出
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d_%H%M%S");
  return oss.str();
}