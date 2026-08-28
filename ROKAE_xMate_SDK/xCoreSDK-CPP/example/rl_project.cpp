/**
 * @file rl_project.cpp
 * @brief 加载和运行RL工程
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#define _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING

#include <iostream>
#include <unordered_map>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include <string>

#ifdef _WIN32
#include <windows.h>
 #endif

using namespace std;
using namespace rokae;

namespace {
 rokae::xMateRobot robot;
}

void printHelp();
static const std::unordered_map<std::string, char> ConsoleInput = {
  {"on", '0'}, {"off", 'x'}, {"quit", 'q'},
  {"info", 'i'}, {"load", 'l'}, {"main", 'm'},
  {"start", 's'}, {"pause", 'p'}, {"tool", 't'},
  {"wobj", 'w'}, {"opt", 'o'}, {"help", 'h'}
}; ///< command -> char

/**
 * @brief 接收RL程序执行行号
 */
void rlExecutionCb(const EventInfo &info) {
  using namespace EventInfoKey::RlExecution;
  std::string task_name = std::any_cast<std::string>(info.at(TaskName));
  std::string lookahead_file = std::any_cast<std::string>(info.at(LookaheadFile));
  int lookahead_line = std::any_cast<int>(info.at(LookaheadLine));
  std::string execute_file = std::any_cast<std::string>(info.at(ExecuteFile));
  int execute_line = std::any_cast<int>(info.at(ExecuteLine));
  std::cout << "[RL执行] Task Name: " << task_name << " , Lookahead: " << lookahead_file << ", " <<
  lookahead_line << ", Executing: " << execute_file << ", " << execute_line << std::endl;
}

std::string UTF8ToGBK(const std::string& utf8Str)
{
#if defined(_WIN32) || defined(_WIN64)
  // 第一步：UTF-8转宽字符
  int wcharSize = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, nullptr, 0);
  if (wcharSize == 0) {
    return "";
  }

  std::vector<wchar_t> wcharBuffer(wcharSize);
  MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, wcharBuffer.data(), wcharSize);

  // 第二步：宽字符转GBK
  int gbkSize = WideCharToMultiByte(CP_ACP, 0, wcharBuffer.data(), -1, nullptr, 0, nullptr, nullptr);
  if (gbkSize == 0) {
    return "";
  }

  std::vector<char> gbkBuffer(gbkSize);
  WideCharToMultiByte(CP_ACP, 0, wcharBuffer.data(), -1, gbkBuffer.data(), gbkSize, nullptr, nullptr);

  return std::string(gbkBuffer.data());
#else
  return utf8Str; // here we assume Linux use UTF-8 encoding as default
#endif
}

/**
 * @brief 控制器日志上报
 */
void logReport(const EventInfo& info) {
  using namespace EventInfoKey::LogReporter;
  int log_ecode = std::any_cast<int>(info.at(Ecode));
  auto log_edetail = std::any_cast<std::string>(info.at(Edetail));
  if (!log_edetail.empty())
  {
    log_edetail = UTF8ToGBK(log_edetail); // 转换编码
  }
  std::cout << "User Log Report - error code: " << log_ecode <<
  (log_edetail.empty() ? "" : ", detail: " + log_edetail) << std::endl;

}

/*
 * @brief 示例 - 导入和删除RL工程
 */
void transferRLProject() {
  error_code ec;

  // 导入RL .zip格式工程
  auto project_name = robot.importProject("ExampleRLProject.zip", true, ec);
  if (!ec) {
  	print(std::cout, project_name);
  }
  else {
  	std::cerr << "导入RL工程发生错误：" << ec.message() << std::endl;
  }

  // 删除工程
  robot.removeProject("test1", ec);
}

/**
 * @brief 示例 - 导入和删除工程文件
 */
void importProjectFile() {
  error_code ec;
  // 将本地的test.mod导入工程 MyRlProject 任务task0下面
  auto ret = robot.importFile(R"(C:\Users\rokae\Desktop\test.mod)", "project/MyRlProject/task0", true, ec);

  if(ec){
    std::cerr << "导入失败" << ": " << ec << std::endl;
  }
  std::cout << "导入后文件名" << ret << std::endl;

  // 将本地的test.mod导入工程 MyRlProject 任务task0下面, 并命名为imported.mod
  robot.importFile(R"(C:\Users\rokae\Desktop\test.mod)", "project/MyRlProject/task0/imported.mod", true, ec);

  // 将工程的工具配置文件导入工程MyRlProject
  robot.importFile(R"(C:\Users\rokae\Desktop\MyRlProject\tool.json)", "project/MyRlProject", true, ec);
  robot.importFile(R"(C:\Users\rokae\Desktop\MyRlProject\_build\tools.sys)", "project/MyRlProject", true, ec);

  // 删除MyRlProject task0下面imported.mod
  robot.removeFiles({"project/MyRlProject/task0/imported.mod"}, ec);
  // 删除MyRlProject task1
  robot.removeFiles({"project/MyRlProject/task1"}, ec);
}

/**
 * @brief 示例 - 设置工具工件的位姿和负载等信息
 */
void setProjectToolWobj() {
  error_code ec;
  // 设置全局工具工件 g_tool_0
  // 手持，X:0, Y:45mm, Z:0, A:0, B:90°, C:0。负载1千克，质心 X:0, Y:20mm, Z:0
  WorkToolInfo g_tool_0("g_tool_0", true, {0, 0.045, 0, 0, M_PI / 2, 0}, {1, { 0, 0.02, 0 }, {}});
  g_tool_0.alias = "tool for job1"; // 工具的附加描述
  robot.setToolInfo(g_tool_0, ec);

  // 设置/创建RL工程下工具 tool_1。需要先加载好一个工程
  WorkToolInfo tool_1("tool_1", true, {0.1, 0.1, 0, 0, M_PI, 0}, {1, { 0.05, 0.05,0 }, {}});
  robot.setToolInfo(tool_1, ec);

  // 设置全局工件g_wobj_0, 外部工件
  WorkToolInfo g_wobj_0("g_wobj_0", false, {0.1, 0.1, 0, 0, M_PI, 0}, {0, {}, {}});
  robot.setWobjInfo(g_wobj_0, ec);
}

/**
 * @brief main program
 */
int main() {
  try {
    std::string ip = "192.168.0.160";
    robot.connectToRobot(ip); // ****   xMate 6-axis
  } catch (const rokae::Exception &e) {
    std::cout << e.what();
    return 0;
  }

  error_code ec;
  robot.setMotionControlMode(MotionControlMode::NrtRLTask,ec);
  // 接收RL执行的任务名和行号
  robot.setEventWatcher(rokae::Event::rlExecution, rlExecutionCb, ec);
  // 控制器日志上报
  robot.setEventWatcher(rokae::Event::logReporter, logReport, ec);

  robot.setOperateMode(OperateMode::automatic, ec);
  printHelp();
  char cmd = ' ';
  while(cmd != 'q') {
    std::string str;
    getline(cin, str);
    if(ConsoleInput.count(str)){
      cmd = ConsoleInput.at(str);
    } else {
      cmd = ' '; }

    switch(cmd) {
      case '0':
        robot.setPowerState(true, ec); cout << "* 机器人上电\n";
        if(ec) break; continue;
      case 'x':
        robot.setPowerState(false, ec); cout << "* 机器人下电\n";
        if(ec) break; continue;
      case 'i': {
        cout << "* 查询工程信息:\n";
        auto infos = robot.projectsInfo(ec);
        if(infos.empty()) { cout << "无工程\n"; }
        else {
          for(auto &info : infos) {
            cout << "名称: " << info.name << " 任务: ";
            for(auto &t: info.taskList) {
              cout << t << " ";}
            cout << endl; }
        }
        if(ec) break; } continue;
      case 'l':{
        cout << "* 加载工程, 请输入加载工程名称: ";
        std::string name, line, task;
        vector<string> tasks;
        getline(cin, name);
        cout << "请输入要运行的任务,空格分割: ";
        getline(cin, line);
        istringstream iss(line);
        while (iss >> task)
          tasks.push_back(task);
        robot.loadProject(name, tasks, ec);
        if(ec) break; } continue;
      case 'm':
        robot.ppToMain(ec);
        cout << "* 程序指针指向main\n";
        if(ec) break; continue;
      case 's':
        robot.runProject(ec); cout << "* 开始运行工程\n";
        if(ec) break; continue;
      case 'p':
        robot.pauseProject(ec); cout << "* 暂停运行\n";
        if(ec) break; continue;
      case 't': {
        cout << "* 查询工具信息\n";
        auto tools = robot.toolsInfo(ec);
        if(tools.empty()) cout << "无工具\n";
        else {
          for(auto &tool : tools) {
            cout << "工具: " << tool.name << ", 质量: " << tool.load.mass << endl;
          } }
        if(ec) break; } continue;
      case 'w': {
        cout << "* 查询工件信息\n";
        auto wobjs = robot.wobjsInfo(ec);
        if(wobjs.empty()) cout << "无工件\n";
        else {
          for(auto &wobj:wobjs) {
            cout << "工件: " << wobj.name << ", 是否手持: " << boolalpha << wobj.robotHeld << endl;}
        }
        if(ec) break; } continue;
      case 'o':{
        cout << "* 设置运行参数, 请依次输入运行速率和是否循环([0]单次/[1]循环), 空格分隔: ";
        double rate; bool isLoop; string line;
        getline(cin, line);
        istringstream iss(line);
        iss >> rate >> isLoop;
        robot.setProjectRunningOpt(rate, isLoop, ec);
        if(ec) break;} continue;
      case 'h':
        printHelp(); continue;
      case 'q':
        std::cout << " --- Quit --- \n"; continue;
      default:
        std::cerr << "无效输入\n"; continue;
    }
    cerr << "! 错误信息: " << ec.message() << endl;
  }
  return 0;
}

/**
 * @brief print help
 */
void printHelp() {
  cout << " --- 运行RL工程示例 --- \n\n"
  << "     命令   \n"
  << "on    机器人上电\n"
  << "off   机器人下电\n"
  << "info  查询工程列表\n"
  << "load  加载工程\n"
  << "main  程序指针指向main\n"
  << "start 开始运行\n"
  << "pause 暂停运行\n"
  << "opt   设置运行参数\n"
  << "tool  查询工具信息\n"
  << "wobj  查询工件信息\n"
  << "help  查看示例程序所有命令\n"
  << "quit  结束\n";
}