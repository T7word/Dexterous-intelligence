# 珞石 xMate ER7Pro-M 与 DexHand021 S 控制项目

本项目的主程序是 PySide6 可视化控制界面，用电脑网线连接珞石机器人控制器，再通过机器人末端 485 接口控制安装在末端的 DexHand021 S 三指灵巧手。电脑直连 USB 转 485 的模式也保留，用于单独调试灵巧手。

## 主程序

```text
ROKAE_xMate_SDK/xCoreSDK-Python/gui/robot_control_gui.py
```

主界面包括：

- xMate ER7Pro-M 机器人 IP 连接、状态读取和关节/笛卡尔控制
- DexHand021 S 的 P1、P2、P3、R 四轴控制
- 角度、Hall、MIT 力矩模式和状态监测
- 压力/触觉、诊断保护、寄存器和动作序列
- 拖动录制与回放、日志和可调整窗口

机器人末端模式不打开电脑 COM 口，也不创建或运行 RL 工程；末端 485 请求通过珞石 xCore SDK 发送。DexHand021 S 的 RTU 帧、CRC、设备 ID 和电机控制模式按随项目资料中的 SDK/说明书执行。

## 运行

Windows 10/11、64 位 Python 3.8–3.12 均可用。不要上传或依赖本地 `venv`；该目录中的环境是本机运行产物。先安装 PySide6 和 pyserial，并从珞石 xCore SDK 的官方发布包取得与 Python 版本对应的 Windows 扩展和 `xCoreSDK.dll`，放入 `ROKAE_xMate_SDK/xCoreSDK-Python/Release/windows/`。仓库只提交源码和资料，不提交 `.pyd`、`.dll`、`.so` 等预编译二进制。

```powershell
Set-Location "C:\Users\sxy18\Desktop\记录留痕\20260810三指灵巧手"
python -m pip install PySide6 pyserial
python ".\ROKAE_xMate_SDK\xCoreSDK-Python\gui\robot_control_gui.py"
```

使用机器人末端 485 前，先在珞石控制器现场确认末端工具 RS485 接线、供电和通信参数；程序不会替用户修改控制器配置。连接机器人后，在界面选择“机器人末端 485（珞石控制器透传）”。

## 目录

详细分类见 [`00_项目索引/项目目录说明.md`](00_项目索引/项目目录说明.md)：

- `ROKAE_xMate_SDK`：珞石 xCore SDK 和主控制程序
- `02_灵巧手SDK`：DexHand/DexRobot SDK 源码
- `03_协议与设备资料`：末端透传、DexHand 和珞石说明资料
- `04_实验与采集`：D435i、标定及实验程序
- `90_本地运行产物_不上传`：日志、缓存和临时输出
- `99_本地归档_不上传`：压缩归档包

## 本地验证

- Python 语法解析：281 个文件，0 个错误
- PySide6 GUI 离屏启动：通过
- xCore SDK Windows 扩展导入：通过
- ROKAE 离线模拟示例：8/8 通过
- DexHand 协议测试：通过（20/20）
- 接口测试：Windows 下 36/38 通过；另外 2 项是 Linux CAN 动态库测试，因 Windows 无法加载 `libusbcanfd.so` 而跳过/失败，不影响末端 485 功能

真实机器人和灵巧手动作仍需在现场接线、供电和安全工作区确认后由操作者手动测试。
