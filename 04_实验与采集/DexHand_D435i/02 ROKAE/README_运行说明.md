# 眼在手上位姿采集

本目录已包含运行所需的珞石 xCore Python SDK 和独立 Python 3.10 环境。

在本目录打开 PowerShell 后运行：

```powershell
.\run_pose_capture.ps1
```

如需指定本机网卡 IP：

```powershell
.\run_pose_capture.ps1 --local-ip "192.168.0.11"
```

按 `C` 或空格键保存当前法兰位姿；按 `Q` 或 `Esc` 退出。数据默认写入 `flange_pose_samples.csv`，仅含六列 `flangeInBase` 原始数据（m、rad）。

不要使用 `py eye_in_hand_pose_capture.py`：系统的 `py` 启动器当前会选中 Python 3.7，该版本与本目录的 xCore SDK 不兼容。
