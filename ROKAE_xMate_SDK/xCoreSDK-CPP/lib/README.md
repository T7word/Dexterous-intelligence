# 预编译库 / Prebuilt Libraries

本仓库**不包含**预编译库文件。库文件通过 GitHub Releases 按平台分发，请打开与 SDK 版本对应的 Release 页面下载。

This repository does **not** include prebuilt library files. Libraries are distributed per platform via GitHub Releases — use the Release page matching your SDK version.

**Release 链接 / Release URL（版本号见根目录 `CMakeLists.txt`）：**

`https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v{VERSION}`

例如当前 v0.7.1：[Release v0.7.1](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)

## 获取步骤 / How to Obtain

1. 克隆本仓库 / Clone this repository
2. 打开与 SDK 版本一致的 [Release 页面](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)（链接规则见上文；运行 `cmake` 时若缺库也会打印对应版本的直达链接）
3. 下载对应平台的库文件包 / Download the package for your platform
4. 在**仓库根目录**解压，使文件落入 `lib/` 目录 / Extract at the **repository root** so files land under `lib/`

### Release 包命名 / Package Names

| 包名 / Package | 适用场景 / Use case |
|---|---|
| `xCoreSDK-{version}-win64-release.zip` | Windows 64-bit Release 编译 |
| `xCoreSDK-{version}-win64-debug.zip` | Windows 64-bit Debug 编译（含 pdb） |
| `xCoreSDK-{version}-win32-release.zip` | Windows 32-bit Release 编译 |
| `xCoreSDK-{version}-win32-debug.zip` | Windows 32-bit Debug 编译（含 pdb） |
| `xCoreSDK-{version}-linux-x86_64.tar.gz` | Linux x86_64 |
| `xCoreSDK-{version}-linux-aarch64.tar.gz` | Linux aarch64 |

> 一般 Release 编译只需下载 `-release` 包；Debug 调试时再额外下载 `-debug` 包。
>
> For Release builds, download the `-release` package only. Download `-debug` when you need Debug symbols.

## 解压后目录结构 / Expected Layout

### Windows

```
lib/Windows/Release/64bit/
  xCoreSDK.dll
  xCoreSDK.lib
  xCoreSDK_static.lib
  xCoreSDK_Upgrade.dll
  xCoreSDK_Upgrade.lib
  xCoreSDK_Upgrade_static.lib
  xMateModel.lib              # 64-bit Release only

lib/Windows/Debug/64bit/
  xCoreSDK.dll
  xCoreSDK.lib
  xCoreSDK_static.lib
  xCoreSDK.pdb
  xCoreSDK_Upgrade.dll
  xCoreSDK_Upgrade.lib
  xCoreSDK_Upgrade_static.lib
  xMateModeld.lib             # 64-bit Debug only
```

32-bit 路径将 `64bit` 换为 `32bit`，且无 xMateModel 库。

### Linux

```
lib/Linux/x86_64/
  libxCoreSDK.so.{version}
  libxCoreSDK.a
  libxMateModel.a
  libxCoreSDK_Upgrade.so.0.1
  libxCoreSDK_Upgrade.a

lib/Linux/aarch64/
  libxCoreSDK.so.{version}
  libxCoreSDK.a
  libxCoreSDK_Upgrade.so.0.1
  libxCoreSDK_Upgrade.a
```

## 验证 / Verify

解压完成后，在仓库根目录执行 CMake 配置。若库文件缺失，CMake 会给出警告并提示下载地址。

After extraction, run CMake configure at the repository root. CMake will warn if libraries are missing.

## 发版说明（维护者） / Release Notes (Maintainers)

库文件打包与发布流程见 [scripts/RELEASE.md](../scripts/RELEASE.md)。

See [scripts/RELEASE.md](../scripts/RELEASE.md) for packaging and publishing instructions.
