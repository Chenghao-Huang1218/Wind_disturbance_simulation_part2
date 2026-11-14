# 安装与运行环境

## 当前支持范围

已验证 Ubuntu 24.04 x86-64、Python 3.12.13、MuJoCo 3.6.0、NumPy 2.4.4、Matplotlib 3.10.9。
其它系统尚未验证。仓库内 PX4 为 Linux x86-64 ELF 程序，不能直接在 Windows/macOS/ARM 上运行。
推荐从源码仓库运行，保留 `runtime/px4`；当前不是可移植的独立 PyPI 应用。

## Python 环境

```bash
conda create -n env_uav python=3.12 -y
conda activate env_uav
python -m pip install -r requirements.txt
python -c "import tkinter, mujoco, numpy, matplotlib; print('Python imports OK')"
```

Conda 的 Tkinter 应使用与该环境匹配的 Tk；系统 Python 可通过 `sudo apt install python3-tk` 安装。
若采用 venv，使用 `python3.12 -m venv .venv` 并激活后安装同一 requirements 文件即可。
Matplotlib 的 TkAgg 需要桌面显示和 Tk，MuJoCo viewer 需要可用的 OpenGL。

## PX4 动态库

仓库包含经过本项目适配的 `runtime/px4/bin/px4` 及 `etc` 启动资源。
**不需要另开 PX4 终端，不需要在正常安装流程中重新编译 PX4。**

检查可执行权限和动态库：

```bash
test -x runtime/px4/bin/px4 || chmod u+x runtime/px4/bin/px4
ldd runtime/px4/bin/px4
```

输出中必须没有 `not found`。当前验证主机提供以下关键 SONAME：

| 库 | 验证主机上的提供者 |
| --- | --- |
| libgz-transport13.so.13 | ros-jazzy-gz-transport-vendor |
| libgz-msgs10.so.10 | ros-jazzy-gz-msgs-vendor |
| libgz-utils2.so.2 | ros-jazzy-gz-utils-vendor |
| libprotobuf.so.32 | Ubuntu libprotobuf32t64 |
| libzmq.so.5 | Ubuntu libzmq5 |
| libstdc++.so.6、libc.so.6 及其它传递依赖 | Ubuntu 24.04 系统库 |

在已按 [ROS Jazzy 官方安装文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
配置好 ROS apt 软件源的 Ubuntu 24.04 上，可安装对应 vendor 包：

```bash
sudo apt update
sudo apt install ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor ros-jazzy-gz-utils-vendor libprotobuf32t64 libzmq5 libgl1
source /opt/ros/jazzy/setup.bash
ldd runtime/px4/bin/px4
```

具体可用包由系统及软件源决定，以 `ldd` 的实际结果为准。
本项目不启动 Gazebo 仿真，也不要求安装完整 ROS 桌面；这些包用于满足现有 PX4 的链接依赖。
不要用不匹配版本的 `.so` 强行改名或软链接。

## 验证和启动

```bash
# 无 PX4、无窗口、无网络的真实模型加载
python scripts/run_simulation.py --check --mode both

# 打开 MuJoCo、面板并启动本启动器拥有的四个 PX4
python scripts/run_simulation.py --mode both --instance-base 4
```

模型检查成功不代表动态库或实际 PX4 闭环成功。`ldd` 成功也不代替飞行闭环验证。
没有桌面的服务器使用 `--headless --auto`，但仍需 PX4 库和本地 TCP/UDP 通信。

```bash
python scripts/run_simulation.py --headless --auto --mode both --duration 3 --timeout 300 --instance-base 4
```

## 常见问题

- **PX4 cannot open shared object**：先 `ldd` 确认缺少的准确 SONAME；补齐匹配的系统包并检查 ROS 环境。
- **没有 Tk 或图形窗口**：确认在已激活环境中 `import tkinter` 成功，且当前会话具有桌面显示权限；无窗口模式使用 `--headless --auto`。
- **端口被占用**：退出自己旧的仿真，或换 `--instance-base 0` 等空闲实例组（四机范围 0–6，双机 0–8）。启动器不会终止其它程序。
- **曲面没有出现**：点击 Start wind field，并等待源机解锁、升空且产生转子推力；地面停转时不会出现。
- **移动被拒绝**：方向控制仅用于已解锁的起飞/悬停飞机；检查任意两机目标水平间距至少 1.50 m，以及四机带载的水平绳长可达性；双机带载仍为 1.50–2.25 m。
- **运动迟缓**：带载指令本身限速 0.25 m/s；另检查 summary 的 realtime_factor。绘图或高负载会使仿真慢于现实时间。
- **自动流程超时**：timeout 是墙钟秒，可适当增加；不要把超时当作完整流程成功。
- **输入 Ah/Az/L 后没有变化**：点击 Apply；非法值保留上一组配置。风场图显示归一化强度，因此 Ah/Az 不改变曲面形状。

## 开发与离线测试

```bash
python -m pip install 'pytest>=8'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg python -m pytest -q
```

测试不启动 PX4，不替代实际 SITL。预览脚本还使用 Pillow（Matplotlib 的依赖）。
显示性能基准的基线和运行方式见 `docs/VALIDATION.md`；基准复用 tests 中的通信替身，不是飞行验收。

当前启动入口默认四机，2×2 方阵与四条 1.6 m 绳。使用 `--aircraft-count 2` 可运行原双机配置，默认绳长 1.2 m。
