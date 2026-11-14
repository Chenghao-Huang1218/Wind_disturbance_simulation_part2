# MultiUAV-windInteraction-simulation

**多无人机风场互扰与协同吊运仿真平台 · MuJoCo + 四 PX4 SITL**

在同一个 MuJoCo 世界中默认运行四架搭载 **Airbot Play 机械臂的 X4-950 四旋翼**，
每架由独立 PX4 SITL 进程闭环控制。四架无人机可分别执行 XYZ 平移，
也可通过夹爪末端的四条弹力绳共同搬运一个物块，并显示各自的扰动强度曲面。

本项目用于多机交互、带载运动、控制接口和人工扰动对照的仿真实验。
当前互扰采用 **“人工设置的距离衰减加速度扰动模型”**，不是经过气动实验标定的真实尾流模型；
显示曲面不代表实际风速或物理势能。PX4 使用仿真真值反馈，不能据此声称已验证真实传感器或实机飞行。

## 四机演示

https://github.com/user-attachments/assets/9a2315fe-3065-4619-844e-3c66ddbbb3b2

[下载原始演示视频](demo/Demo%20video.mp4)。上方为同一段四机演示的 H.264 网页播放版，保留完整时长。

![四机 MuJoCo 场景与共同负载](demo/Mujoco%20model.png)

<details>
<summary>展开四机独立控制面板</summary>

![四机独立控制面板](demo/Contorl%20UI.png)

</details>

[四源曲面示意](docs/media/four-uav-fields/disturbance-surface-close.png) ·
[已验证运行的吊运姿态回放](docs/media/four-uav-payload-sitl.png)。前者为离线示意姿态，后者读取实际 SITL 日志；均不代表气动标定。

## 功能与用途

- **共享物理世界**：四套完整无人机/机械臂模型，公共地面、接触及负载动力学；每个物理步只推进一次 MuJoCo。
- **四个独立飞控**：同一个预编译 PX4 SITL 程序启动四个实例，身份、通信端口、参数与日志目录隔离。
- **四机独立控制**：同步起飞/降落，分别设置每架飞机的世界坐标 XYZ 位置目标。
- **协同吊运**：四机朝同一方向，通过夹爪末端的四条弹力绳连接物块顶面的四个独立吊点。
- **可调人工互扰**：关闭、仅水平、仅竖直、水平＋竖直四种模式，运行时修改幅值和衰减长度。
- **扰动可视化**：四张半透明强度曲面与等值线，分别跟随机体、位于机体水平面以下，水平显示半径 10 m。
- **自动流程与记录**：起飞、吊起、共同平移、保持、降落及退出；记录四机状态、四绳张力、互扰加速度和实时倍率。

机械臂保持初始姿态；绳索预先连接，当前不包含主动抓取、机械臂轨迹规划或真实流体求解。

## 运行依赖

当前验证平台为 **Ubuntu 24.04、Linux x86-64、Python 3.12**。Windows、macOS、ARM 和其它 Linux 发行版未验证。

| 类别 | 依赖 |
| --- | --- |
| Python 数值与仿真 | MuJoCo 3.6.0、NumPy 2.4.4 |
| 控制面板 | Matplotlib 3.10.9、Tk/Tkinter、可用的桌面与 OpenGL 驱动 |
| 通信 | 仓库内附 pymavlink 2.4.47，以及 lxml、pyserial |
| 飞控 | 仓库内附 Linux x86-64 PX4 SITL 运行包，不需要另行下载或编译 PX4 |
| PX4 系统动态库 | Gazebo Transport 13、Messages 10、Utils 2、Protobuf 32 及其传递依赖；当前通过 ROS Jazzy vendor 包提供 |
| 离线测试 | pytest ≥ 8 |

**Python 包安装不会同时安装 PX4 的系统动态库。** 完整安装步骤、动态库检查和常见问题见
[安装说明](docs/INSTALLATION.md)。仿真物理由 MuJoCo 计算，Gazebo vendor 包仅提供现有 PX4 二进制链接的库。

## 安装与启动

以下命令从仓库根目录运行，不依赖开发者电脑上的其它项目目录。

```bash
git clone https://github.com/Chenghao-Huang1218/MultiUAV-windInteraction-simulation.git
cd MultiUAV-windInteraction-simulation

conda create -n env_uav python=3.12 -y
conda activate env_uav
python -m pip install -r requirements.txt

# 必须没有 “not found”；详情见 docs/INSTALLATION.md
ldd runtime/px4/bin/px4

# 仅加载模型和配置：不启动 PX4、不打开窗口、不使用网络
python scripts/run_simulation.py --check --mode both

# 启动四个 PX4、MuJoCo 窗口和紧凑控制面板
python scripts/run_simulation.py --aircraft-count 4 --mode both --instance-base 4
```

不需要单独启动 PX4 终端任务。启动器只管理自己创建的四个子进程，端口被占用时会报错，不终止其它会话。
CLI 默认互扰模式是 `off`，上面的命令显式开启 `both`。

已有本机环境时：

```bash
conda activate env_uav
cd MultiUAV-windInteraction-simulation  # 替换为你的实际项目路径
python scripts/run_simulation.py --aircraft-count 4 --mode both --instance-base 4
```

## 控制面板使用方法

四机面板默认约 900×640（随 DPI 缩放），A/B 在上、C/D 在下，采用紧凑 2×2 布局。推荐操作顺序：

1. 等待四机状态均显示 **connected**。
2. 点击 **Takeoff all**；按钮显示 **Arming all...**，收到四机真实解锁确认后执行起飞。
3. 等待升高、绳索拉紧并吊起物块，再使用各自的 XYZ 方向键移动。
4. 点击 **Start wind field** 显示扰动曲面；再次点击 **Stop wind field** 关闭显示。
5. 完成后点击 **Land all**，等待四机落地并解除解锁；最后点击 **Exit**。

| 控件 | 功能 |
| --- | --- |
| A/B/C/D 的 X+/X− | 仅改变所选飞机的世界 X 位置目标；蓝色 |
| A/B/C/D 的 Y+/Y− | 仅改变所选飞机的世界 Y 位置目标；绿色 |
| A/B/C/D 的 Z+/Z− | 仅改变所选飞机的世界 Z 位置目标；橙色；+Z 向上 |
| Step：0.10 / 0.20 / 0.50 | 四机共用的单次位移步长，单位 m；默认 0.20 m |
| Takeoff all / Land all | 四机共同起飞 / 降落 |
| Emergency | 立即停止仿真转子并请求解除解锁，不执行受控降落 |
| Exit | 退出并清理本次启动器拥有的四个 PX4；Ctrl+C 同样走清理流程 |
| Start wind field / Stop wind field | 仅开关曲面显示，不改变互扰受力或吊绳力 |
| off / horizontal / vertical / both | 互扰模式草稿；点击 Apply 后生效 |
| Ah / Az / L 与 Apply | 一次校验并应用模式和三个参数；非法输入保留上一组有效设置 |
| Pos / Tgt、Distance、Cargo、Rope A/B/C/D | 当前位置/最终目标、最小水平机间距、物块中心高度、四条绳张力 |

点击方向键只改变所选飞机的目标，其余三架的目标不变；四机仍会通过绳索和物块产生物理耦合。
方向键只接受已经解锁且处于起飞/悬停阶段的飞机。任意两机目标水平间距至少为 **1.50 m**。四机带载时还检查
各夹爪目标到共同顶面吊点布局的水平跨度小于绳长，避免不可达目标；
无载任意两机目标水平间距不超过 **10 m**。拒绝时所有目标保持原样。
这只是目标设置约束，未经实机安全验证，也不保证任意高度差和动态姿态下的绳索稳定。
原双机模式保留带载水平间距 **1.50–2.25 m** 的范围。

带载位置目标按仿真时间平滑发送：速度上限 **0.25 m/s**、常规加速度渐变 **0.1 m/s²**。
因此按下按钮后飞机不会瞬移到 Tgt。运行速度还受主机负载和绘图开销影响，当前不保证实时倍率达到 1。

## 实现方法

### 共享世界与四 PX4

四套模型使用 `a/`、`b/`、`c/`、`d/` 命名空间，共享 MuJoCo 世界和 0.005 s 物理步长。
每步刷新同一时刻的状态，计算所有其它有效源的人工扰动及四条绳力，向各自 PX4 交换状态/执行器数据，最后只推进一次物理。
PX4 保留原生控制回路，状态估计使用现有的仿真真值反馈适配，EKF2 传感器链路被旁路。

四机初始按边长 2 m 的方阵排列，A=(-1,-1)、B=(-1,+1)、C=(+1,-1)、D=(+1,+1) m，
机头统一朝 +X，初始 root 高度 0.48 m。每机含机械臂质量约 8.0004 kg。
同一 PX4 可执行文件启动四份进程，各自独立控制一架无人机。

### 人工设置的距离衰减加速度扰动模型

对接收机 $i$ 和其它源机 $j$，采用水平距离 $d_{ij}$，从源机指向接收机的水平单位向量为 $\mathbf e_{j\to i}$：

$$
d_{ij}=\sqrt{(x_i-x_j)^2+(y_i-y_j)^2},
\qquad
 g_{ij}=\frac{1}{1+(d_{ij}/L)^2}
$$

$$
\mathbf e_{j\to i}=\frac{\mathbf p_{i,xy}-\mathbf p_{j,xy}}{d_{ij}}
$$

令 $\mathcal S_i$ 为除自身之外、满足飞行与推力条件的有效源机集合；两种扰动通道均开启时：

$$
\begin{aligned}
\mathbf a_{i,xy}&=\sum_{j\in\mathcal S_i} A_h g_{ij}\,\mathbf e_{j\to i},\\
a_{i,z}&=-\sum_{j\in\mathcal S_i} A_z g_{ij},\\
\mathbf F_i&=m_i\mathbf a_i.
\end{aligned}
$$

每架叠加其余三架有效源的贡献，不含自身项。原双机模式仍只有一个其它源。

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| Ah | 单源水平向外峰值加速度 | 0.5 m/s² |
| Az | 单源竖直向下峰值加速度 | 0.5 m/s² |
| L | 水平距离为 L 时，强度降为峰值一半 | 2 m |

模式关闭相应通道；`off` 清除旧互扰载荷。源机需处于飞行阶段、离地且有实际仿真转子推力，
接收机需满足飞行阶段及离地条件。不按源机数量平均，相同 Ah/Az 下总扰动可能大于双机。模型使用水平距离，不含高度差、姿态和转速相关的尾流解析项。

加速度按各子 body 质量分配为 $\mathbf F_b=m_b\mathbf a_i$，作用于各 body 质心，
总力为整机 $m_i\mathbf a_i$，关于实时整机质心的互扰合矩为零。吊绳连接点力矩另行计算。

可视化以源机为中心，水平半径 $r$ 内的曲面为：

$$
g(r)=\frac{1}{1+(r/L)^2}
$$

$$
z_{\mathrm{vis},i}(r)=z_i-0.05-1.35\bigl[1-g(r)\bigr],
\qquad 0\le r\le 10\,\mathrm{m}
$$

曲面最高点在机体原点下方 0.05 m，边缘渐隐，源停转后在 0.6 s 仿真时间内淡出。
半径 10 m 仅限制显示，不截断受力模型的衰减尾部。曲面高度编码归一化人工强度，
不是风速场，不能用曲面梯度代替受力公式。Ah/Az 不改变归一化曲面的高度。

### 四条弹力绳与共同物块

默认负载为 **0.5 kg、0.40×0.30×0.15 m** 的自由刚体箱体。
四绳连接各自 Airbot Play 的 `link6/endpoint` 与物块顶面四个独立吊点，初始松弛。
四个顶面吊点局部坐标依次为 `(-.15,-.10,.075)`、`(-.15,+.10,.075)`、
`(+.15,-.10,.075)`、`(+.15,+.10,.075)` m。
四机参数：原长 `L0=1.6 m`、刚度 `k=100 N/m`、阻尼 `c=3 N·s/m`。

$$
\mathbf r=\mathbf p_{\mathrm{box,site}}-\mathbf p_{\mathrm{gripper}},
\qquad \ell=\lVert\mathbf r\rVert,
\qquad \mathbf n=\frac{\mathbf r}{\ell}
$$

$$
\dot\ell=\bigl(\mathbf v_{\mathrm{box,site}}-\mathbf v_{\mathrm{gripper}}\bigr)\cdot\mathbf n
$$

$$
T=\begin{cases}
\max\left\{0,\;k(\ell-L_0)+c\dot\ell\right\}, & \ell>L_0,\\
0, & \ell\le L_0.
\end{cases}
$$

$$
\mathbf F_{\mathrm{gripper}}=T\mathbf n,
\qquad \mathbf F_{\mathrm{box}}=-T\mathbf n
$$

零长度时力为零。连接点速度通过真实 site Jacobian 计算，点力通过 `mj_applyFT` 成对施加，
保留对机械臂、机体和物块的力矩。可见绳索只作绘制，不重复施力。
这是单向弹簧阻尼模型，不含绳下垂、碰撞、断裂或主动卷扬，参数未做绳材料标定。

## 其它运行方式

```bash
# 四机无物块/无吊绳
python scripts/run_simulation.py --no-payload --mode both --instance-base 4

# 原双机模式（默认绳长回到 1.2 m，按钮显示 both）
python scripts/run_simulation.py --aircraft-count 2 --mode both --instance-base 4

# 指定互扰参数
python scripts/run_simulation.py --mode both --horizontal 0.5 --vertical 0.5 --length 2 --instance-base 4

# 无窗口完整搬运流程；timeout、duration 单位均为墙钟秒
python scripts/run_simulation.py --headless --auto --mode both --duration 3 --timeout 300 --instance-base 4

# 更低的面板读数频率（不降低鼠标事件/物理/曲面频率）
python scripts/run_simulation.py --mode both --instance-base 4 --panel-hz 0.5

# 查看所有参数
python scripts/run_simulation.py --help

# 离线测试：不启动 PX4
python -m pip install 'pytest>=8'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg python -m pytest -q
```

自动流程按实测状态验收：起飞 → 物块离地及张力/速度/位置稳定 → 共同 +X 平移 0.5 m →
实际物块位移及稳定性满足条件 → 保持 → 降落、物块接地、绳松弛且四机解除解锁。
较慢主机可能触发墙钟超时；示例中的 300 s 不改变默认 80 s 或验收条件。

## 日志、验证与目录

四机面板读数默认 1 Hz（双机 2 Hz），事件 30 Hz、MuJoCo 同步 20 Hz、曲面更新 10 Hz，独立调度。

每次运行自动创建 `outputs/runs/<运行标识>/`，包含：

- `metadata.json`：参数、通信身份、显示频率等配置。
- `states.jsonl`：50 Hz 仿真时间状态，含四机、关节、物块、四绳张力、人工互扰、全对水平距离和墙钟时间。
- `summary.json`：各阶段完成情况、异常、仿真/现实耗时及 `realtime_factor`。
- `px4/a/`、`px4/b/`、`px4/c/`、`px4/d/`：四机独立参数、控制台与 PX4 日志。

四机默认实例 0/1/2/3；示例使用 4/5/6/7，对应系统 ID 5/6/7/8、TCP 4564–4567、Offboard UDP 14584–14587。
四机 `--instance-base` 范围为 0–6；双机为 0–8。
已有输出目录会被拒绝，避免覆盖。原始运行日志、虚拟环境和缓存不提交到 GitHub。

```text
src/dual_uav_interaction/  模型组装、共享仿真、互扰、吊绳、桥接、显示及面板
src/pymavlink/             随项目保留的通信依赖与原始声明
runtime/px4/              预编译 Linux x86-64 SITL 与启动资源
scripts/                  启动、回放、预览、面板验证及显示基准
tests/                   离线回归测试
demo/                    用户提供的两张截图与视频
docs/                    安装、来源、验证记录与历史开发说明
licenses/                第三方许可证
```

四机扩展的当前离线记录为 **87 项测试通过**；已有实际四 PX4 的起飞、四绳吊物、共同平移及降落成功记录，
物块向 +X 实际位移约 0.505 m。四机图形与独立移动验证结果见下述验证记录。
四机验证结果见 [四机验证说明](docs/FOUR_UAV_VALIDATION.md)；历史记录见 [VALIDATION](docs/VALIDATION.md)，资源及适配来源见 [PROVENANCE](docs/PROVENANCE.md)。
历史文档中的 `outputs/validation/` 指开发机保留的运行目录，原始日志没有随仓库分发。
上述证据不代表气动精度、真实传感器、硬件或实机飞行验收。

## 许可证与引用

项目自有代码、文档、演示素材及已授权的 RAIDS Lab 模型按 [MIT](LICENSE) 发布。
PX4、pymavlink 和其它第三方素材继续适用各自原始许可证，见
[THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md)。MIT 不覆盖这些独立的第三方材料。

研究中使用时，请注明项目名称、仓库地址和使用的提交版本，并如实说明人工扰动模型及真值反馈范围。
