# 历史开发说明（2026-11-14 开源整理前）

以下为整理前的项目说明，安装与当前使用方法以仓库根目录 README.md 为准。

# Dual UAV Interaction

两架 **X4-950 + Airbot Play** 的独立 MuJoCo / PX4 SITL 仿真。
一个共享物理世界、两个独立飞控实例，机械臂保持初始姿态。
默认提供双机弹力绳共同吊运物块，同时保留可调的双机平动互扰。
两机沿世界 Y 轴并排，机头都朝 +X；夹爪与物块通过预先连接的绳模拟，不包含主动抓取/释放。
场景采用原单机仿真的蓝色格子地板、渐变天空及灯光配置，读取本项目已有的 `ground.xml` 副本。

## 启动

在桌面终端运行一个命令即可同时启动两个受管理 PX4、MuJoCo 窗口和控制面板：

```bash
source /home/raids/miniconda3/etc/profile.d/conda.sh
conda activate env_uav
cd /home/raids/Dual_UAV_Interaction
python scripts/run_simulation.py
```

不需要再单独启动 PX4。当前运行包面向本机 Linux x86-64，依赖与原项目相同的系统动态库。
环境依赖见 `requirements.txt`；完整 PX4 源码没有复制或重编译。

操作顺序：等待 A/B 均显示 connected → **Takeoff both**（自动请求两机解锁，确认后同步起飞）→
使用各自 XYZ 方向键 → **Land both** → **Exit**。
起飞等待期间按钮显示 **Arming both...**；降落、急停或退出会取消尚未完成的起飞请求。
Emergency 直接停止仿真转子并请求解除解锁。Exit/Ctrl+C 只停止本启动器创建的两个 PX4。
降落将高度目标设在地面支撑位置以下 0.2 m，确认真实地面接触、低速并持续稳定 0.7 秒仿真时间后解除解锁。

面板采用约 600 × 560 的紧凑窗口（随系统 DPI 缩放），浅灰背景和彩色三角方向键。
A 在上、B 在下，各控制区面积约为原横排版的一半；公共按钮、步长和参数集中在底部，
每架各有独立 **X+/X−、Y+/Y−、Z+/Z−**；蓝色 X、绿色 Y、橙色 Z。
所有方向都是公共 MuJoCo 世界坐标，点击只改变所选飞机的位置目标，另一架目标保持不变；
两机之间仍会通过物块和弹力绳产生物理耦合。仅起飞/悬停且已解锁的飞机接受平移。
两组方向键共用步长 **0.10 / 0.20 / 0.50 m**，默认 0.20 m。
每机状态区显示当前位置和最终目标；沿用原有带载平滑位置指令。

独立位移需要满足目标水平间距：带载 **1.50–2.25 m**、无载 **1.50–10 m**，
越界整次拒绝并在面板显示原因。这是场景设置，不是经过实物验证的安全距离。
机械臂位置保持控制保留，不锁死关节。
面板下方保留四种互扰模式及 Ah/Az/L，修改后点击 **Apply** 生效。

**Start wind field** 启动扰动强度曲面显示，按钮随即变为 **Stop wind field**；再次点击立即清除显示。
启动时默认不显示；需要源机已起飞且有推力才出现曲面。此按钮独立于互扰受力模式，
不会启用/关闭人工加速度扰动或改变吊绳力。
[当前精简面板预览](media/control-panel-minimal.png)。

显示更新已分开调度：面板读数 **2 Hz**、鼠标事件 **30 Hz**、MuJoCo 状态同步 **20 Hz**、
风场几何更新 **10 Hz**。读数只重绘发生变化的状态区域，按钮/输入草稿保留；
窗口缩放后若文字边界不适合局部清除，则自动回退到 2 Hz 完整重绘。
曲面使用缓存几何，仅更新位置/透明度，仍保留完整网格和等值线。
此优化保持物理步长及飞行限速，不保证所有主机负载下达到实时；
每轮 summary 记录 `simulated_time_s` / `elapsed_wall_s` / `realtime_factor`，
每条状态记录增加 `wall_elapsed_s`，可据此区分仿真变慢与飞行限速。

## 双机弹力绳搬运

默认物块为 0.5 kg、0.40 × 0.30 × 0.15 m 的自由刚体箱体，初始底面位于地面。
两条无质量直绳从各自 Airbot Play 的 `link6/endpoint` 连接到物块顶面局部
`(0, -0.12, 0.075)` 和 `(0, +0.12, 0.075)` m。原长 1.2 m、刚度 100 N/m、阻尼 3 N·s/m。
初始绳松弛；带载初始和目标间距限制为 1.50–2.25 m。黄色短标记指示两机 +X 机头方向。

令 `r=p_box_site-p_gripper`、`ell=|r|`、`n=r/ell`、
`ell_dot=(v_box_site-v_gripper)·n`。仅当 `ell>L0` 时，
`T=max(0, k*(ell-L0)+c*ell_dot)`，否则 `T=0`。
夹爪受到 `+T*n`，物块连接点受到 `-T*n`；零长度时力为零。
这是沿用参考单机实现的单向弹簧阻尼模型，系数是人工仿真设置，未做绳材料标定。
参考位置见 [PROVENANCE](PROVENANCE.md)。

力在世界系的真实连接点通过 `mj_applyFT` 施加，保留对关节和机体的力矩；
点速度包含机体转动和机械臂运动。广义外力每步清零并累加两条绳力；显示用空间 tendon 的刚度/阻尼均为零，
不会重复施力。人工气动互扰继续按整机质量分布到各部件，其零质心合矩约定仅适用于互扰力。
PX4 推力换算仍使用每机自身 8.0004 kg，没有把半个物块质量加入标定。

带载位置指令按仿真时间渐变到最终目标，速度限制 0.25 m/s、常规加速度渐变 0.1 m/s²，
终点处截断到准确目标，避免突然拉紧绳索；不增加速度/加速度前馈。
面板显示物块高度、底部离地高度、地面接触和两绳张力。无载模式使用 `--no-payload`，目标间距范围为 1.5–10 m。
可用 `--payload-mass`、`--rope-length`、`--rope-stiffness`、`--rope-damping` 调整参数；
有限正值不保证任意设置的闭环稳定性，初始预拉伸配置会被拒绝。

完整自动流程：两机解锁起飞 → 物块离地 >0.15 m、两绳有张力、物块线速度 <0.2 m/s、
两机位置误差 <0.15 m 持续 1 秒仿真时间 → 共同向 +X 平移 0.5 m →
实际物块 X 位移 ≥0.35 m、距目标 <0.15 m 且满足上述条件持续 1 秒 → 保持 →
两机降落解除解锁，物块地面接触/低速/绳松弛持续 0.7 秒。
从吊起验收通过到主动降落前，物块触地或失去规定离地间隙直接判为失败。
空中验收针对平动，不保证物块姿态稳定。绳没有下垂、碰撞、断裂或主动卷扬模型。

```bash
python scripts/run_simulation.py --headless --auto --mode both --duration 3 --timeout 140
python scripts/run_simulation.py --no-payload
```

实际运行状态保存在独立输出目录的 `metadata.json`、`states.jsonl` 和 `summary.json`。
验证范围与结果见 [VALIDATION](VALIDATION.md)。

2026-11-13 实际无窗口双 PX4 SITL：`off` 和 `both` 均完成全部搬运阶段，
物块最终 X 位移分别为 0.547 m 和 0.494 m；无载回归通过，离线套件 54 passed。
[实测姿态回放](media/payload-carry-sitl.gif)、[吊运画面](media/payload-carry-sitl.png)、
[带载面板](media/control-panel-payload.png)。回放读取 `both` 成功运行的实际位置、姿态和关节日志，
不重放 PX4 或推进物理；为看清吊绳没有叠加强度曲面，转子转动相位未记录。
可重新生成：

```bash
MUJOCO_GL=egl python scripts/preview_payload_run.py outputs/validation/carry_both_20261113_final
```

## 四种模式与参数

当前暂用**人工设置的距离衰减加速度扰动模型**，用于程序验证与人工扰动对照。
该模型并非由实验数据拟合得到，也未经过 X4-950 气动标定，不能据此声称复现了真实双机气动互扰。

| 模式 | 面板/CLI 名称 | 作用 |
| --- | --- | --- |
| 关闭 | off | 所有互扰外载荷清零 |
| 仅水平 | horizontal | 沿两机水平连线分别向外 |
| 仅竖直 | vertical | 两机均受到向下扰动 |
| 水平＋竖直 | both | 同时启用两通道 |

参数均为**人工设置的试验参数，不是 X4-950 气动标定值**：

- Ah：水平峰值加速度，m/s²，非负有限；默认 0.5。
- Az：向下峰值加速度，m/s²，非负有限；默认 0.5。
- L：水平间距为 L 时强度减半，m，正有限；默认 2。

以水平中心距 d 定义 `g = 1 / (1 + (d/L)^2)`。
A、B 水平加速度大小均为 `Ah*g`、方向相反；竖直分量均为 `-Az*g`。
默认间距 2 m、L=2 m 时，每个已开启通道大小为 0.25 m/s²。
这不是实际流速场，没有将风速 m/s 直接当作加速度。

模式和三个字段在 Apply 时一起校验并生效。错误输入保留上一组配置，状态刷新不覆盖正在编辑的草稿。
默认关闭；命令行也可预设模式及参数：

```bash
python scripts/run_simulation.py --mode both --horizontal 0.5 --vertical 0.5 --length 2 --separation 2
```

源机没有实际仿真推力时不生成互扰；地面接收机不施加载荷。
源/接收机需要处于起飞、悬停或降落阶段，且离初始 root 高度超过 0.15 m；这是首版飞行阶段门控。
关掉模式立即清除所有相关外力，没有沿用原风项目的 EMA、XY 投影或 3 m/s 限制。

每个物理步先刷新同一世界状态，计算 A→B 与 B→A，再把每机加速度按子 body 质量分配外力，
保证合力为 `m*a`、关于整机当前质心的互扰合矩为零，最后只推进一次 MuJoCo。
不额外添加气动力矩；PX4 自身的姿态变化和机械臂动力学仍然存在。

## 半透明扰动强度曲面

点击面板 **Start wind field** 后，MuJoCo 窗口显示两机各自的半透明曲面与细等值线，不使用箭头。
A 的曲面为珊瑚红，B 为琥珀橙。曲面跟随当前机体位置，中心高、向外降低，整体位于机体水平面下方。
当前曲面最高处为机体原点下方 0.05 m，较旧版上移 0.60 m；在水平姿态下约距桨叶底面 0.116 m。
高度编码现有 `g(r) = 1 / (1 + (r/L)^2)` 的归一化人工扰动强度，L 跟随面板已应用值；
高度的偏移/缩放仅用于显示，不是实际风速、物理势能或气流边界，不能用其梯度代替当前受力计算。
Ah/Az 控制实际人工加速度幅值，图中显示的是归一化 g(r) 形状。

**10 m 指水平图域半径**，与衰减公式的水平距离定义一致；边缘渐隐。
显示范围不截断实际互扰模型的衰减尾部；两张曲面的叠放也不改变受力模型。
已解锁、处于起飞/悬停/降落、离初始 root 高度超过 0.15 m 且有转子推力的源机才显示。
源机停转后在 0.6 秒仿真时间内淡出。面板 **Start wind field / Stop wind field** 可独立开关，关闭立即清除显示。
此显示与四种互扰模式解耦：`off` 时有效源机也可以显示归一化曲面。
显示层不增加碰撞体、不写外力、不推进物理；无窗口模式跳过显示层。

[近景预览](media/field-height-adjusted/disturbance-surface-close.png)、[范围总览](media/field-height-adjusted/disturbance-surface-overview.png)、
[相机环绕动画](media/field-height-adjusted/disturbance-surface.gif)与[面板布局](media/control-panel-minimal.png)
使用真实模型的离线示意姿态，不能替代飞行验收。
[带载高度调整预览](media/payload-field-height-adjusted.png) 使用之前实际运行的物块/机体/关节姿态重建。
重新生成模型预览无需启动 PX4：

```bash
MUJOCO_GL=egl /home/raids/miniconda3/envs/env_uav/bin/python scripts/preview_disturbance.py --output-dir docs/media/field-height-adjusted
```

## 端口与独立资源

默认实例 0/1，MAV_SYS_ID 为 1/2，TCP 为 4560/4561，Offboard UDP 为 14580/14581。
状态、参数、日志在本项目 `outputs/runs/<本次运行>/px4/a,b/` 中分开保存。
如果旧单机仿真正在使用默认端口，可为新项目指定 `--instance-base 4`，使用实例 4/5 与系统 ID 5/6。
端口占用时启动器报错，不停止其它会话。已有输出目录会被拒绝，避免覆盖。

资源、pymavlink 和 PX4 bin/etc 均复制到本项目；PX4 内部相对命令别名保留，均解析到本项目内部。
运行不依赖 `/home/raids/Wind_field_simulation`，原项目代码、资源与状态不被修改。
来源与授权见 [PROVENANCE](PROVENANCE.md) 和 [第三方声明](../THIRD_PARTY_NOTICES.md)。

## 离线与闭环检查

```bash
# 真实 MuJoCo 模型/参数检查，无 PX4、无窗口、无网络
python scripts/run_simulation.py --check --mode both

# 离线测试
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg python -m pytest -q

# 实际双 PX4 自动解锁/起飞/吊物/共同平移/保持/降落/退出，无窗口
python scripts/run_simulation.py --headless --auto --mode both --duration 8 --instance-base 4
```

每次运行保存 metadata.json、50 Hz 同时刻 states.jsonl、summary.json 和各 PX4 控制台/ULog。
50 Hz 按仿真时间采样；渲染较慢时实际运行可能慢于实时。summary 区分初始模式与最终设置，逐步日志记录每次参数变更。
悬停 RMSE 包含进入高度容差后的瞬态，仅作程序诊断，不作为气动精度或控制器性能结论。
验证范围和未验证项见 [VALIDATION](VALIDATION.md)。
