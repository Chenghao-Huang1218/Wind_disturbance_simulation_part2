# 来源与接口

本项目从 `/home/raids/Wind_field_simulation` 只读复制必要资源，未复制整个 PX4 源码。
原始文件的 SHA256 清单保存在 protected-original-sha256.json，用于最终确认原工程未变更。

- assets/robot_models、assets/scenes、assets/configs：原 X4-950 + Airbot Play 资源，版权头保留。
- src/pymavlink：原项目兼容的 pymavlink 2.4.47；COPYING/嵌套声明保留。
- runtime/px4/bin 与 etc：原项目的 Linux x86-64 运行包，字节未改，命令别名为内部相对链接。
- px4_bridge.py：从原 simulation.py 抽取必要坐标、推力、状态反馈和 MAVLink 类，
  增加显式目标/发送者 ID、接收来源过滤和各实例固定 seed。没有导入原包，也没有 PINN。
- viewer.py：沿用原项目针对 MuJoCo 3.6 的渲染线程关闭实现。
- interaction.py、scene.py、simulation.py、panel.py、processes.py、cli.py：本轮双机实现。
- disturbance_visualization.py：独立显示层；当前使用人工距离衰减增益的半透明高度曲面和等值线，
  不是流体求解或势能模型。参考图仅用于用户选择显示风格，没有引入图中势函数。
- 共享世界的蓝色格子地板、天空及灯光读取已有的 assets/robot_models/scenes/ground.xml，
  与原单机工程的该配置一致，不增加运行时跨项目依赖。
- payload.py 的单向弹簧阻尼张力、Jacobian 连接点速度与 mj_applyFT 成对点力方式，
  参考只读文件 `/home/raids/PX4-Autopilot/dev_huang/apps/robot_control_agent/uav_control_with_px4.py`。
  原参考把绳绑定机体，当前按用户要求改为真实夹爪 endpoint/link6；物块顶面两个独立吊点。
  原参考 XML 的零重力没有沿用，本世界保留 -9.80665 m/s²；未引入参考中的球形负载和其他控制算法。
  绳参数为人工设置，不声称来自绳材料实验。transport.py 为本项目的实测状态验收流程。
- panel.py 当前双机独立控制布局参考同一只读文件的 create_control_panel：浅灰背景、
  蓝 X/绿 Y/橙 Z 三角方向键、鼠标形状命中/悬停和步长按钮风格。
  本项目扩展为 A/B 独立目标及公共起飞/显示控制；初版步长 0.05/0.10/0.25 m，
  后按用户要求翻倍为 0.10/0.20/0.50 m。2026-11-14 按用户要求压缩为上下排列的紧凑布局，
  默认 6×5.6 inch；不改变原有控制语义。
  未引入参考程序的手柄、相机、轨迹库或 L1 控制。原始参考文件没有改动。
  一键起飞沿用现有桥接解锁协议，等待两机 armed 真正确认；不据命令发送即判定起飞。
- display.py、panel_rendering.py 为本项目显示性能优化：独立墙钟刷新节拍及纯显示局部重绘。
  disturbance_visualization.py 对独占非 append 用户场景复用原生几何；append 预览保持完整构建。
  没有引入其它框架、简化网格或修改模型/物理/飞行控制。优化前的两个显示模块备份保存在
  `docs/provenance/display-before-optimization/`，仅供 `scripts/benchmark_display.py` 对照，
  正常运行不导入该目录。

PX4 官方基线为 `c2fb48990aacebc938c4e9998c7d5348f858fd22`。
现有二进制含原项目的 MuJoCo 浮点真值反馈/X4 配置适配，估计器被旁路；控制器仍为 PX4 原生控制回路。
因此本项目不验证真实传感器 EKF2 链路。复制的最小补丁/散列位于 docs/provenance。

MuJoCo 世界 z 向上，内部沿用原桥接的 `(x,-y,-z)` 世界/机体分量转换；
该世界可作为本地任务坐标，不另行宣称绝对地理 ENU/NED 对齐。
姿态顺序为 w,x,y,z。每机 PX4 本地原点设为各自初始 root 位置；共同任务位置在 MuJoCo 世界中表达。
HIL_SENSOR、LOCAL_POSITION_NED 和 HIL_STATE_QUATERNION 保留原发送顺序与浮点配对所需身份一致性。

原模型 root 上的总外力会偏离含机械臂整机质心。本实现给每个子 body 施加 `body_mass * acceleration`，
世界系力作用于该 body 的质心，无附加局部力矩。利用质心定义，其总合矩在整机质心为零。
这一做法是用户批准的平动现象模型，不是逐部件气动求解。

PX4 预编译程序含构建期 `PX4_SOURCE_DIR`。启动器显式传入 `-t <本次运行>/test_data`，
预建空的本地目录，避免 POSIX 启动代码自动链接构建源的单元测试目录；飞行启动不需要该测试数据。
`-d` 关闭 stdin EOF 导致的 pxh 交互提示符循环。归档验证输出保留普通文件，
不复制 PX4 自动生成的目录链接；实际新会话使用当前项目的资源路径。

## 2026-11-14 公开分发授权与文档整理

项目权利人已确认有权公开自有代码、RAIDS Lab 模型及 demo，并选择 MIT 许可证。
根目录 LICENSE 为自有材料的显式许可；保留原有模型版权署名。第三方材料维持原许可证，
详见 THIRD_PARTY_NOTICES.md。新增安装说明明确预编译 PX4 的主机动态库依赖。
GitHub 分发排除 outputs、缓存、虚拟环境和本地凭据目录；历史验证中的原始输出路径指开发机归档。
本次整理未修改控制/物理实现、原单机工程或 PX4 二进制。
