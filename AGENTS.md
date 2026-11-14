# Dual UAV Interaction 开发约定

独立项目根目录 `/home/raids/Dual_UAV_Interaction`。先读 README.md、docs/PROVENANCE.md、docs/VALIDATION.md。
未经用户要求，不修改旁边的 Wind_field_simulation、PX4-Autopilot 或 PINN 项目。
当前是人工加速度互扰模型；不描述为经标定的流场，也不自行扩展训练或实机测试。

Python 使用类型提示和关键函数 Google Style Docstrings；遵守 PEP 8。
新实验随机过程显式设 seed；大型数值操作优先 NumPy。深度学习如需引入，默认 PyTorch，但当前不需要。
模型/接口变化保留必要诊断；先解释目的、输入、输出及验证范围。

离线命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg /home/raids/miniconda3/envs/env_uav/bin/python -m pytest -q`。
实际 SITL 与离线检查分别报告。进程只由启动器拥有和清理，不按名称批量终止。
共享世界每步只推进一次；A/B 状态、关节、执行器、通信身份、参数目录均隔离。
外力/矩的参考点和坐标要明确；模式关闭必须清除旧载荷。保留第三方声明。
