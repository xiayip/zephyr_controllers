# mpc_controller

支持固定底座单机械臂，或单机械臂加移动底盘的 MPC 控制器，直接依赖原版 OCS2。
不再需要安装或构建 `arms_ros2_control` 的任何包。包名、控制器类名和插件名均为小写。

## 组成与依赖

| 文件 | 功能 |
| --- | --- |
| `src/mpc_controller.cpp` | 生命周期、求解线程、暂停及服务注册 |
| `src/control/TcpKinematics.cpp` | 基于 Pinocchio 的标准 FK/IK 服务，不执行运动 |
| `include/mpc_controller/control/JointInterfaces.h` | 关节接口句柄及最近位置命令；不维护无用的导数历史 |
| `include/mpc_controller/runtime/` | 线程调度、执行周期计算和可选耗时统计 |
| `src/control/CtrlComponent.cpp` | 模型加载、实测观测、求解结果执行、保护与工作线程协调 |
| `src/control/TcpReferenceManager.cpp` | TCP 目标输入、坐标转换与 OCS2 参考轨迹生成 |
| `src/control/ContinuousTcpPath.cpp` | 显式连续路径的几何插值、时间参数化与偏离检查，不依赖 ROS 或硬件 |
| `src/control/WholeBodyReference.cpp` | 可选底盘／关节姿态参考、同时间轴插值和状态代价；不包含任务流程 |
| `src/control/TcpTrajectoryRecorder.cpp` | 实测/预测 TCP 的缓冲与 CSV 写出 |
| `include/mpc_controller/control/BaseKinematics.h` | 旧底盘后端的横向速度约束；不安排底盘/机械臂动作顺序 |
| `src/visualization/mpc_visualizer.cpp` | TCP 位姿、两色轨迹、分身及其时间对齐 |
| `plugin/tcp_rviz.cpp` | 可拖动的单 TCP 坐标轴、坐标输入、完整位姿输入和误差显示 |

控制模块的声明放在同名头文件中。目标处理、CSV 记录和可视化各自独立成文件，
由 `CtrlComponent` 协调，仍属于同一个包；没有增加新的抽象层或独立节点。
交互坐标轴由 RViz 面板共享，使用 RViz 已有的 ROS 节点。
三个 RViz 界面类在 `plugin/tcp_rviz.hpp` 中声明，在 `plugin/tcp_rviz.cpp` 中实现；Qt 元对象代码由 AUTOMOC 自动生成。
TCP 接入 MPC 的主要链路为：
`tcp_target/stamped → TcpReferenceManager → OCS2 参考轨迹 → 求解器 → CtrlComponent → 关节与底盘命令`。

MPC 求解器与优化模型使用未修改的 `ocs2_mobile_manipulator`、`ocs2_ddp`、`ocs2_mpc`。
关节接口、调度/计时及单 TCP 误差 HUD 是本包内的少量辅助代码；不复制 MPC 算法。
普通目标话题及 FK/IK 复用标准 ROS 类型；有限运动 Action 和运行配置接口依赖 `zephyr_msgs`。
画圆等形状生成放在上层示例，不属于控制器的执行接口。
代码来源、迁移基线和维护边界见 [UPSTREAM.md](UPSTREAM.md)。

## 注册与兼容

控制器插件类型为 `mpc_controller/mpc_controller`，
C++ 类为 `zephyr::mpc::mpc_controller`。
目标面板插件为 `mpc_controller/target_coordinate_panel`。

单机械臂 mock 使用实例名 `mpc_controller`，只加载一个 MPC 控制器。
RViz 的本地 TCP 面板直接连接 TCP 话题，不再使用控制器别名或左右臂重映射。

ROS 包、C++ 头文件路径、插件 XML、共享库和 RViz 插件均已使用新名称；
不提供旧包名兼容别名。已有外部 YAML、RViz 配置或客户端需同步迁移。
本次服务类型有不兼容变更，见下文服务迁移表。
从旧包切换时请在新终端加载新的安装环境，避免旧安装残留干扰插件发现。

固定底座模型任务在 `zephyr_arm_description/config/ocs2/task.info`，控制器参数在
`zephyr_arm_bringup/config/mpc_mock_controllers.yaml`。Z2 的整机配置位于
`z2_bringup/config/ocs2/mpc_real.info`，通过 Z2 bringup 默认加载为 inactive。

任务文件路径为 `<task_config_package 的 share 目录>/config/ocs2/<info_file_name>.info`。
`task_config_package` 默认是 `<robot_name>_description`，Z2 显式设为 `z2_bringup`；
`info_file_name` 默认是 `task`，不包含 `.info` 扩展名。机器人模型仍由
`planning_urdf_path` 或机器人描述包提供，与任务配置包独立。
这两个参数只在控制器初始化时读取，切换配置需要卸载/重新加载控制器，或重启 bringup；
仅停用/激活不会重新读取文件，也不支持运行中通过 `ros2 param set` 热切换。

## 编译

基线为 ROS Jazzy/aarch64、OCS2 `1.4.0`。升级 OCS2 需要单独验证 API/ABI，
不要混用未验证的 `1.4.2.main` 预发布库。来源提交只用于归属与对照，不是构建依赖。

| 依赖 | 获取方式 / 是否适合放 Dockerfile |
| --- | --- |
| OCS2 算法及 ROS 桥接：`ocs2_core/oc/mpc/ddp/mobile_manipulator`、`ocs2_pinocchio_interface`、`ocs2_self_collision`、`ocs2_self_collision_visualization`、`ocs2_ros_interfaces`、`ocs2_msgs` | 固定版本的二进制包，或匹配源码构建的 underlay；可在镜像构建阶段安装，不能以普通 ROS apt 包存在为前提 |
| ROS 2、ros2_control、TF2、realtime_tools、标准消息、`moveit_msgs` | 按 `package.xml` 由 rosdep/apt 安装，适合 Dockerfile |
| Eigen、Boost、Pinocchio | rosdep/apt；Pinocchio 必须与 OCS2 的构建版本兼容 |
| RViz、Qt、interactive_markers | 可选界面构建依赖，默认开启；适合 Dockerfile |

`moveit_msgs` 只提供服务/消息定义，不需要启动 MoveIt、move_group 或 Servo。
本包不再生成 rosidl 接口。下面的命令假设 OCS2 underlay 和其他 Zephyr 依赖已安装；
新终端只加载需要的环境，不要沿用旧 `arms_*` underlay：

```bash
cd /workspaces/zephyr-dev/zephyr_ws
source /opt/ros/jazzy/setup.bash
# 若 OCS2 不在系统前缀，先 source 对应 underlay 的 setup.bash。
source install/setup.bash
colcon build \
  --packages-select mpc_controller zephyr_arm_description zephyr_arm_bringup \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

只构建控制器核心、不编译 Qt/RViz 插件时，给构建命令增加
`-DBUILD_MPC_RVIZ_PLUGIN=OFF`。它与运行时的 `show_rviz:=false` 不同；
请使用新的 build/install 目录，避免旧的 RViz 插件安装产物残留。
`package.xml` 仍声明默认 GUI 构建所需依赖；此 CMake 开关不会自动过滤 rosdep 安装列表。

验证改名和 mock 配置：

```bash
colcon test --packages-select mpc_controller zephyr_arm_description zephyr_arm_bringup
colcon test-result --verbose
```

## 启动

以下单臂入口位于配套 `zephyr_arm_ros2` 工作区改动中，目前暂留本地，
**不随本次控制器 `dev_mpc` 推送发布**。仅拉取本控制器仓库不会获得该 launch/task 配置；
本包本身只提供控制器及 RViz 插件，不需要另建自定义消息。

固定底座单臂 mock（默认打开 RViz）：

```bash
ros2 launch zephyr_arm_bringup mpc_mock.launch.py
# 无窗口
ros2 launch zephyr_arm_bringup mpc_mock.launch.py show_rviz:=false
```

不要同时运行以上两条命令，也不要在同一 ROS domain 混入真机 controller manager。
此 launch 只接受 `mock_components/GenericSystem`，没有切换真机的参数，
不会启动 CAN、MIT 控制器、相机驱动或底盘；原 real_world/mock 启动链不变。

- 初始关节为 `[0, 1.6, 1.0, 0, -0.5, 0]`，直接初始化理想 mock，不是真机 JTC 准备动作。
- `manipulatorModelType=0`，只有六个机械臂状态；控制器只认领六轴 `position`。
- 同时激活 `joint_state_broadcaster` 和 `zephyr_arm_gripper_controller`。
- 包含法兰、Gemini 305、OmniPicker 的几何，TCP 为 `omni_picker_tcp`。
- MPC 中夹爪固定闭合，mock 初始夹爪归一化位置为 `1.0`；本轮不要开合夹爪，
  否则显示与规划几何会不一致。夹爪实时几何更新尚未接入。
- 当前本地单臂 mock 的 `selfCollision.activate=false`：暂时关闭 MPC 自碰撞代价和
  运行时自碰撞检查，隔离验证控制链。原有 52 对配置保留，没有修改 SRDF 排除规则。
  碰撞模型简化并验证后再启用；不含桌面、Z2 车厢或动态环境，不能用作安全证明。
  控制器库没有全局关闭开关改动，其他配置仍按各自的 `selfCollision.activate` 工作。
- GenericSystem 是理想位置跟随，不模拟电机动力学、MIT 力矩、刚度或通信延迟。

查看加载状态与 TCP：

```bash
ros2 control list_controllers
ros2 topic echo /tcp_current_pose --once
ros2 topic echo /mpc_controller/motion_status --once
```

控制器类型应为 `mpc_controller/mpc_controller`；生命周期 `active` 不代表 MPC
已就绪，需要同时查看 `~/motion_status`（IDLE、无故障且反馈有效）和诊断日志。
激活后自动准备，无额外 enable/resume；TCP 指令与停止接口见下文。

### 生命周期简化验证（2026-11-12）

88 项 C++ 用例及画圆示例的 10 项 Python 用例通过。隔离 mock 验证了：
激活自动就绪、Action 取消后接受新目标、连续目标 stop 后丢弃旧参考、
运行配置跨重新激活自动恢复，以及 RViz 未就绪只预览、就绪后不自动发送旧预览。
Z2 的底盘＋机械臂 mock 也验证了 stop 后底盘零速保持、无 resume 执行下一目标，
以及停止后切回 JTC 的保持确认。底盘发送器不再另设 LifecyclePublisher 激活门槛，
统一由 controller 管理；退出时仍需验证最终保持写入及零速发布。
这些是控制链功能测试，不代表真机动力学、避障或物理急停验收。

### 历史 mock 验证（2026-11-05）

控制器核心与 RViz 插件重新编译通过，链接和导出的构建依赖不再包含 `zephyr_msgs`。
65 项 C++ 测试及 19 项 bringup Python 测试通过，覆盖标准路径输入、无效位姿拒绝、
暂停后忽略新目标、RViz 位姿流和非阻塞参考交接；不改求解器、控制增益及原保护逻辑。

当时隔离 headless mock 中三个控制器 active、MPC 正常跟踪（旧 `/fsm_state=3`）：
- 标准 `/target_path` 执行 TCP +Z 1 cm 及返回，4 秒后末端位置误差分别约 0.051/0.054 mm。
- 上层示例以 50 Hz 发布位姿目标，YZ 平面半径 2 cm、10 秒一圈；
  近时反馈位置误差 RMS 约 2.20 mm，回到起点并保持 3 秒后误差约 0.053 mm。
- ROS 图中两个自定义运动服务均不存在。测试进程已退出。
- 前一轮已验证 RViz 可拖动小球，并通过交互标记完成 TCP 向上 2 cm 及返回；
  本轮标准位姿流单测保留该入口，不改标记插件。

这些数据只验证理想 mock 连通性，非严格同步的性能测量，不代表真机精度或避障能力。
此前高精度 STL 的 52 对碰撞距离查询约 97 ms、首次求解约 15.9 s，曾触发
`collision check stale`。当前仍按开发安排仅在单臂 mock 暂关碰撞，
模型简化和性能验收待完成。夹爪 mock 已改为仅 position 命令以匹配 GenericSystem，
没有改变原真机接口或放宽限位。

## 后续更新上游

升级 OCS2 后重新构建本包并运行回归测试；API 适配留在本包，不修改上游源码。
`arms_ros2_control` 仅作为代码来源参考，更新它不会影响当前控制器。
迁入的辅助代码需按 UPSTREAM.md 的来源表人工对比，不会自动跟随上游更新。

迁移前备份（不在 colcon 源码扫描范围内）：
`/home/mscape/zephyr_migration_backups/20261026T073726Z`。
其中包含来源提交、完整差异补丁和逐文件校验后的源文件归档。

## 无界面运行与移动底盘输出

单臂 `mpc_mock.launch.py show_rviz:=false` 可关闭窗口。以下是移动底盘输出配置说明，
不能直接用于本节的固定底座 mock；Z2 的集成启动与真机验收开关见 `z2_bringup/README.md`。

按底盘驱动能力选择 profile，而不是按真机/mock 区分。支持完整全向命令的后端使用
`base_command_profile: omni`，允许带符号的 `vx/vy/wz`，支持倒退、横移、斜移和旋转。
不支持横移与旋转同时执行的旧后端使用 `ranger`：在优化问题中始终约束车体横向速度 `vy=0`，
执行端同步去除数值残差，允许前后移动、转向与机械臂同时优化。不再在输出端临时切换横移/转向模式。
当前 Z2 配置仍采用此保守 profile；切到 `omni` 前必须验证底盘后端的实际组合运动能力。
`base_max_linear_speed`、`base_max_angular_speed` 分别限制平面速度和角速度；0 表示不额外限幅。
默认 `omni` 不增加这些非完整运动约束。这些设置在初始化时读取，修改后需重新加载控制器。

## 单 TCP 接口

本包现为单机械臂控制器，已删除本地的第二条机械臂、第二组末端轨迹及记录分支。
若 task.info 启用了 dualArmMode，初始化会明确拒绝该配置。底盘的左右车轮、
夹爪两侧指尖属于真实硬件几何，与第二条机械臂无关，保持各自原名称。

公开话题统一为：

| 功能 | 新接口 |
| --- | --- |
| 当前 TCP 位姿（PoseStamped） | `/tcp_current_pose` |
| 当前跟踪目标（PoseStamped） | `/tcp_current_target` |
| 位姿目标（Pose） | `/tcp_target` |
| 带坐标系的位姿目标（PoseStamped） | `/tcp_target/stamped` |
| 速度目标（Twist） | `/tcp_target/twist` |
| 相对位移目标（TwistStamped） | `/tcp_target/relative` |
| 分段位姿路径（Path） | `/target_path` |
| 连续通过路径（Path，显式选择） | `/target_path/continuous` |

`tcp_ee_frame` 是本包的末端配置参数，默认读取 task.info 的 eeFrame。
当前两个 Zephyr 模型的真实 TF 末端仍为 `omni_picker_tcp`，本次仅统一控制接口命名，
保留实际安装几何和 142 mm TCP 偏移。修改末端帧参数时需重新加载控制器。
本地目标管理、FK/IK、交互坐标轴、位姿面板和误差订阅均直接使用单 TCP 接口。
不再声明 `left_ee_frame`，不再重映射任何左右臂话题，也不再安装
`tcp_compatibility.h` 或 `zephyr_mpc_launch.tcp_interfaces`。

- `TcpReferenceManager.cpp` 直接连接 OCS2 的 `ReferenceManagerInterface`，处理绝对位姿、
  带坐标系位姿、速度流、相对位移和路径；坐标转换失败会拒绝目标。
- `tcp_rviz.cpp` 中的 `TcpTargetManager` 创建名称为实际 TCP 帧的六自由度交互坐标轴，
  支持连续拖动和右键手动发送。XYZ 和完整位姿两个面板在同一 RViz 节点中共享一个服务端。
  不再安装或启动独立的 `tcp_target_manager` 可执行程序。现有 InteractiveMarkers 显示器的
  `Interactive Markers Namespace` 应为 `/tcp_target_manager`（Jazzy 不使用旧 `Update Topic` 字段）。
  关闭全部目标输入面板或退出 RViz 后，交互服务端随之销毁；控制器继续接受其他程序的 TCP 命令。
  `/tcp_current_target` 使用 transient-local 缓存，后打开的界面可获得最近一次目标。
- `tcp_pose_panel` 提供完整位置/四元数输入和相对位姿命令；`target_coordinate_panel` 保留 XYZ 快捷输入。
- `tcp_error_display` 使用本地单 TCP 误差 HUD，显示位置/角度误差及趋势图；没有 body/head 卡片。
- FK/IK 使用标准 MoveIt 服务类型；内部仍由 Pinocchio 求解，不经过 MoveIt 规划器。
- CSV 记录文件为 `real_tcp.csv`、`pred_tcp.csv`，没有左右臂分支。

路径统一发布到 `/target_path`（`nav_msgs/Path`），不再提供两个自定义运动服务。
每个 Pose 的 frame_id 优先于 Path 的 frame_id；均为空时使用 MPC 基坐标系。
新路径替换当前参考，不排队。路径从当前采样参考开始，按段做五次时间插值及四元数球面插值，
每段在途径点处减速到零；`movel_duration` 是整条路径的基础时长，速度、加速度和 jerk
限制可以延长它。参数在初始化时读取，**Path 的时间戳不作为轨迹时间表**。

### 连续通过参考帧

有限运动优先使用 `/mpc_controller/execute_motion` 的 `CONTINUOUS_PATH=3` 模式；
也可发送 `nav_msgs/msg/Path` 到 `/target_path/continuous`。旧 `PATH=2` 和
`/target_path` 仍逐段起停，默认行为不变。此轮仅在 Action 中新增一个常量值，
未新增字段；发送端和机器人端仍应同步更新 `zephyr_msgs`，重新编译控制器及使用该
Action 的客户端（包括 BT/RViz）。
旧服务端不支持模式 3，不能混用来验证新功能。

新模式把整条路径作为一个有限运动：自然三次样条穿过中间位姿，整条路径共用
一个五次时间律，首尾参考速度及加速度为零，中间点不再强制归零。
它不保证恒速：曲率、反向和速度/加速度/jerk 约束仍会要求减速。
重复位姿会合并，不表示停留。需要停稳、接触确认或严格保持折线时仍用旧 PATH
或分成独立有限运动；本轮不实现阶段门槛、混合停止点或活动 Action 的在线续接。

- 时间参数化使用现有 `movel_max_*`，Action 的 `speed_scale` 对这些限制同时缩放。
  Action 时长由限制决定；topic 路径另以 `movel_duration` 为最短总时长。
- `movel_continuous_max_deviation` 默认 **0.0（关闭折线偏离限制）**，支持稀疏、长距离
  的参考帧。在初始化读取、禁止在线更改；设置有限正数（单位 m，例如 `0.005`）
  可重新启用 Bezier 控制凸包的保守偏离检查，超限明确拒绝，不退化成其他模式。
  该上界可能比曲线实际偏离更大；关闭后样条仍穿过各途经点，但点间可能明显弯曲或
  过冲，不能将输入折线的空闲空间当成平滑路径的空闲空间。它不是碰撞检查。
- 姿态在起点的局部旋转向量坐标中插值，再映射回单位四元数。起点相对转角和
  整段控制凸包须小于 **3 rad**；跨分支/大转动路径会拒绝，请拆成停止路径。
  `q` 与 `-q` 视为相同姿态，不支持多圈旋转意图。
- 解析曲线采用保守导数上界确定总时长，默认 `movel_sample_interval=0.04 s` 采样，
  并额外保留每个原始途经点。OCS2 上游仍对离散位置线性插值、姿态 SLERP；
  因此不能将解析曲线的 C2 连续性或 jerk 上界视为实际关节输出的严格保证。
- 有限 Action 从实测 TCP 起点建立参考，有接收/到位/取消反馈；topic 从当前参考起点
  替换整条路径，无到位确认，也不保证移动中反复替换的速度连续性。不要用该 topic
  冒充已经实现了滚动参考拼接。需要取消时使用 Action cancel 或 `~/stop`。
- 平滑偏离检查不等于碰撞检查、可达性检查或全身安全认证。最终到位要求实测
  TCP 误差及全身速度连续达标，见下文“有限运动的停稳判定”；中间点不触发该判定。
- 仅 `mock_debug_enabled=true` 时发布 `/tcp_reference_path`（Path，transient-local），
  显示或记录已提交的整条参考及其时间戳；这是非实时诊断，不是实际执行确认。
  无路径的保持目标发布空 Path 清除旧显示。真机默认不创建该 publisher。

OCS2 接入复用现有 `TargetTrajectories` 和 `ReferenceManager`：完整轨迹在非实时
回调中生成并写入缓冲，下一次 `preSolverRun` 才统一生效。控制器的参考生成不修改
模型、求解器、执行保护或上游源码；预测时域由各机器人 `.info` 配置选择。
`ModeSchedule` 用于具有模式切换的动力学模型；
当前运动学模型不因为任务进入“按压”就切换动力学，所以本轮不引入伪模式序列。
后续任务阶段约束应与参考版本同步，不能在求解过程中直接改共享代价对象。

仅用于已启动的**单臂 mock** 示例，以下坐标需改为当前 TCP 附近的可达坐标：

```bash
ros2 action send_goal /mpc_controller/execute_motion \
  zephyr_msgs/action/ExecuteCartesianMotion \
  "{mode: 3, waypoints: [
    {header: {frame_id: base_link}, pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {w: 1.0}}},
    {header: {frame_id: base_link}, pose: {position: {x: 0.30, y: 0.01, z: 0.30}, orientation: {w: 1.0}}},
    {header: {frame_id: base_link}, pose: {position: {x: 0.30, y: 0.02, z: 0.30}, orientation: {w: 1.0}}}
  ], speed_scale: 0.3, timeout: 0.0}" --feedback
```

这不是按钮接触轨迹，也不应原样用于真机。可先用 `ros2 topic echo /tcp_current_pose --once`
取得当前位置和四元数，用相同四元数构造小幅直线路径，再比较 `mode: 2` 与 `mode: 3`。

连续遥操或圆轨迹使用 `/tcp_target`（`geometry_msgs/Pose`）位姿流，坐标必须是 MPC 基坐标系。
该入口直接更新参考，不经过上述路径插值/限速；发送端负责连续性和目标速度，
控制器原有关节约束及保护仍生效。不要同时运行多个目标发送器。
配套 mock 画圆示例位于 `zephyr_arm_bringup/scripts/mpc_circle_demo.py`：

```bash
# 已启动 mpc_mock.launch.py，motion_status 为 IDLE；期间不要拖动 RViz 小球。
ros2 run zephyr_arm_bringup mpc_circle_demo.py --plane yz --radius 0.02 --duration 10
```

此示例只允许 GenericSystem，保持当前 TCP 姿态，绕一圈返回起点后保持 3 秒并退出。
普通 topic 没有接收确认、到位反馈或目标取消语义；通过实际 TCP 和 motion_status 判断跟踪情况。
停止发送只会保留最后目标；停止跟踪时先停止发布，再调用控制器的 `~/stop`。

### 服务迁移与 FK/IK 约束

| 服务 | 当前类型 | 迁移要求 |
| --- | --- | --- |
| `~/stop` | `std_srvs/srv/Trigger` | 替代旧 `~/pause`；重新就绪后可接收新目标，无 resume |
| `~/compute_fk` | `moveit_msgs/srv/GetPositionFK` | 替代旧 `~/kinematics_service` 的 FK 分支 |
| `~/compute_ik` | `moveit_msgs/srv/GetPositionIK` | 替代旧 `~/kinematics_service` 的 IK 分支 |

已删除 `ExecuteTcpPath.srv` 和 `ExecuteTcpCircle.srv`，旧客户端改发标准话题；
没有保留旧服务类型的兼容包装，否则仍会引入被解除的依赖。以上非私有名称在 launch
设置命名空间时跟随命名空间；`~/compute_*` 的前缀是控制器实例名，默认 `/mpc_controller`。

FK/IK 是**离线运动学查询，不发送关节命令**。请求须提供全部已配置机械臂关节的
`robot_state.joint_state.name/position`，允许重排，拒绝缺失、重复、未知关节及非有限值。
`is_diff` 必须为 false，不接受附着物或多自由度状态；移动底盘不包含在这组关节中。
FK 支持模型内存在的帧；IK 只支持配置的单 TCP。输入/输出坐标系只能是配置的 `baseFrame`
或空字符串（表示 baseFrame），不做任意 TF 查询。
IK 的 `group_name` 默认 `arm`；`ik_link_name` 为空时使用配置 TCP。
`avoid_collisions=true`、额外约束、多末端请求会明确返回错误，**不会默默忽略**。
本服务不能用于证明姿态/路径无碰撞。成功时 `error_code.val=1`，IK 解保留请求关节顺序。

求解设置位于控制器的 `ros__parameters`，只在初始化读取，在线修改会被拒绝：

```yaml
kinematics:
  group: arm
  solver: auto             # auto: 先 DLS，必要时 BFGS；也可 dls/bfgs
  max_iterations: 300      # 每种方法、每次尝试的上限，共用请求的超时预算
  tolerance: 0.0001        # SE(3) 残差范数，非碰撞/路径容差
  damping: 0.01
  timeout: 0.1             # 请求 timeout=0 时使用的总计算预算，秒
  random_restart: false    # true 最多四次尝试，要求有限关节限位
```

请求的 `timeout` 覆盖默认值，范围 `(0, 60]` 秒；包含等待运动学模型互斥锁的时间。
到期返回 `TIMED_OUT`，未收敛返回 `NO_IK_SOLUTION`，失败不返回可执行关节解。

只查询 ready 姿态的 FK（不会移动机械臂）：

```bash
ros2 service call /mpc_controller/compute_fk moveit_msgs/srv/GetPositionFK \
  "{header: {frame_id: base_link}, fk_link_names: [omni_picker_tcp], robot_state: {
    joint_state: {name: [joint1, joint2, joint3, joint4, joint5, joint6],
                  position: [0.0, 1.6, 1.0, 0.0, -0.5, 0.0]}}}"
```

IK 调用用同一组 `robot_state` 作为种子，将 FK 返回的 `pose_stamped[0]` 放入
`GetPositionIK` 的 `ik_request.pose_stamped`，并设 `group_name: arm`、
`ik_link_name: omni_picker_tcp`、`avoid_collisions: false`。不要把 `moveit_msgs`
接口类型理解成已经运行了 MoveIt 的 PlanningScene 碰撞检查。

运动学服务的机械臂求解器使用独立的降维模型：锁定底盘的虚拟自由度，
并把参考原点移到 task.info 指定的 baseFrame，避免把六维关节向量传给九维移动模型。
这只影响机械臂 IK/FK 服务，MPC 保留完整底盘与机械臂状态。服务求解异常会返回失败响应。


## 观测、线程和参数更新

### 接入现有位置适配器

`command_prefix: zephyr_arm_position_adapter` 配合
`command_interfaces: [position, velocity]` 会认领适配器的参考接口，
状态仍读取 `joint1/position` 等真实关节接口。MPC 同时发送预测位置和速度；
暂停/停用将速度参考归零。此组合仍是位置输出模式，不计算额外重力力矩，
Kp/Kd、重力与摩擦前馈继续由下游 Position Adapter 管理。
只配置 `[position]` 的 mock/CSP 输出保持原行为。
切换时不能让 JTC/Servo 与 MPC 同时认领同一适配器接口；Z2 真机入口默认
只加载 MPC 为 inactive，保留 JTC，见 `z2_bringup/README.md`。

控制观测始终来自实测关节和里程计，包括暂停后重新初始化；不再用上一周期的
关节命令覆盖反馈。旧 YAML 必须删除观测缓存参数，或明确设置
`cached_ob_state: false`、`hardware_latency: 0.0`。其他值会拒绝初始化，
不会静默启用旧行为。这两个名称只保留用于迁移检查，不提供延迟补偿功能。

控制线程先读取完整的一组关节位置和底盘里程计，全部有效才提交观测。
关节读取失败、NaN/Inf、里程计接收超时（500 ms）、无效四元数或
`header.frame_id` 与 `base_odom_frame` 不一致时，保留上一组有效观测，
暂停 MPC 执行，保持最近一次关节位置命令，并向底盘发送零速度。
该位置反馈保护依据本机接收时间；Action 停稳判定另检查里程计源时间戳与速度，见下文。
首次激活时关节反馈无效会拒绝激活。
正常反馈恢复后从当前实测 TCP 位姿重新开始 MPC。此处检查的是控制器使用的位置反馈，不代替硬件故障诊断。

实时 TCP FK 使用 `rt_data_`；预测、轨迹记录使用显示工作线程自己的 `viz_data_`。
轨迹清理也交给该工作线程，避免控制线程修改它正在使用的缓存。
控制线程交接最新实测观测快照与不可变预测快照；显示线程按独立频率运行，
不积压中间帧。退出 MPC 时通过版本号丢弃旧轨迹，防止分身重新出现。
TCP 发布仍由 `dds_publish_hz` 控制，不再随 MPC/分身刷新周期改变。

TCP 参考轨迹生成、Twist 积分和目标发布只在非实时回调执行。控制周期只更新原子时钟；
暂停不等待参考轨迹的互斥锁。恢复通过固定大小的非阻塞邮箱请求目标重置，等待
非实时处理确认之后才启动求解；等待期间保持关节并发送底盘零速。
暂停前已开始、但尚未获得处理锁的目标回调不能跨会话进入恢复后的控制。
这消除了参考轨迹长任务对控制线程的锁阻塞，但不表示整个插件已通过硬实时认证：
现有观测/显示快照分配及部分诊断发布仍需后续实测评估。

| 参数类别 | 更新规则 |
| --- | --- |
| `planning_urdf_path` | 必须指向已展开的 URDF 文件；保留文件/模型校验，不再需要固定值标记 `planning_urdf_variant` |
| `cached_ob_state`、`hardware_latency` | 仅接受 `false`、`0.0`；迁移检查参数，不支持在线启用旧功能 |
| `policy_max_age` | 默认 `0.5` 秒，策略 ROS 时间年龄和接收后的单调时钟年龄上限；初始化读取 |
| `collision_check_timeout` | 默认 `0.25` 秒，碰撞检查所用观测的最大年龄；初始化读取，须覆盖至少两个检查周期 |
| `mock_debug_enabled` | 默认 `false`，仅 mock 配置设为 `true`，初始化后不可切换；不负责切换硬件插件 |
| `rt_timing_enabled` | 可在线修改，只接受布尔值 |
| `traj_record_enabled` | 默认 `false`；仅 mock 调试配置允许在线启用，普通/真机配置拒绝启用 |
| `traj_record_dir` | 非空字符串；更换目录前先停止记录，下次开启记录时使用新目录 |
| 模型/接口、底盘话题与坐标系、底盘限速/死区、线程配置、MPC/DDS 频率、预测显示开关、观测缓存与未来采样配置 | 只在初始化读取；在线更改会返回失败，修改 YAML/launch 后须重新加载控制器 |

动态设置在整批参数通过所有校验后才应用；拒绝的参数批次不会修改本包内部配置。
`movel_*` 速度、加速度、jerk、采样间隔和时长在初始化时读取，在线更改会明确拒绝。
以下旧参数没有实际控制作用，已删除声明、成员和配置，旧 YAML 不应继续传入：

| 已删除参数 | 当前有效行为 |
| --- | --- |
| `joint_speed_threshold` | 不设置该门槛；实际限速由任务文件及硬件/控制器限制决定 |
| `movel_trajectory_duration` | 使用 `movel_duration`；路径服务也可以显式指定 `trajectory_duration` |
| `movel_auto_extend_duration` | 按速度、加速度和 jerk 限制自动延长时长，不再提供无效开关 |

单 TCP 可视化也不再包含类人机器人的 body/head 帧、额外 FK 结果和
`body_current_pose` / `head_current_pose` 发布器。只计算并发布实际 TCP，
保持原有发布节流及实时/显示线程独立 FK 缓存。移动底盘的里程计与协同控制不受影响。
本包已移除 HOME、HOLD、MOVEJ 状态类和对应动作；参考轨迹直接交给 OCS2 公共接口。

### 输出与故障边界

- 接收新策略时检查时间顺序、状态/输入维度和全部有限数值；每周期检查
  `当前时间 + future_time_offset / update_rate` 是否仍在策略时域内。不允许末端外推，
  策略过期或数据无效当周期保持关节、清零关节速度并请求底盘零速，锁存暂停。
- 运行时碰撞监控同时检查实测关节与最近命令。启用检查却缺少几何或碰撞对时拒绝初始化；
  检查失败、检测到碰撞或观测过期均锁存暂停。首次有效检查到达前不执行运动。
  碰撞结果的年龄从观测采集时刻计算，重复检查旧快照不会刷新有效期。
  目前检查与显示共用工作线程（频率为 `prediction_publish_hz`），但关闭预测显示
  不关闭已配置的碰撞检查。慢 FCL/显示负载可能触发监控超时，不会被当作安全。
- 求解、参考处理和上述策略/碰撞故障必须排查后显式停用/重新激活控制器，`~/stop` 不清错。
  首条策略尚未到达时只保持，不会输出运动命令。
- MIX 模式按 Pinocchio 关节名对应的 `idx_v` 选取重力力矩，跳过移动底盘自由度。
  初始化校验六轴顺序和模型维度，拒绝静默错位。保持状态不失能电机，也不清掉
  已输出的重力前馈；但这些软件行为不是未经真机验证的“不下坠”保证。

保护逻辑回归测试在 `test/test_control_safety.cpp`，覆盖策略过期/无效数值、
移动与固定底座力矩映射、碰撞检查失效/陈旧结果、暂停会话隔离及邮箱并发一致性。
`test/test_reference_handoff.cpp` 使用真实 ROS 执行器验证重置确认，并故意阻塞
参考处理，检查控制侧观测更新、暂停及重置请求仍不等待该锁；不启动硬件或求解器。
`test/test_tcp_visualizer.cpp` 用解析关节模型验证单 TCP 的 FK、话题、坐标系和节流；
不加载机器人网格、求解器或硬件。
依赖齐备后运行：

```bash
colcon test --packages-select mpc_controller
colcon test-result --verbose
```

## 底盘与机械臂联合优化

移动底盘模型将底盘位姿和关节角放入同一个 OCS2 问题，TCP 代价与输入代价持续生效。
底盘和机械臂从收到目标起即可同时运动；实际如何分配运动由模型、限位和代价决定，
并不强制每次都移动底盘，也不保证两者在指定时刻一起到位。

本地的 `StagedMotion` 已删除，包括距离球判定、停车点引导、先底盘后机械臂的输入锁定、
停滞后才协同的状态机及 `/mpc_controller/motion_phase` 话题。
旧的 `staged_motion_enabled`、`arm_workspace_frame`、`arm_reach_radius`、
`reach_switch_margin`、`base_parking_*`、`base_heading_tolerance`、
`base_approach_keep_heading`、`tracking_stall_seconds`、`tracking_position_tolerance`、
`tracking_orientation_tolerance`、`tracking_progress_threshold`、
`assist_no_progress_seconds` 和 `base_settle_seconds` 不再使用，请从私有配置中移除。
不再提供基于估算操作半径的 `unreachable_height` 拦截；上层仍须验证目标可达性和工作空间。
不要把求解器返回策略等同于目标可达或路径无碰撞。

`ranger` 仅保留独立的底盘运动学约束 `vy=0`，不锁机械臂，不指定停车点或转向时序；
`omni` 不添加该约束。两者均保留既有输出限速、死区、里程计检查、策略超时和暂停/故障处理。
单臂模型不添加底盘约束。没有修改上游 OCS2，也没有增加新的同步到达算法。

`task.info` 中 `inputCost.R.base/arm` 影响运动分配，`endEffector` 与
`finalEndEffector` 影响 TCP 跟踪；不同量纲的权重不能直接解释为运动比例。
当前 `mpc.timeHorizon` 是滚动预测时域，不是完成期限。
删除分阶段策略不会自动启用碰撞检查；原有 commissioning 配置的避障限制仍然有效。

## 运行时运动配置与底盘固定

`~/set_motion_profile`（`zephyr_msgs/srv/SetMotionProfile`）用于空闲时切换同一
MPC 控制器的运动约束。当前支持：

| 字段 | 取值 | 含义 |
| --- | --- | --- |
| `base_mode` | `COORDINATED`（0，默认） | 底盘和机械臂联合优化；Ranger 保留 `vy=0` |
| `base_mode` | `LOCKED`（1） | MPC 求解约束 `vx=vy=wz=0`，MPC 底盘输出为零速 |
| `tcp_cost_scale` | 默认 `1.0` | `.info` 中末端中间/终端代价的倍率，范围 0.1～10 |
| `input_cost_scale` | 默认 `1.0` | `.info` 中输入代价的倍率，范围 0.1～10 |

这些倍率不是速度或加速度限制。有限运动仍使用 Action 的 `speed_scale`；
机器人模型、维度、硬件接口和求解器结构仍在初始化时读取。
默认配置保留原来的权重和联合运动行为，固定机械臂模型不会把前三个臂关节误当底盘锁定。

控制器需已激活、反馈有效且无正在执行的 Action。配置请求会清除旧目标并保持输出，
以实测 TCP 建立保持目标，再把新配置交给求解线程。参考与配置在一次求解内保持一致，
MRT 策略携带对应配置版本；控制线程只有取得匹配版本的策略且反馈有效后才放行。

```bash
ros2 service call /mpc_controller/set_motion_profile zephyr_msgs/srv/SetMotionProfile \
  '{profile: {base_mode: 1, tcp_cost_scale: 1.0, input_cost_scale: 1.0}}'
ros2 topic echo /mpc_controller/motion_profile_status
```

服务返回 `accepted=true` 只表示接收请求。任务层必须等待
`motion_profile_status.applied=true`，并确认 `active_revision` 等于服务返回的
`revision`，才发送新的 `execute_motion` 或连续目标。切换期间普通目标和新 Action 均被拒绝。
恢复联合运动使用同一服务，设置 `base_mode: 0`。切换不会恢复旧的 MPC 目标，
需发送新目标；停用并重新激活 MPC 后保留已选配置，等待本次激活的新策略确认，
不需要重复调用配置服务。旧策略/旧反馈不能跨激活用于放行输出。

`applied` 仅确认 MPC 配置及对应策略已生效，不代表底盘实际停稳或底盘控制权已独占。
这一版不修改底盘驱动，也不引入底盘指令仲裁；`LOCKED` 只约束 MPC 的预测和输出，
其他节点仍可向底盘发送速度。任务层需要协调其他速度源，并通过实际反馈判断停稳。

这一阶段尚未增加连续视觉目标的任务会话、定时底盘目标或巡航约束，
也不改变 `.info` 中的碰撞开关。任务语义与抓放阶段仍由 BT/任务节点负责。

## 预测轨迹与分身的时间对齐

求解工作线程在一次 MPC 求解结束后，通过 OCS2 公共 `getPrimalSolution(getFinalTime(), ...)`
单独取得完整预测时域，并同时保留时间和状态。MRT 的 `solutionTimeWindow` 仍只决定控制策略导出窗口；
例如优化时域为 2 秒、控制导出窗口为 0.2 秒时，显示使用完整 2 秒预测。
完整轨迹复制在求解线程完成，控制线程只交接共享指针；不更改上游 OCS2 代码。

显示只接受与当前 MRT 策略起始时间、暂停/恢复版本号相符的预测。
每个显示周期都以当前 ROS 时间重新采样：第 k 个分身对应 `现在 + k × prediction_ghost_interval`，
过去的点不显示，超过优化时域末端的点不外推。
单臂 mock 配置 1 秒时域、0.2 秒间隔和最多 5 个分身；实际数量受剩余预测时域限制。
Z2 的 `z2_bringup/config/ocs2/mpc_{mock,real}.info` 配置 5 秒时域；
Z2 mock 的分身间隔为 1 秒、最多 5 个，真机仍关闭预测显示。
预测终点是当前有限时域末端，不保证已经到达远处目标。
蓝色 TCP 路径从最新实测 TCP 连向剩余预测轨迹，橙色段表示下一控制周期的预测位置。
路径最多做 64 次预测 FK，分身在相邻预测状态之间按绝对时间插值。

| 控制器参数 | 默认值 | 作用 |
| --- | --- | --- |
| `prediction_publish_hz` | `20.0` | 显示工作线程刷新频率（1～60 Hz），独立于 MPC 求解频率 |
| `prediction_ghost_interval` | `0.2` | 相邻未来姿态的时间间隔，单位秒 |
| `prediction_ghost_count` | `5` | 最多显示的未来姿态数，实际还受剩余时域限制 |
| `prediction_max_age` | `0.5` | 预测起点或实测观测落后当前时间超过此秒数时清除显示；预测还检查导出后的本机经过时间 |

以上参数在初始化时读取，放在控制器 YAML 的 `ros__parameters` 中，修改后重新启动/加载控制器。
这些参数不改变 MPC 权重、求解频率、TCP 偏移或安装变换。

单臂预测跨度在 `zephyr_arm_description/config/ocs2/task.info` 的 `mpc.timeHorizon` 中设置，
分身数量和间隔在 `zephyr_arm_bringup/config/mpc_mock_controllers.yaml` 中设置。
仅增加分身上限不会延长预测时间；需同步增加真实优化时域。
延长 `timeHorizon` 会增加求解计算量，并可能改变 MPC 规划的动作。

两色轨迹和分身的 ADD 标记都有有限生存期：`max(0.15, 3 / prediction_publish_hz)` 秒。
暂停、预测过期或分身数量减少时也主动发送 DELETE，避免旧分身卡住；
即使发布进程停止，RViz 也会在生存期结束后移除标记（需其 ROS 时间正常推进）。
单臂 `mpc_mock.rviz` 对 `/trajectory_markers` 使用 Depth=1、Best Effort，
尽量显示最新帧；发布端保留 Reliable，兼容已有的 Reliable 订阅配置。
显示刷新率不代表 MPC 求解频率，也不保证远程 RViz 的实际渲染帧率。

## 独立控制预测显示

控制器默认 `mock_debug_enabled: false`、`prediction_visualization_enabled: false`、
`traj_record_enabled: false`。普通/真机配置不加载预测分身、不导出预测显示快照、不录制 CSV；
此时 YAML 强行开启预测/录制会拒绝初始化，在线开启录制也会明确拒绝。
`mock_debug_enabled` 是不可在线修改的配置许可，不是硬件探测或切换开关；不要在真机配置中开启。
单臂专用 `mpc_mock.launch.py` 会检查模拟硬件插件，配套 mock YAML 才开启该许可；Z2 已移除独立 mock 启动，由 `mock.launch.py load_mpc_controller:=true` 在代码中为 mock 开启。

`prediction_visualization_enabled` 同时控制两色预测轨迹与预测机器人分身，
独立于 `show_rviz`。关闭后不加载分身的 visual mesh，不获取预测轨迹快照、
不计算/发布预测轨迹和分身；TCP 当前位姿、目标管理、已配置的自碰撞检查和可选 CSV 记录继续工作。
若碰撞检查和记录也未开启，显示工作线程空闲，不再创建观测显示快照。
该参数在启动时生效，修改后需重新加载控制器。

单臂与 Z2 mock 默认开启预测显示，但仍默认关闭录制。如需关闭显示，修改
`zephyr_arm_bringup/config/mpc_mock_controllers.yaml` 中的该参数并重新启动。
`show_rviz:=false` 只关闭窗口，不修改这个控制器参数。
单机械臂入口生成临时的规划 URDF 和控制器参数文件，退出时清理；
不覆盖正常控制器 YAML，也不修改上游包。

仅在 mock 中需要录制时：

```bash
ros2 param set /mpc_controller traj_record_dir /tmp/traj_record
ros2 param set /mpc_controller traj_record_enabled true
# 完成采集后关闭录制，将缓存写入 CSV。
ros2 param set /mpc_controller traj_record_enabled false
```

文件为 `real_tcp.csv`（反馈 FK）和 `pred_tcp.csv`（控制采样预测 FK），不包含输入目标轨迹。
写入失败会报告错误并停止录制，不会让控制器进程退出；该次 CSV 可能缺失或不完整。


## 生命周期、停止与故障

`src/FSM` 和对应头文件已经删除，直接 OCS2 执行已合入 `src/mpc_controller.cpp`。
激活时启动后台求解线程；完整观测及首个有效碰撞检查到达后，以实测 TCP 初始化目标。
等待参考重置确认和第一条策略期间保持位置。
controller 生命周期是唯一启用入口，不再提供 `/fsm_state` 或 `~/pause`。
`~/motion_status` 是从执行结果派生的状态反馈，不是第二个 enable 开关。
无效观测、停止请求、碰撞监控故障、策略失效、参考处理和求解异常均停止运动输出。
普通停止不锁存：有限 Action 使用 cancel；连续 Pose/Twist/Path 或任意当前运动使用 `~/stop`：

```bash
ros2 service call /mpc_controller/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /mpc_controller/motion_status
```

Trigger 的 `success=true` 仅表示受理，不表示已经物理停止。状态先进入 STOPPING，
控制循环确认保持指令/底盘零速输出后，丢弃旧参考并重新准备至 IDLE。
此后可发送新目标，无需 resume；停止服务会中止当前 Action，Action cancel 则返回 CANCELED。
停止确认等待期间拒绝新输入；旧目标不会在恢复就绪或重连 RViz 后自动重发。
停止参考重置前等待旧求解完成，避免并发 reset/solve。
连续发送器应先停止发布，再调用 `~/stop`；否则重新 IDLE 后到达的新消息仍会被执行。
该服务不持久禁用控制器，也不是急停或遥控断链保护。

除临时无效观测外，求解/策略/参考/碰撞故障仍锁存为 FAULT；新目标和 `~/stop`
不会清错。先排查原因，再通过 controller_manager 明确停用/重新激活 MPC；
不得自动重试。真机必须先确认保持和控制链交接条件，不能把停用等同于安全失能。
停用控制器时先发保持/底盘零速并停止接受目标，再 join 求解线程、显示线程，最后释放硬件接口；
重新激活后自动启动。
退出整个 launch 前，先停止上游输入、完成 `~/stop` 确认及控制器交接。
直接 Ctrl+C 可能先使 ROS 通信上下文失效，导致底盘保持无法确认；
此时保留告警，不将“进程已退出”当作物理停稳证明。

RViz 统一订阅 `/mpc_controller/motion_status`（可用 `motion_status_topic` 参数改路径）。
未就绪时小球仍可预览拖动，但不发送；面板显示不可用原因并禁用发送按钮。
只有 IDLE、无故障、反馈有效且没有 Action 所有者时才允许发送。
状态超过 1 秒未更新会禁用发送，恢复状态不会自动发送预览目标。
这只是客户端防误操作，不替代控制器校验，也不是遥控断链急停。

### RViz 一键切换 JTC / MPC

`MPC target` 面板顶部显示 controller_manager 中的实际生命周期状态，并提供
“切换到 MPC / 切回 JTC”按钮。确认弹窗后，
以 STRICT、5 秒切换超时调用 `switch_controller`：只激活目标控制器、停用当前
active 的另一个控制器；Position Adapter、MIT、底盘驱动和夹爪不变。
返回 JTC 使用 MPC 的生命周期退出保持流程，不另加内部 enable/FSM。

先停止 BT/Servo/其他目标发送器，确认机械臂已停稳、周围安全后再点击。
按钮不发送 ready/home 姿态，不自动加载或配置控制器，不抢占其他控制器的接口，
也不自动重试、回滚或清故障。切换期间本 RViz 禁止发送目标，服务返回后再次读取
实际状态；MPC 激活后仍需等新策略就绪。网络超时不代表机器人未执行切换。
该按钮不是物理急停或底盘控制仲裁，真机保持与平滑交接仍需实机验收。

RViz 节点可配置以下参数（不改变机器人端配置）：

| 参数 | 默认值 |
| --- | --- |
| `controller_manager_name` | `/controller_manager` |
| `mpc_controller_name` | `mpc_controller` |
| `trajectory_controller_name` | `zephyr_arm_trajectory_controller` |

自定义 MPC 名称时也应设置对应的 `motion_status_topic`。独立单臂 `mpc_mock.launch.py`
默认只加载 MPC、不加载 JTC，因此切换按钮禁用并提示缺少控制器；Z2 普通 bringup
已加载这两个控制器，可直接使用。不需要为此改动 MPC 控制算法。

## 通用有限运动接口

`~/execute_motion` 使用 `zephyr_msgs/action/ExecuteCartesianMotion`，支持：

- `POSE`（0）：`target` 指定配置 TCP 的目标位姿。
- `RELATIVE`（1）：`relative` 指定相对实测 TCP 的平移和旋转向量；单位 m/rad。
- `PATH`（2）：`waypoints` 指定位姿序列（1～1000 个）；逐段插值，不继承点的时间作为轨迹时序。
- `CONTINUOUS_PATH`（3）：平滑穿过中间位姿，仅最终目标要求停稳，限制见“连续通过参考帧”。

每个动作提供 `speed_scale`、位置/旋转容差、`settle_time` 和 `timeout`。
`speed_scale` 是配置参考速度、加速度、jerk 上限的比例，不改变底层关节硬限位，
也不保证实测 TCP 速度严格等于参考速度。有限动作不沿用 `movel_duration` 的最短总时长，
而是根据当前反馈、路径距离与参考限值计算时长。`.info` 和模型参数仍在初始化时读取。

`timeout` 默认 **0.0**：按已生成参考的时长 `T` 自动计算有限截止时间
`T + settle_time + max(5 s, 0.25 × T)`，避免长路径被固定 30/40 秒误截断。
这不是无限等待；不可达、长期受阻或未停稳仍会超时。显式正数仍是从 Action 受理开始
计算的严格墙钟截止时间，**不会自动延长**。因此长距离命令请省略 timeout 或明确写
`timeout: 0.0`；原命令里的 `timeout: 40.0` 仍会在 40 秒停止。
负数、NaN 和 Inf 拒绝。日志显示参考时长、实际截止时间及 automatic/explicit。
旧编译客户端可能仍使用原默认 30 秒，需重新构建或显式发送 0；没有增加消息字段。

长路径不会自动提高移动速度，也不会关闭关节限位、速度/加速度/jerk、反馈新鲜度、
求解故障和已配置的碰撞保护。仍保留 1000 个途经点、采样内存预算及旋转向量分支限制；
超大任务需切成合适的参考段，当前尚未实现活动 Action 的在线拼接。

2026-11-18 长路径 mock 回归：约 1.25 m、一个途经 pose 加一个终点 pose，保持
原 0.05 m/s 参考速度上限，自动截止时间 59.08 s，参考时长 46.94 s，Action 成功。
TCP 同时间参考误差 RMS 2.12 mm、最大 5.82 mm，途经点实测速度约 0.046 m/s；
底盘移动约 1.13 m，并与机械臂同时运动。14 个测试程序、121 项当前用例通过。
这些为 GenericSystem mock 数据，不代表真机精度或避障验收。

位姿非空 frame 按其时间戳转换，零时间戳表示最新 TF；空 frame 使用 MPC 参考系。
移动底盘通常为 `odom`，固定底座为模型 base frame。相对动作的 TCP frame 表示
实测工具轴；其他 frame 只用于表达平移/旋转向量。目标在接收时固定，
不是持续视觉跟踪入口。超过采样预算、无效四元数或不可用 TF 会失败。

单个 Action 独占参考：并发 Action 被拒绝，运行期间普通 Pose、PoseStamped、
relative、twist 和 path topic 输入均忽略。成功后恢复原有 topic 输入；
多阶段 BT 应避免阶段之间混入外部手动目标。既有 topic 的空闲行为保持兼容。

成功要求参考轨迹结束，并且实测 TCP 位置/姿态误差及全身速度连续达标达到 `settle_time`。
输入可达性、避障和实际任务成功不由这一到位判据保证。超时、反馈失效、停止、
停用或求解错误使任务失败，不自动重试，重新就绪也不会继续之前的 Action。

取消/失败先请求控制循环保持关节指令、底盘输出零速度，收到停止确认后才返回终态。
该确认表示控制循环已执行保持，不表示已经由轮速/力传感器确认物理静止。
如果控制循环不再响应，`STOP_UNCONFIRMED` 会出现在状态中，Action 保持 STOPPING
并保留所有权；不能把取消请求的响应当成已停止。后续有效停止确认或控制器停用后
才能释放。取消后控制器从当前实测状态重新准备，旧参考不会恢复。

`~/motion_status` 使用 `zephyr_msgs/msg/ControllerMotionStatus`，含状态、原因、
Action UUID、实测 TCP、反馈有效性及误差。状态区分 INACTIVE、INITIALIZING、IDLE、
EXECUTING、SETTLING、STOPPING、FAULT。消息中历史 PAUSED/PAUSE_REQUESTED 常量暂保留兼容，
本控制器不再使用，不修改消息字段或枚举编号；旧 `/fsm_state` 客户端必须同步升级。
反馈快照超过 200 ms 不用于任务完成判断。具体求解/碰撞异常细节仍可查控制器日志。

动作协议处理、TF、插值和结果发布在 ROS executor；控制线程只写固定大小的反馈快照
并处理停止请求，不执行 Action 回调或等待客户端。

示例（明确选择移动模型后才执行；不会切换控制器或打开碰撞检查）：

```bash
ros2 action send_goal /mpc_controller/execute_motion \
  zephyr_msgs/action/ExecuteCartesianMotion \
  '{mode: 1, relative: {header: {frame_id: omni_picker_tcp}, twist: {linear: {z: 0.01}}}, speed_scale: 0.2, timeout: 15.0}' --feedback
ros2 topic echo /mpc_controller/motion_status
```

底盘固定通过前述 `~/set_motion_profile` 在动作开始前配置。MoveIt 场景检查、
动态 `.info` 切换和连续视觉目标任务会话留待后续实现。
`ArmTapBT` 在任务层组合接近/按压/退回；控制器中没有按钮专用阶段和业务判断。

### 有限运动的停稳判定

以上四种有限 Action 共用此判据，不新增控制 FSM 或 enable 开关。TCP 到达但底盘、
关节仍在运动时不会提前返回成功。仅在最终参考结束后检查，中间参考点不因此减速。
`SETTLING` 表示几何误差已经达标、正在等待速度及持续时间；速度未达标或无效会清零
持续时间计数，并在 `motion_status.message` 中给出速度和有效标志。

| 初始化参数 | 默认值 | 含义 |
| --- | --- | --- |
| `settle_tcp_linear_velocity` | `0.005` m/s | 实测 TCP 平移速度上限 |
| `settle_tcp_angular_velocity` | `0.03` rad/s | 实测 TCP 角速度上限 |
| `settle_joint_velocity` | `0.03` rad/s | 机械臂关节运动速度上限 |
| `settle_base_linear_velocity` | `0.01` m/s | 移动底盘平面速度上限 |
| `settle_base_angular_velocity` | `0.03` rad/s | 移动底盘转向速度上限 |

参数放在控制器 `ros__parameters` 下，必须为有限正数，修改需重新加载控制器；
在线改值会拒绝。这些是**到位判定阈值，不是运动限速**，默认值尚未经真机噪声标定。

速度来自实测关节位置、TCP FK 和底盘位置的约 50 ms 累计绝对运动量，不使用命令或
预测速度；关节量累计每周期最大的关节位移，保守覆盖各关节，避免零空间运动漏判。
来回运动不会因净位移为零而抵消。移动模型还取里程计 twist 与位置估计速度中较大者；
里程计时间戳必须有效，与控制器 ROS 时钟相差不超过过去 200 ms / 未来 20 ms。
零时间戳、过期或非有限速度均不能当作静止，直到有效反馈恢复才重新累计停稳时间。
固定底座不要求里程计；激活、时钟跳变或无效观测会清空速度窗口，先等待新测量。

这是有采样/窗口延迟的任务到位判据，不是安全认证的静止检测，也不要求底盘与机械臂
在同一毫秒停止。取消/故障的“保持输出已确认”语义不变，不能将其等同于物理停稳。

2026-11-18 完整 Z2 GenericSystem mock 验证：同一条 TCP +X 25 cm、+Z 6 cm 的
六点连续路径，以参考峰值 0.025 / 0.05 m/s 各往返一次，4 次 Action 均成功。
底盘与机械臂存在同时运动区间，底盘每程约移动 22 cm；TCP 同时间参考误差 RMS
2.22～2.46 mm，最大 6.04 mm，内部途经点没有强制停顿。14 个测试程序、117 项当前用例通过。
未改控制增益/底盘限速；碰撞检查沿用当前关闭配置。此结果不代表真机动力学精度、
避障能力或底盘最终朝向已经满足按电梯任务要求。

### 可选全身参考（剪影，M2 原型）

仍使用 `~/execute_motion` / `ExecuteCartesianMotion`。空的 `whole_body_references`
和默认零权重保持 TCP-only 行为；没有新增控制器、业务 FSM 或 OCS2 fork。
**Action 新增了字段，DDS 消息定义有变化：机器人、RViz/BT 和发布端必须同步构建
`zephyr_msgs` 及依赖它的客户端，旧二进制不保证线协议兼容。**

当前支持 `POSE` 的一个参考，或 `CONTINUOUS_PATH` 中与 `waypoints` 一一对应的参考。
此扩展暂不接受 `PATH` / `RELATIVE`；这两种原有 TCP-only 模式仍可正常使用。

每个 `zephyr_msgs/WholeBodyReference` 包含：

- `base_pose`：`geometry_msgs/PoseStamped`，位置为平面 x/y，姿态仅 yaw；z/roll/pitch 必须为零。
  frame 必须显式等于 MPC 固定参考系（移动模型通常 `odom`），不接受 `base_link` 等移动系。
- `posture`：`sensor_msgs/JointState` 的 name/position，支持机械臂关节子集；所有点名称和顺序一致。
  拒绝重复名、未知关节、非有限值、越模型限位目标以及 velocity/effort。
  这是关节名义姿态偏好，不是独立下发的关节命令。

Action 中 `base_position_weight`、`base_yaw_weight`、`posture_weight` 为全程固定的非负权重
（0 关闭，当前输入范围 0～1000）；内部新增运行及终端状态代价
`0.5 * Σ weight * error²`，yaw 误差取最短角差，有限行程关节不做 2π 环绕。
固定底座模型支持 posture，但拒绝底盘参考。没有配置的维度保持自由，不自动锁住整机。

底盘位置／yaw 启用后，成功判定还要求最终实测误差分别小于
`base_position_tolerance`（默认 0.02 m）／`base_yaw_tolerance`（默认 0.03 rad）。
Action feedback/result 返回 `base_position_error`、`base_yaw_error`，禁用项为零。
posture 默认只作为软偏好，不要求逐关节精确到位；TCP 误差、实测速度、持续时间、取消和超时判据不变。
权重是加权折中，不是严格优先级，也不是碰撞或可达性保证。当前实现的是底盘**点参考加到位容差**，
不是“区域内部零代价”的区域约束；不一致的 TCP/底盘/姿态输入可能无法满足完成条件。

参考从同一实测 TCP／整机状态快照开始。TCP 与全身参考共用全局五次时间进度，
底盘／关节使用保形 C1 插值，单调经过的中间点不强制停止；反向或极值处允许减速。
`whole_body_max_joint_velocity` 是名义关节参考速度上限（初始化参数，默认 0.2 rad/s；Z2 mock 0.8）。
底盘参考速度使用 `base_max_linear_speed` / `base_max_angular_speed`，乘以 `speed_scale`；
旧配置若将输出限速设为 0，新底盘参考分别采用保守的 0.1 m/s / 0.2 rad/s；不改变旧输出语义。
必要时整条 TCP 与全身参考一起延长，保持关键帧同时到达。它不保证全身加速度／jerk 有界，
也不表示实际运动严格按参考时刻到达。全身扩展的总参考时长上限为 1 小时。

完整参考在 executor 构造，TCP 和全身快照在同一求解边界交换，整个 solve 内不可变。
停止、重置、profile hold 或新 TCP-only 目标会清除旧的全身代价；成功后保持最后参考。
全身轨迹不是在线流式接口，不能在活动 Action 中用新目标或 profile service 偷换任务。
运行时阶段门槛、按压授权、参考在线更新和自动剪影生成由后续任务层实现。

mock-only 调试输出：`/tcp_reference_path` 和 `/base_reference_path` 为带时间戳、transient-local
的 `nav_msgs/Path`；后者包括名义底盘姿态（零权重维度不代表实际被约束）。旧参考被清除时发布空路径。
`~/solve_time_ms` 为 `std_msgs/Float64`，由求解线程发布每次 solve 加可选预测导出的耗时，
不在 ros2_control 更新循环发布；真机默认不创建这些调试话题。

完整 Z2 mock 运行及 CSV 示例见 `z2_bringup/README.md` 的全身参考测试段。

2026-11-18 两次干净启动各完成 5 项验证（10/10 成功）：同一 TCP 下改变底盘 yaw，
以及底盘 60 cm、机械臂 J2/J3 同时伸展的往返。后者 TCP 同时间参考误差 RMS
1.04～1.16 mm，最终底盘误差 1.17～3.49 mm；原停稳门槛未放宽。
这是关闭碰撞检查的 GenericSystem 结果，不代表真机精度。
5 s horizon 下，部分 **TCP-only** 长路径仍会因残余运动停稳超时；新增参考能力
没有宣称解决该基线问题。需单独继续诊断，不以修改到位判据替代修复。
