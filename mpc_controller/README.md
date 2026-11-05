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
| `src/control/TcpTrajectoryRecorder.cpp` | 实测/预测 TCP 的缓冲与 CSV 写出 |
| `src/control/StagedMotion.cpp` | 机械臂优先、底盘接近和停稳后的任务切换 |
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
目标输入和服务均复用标准 ROS 类型，不依赖 `zephyr_msgs`。
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
`zephyr_arm_bringup/config/mpc_mock_controllers.yaml`。本次没有改 Z2 启动链；
当前工作区未提供可直接启动本控制器的 Z2 MPC launch，下文底盘部分说明的是代码能力，
不代表已经完成整机接线或验证。

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
ros2 topic echo /fsm_state --once
```

控制器类型应为 `mpc_controller/mpc_controller`；生命周期 `active` 不代表 MPC
未暂停，需要同时查看 `/fsm_state` 和诊断日志。TCP 指令与暂停接口见下文。

### 当前 mock 验证状态（2026-11-05）

控制器核心与 RViz 插件重新编译通过，链接和导出的构建依赖不再包含 `zephyr_msgs`。
65 项 C++ 测试及 19 项 bringup Python 测试通过，覆盖标准路径输入、无效位姿拒绝、
暂停后忽略新目标、RViz 位姿流和非阻塞参考交接；不改求解器、控制增益及原保护逻辑。

隔离 headless mock 中三个控制器 active、`/fsm_state=3`：
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
不能直接用于本节的固定底座 mock，也不表示已有 Z2 真机启动入口。

按底盘驱动能力选择 profile，而不是按真机/mock 区分。Z2 的全向 swerve 控制器使用
`base_command_profile: omni`，允许带符号的 `vx/vy/wz`，支持倒退、横移、斜移和旋转。
只有不支持横移与旋转同时执行的旧后端才选择 `ranger`。启用分阶段控制时，该 profile 在接近和协同阶段
约束横向速度为零，使用前后移动、原地转向或弧线转向，匹配该旧后端的命令组合。
未启用分阶段控制时，原输出适配仍在横移命令存在时抑制旋转。
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

连续遥操或圆轨迹使用 `/tcp_target`（`geometry_msgs/Pose`）位姿流，坐标必须是 MPC 基坐标系。
该入口直接更新参考，不经过上述路径插值/限速；发送端负责连续性和目标速度，
控制器原有关节约束及保护仍生效。不要同时运行多个目标发送器。
配套 mock 画圆示例位于 `zephyr_arm_bringup/scripts/mpc_circle_demo.py`：

```bash
# 已启动 mpc_mock.launch.py，FSM=3；期间不要拖动 RViz 小球。
ros2 run zephyr_arm_bringup mpc_circle_demo.py --plane yz --radius 0.02 --duration 10
```

此示例只允许 GenericSystem，保持当前 TCP 姿态，绕一圈返回起点后保持 3 秒并退出。
普通 topic 没有接收确认、到位反馈或目标取消语义；通过实际 TCP/FSM 判断跟踪情况。
暂停时忽略新目标。停止发送只会保留最后目标，暂停跟踪请调用控制器的 `~/pause`。

### 服务迁移与 FK/IK 约束

| 服务 | 当前类型 | 迁移要求 |
| --- | --- | --- |
| `~/pause` | `std_srvs/srv/SetBool` | 不变 |
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
该超时依据本机接收时间；当前未额外判断消息源时间戳。首次激活时关节反馈无效会拒绝激活。
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
- 求解、参考处理和上述策略/碰撞故障必须排查后显式调用 `~/pause {data: false}` 恢复。
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

## 机械臂优先与底盘接近

移动底盘模型可以启用 `staged_motion_enabled`（单臂 mock 关闭）。
MPC 根据目标到机械臂 `base_link` 原点的三维距离分配运动：

- 目标在完整操作半径 `R` 内时，先由机械臂跟踪，不以停车半径限制正常跟踪。
- 距离超过 `R + reach_switch_margin` 后，底盘平移接近；停车目标设为约 `0.65 × R`，
  到停车点 2.5 cm 内才停止接近，不会刚进入最大伸展半径就停车。
  目标高度较高时，停车距离适当增大以保留水平距离；这不是底盘碰撞避障距离。
- 底盘接近期间允许平移和转向，使车身正前方逐渐朝向目标，机械臂保持进入该阶段时的关节姿态。
  停车位置会补偿机械臂安装偏移随车身旋转的变化；位置误差与朝向误差都满足容差后才停车。
  朝向采用最短角度差，跨越 ±π 时不会绕远路。
- 若设置 `base_approach_keep_heading: true`，接近阶段改为保持开始接近时的车头朝向，
  停车点仍沿机械臂安装点到目标的方向选择，不把平移方向绑定到车头方向。
  配合 `omni` 可直接倒退、横移和斜移；同一目标下不会因轻微朝向漂移反复重设目标朝向。
  这只约束接近阶段，后续 `whole_body_tracking` 仍可由 MPC 联合优化旋转与机械臂。
- TCP 位置误差大于 2 cm 或姿态误差大于 0.12 rad，且综合误差连续 2 秒未改善至少 3 mm，
  自动进入 `whole_body_tracking`：恢复 TCP 代价，释放底盘及机械臂输入，由 MPC 协同跟踪。
  停滞判断使用实测关节状态的 FK；综合误差为 `hypot(位置误差, 0.1 × 姿态误差)`。
  目标位置相对检测起点变化超过 3 cm，或姿态变化超过 0.15 rad，会重新计时。
- 协同阶段到达上述误差阈值后进入停稳等待；若连续 4 秒也没有进一步改善，结束该次协同尝试，
  同一目标不会反复启动底盘。更换目标或误差重新收敛后允许下一次尝试。
- 停车或协同结束后，里程计平移速度低于 0.01 m/s、角速度低于 0.02 rad/s，
  且连续保持 0.25 秒，才恢复单臂 TCP 跟踪。新选的范围内目标会取消旧停车任务并等待停稳。

`R` 在控制器初始化时从当前规划 URDF 估算：在六个关节限位内做 4096 个确定性采样，
再局部细化最大 TCP 距离。计算采用当前 `omni_picker_tcp` 和实际安装变换。
这是位置包络估计，不能保证范围内任意末端姿态均可达；可用 `arm_reach_radius`（米）覆盖。
若目标高度本身超出整个半径，则保持位置并报告 `unreachable_height`，新目标回到可达高度后自动恢复。

移动底盘接线时可在控制器参数中配置下列值，修改后重新加载控制器；
示例不代表当前 Z2 仓库的默认参数或真机验收结果：

| 参数 | 示例配置 | 说明 |
| --- | --- | --- |
| `staged_motion_enabled` | `true` | 启用分阶段控制；单机械臂默认关闭 |
| `arm_workspace_frame` | `base_link` | 位于所有机械臂活动关节之前的安装坐标系 |
| `arm_reach_radius` | `0.0` | 0 表示按 URDF 估算，正数指定半径（米） |
| `reach_switch_margin` | `0.02` | 超出半径多少米才启动远目标接近 |
| `base_parking_radius_ratio` | `0.65` | 远目标停车半径比例，不限制单臂跟踪范围 |
| `base_parking_tolerance` | `0.025` | 底盘到停车点的距离容差，米 |
| `base_heading_tolerance` | `0.08` | 车身停车朝向容差，弧度，约 4.6° |
| `base_command_profile` | `omni` | 全向 swerve；`ranger` 是横移/旋转互斥的旧后端约束 |
| `base_approach_keep_heading` | `true` | 接近阶段保持入口朝向；默认 false 保留朝向目标的旧行为 |
| `tracking_stall_seconds` | `2.0` | 跟踪无明显进展后允许底盘协助的等待时间，秒 |
| `tracking_position_tolerance` | `0.02` | 停滞判断的位置误差阈值，米 |
| `tracking_orientation_tolerance` | `0.12` | 停滞判断的姿态误差阈值，弧度 |
| `tracking_progress_threshold` | `0.003` | 重置停滞计时所需的综合误差改善量，米 |
| `assist_no_progress_seconds` | `4.0` | 协同阶段无进一步改善时结束本次尝试，秒 |
| `base_settle_seconds` | `0.25` | 底盘连续停稳的等待时间 |
| `base_max_linear_speed` | `0.15` | 底盘平移合速度上限，m/s |
| `base_max_angular_speed` | `0.3` | 旋转速度上限，rad/s；真机值需单独验证 |

本地 `control/StagedMotion.cpp` 在上游问题的副本中添加输入等式约束。
接近阶段暂停 TCP 代价，使用底盘位置与朝向代价，并以关节速度等式约束锁住机械臂。
全向底盘的平移仍由 MPC 优化，转向采用有界朝向反馈，并作为输入等式约束进入预测；
执行端使用同一反馈和实测朝向，避免预测转向与输出限速不一致造成反复修正。
朝向误差在 0.02 rad 内停止修正，外侧平滑增加角速度；停车朝向容差仍为 0.08 rad。
朝向代价为 `0.5 × 1.0 × wrap(yaw - yaw_goal)^2`，位置代价仍为 `0.5 × 50 × ||p - p_goal||²`。
旧后端配置 `base_command_profile: ranger` 在接近与协同阶段额外约束横向速度为零，采用前后移动与转向，
Ranger 接近阶段使用停车点引导：转向速度随朝向误差平滑变化，平移速度随到点距离及转向角减小，
防止受转向限速影响而绕着停车点打圈。该引导作为 MPC 输入等式约束用于预测，输出端执行同一引导；
前进或后退方向在选定停车点时固定，避免经过侧向位置时反复翻转转向要求；朝向偏差超过 90° 时先转向再平移。
接近阶段不再单独优化 Ranger 的速度输入，协同跟踪阶段仍由 MPC 优化前后移动、转向及机械臂。
仅在执行 Ranger 接近任务时允许低于空闲死区的减速命令，到点后由停稳阶段明确发送零速度。
驶入停车容差的 0.6 倍范围后锁住平移、原地完成最终朝向；只有偏离超过容差的 2 倍才重新驶入，
避免停车点附近反复切换驶入与对正。正常停车位置容差不变。
该 profile 的规划和输出都避免同时发送横移与旋转；`omni` 不受此限制。
输出端同步保持进入该阶段时的关节位置，避免旧策略或插值引入关节动作。
停稳后恢复单臂 TCP 跟踪并锁住底盘；协同阶段直接使用原 TCP 代价并开放底盘和机械臂输入。
预测与控制使用同一次求解，分身反映接近阶段的关节保持及协同阶段的跟踪动作。
扩展使用 OCS2 公共接口，无需修改上游目录。

`/mpc_controller/motion_phase` 发布当前阶段：
`arm_tracking`、`base_approach`、`whole_body_tracking`、`base_settling` 或 `unreachable_height`。
暂停/恢复会重置阶段，并以当前 TCP 初始化目标。

## 预测轨迹与分身的时间对齐

求解工作线程在一次 MPC 求解结束后，通过 OCS2 公共 `getPrimalSolution(getFinalTime(), ...)`
单独取得完整预测时域，并同时保留时间和状态。MRT 的 `solutionTimeWindow` 仍只决定控制策略导出窗口；
例如优化时域为 2 秒、控制导出窗口为 0.2 秒时，显示使用完整 2 秒预测。
完整轨迹复制在求解线程完成，控制线程只交接共享指针；不更改上游 OCS2 代码。

显示只接受与当前 MRT 策略起始时间、暂停/恢复版本号相符的预测。
每个显示周期都以当前 ROS 时间重新采样：第 k 个分身对应 `现在 + k × prediction_ghost_interval`，
过去的点不显示，超过优化时域末端的点不外推。
单臂 mock 配置 1 秒时域、0.2 秒间隔和最多 5 个分身；实际数量受剩余预测时域限制。
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
单臂及 Z2 的专用 `mpc_mock.launch.py` 会检查模拟硬件插件，配套 mock YAML 才开启该许可。

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


## 自动运行与暂停

`src/FSM` 和对应头文件已经删除，直接 OCS2 执行已合入 `src/mpc_controller.cpp`。
激活时启动后台求解线程；完整观测及首个有效碰撞检查到达后，以实测 TCP 初始化目标。
等待参考重置确认和第一条策略期间保持位置。
暂停、无效观测、碰撞监控故障、策略失效、参考处理和求解异常均停止运动输出；
除临时无效观测外，上述故障均锁存暂停，需显式恢复。
暂停后恢复前先等待旧求解完成和非实时参考重置确认，再重置 MPC，避免并发 reset/solve。

```bash
ros2 service call /mpc_controller/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /mpc_controller/pause std_srvs/srv/SetBool '{data: false}'
```

单机械臂使用相同的暂停/恢复服务路径。
停用控制器时先发保持/底盘零速并停止接受目标，再 join 求解线程、显示线程，最后释放硬件接口；
重新激活后自动启动。
`/fsm_state` 仅作为原版 RViz/目标管理器的兼容状态发布（3 运行、2 暂停），
控制器不再接收 `/fsm_command`，也不再提供 HOME、MOVEJ、腰部或原状态机动作接口。
本地交互坐标轴直接订阅该状态话题，以支持启动后拖动目标。
