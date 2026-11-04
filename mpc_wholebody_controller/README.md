# mpc_wholebody_controller

Zephyr 机械臂和 Z2 底盘的独立 MPC 控制器，依赖原版 OCS2 和
`arms_ros2_control` 公共库。包名、控制器类名和导出的插件名均为小写。

## 组成与依赖

| 文件 | 功能 |
| --- | --- |
| `src/mpc_wholebody_controller.cpp` | 生命周期、求解线程、暂停以及单 TCP FK/IK 服务 |
| `src/control/CtrlComponent.cpp` | TCP 目标与参考轨迹、实测状态送入 MPC、执行求解结果、底盘限速/死区与可选 CSV 记录 |
| `src/control/StagedMotion.cpp` | 机械臂优先、底盘接近和停稳后的任务切换 |
| `src/visualization/mpc_visualizer.cpp` | TCP 位姿、两色轨迹、分身及其时间对齐 |
| `plugin/tcp_rviz.cpp` | 可拖动的单 TCP 坐标轴、坐标输入、完整位姿输入和误差显示 |

控制模块的声明放在同名头文件中。`TcpReferenceManager` 类的声明与实现已合入
`CtrlComponent.h/.cpp`，保留独立类职责；`mpc_visualizer.h/.cpp` 继续独立。
交互坐标轴由 RViz 面板共享，使用 RViz 已有的 ROS 节点。
三个 RViz 界面类在 `plugin/tcp_rviz.hpp` 中声明，在 `plugin/tcp_rviz.cpp` 中实现；Qt 元对象代码由 AUTOMOC 自动生成。
TCP 接入 MPC 的主要链路为：
`tcp_target/stamped → TcpReferenceManager → OCS2 参考轨迹 → 求解器 → CtrlComponent → 关节与底盘命令`。

MPC 求解器与优化模型使用未修改的 `ocs2_mobile_manipulator`、`ocs2_ddp`、`ocs2_mpc`。
调度与公共控制接口继续调用 `ocs2_controller_common` 和 `arms_controller_common`。
本地只实现单 TCP 接口及 Zephyr 的观测、执行和显示逻辑。
代码来源、迁移基线和维护边界见 [UPSTREAM.md](UPSTREAM.md)。

## 注册与兼容

控制器插件类型为 `mpc_wholebody_controller/mpc_wholebody_controller`，
C++ 类为 `zephyr::mpc_wholebody::mpc_wholebody_controller`。
目标面板插件为 `mpc_wholebody_controller/target_coordinate_panel`。

Z2 与单机械臂入口均使用实例名 `mpc_wholebody_controller`，只加载一个 MPC 控制器。
RViz 的本地 TCP 面板直接连接 TCP 话题，不再使用控制器别名或左右臂重映射。

Z2 的参数仍在 `z2_bringup/config/controllers.yaml`，URDF、任务权重和限制仍在
`z2_description`；固定底座机械臂的参数在 `zephyr_arm_description`。
两者的控制器插件类型都已切换到本包。夹爪桥接继续使用
`zephyr_arm_bringup/gripper_rviz_bridge`。

## 编译

在 **VSCode 的 zephyr_dev Dev Container 终端** 执行：

```bash
cd /workspaces/zephyr-dev/zephyr_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build \
  --base-paths src/ocs2/arms_ros2_control/libraries/ocs2_controller_common \
               src/ocs2/arms_ros2_control/controller/ocs2_arm_controller \
               src/ocs2/arms_ros2_control/command/arms_rviz_control_plugin \
               src/zephyr_common/zephyr_controllers/mpc_wholebody_controller \
               src/zephyr_z2/z2_description src/zephyr_z2/z2_bringup \
               src/zephyr_common/zephyr_arm_ros2/zephyr_arm_description \
               src/zephyr_common/zephyr_arm_ros2/zephyr_arm_bringup \
  --packages-select ocs2_controller_common ocs2_arm_controller \
                    arms_rviz_control_plugin mpc_wholebody_controller \
                    z2_description z2_bringup \
                    zephyr_arm_description zephyr_arm_bringup --symlink-install
source install/setup.bash
```

以上命令用于其他依赖已经构建好的工作区，包含本次迁移需重新构建的上游库、
原控制器、RViz 插件及 Zephyr 包，避免旧的定制二进制掩盖源码恢复结果。

## 启动

同一个 Dev Container 终端中：

```bash
export ROS_DOMAIN_ID=12
ros2 launch z2_bringup MPCmockdemo.launch.py
```

仅机械臂的现有入口仍为：

```bash
ros2 launch zephyr_arm_bringup MPCmockdemo.launch.py
```

`ros2 control list_controllers` 中，Z2 和单机械臂的实例均为 `mpc_wholebody_controller`，类型应为
`mpc_wholebody_controller/mpc_wholebody_controller`。
控制器激活后自动开始 OCS2；RViz 保留夹爪、目标坐标、跟踪误差和预测显示，移除了原 FSM 面板。

## 后续更新上游

保持 `src/ocs2/arms_ros2_control` 工作树干净，在该仓库更新到所需上游版本后，
重新构建变动的公共库和本包。算法库中的兼容优化可以直接复用；若公共 API 改变，
在本包做适配。迁入的控制器胶水代码需要按 UPSTREAM.md 中的来源表对比上游变化。

迁移前备份（不在 colcon 源码扫描范围内）：
`/home/mscape/zephyr_migration_backups/20261026T073726Z`。
其中包含来源提交、完整差异补丁和逐文件校验后的源文件归档。

## 无界面运行与 Ranger 真机输出

Z2 的启动参数以 `z2_bringup/README.md` 为准。
`MPCmockdemo.launch.py show_rviz:=false` 可关闭窗口，`rviz.launch.py` 可独立打开显示。

真机配置设置 `base_command_profile: ranger`。启用分阶段控制时，MPC 在接近和协同阶段
约束横向速度为零，使用前后移动、原地转向或弧线转向，匹配 Ranger 支持的命令组合。
未启用分阶段控制时，原输出适配仍在横移命令存在时抑制旋转。
`base_max_linear_speed`、`base_max_angular_speed` 分别限制平面速度和角速度；0 表示不额外限幅。
默认 `omni` 保留仿真的全向平移和旋转。这些设置在初始化时读取。

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
| 单 TCP 圆弧服务 | `/execute_tcp_circle` |

`tcp_ee_frame` 是本包的末端配置参数，默认读取 task.info 的 eeFrame。
当前两个 Zephyr 模型的真实 TF 末端仍为 `omni_picker_tcp`，本次仅统一控制接口命名，
保留实际安装几何和 142 mm TCP 偏移。修改末端帧参数时需重新加载控制器。
本地目标管理、FK/IK、交互坐标轴、位姿面板和误差订阅均直接使用单 TCP 接口。
不再声明 `left_ee_frame`，不再重映射任何左右臂话题，也不再安装
`tcp_compatibility.h` 或 `zephyr_mpc_launch.tcp_interfaces`。

- `CtrlComponent.cpp` 中的 `TcpReferenceManager` 直接连接 OCS2 的 `ReferenceManagerInterface`，处理绝对位姿、
  带坐标系位姿、速度流、相对位移和路径；坐标转换失败会拒绝目标。
- `tcp_rviz.cpp` 中的 `TcpTargetManager` 创建名称为实际 TCP 帧的六自由度交互坐标轴，
  支持连续拖动和右键手动发送。XYZ 和完整位姿两个面板在同一 RViz 节点中共享一个服务端。
  不再安装或启动独立的 `tcp_target_manager` 可执行程序。现有 InteractiveMarkers 显示器的
  Namespace 仍为 `/tcp_target_manager`，不需要修改 RViz 配置。
  关闭全部目标输入面板或退出 RViz 后，交互服务端随之销毁；控制器继续接受其他程序的 TCP 命令。
  `/tcp_current_target` 使用 transient-local 缓存，后打开的界面可获得最近一次目标。
- `tcp_pose_panel` 提供完整位置/四元数输入和相对位姿命令；`target_coordinate_panel` 保留 XYZ 快捷输入。
- `tcp_error_display` 直接订阅 TCP 当前位姿与目标，复用上游误差图形绘制组件，保留误差数值和趋势图。
- FK/IK 服务继续使用 `arms_ros2_control_msgs/srv/KinematicsService` 消息格式，
  其中 `arm_type` 填 `omni_picker_tcp`、`tcp` 或留空；不会转换为上游机械臂选择字符串。
  支持本地 DLS、BFGS 和 AUTO 求解，使用当前模型的关节限位。
- CSV 记录文件为 `real_tcp.csv`、`pred_tcp.csv`，没有左右臂分支。

路径可发布到 `/target_path`（`nav_msgs/Path`），也可调用 `/execute_path`。
该服务现使用本包的 `mpc_wholebody_controller/srv/ExecuteTcpPath`：请求字段为 `path`
和 `trajectory_duration`，旧的双臂 `ExecutePath` 请求需要迁移。
`/execute_tcp_circle` 沿用上游 `ExecuteCircle` 消息定义以复用传输格式，
`circle_params.arm_name` 填实际 TCP 帧、`tcp` 或留空；只处理单 TCP 几何字段。
路径插值使用五次时间曲线和四元数球面插值，并根据速度、加速度及 jerk 配置延长段时长。

运动学服务的机械臂求解器使用独立的降维模型：锁定底盘的虚拟自由度，
并把参考原点移到 task.info 指定的 baseFrame，避免把六维关节向量传给九维移动模型。
这只影响机械臂 IK/FK 服务，MPC 保留完整底盘与机械臂状态。服务求解异常会返回失败响应。


## 观测、线程和参数更新

控制线程先读取完整的一组关节位置和底盘里程计，全部有效才提交观测。
关节读取失败、NaN/Inf、里程计接收超时（500 ms）、无效四元数或
`header.frame_id` 与 `base_odom_frame` 不一致时，保留上一组有效观测，
暂停 MPC 执行，保持最近一次关节位置命令，并向底盘发送零速度。
该超时依据本机接收时间；当前未额外判断消息源时间戳。首次激活时关节反馈无效会拒绝激活。
正常反馈恢复后从当前实测 TCP 位姿重新开始 MPC。此处检查的是控制器使用的位置反馈，不代替硬件故障诊断。

实时 TCP FK 使用 `rt_data_`；预测、轨迹记录使用显示工作线程自己的 `viz_data_`。
轨迹清理也交给该工作线程，避免控制线程修改它正在使用的缓存。
控制线程每周期只交接最新实测观测与不可变预测快照的指针；显示线程按独立频率运行，
不积压中间帧。退出 MPC 时通过版本号丢弃旧轨迹，防止分身重新出现。
TCP 发布仍由 `dds_publish_hz` 控制，不再随 MPC/分身刷新周期改变。

| 参数类别 | 更新规则 |
| --- | --- |
| `hardware_latency` | 可在线修改；必须是有限、非负的浮点秒数，以原子变量传给实时线程 |
| `rt_timing_enabled`、`traj_record_enabled` | 可在线修改，只接受布尔值 |
| `traj_record_dir` | 非空字符串；更换目录前先停止记录，下次开启记录时使用新目录 |
| 模型/接口、底盘话题与坐标系、底盘限速/死区、线程配置、MPC/DDS 频率、预测显示开关、观测缓存与未来采样配置 | 只在初始化读取；在线更改会返回失败，修改 YAML/launch 后须重新加载控制器 |

`hardware_latency` 等动态设置在整批参数通过所有校验后才应用；拒绝的参数批次不会修改本包内部配置。
本包已移除 HOME、HOLD、MOVEJ 状态类和对应动作；OCS2 参考轨迹管理仍调用上游公共库。

## 机械臂优先与底盘接近

Z2 默认启用 `staged_motion_enabled`。MPC 根据目标到机械臂 `base_link` 原点的三维距离分配运动：

- 目标在完整操作半径 `R` 内时，先由机械臂跟踪，不以停车半径限制正常跟踪。
- 距离超过 `R + reach_switch_margin` 后，底盘平移接近；停车目标设为约 `0.65 × R`，
  到停车点 2.5 cm 内才停止接近，不会刚进入最大伸展半径就停车。
  目标高度较高时，停车距离适当增大以保留水平距离；这不是底盘碰撞避障距离。
- 底盘接近期间允许平移和转向，使车身正前方逐渐朝向目标，机械臂保持进入该阶段时的关节姿态。
  停车位置会补偿机械臂安装偏移随车身旋转的变化；位置误差与朝向误差都满足容差后才停车。
  朝向采用最短角度差，跨越 ±π 时不会绕远路。
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

参数位于 Z2 的 `controllers.yaml`，修改后重新加载控制器：

| 参数 | Z2 配置 | 说明 |
| --- | --- | --- |
| `staged_motion_enabled` | `true` | 启用分阶段控制；单机械臂默认关闭 |
| `arm_workspace_frame` | `base_link` | 位于所有机械臂活动关节之前的安装坐标系 |
| `arm_reach_radius` | `0.0` | 0 表示按 URDF 估算，正数指定半径（米） |
| `reach_switch_margin` | `0.02` | 超出半径多少米才启动远目标接近 |
| `base_parking_radius_ratio` | `0.65` | 远目标停车半径比例，不限制单臂跟踪范围 |
| `base_parking_tolerance` | `0.025` | 底盘到停车点的距离容差，米 |
| `base_heading_tolerance` | `0.08` | 车身停车朝向容差，弧度，约 4.6° |
| `tracking_stall_seconds` | `2.0` | 跟踪无明显进展后允许底盘协助的等待时间，秒 |
| `tracking_position_tolerance` | `0.02` | 停滞判断的位置误差阈值，米 |
| `tracking_orientation_tolerance` | `0.12` | 停滞判断的姿态误差阈值，弧度 |
| `tracking_progress_threshold` | `0.003` | 重置停滞计时所需的综合误差改善量，米 |
| `assist_no_progress_seconds` | `4.0` | 协同阶段无进一步改善时结束本次尝试，秒 |
| `base_settle_seconds` | `0.25` | 底盘连续停稳的等待时间 |
| `base_max_linear_speed` | `0.15` | 底盘平移合速度上限，m/s |
| `base_max_angular_speed` | `0.3` | 仿真旋转速度上限，rad/s；真机覆盖为 0.2 |

本地 `control/StagedMotion.cpp` 在上游问题的副本中添加输入等式约束。
接近阶段暂停 TCP 代价，使用底盘位置与朝向代价，并以关节速度等式约束锁住机械臂。
全向底盘的平移仍由 MPC 优化，转向采用有界朝向反馈，并作为输入等式约束进入预测；
执行端使用同一反馈和实测朝向，避免预测转向与输出限速不一致造成反复修正。
朝向误差在 0.02 rad 内停止修正，外侧平滑增加角速度；停车朝向容差仍为 0.08 rad。
朝向代价为 `0.5 × 1.0 × wrap(yaw - yaw_goal)^2`，位置代价仍为 `0.5 × 50 × ||p - p_goal||²`。
真机 `base_command_profile: ranger` 在接近与协同阶段额外约束横向速度为零，采用前后移动与转向，
Ranger 接近阶段使用停车点引导：转向速度随朝向误差平滑变化，平移速度随到点距离及转向角减小，
防止受转向限速影响而绕着停车点打圈。该引导作为 MPC 输入等式约束用于预测，输出端执行同一引导；
前进或后退方向在选定停车点时固定，避免经过侧向位置时反复翻转转向要求；朝向偏差超过 90° 时先转向再平移。
接近阶段不再单独优化 Ranger 的速度输入，协同跟踪阶段仍由 MPC 优化前后移动、转向及机械臂。
仅在执行 Ranger 接近任务时允许低于空闲死区的减速命令，到点后由停稳阶段明确发送零速度。
驶入停车容差的 0.6 倍范围后锁住平移、原地完成最终朝向；只有偏离超过容差的 2 倍才重新驶入，
避免停车点附近反复切换驶入与对正。正常停车位置容差不变。
规划和输出都避免向 Ranger 驱动同时发送横移与旋转；全向仿真仍可横移。
输出端同步保持进入该阶段时的关节位置，避免旧策略或插值引入关节动作。
停稳后恢复单臂 TCP 跟踪并锁住底盘；协同阶段直接使用原 TCP 代价并开放底盘和机械臂输入。
预测与控制使用同一次求解，分身反映接近阶段的关节保持及协同阶段的跟踪动作。
扩展使用 OCS2 公共接口，无需修改上游目录。

`/mpc_wholebody_controller/motion_phase` 发布当前阶段：
`arm_tracking`、`base_approach`、`whole_body_tracking`、`base_settling` 或 `unreachable_height`。
暂停/恢复会重置阶段，并以当前 TCP 初始化目标。

## 预测轨迹与分身的时间对齐

求解工作线程在一次 MPC 求解结束后，通过 OCS2 公共 `getPrimalSolution(getFinalTime(), ...)`
单独取得完整预测时域，并同时保留时间和状态。MRT 的 `solutionTimeWindow` 仍只决定控制策略导出窗口；
例如 Z2 中优化时域为 2 秒、控制导出窗口为 0.2 秒，显示使用完整 2 秒预测。
完整轨迹复制在求解线程完成，控制线程只交接共享指针；不更改上游 OCS2 代码。

显示只接受与当前 MRT 策略起始时间、暂停/恢复版本号相符的预测。
每个显示周期都以当前 ROS 时间重新采样：第 k 个分身对应 `现在 + k × prediction_ghost_interval`，
过去的点不显示，超过优化时域末端的点不外推。Z2 在自己的控制器 YAML 中设置最多 6 个分身，
间隔为 0.3 秒，对应当前时刻之后 0.3、0.6、0.9、1.2、1.5、1.8 秒；实际数量受剩余预测时域限制。
单机械臂保留 1 秒时域、0.2 秒间隔和最多 5 个分身，通常可见 4 个。
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

Z2 的预测跨度在 `z2_description/config/ocs2/task.info` 的 `mpc.timeHorizon` 中设置，
分身数量和间隔在 `z2_bringup/config/controllers.yaml` 的 `mpc_wholebody_controller.ros__parameters` 中设置。
仅增加分身上限不会延长预测时间；需同步增加真实优化时域。
延长 `timeHorizon` 会增加求解计算量，并可能改变 MPC 规划的动作；本次 Z2 从 1 秒增加到 2 秒，
20 Hz 显示刷新、控制导出窗口、求解请求频率和过期清理保持原配置。

两色轨迹和分身的 ADD 标记都有有限生存期：`max(0.15, 3 / prediction_publish_hz)` 秒。
暂停、预测过期或分身数量减少时也主动发送 DELETE，避免旧分身卡住；
即使发布进程停止，RViz 也会在生存期结束后移除标记（需其 ROS 时间正常推进）。
随包提供的两个 RViz 配置对 `/trajectory_markers` 使用 Depth=1、Best Effort，
尽量显示最新帧；发布端保留 Reliable，兼容已有的 Reliable 订阅配置。
显示刷新率不代表 MPC 求解频率，也不保证远程 RViz 的实际渲染帧率。

## 独立控制预测显示

`prediction_visualization_enabled` 同时控制两色预测轨迹与预测机器人分身，
独立于 `show_rviz`。关闭后不加载分身的 visual mesh，不获取预测轨迹快照、
不计算/发布预测轨迹和分身；TCP 当前位姿、目标管理、已配置的自碰撞检查和可选 CSV 记录继续工作。
若碰撞检查和记录也未开启，显示工作线程空闲，不再创建观测显示快照。
该参数在启动时生效，修改后需重新加载控制器。

Z2 和单机械臂的 `MPCmockdemo.launch.py` 默认开启预测显示；
Z2 总入口的真机模式默认关闭，设置 `enable_mpc:=true` 选择 MPC。可在现有启动命令后追加：

```bash
# 无窗口，也不计算预测显示
ros2 launch z2_bringup MPCmockdemo.launch.py show_rviz:=false prediction_visualization_enabled:=false

# 机器人端无窗口，仍向另一台电脑上的 RViz 提供预测显示数据
ros2 launch z2_bringup z2_launch_all.launch.py enable_mpc:=true show_rviz:=false prediction_visualization_enabled:=true

# 固定底座机械臂同样支持独立开关
ros2 launch zephyr_arm_bringup MPCmockdemo.launch.py show_rviz:=false prediction_visualization_enabled:=false
```

单机械臂入口直接生成本地控制器临时参数文件，退出时清理。
`robot_profile/control.patch/mpc_wholebody_controller/ros__parameters` 可覆盖 MPC 参数。
无需更改 `src/ocs2`。


## 自动运行与暂停

`src/FSM` 和对应头文件已经删除，直接 OCS2 执行已合入 `src/mpc_wholebody_controller.cpp`。
激活时启动后台求解线程；首个完整观测到达后以实测 TCP 初始化目标，等待第一条策略期间保持位置。
暂停、无效观测、自碰撞和求解异常均停止运动输出；自碰撞和求解异常需显式恢复。
暂停后恢复前先等待旧求解完成，再从当前 TCP 重置 MPC，避免并发 reset/solve。

```bash
ros2 service call /mpc_wholebody_controller/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /mpc_wholebody_controller/pause std_srvs/srv/SetBool '{data: false}'
```

单机械臂使用相同的暂停/恢复服务路径。
停用控制器时先停止并 join 求解线程、显示线程，再释放硬件接口；重新激活后自动启动。
`/fsm_state` 仅作为原版 RViz/目标管理器的兼容状态发布（3 运行、2 暂停），
控制器不再接收 `/fsm_command`，也不再提供 HOME、MOVEJ、腰部或原状态机动作接口。
本地交互坐标轴直接订阅该状态话题，以支持启动后拖动目标。
