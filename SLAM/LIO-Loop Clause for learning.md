# LIO-Loop Clause for learning

## 1. 主题与目标

本笔记记录 Small Point-LIO 前端、local_map_feedback_node 局部反馈后端、全局 PGO 和地图节点之间的闭环设计。

重点不是"收到 PGO 结果后重建一次 iVox"，而是解决关键帧在 scan-to-map 前后发生位姿修正时的历史一致性问题。必须避免以下状态同时存在：

- ESKF 已经使用修正后位姿；
- 关键帧仍保存修正前 raw pose；
- 旧 iVox 仍由修正前点云组成；
- 新点已经按修正后位姿写入；
- 后台旧 PGO 或旧地图任务又覆盖当前状态。

本设计的目标是让关键帧位姿、tracking iVox、ESKF 状态和 PGO 结果都具有明确的时间边界、版本边界和提交边界。

## 2. 总体闭环流程（文字说明）

前端持续消费 IMU 和 LiDAR。每个 LiDAR packet 在数据真正消费完成后生成一次 correction 证据，同时关键帧桥接节点保存候选关键帧的原始 odom 位姿和 body 坐标系点云。关键帧刚生成时只属于候选状态，不立即视为已经稳定的历史地图帧。

后端接收 correction 和关键帧候选后，按时间将两条异步数据流关联起来。只有当关键帧能够关联到同一个 tracking-map version 下的 correction-free prediction，并且 correction 质量和修正幅度达到触发条件，后端才创建局部 BuildJob。BuildJob 会冻结 correction、tracking map、PGO graph 和运行代次，并复制 active keyframes 与附近 frozen history，保证后台优化看到的是一致快照。

后台只优化 active 尾部关键帧，frozen history 只作为重定位地图几何。优化后的 active 位姿重新投影关键帧点云，生成新的局部 tracking cloud。后端将地图作为 LocalTrackingMap 发给前端，同时把优化后的 active 位姿放入 PendingPoseCommit，但暂不正式覆盖后端的局部位姿缓存。

前端收到地图后先检查 source/target tracking-map version。检查通过后，后台创建新的 iVox；与此同时前端继续处理新 IMU/LiDAR，并把新点写入 tail journal。只有在一个完整 LiDAR packet 结算的安全边界，前端才检查运行代次、地图版本、correction lag 和 journal 覆盖范围。检查通过后，前端把 cutoff 之后的点重放到新 iVox，然后一次性替换 estimator.ivox，并重置 correction-free epoch。

地图切换完成后，前端在下一次 ScanToMapCorrection 中携带新的 tracking-map version 和 active-map source correction sequence。后端把这个版本变化作为隐式 ACK，验证它和 PendingPoseCommit 的 source/target version、source correction sequence 一致后，才提交局部优化后的关键帧位姿。这样，优化位姿和包含这些位姿的 tracking map 共同生效，形成一次完整闭环。

## 3. 这样做的原因

### 3.1 关键帧不能立即成为最终地图

关键帧应经过以下状态：

~~~
candidate
  → active local window
  → local PoseGraph optimized
  → tracking map applied
  → pose commit acknowledged
  → stable local pose
~~~

如果关键帧点云在 scan-to-map 修正前就被永久写入世界坐标地图，而位姿随后又被局部优化修改，就会产生旧位置残留。保存 body 点云、延迟提交并按最新位姿重建，可以避免这种不可逆错误。

### 3.2 地图和 ESKF 必须属于同一个 epoch

tracking_map_version 将前端 iVox 的替换划分为不同 epoch。同一个 epoch 内：

- correction 使用同一版 iVox；
- candidate 的 epoch prediction 来自同一版 iVox；
- active window 只能选择同一版 prediction；
- 新地图声明 source version；
- 地图切换后才进入 target version。

### 3.3 地图和位姿必须事务式提交

LocalTrackingMap 负责前端构建和切换 iVox，PendingPoseCommit 负责后端暂存局部优化位姿。二者都携带 source correction sequence、source/target tracking-map version 和 PGO graph version。

前端没有确认地图切换之前，后端不提交 pose override，避免出现"位姿已更新但 scan-to-map 仍使用旧地图"的中间状态。

### 3.4 后台任务必须可失效

BuildJob 保存 runtime_generation、pgo_graph_version、tracking_map_version 和 source_correction_sequence。任务结束时重新检查这些值，旧 reset、旧 PGO、旧地图和旧 correction 的结果都不能覆盖当前状态。

### 3.5 异步建图期间不能丢实时点

地图构建期间前端仍然处理新点：

~~~
后台：构建新 iVox
前端：继续处理 LiDAR，并记录 tail points
应用：replay cutoff 之后的 tail points
~~~

tail journal 使新地图可以在安全边界补齐实时数据。

## 4. 版本和编号

~~~text
correction_sequence
    前端 packet correction 的运行内编号

point_insertion_sequence
    点进入 tail journal 的顺序编号

tracking_map_version
    前端当前 iVox 的版本

pgo_graph_version
    全局 PGO 优化快照的版本

reset_generation / runtime_generation
    前后端各自的运行代次
~~~

这些编号互相独立。

### 4.1 correction 相关

- CorrectionSnapshot：保存 correction sequence 及该 packet 边界累计修正。
- correction_history：保存近期 CorrectionSnapshot，供地图应用时按 source sequence 查找。
- source_correction_sequence：某张地图构建所依据的 correction 序号。
- active_map_source_correction_sequence：当前前端 iVox 的来源 correction 序号。

### 4.2 tracking-map 相关

- source_tracking_map_version：构建开始时的前端地图版本。
- target_tracking_map_version：地图应用成功后应成为的版本。
- prediction_map_version：关键帧 epoch prediction 所属的地图版本。
- active_tracking_map_version_：后端记录的前端当前地图版本。

必须满足：

~~~text
target_tracking_map_version == source_tracking_map_version + 1
source_tracking_map_version == 当前前端 active version
~~~

### 4.3 PGO 和运行代次

- pgo_graph_version_：后端当前收到的全局 PGO 版本。
- applied_pgo_graph_version_：已经随前端地图切换确认的 PGO 版本。
- pgo_rebuild_pending_：PGO 已更新但局部地图还没有吸收。
- reset_generation：前端 reset 后递增，使旧地图任务失效。
- runtime_generation_：后端 reset 或时间跳变后递增，使旧 BuildJob 失效。

## 5. 前端时间边界和 correction

### 5.1 packet boundary

LiDAR 回调先调用 register_packet_boundary，再交给预处理器。该函数取一次点云回调中所有有限点时间戳的最大值，保存到 packet_end_boundaries。

packet_boundary_ready 同时检查：

1. 稀疏点队列中 boundary 之前的数据已经消费；
2. 稠密点队列中 boundary 之前的数据已经消费；
3. IMU 队首已经推进到 boundary 之后。

finalize_ready_packet_boundaries 只在上述条件成立后调用 finalize_packet。它可以在 handle_once 的数据消费过程中多次尝试，但不代表每次都会完成一个 packet。

### 5.2 三个位姿基线

~~~text
estimator.kf
    IMU + LiDAR 点更新后的正常 ESKF

prediction_only_kf
    当前地图 epoch 以来只接收 IMU

packet_prediction_kf
    上一个 packet 边界以来只接收 IMU
~~~

finalize_packet 中：

~~~text
corrected
    = estimator.kf 当前位姿

epoch_predicted
    = prediction_only_kf 当前位姿

packet_predicted
    = packet_prediction_kf 当前位姿
~~~

对应修正量：

~~~text
packet_correction
    = corrected * packet_predicted.inverse()

cumulative_correction
    = corrected * epoch_predicted.inverse()
~~~

packet correction 用于即时修正和安全判断；cumulative correction 用于累计漂移、地图 rebase 和 lag 检查。

### 5.3 correction 发布

finalize_packet：

1. 递增 correction_sequence；
2. 填充 ScanToMapCorrection；
3. 保存 CorrectionSnapshot；
4. 放入 correction_history；
5. 通过 scan_to_map_correction_callback 发给 ROS 节点和后端。

## 6. 后端的时间关联和触发

correction_callback 将 ROS correction 复制为 CorrectionRecord，检查 frame、时间、位姿和 tracking-map version，然后存入 corrections_。

candidate_callback 保存：

~~~text
candidate.id
candidate.stamp
candidate.raw_pose
candidate.cloud_body
~~~

associate_candidate_prediction_locked 在 correction 缓冲中寻找时间最近的记录，并保存：

~~~text
candidate.epoch_prediction
candidate.prediction_map_version
candidate.has_epoch_prediction
~~~

后续 active window 只接受 prediction_map_version 与最新 correction tracking-map version 相同的关键帧。

schedule_if_needed_locked 的检查顺序：

1. 是否等待上一张地图应用；
2. 等待是否超时；
3. 是否已有后台任务；
4. 是否处于地图应用后的 cooldown；
5. packet/cumulative correction 是否超过安全和触发阈值；
6. correction 质量是否满足要求；
7. 是否有 PGO 重建请求；
8. 是否有足够的同步关键帧。

触发原因：

~~~text
cumulative_scan_correction
instant_scan_correction
pgo
~~~

make_build_job_locked 会冻结：

~~~text
job.correction
job.runtime_generation
job.pgo_graph_version
job.trigger_reason
job.active_frames
job.frozen_history
~~~

active_frames 参与局部优化，frozen_history 只提供附近历史几何。

## 7. PGO、局部优化和地图构建

全局 PGO 成功后，loop detector 递增 pgo_graph_version_ 并发布 OptimizedKeyFrames。optimized_callback 检查版本是否更新，保存 optimized_poses_，判断 PGO 形变是否显著，必要时设置 pgo_rebuild_pending_。

worker_loop 取出 BuildJob 后调用 execute_build_job。

### 7.1 seed_poses_locked：PGO 位姿到 odom 系的映射

make_build_job_locked 在构建 active_frames 之前调用 seed_poses_locked，将 PGO 优化位姿映射到前端 odom 坐标系，作为 active frame 的 initial_pose。

~~~text
1. 在 candidates_ 中找存在于 optimized_poses_ 的帧，取时间戳最大者作为 anchor
2. odom_from_map = anchor.raw_pose * optimized_poses_[anchor.id].inverse()
   将 PGO 系位姿变到 odom 系

3. 对每个 candidate：
   a. 若 candidate ∈ optimized_poses_（PGO 直接优化过）：
      seed = odom_from_map * optimized_poses_[candidate.id]
   b. 若 candidate ∉ optimized_poses_ 但落在两个 PGO 帧之间：
      在前后两个 PGO 帧之间线性插值修正量 correction
      seed = interpolated_correction * candidate.raw_pose
   c. 若无任何 PGO 帧：
      使用 local_pose_overrides_ 或 raw_pose
~~~

这个 seed 就是 active frame 的 initial_pose，后续局部优化和 reanchor 判定都基于它。

### 7.2 pgo_covered：PGO 时间覆盖标记

~~~text
frame.pgo_covered = new_pgo_snapshot
    && isfinite(latest_pgo_stamp)
    && candidate.stamp <= latest_pgo_stamp + epsilon
~~~

latest_pgo_stamp = candidates_ 中存在于 optimized_poses_ 的最大时间戳。

pgo_covered = true 表示该关键帧时间戳落在 PGO 覆盖范围内，即使该帧不在 PGO 图中也能通过插值获得 PGO 修正位姿。未被覆盖的帧（时间戳晚于 PGO 最新帧）用 raw_pose 作为 initial_pose。

### 7.3 PGO 形变两层阈值

第一层：是否触发重建（pgo_deformation_significant_locked）

~~~text
比较历史关键帧在旧/新 PGO 图中的局部位置变化
阈值：pgo_deformation_trigger_translation_m = 0.05m
      pgo_deformation_trigger_rotation_deg  = 0.5deg
任一帧位移 ≥ 阈值 → pgo_rebuild_pending_ = true
~~~

第二层：reanchor 还是 small_correction（在 make_build_job_locked 中）

~~~text
计算 active window 内 PGO 覆盖帧相对于 anchor 的相对位姿变化
阈值：max_local_translation_threshold = 0.30m
      max_local_rotation_deg_threshold = 1.50deg
任一帧相对形变 ≥ 阈值 → pgo_reanchor = true
~~~

两层阈值度量的是局部相对形变，不是全局绝对修正量。PGO 对整条轨迹的刚体变换会被消掉，只有相邻帧之间相对几何发生改变时才触发 reanchor。

### 7.4 optimize_active_window

局部位姿图结构：

~~~text
节点：
  0..N-1: active frame, pose = initial_pose (seed)
  N (终端): pose = corrected, fixed = true (始终)

  第 0 个节点 fixed (fix_first_node = true)
  其余节点 fixed 取决于模式（见 7.5）

边：
  (i-1, i) 帧间边: relative = raw_pose[i-1]⁻¹ × raw_pose[i]
  (N-1, N) 终端边: relative = raw_pose[N-1]⁻¹ × packet_predicted
~~~

终端边测量值使用 packet_prediction_kf（影子 ESKF）的纯 IMU 积分结果，是独立测量。终端节点固定在 corrected（主 ESKF 含点更新后的位姿）。终端边残差编码了 packet 级 scan-to-map 修正量。

帧间边使用 raw_pose（主 ESKF 含点更新后的里程计），不是独立测量，起正则化/平滑作用。

epoch_prediction 不参与位姿图边构造，仅用于 active window 筛选和有限性检查。

局部优化改变顶点估计，不改变原始 odom 测量。优化后 odom edge 存在非零残差是正常的，不应根据优化后位姿重算 raw odom edge。

### 7.5 reanchor vs small_correction 双模式

~~~text
small_correction (pgo_reanchor = false):
  - 所有 active 节点均为自由节点
  - 所有帧间边用 raw_pose 相对位姿
  - 局部优化器完全运行，将 PGO 位姿作为初值与当前里程计平滑混合
  - 适合刚体修正（全局轨迹平移旋转，局部几何不变）

reanchor (pgo_reanchor = true):
  分支 A：所有 active 被 PGO 覆盖
    → 跳过局部优化，直接用 initial_pose (PGO 位姿)
  分支 B：部分 active 未被 PGO 覆盖
    → PGO 覆盖帧 fixed = true，作为固定约束
    → PGO 覆盖帧之间的边用 initial_pose 相对位姿（锁定 PGO 几何）
    → 未覆盖帧自由，用 raw_pose 边
    → 局部优化器运行但 PGO 段被锁定
  - 适合非刚体形变（局部轨迹弯曲，相邻帧相对位置剧变）
~~~

两种模式都会重建并发送局部 tracking cloud 给前端。区别仅在于 active window 位姿的计算策略。

### 7.6 build_tracking_cloud

处理顺序：

~~~text
frozen history 的 body 点云
    → 使用固定 initial pose 变换到 odom

active frame 的 body 点云
    → 使用 optimized pose 变换到 odom

统一体素降采样
    → 生成局部 tracking cloud
~~~

frozen history 先插入，用于保留重定位几何。

### 7.7 execute_build_job

该函数生成两份结果：

- LocalTrackingMap：通过 ROS 发给前端；
- PendingPoseCommit：后端内部暂存的优化位姿。

地图 cutoff 应取实际加入地图的 active/frozen keyframe 最大时间，而不能简单使用 correction 时间。这样 tail replay 的时间边界与地图实际内容一致。

worker 发布前必须检查：

~~~text
job.runtime_generation == runtime_generation_
job.pgo_graph_version == pgo_graph_version_
job.correction.tracking_map_version == active_tracking_map_version_
当前没有等待另一张地图应用
~~~

## 8. 前端地图事务

### 8.1 local_tracking_map_subscription

前端收到 LocalTrackingMap 后：

1. 检查消息和点云 frame；
2. 复制 cutoff、source/target version、PGO version 等元数据；
3. 将外部 ROS 点云坐标转换回 Point-LIO 内部 LiDAR-odom 基准；
4. 调用 queue_local_tracking_map。

map、odom、base_link、LiDAR frame 和 Point-LIO 内部 iVox 基准不能重复应用同一个 correction。

### 8.2 queue_local_tracking_map

前端检查：

~~~text
cutoff_timestamp 有效
points_odom 非空
target == source + 1
source == 当前 active tracking_map_version
~~~

通过后创建 MapBuildRequest，并保存当前 reset_generation。若已有排队任务，只保留 source correction sequence 更新的地图。

### 8.3 map_builder_loop

后台创建新的 SmallIVox，加入 points_odom，检查体素数量和 reset_generation，然后将结果保存为 BuiltMapUpdate，再放入 built_map_update。

后台线程不直接替换当前 estimator.ivox。

### 8.4 try_apply_built_map

该函数在 packet boundary 调用，依次检查：

1. reset_generation；
2. source tracking-map version；
3. target tracking-map version；
4. source correction 是否仍在 correction_history；
5. correction lag 是否超过阈值；
6. tail journal 是否覆盖 cutoff；
7. replay cutoff 之后的新点；
8. 替换 estimator.ivox；
9. 更新 active_map_source_correction_sequence；
10. 写入消息提供的 target_tracking_map_version；
11. 调用 reset_prediction_epoch。

这里不是对 tracking_map_version 做简单加一，而是写入后端消息中的目标版本。

**PGO 触发的地图跳过 lag 检查**：若 trigger_reason == "pgo"，则跳过第 4 步（source correction 序列查找）和第 5 步（correction lag 检查）。因为 PGO 重建基于全局优化结果而非单次 correction 快照，lag 判定不适用。非 PGO 触发（normal/instant）仍需完整检查。

lag 阈值参数：

~~~text
local_map_max_apply_lag_translation_m = 0.10
local_map_max_apply_lag_rotation_deg  = 1.0
~~~

## 9. 隐式 ACK 和位姿提交

当前系统没有独立的地图 ACK topic。

前端完成地图切换后，在下一次 ScanToMapCorrection 中带出：

~~~text
新的 tracking_map_version
新的 active_map_source_correction_sequence
~~~

后端发现版本升高后调用 acknowledge_map_version_locked。

只有以下条件全部满足，才提交 PendingPoseCommit：

~~~text
active_version == pending.target_tracking_map_version
pending.source_tracking_map_version + 1 == active_version
active_map_source_correction_sequence
    == pending.source_correction_sequence
~~~

确认成功后：

- 将优化后的 active poses 写入 local_pose_overrides_；
- 更新 applied_pgo_graph_version_；
- 清理 awaiting 状态；
- 允许后续 BuildJob 使用这些局部位姿。

这一步完成前端地图和后端位姿的闭环同步。

## 10. 完整闭环数据流

~~~text
LiDAR packet
    ↓
finalize_packet()
    ↓
correction_sequence++
    ↓
ScanToMapCorrection
    ↓
correction_callback()
    ↓
CorrectionRecord
    ↓
associate_candidate_predictions_locked()
    ↓
schedule_if_needed_locked()
    ↓
BuildJob
    ├── source correction sequence
    ├── tracking-map version
    ├── PGO graph version
    └── runtime generation
    ↓
seed_poses_locked()
    ├── PGO 直接命中 → odom_from_map × PGO 位姿
    └── 未命中 → 插值修正量 × raw_pose
    ↓
optimize_active_window()
    ├── small_correction: 全自由节点 + raw_pose 边
    └── reanchor: PGO 帧固定 + initial_pose 边
    ↓
build_tracking_cloud()
    ↓
LocalTrackingMap + PendingPoseCommit
    ↓
local_tracking_map_subscription
    ↓
queue_local_tracking_map()
    ↓
map_builder_loop()
    ↓
try_apply_built_map()
    ├── PGO 触发: 跳过 lag 检查
    └── 其他触发: 完整 lag 检查
    ↓
estimator.ivox 安全替换
    ↓
下一条 correction 带出新 tracking-map version
    ↓
acknowledge_map_version_locked()
    ↓
提交 PendingPoseCommit
~~~

## 11. 关键不变量和常见坑

### 11.1 版本不能混用

~~~text
correction_sequence       事件顺序
tracking_map_version      前端地图 epoch
pgo_graph_version         全局 PGO 快照
reset/runtime_generation  运行生命周期
~~~

### 11.2 不要重算 raw odom edge

raw odom edge 是前端测量，局部优化后的位姿是估计结果。二者允许有非零残差。用优化后位姿重算 edge 会丢失测量语义并重复使用地图信息。

### 11.3 不要在 packet 中途切换地图

地图和 ESKF 状态必须在 packet boundary 统一切换，避免一个 packet 前后使用不同 iVox。

### 11.4 不要在地图未确认前提交位姿

PendingPoseCommit 必须等待前端通过新 correction 报告目标地图版本。

### 11.5 cutoff 必须对应地图实际内容

当前地图由关键帧组成，所以 cutoff 应覆盖实际进入地图的最大关键帧时间。cutoff 过早会重复 replay，cutoff 过晚会漏点。

### 11.6 tail journal 可能失效

如果建图耗时过长，journal 可能已经裁剪掉 cutoff 之前的数据。此时地图不能强行应用，应丢弃并等待下一次重建。

### 11.7 旧 PGO 和旧 worker 结果必须丢弃

worker 开始和结束都需要检查版本。任何新 PGO、新地图或新运行代次出现后，旧任务不能覆盖当前状态。

### 11.8 active 和 frozen 不能混淆

~~~text
active_frames   参与局部优化，也用于重建地图
frozen_history  不参与本轮优化，只提供附近历史几何
~~~

### 11.9 全局地图和 tracking iVox 不是同一个对象

全局地图是由关键帧局部点云和 PGO 位姿重建的地图视图；tracking iVox 是前端实时 scan-to-map 使用的局部地图。二者都应保留局部点云和位姿元数据，不能只保存不可回溯的世界坐标点块。

### 11.10 reanchor 判定度量局部相对形变，不是全局绝对修正

reanchor 阈值（0.30m / 1.50deg）度量的是 active window 内相邻关键帧相对于 anchor 的相对位姿变化。全局刚体修正（map→odom 跳变）会被消掉。只有 PGO 导致局部非刚体弯曲时才触发 reanchor。

### 11.11 终端边是驱动约束，帧间边是正则化

终端边使用 packet_prediction_kf（纯 IMU 积分，独立测量），编码 packet 级 scan-to-map 修正量。帧间边使用 raw_pose（主 ESKF 含点更新，非独立），起平滑/正则化作用。epoch_predicted 不参与位姿图边构造。

### 11.12 PGO 触发的地图跳过 lag 检查

trigger_reason == "pgo" 时，try_apply_built_map 跳过 source correction 序列查找和 lag 阈值检查。非 PGO 触发仍需完整检查。这是因为 PGO 重建基于全局优化而非单次 correction 快照。

## 12. 最终结论

前后端闭环可以概括为：

~~~text
候选关键帧
    → 关联同一地图 epoch 的 correction
    → seed_poses 将 PGO 位姿映射到 odom 系（直接命中或插值）
    → 判定 reanchor 还是 small_correction（局部相对形变阈值）
    → 局部 active window 优化（双模式：自由/固定）
    → 生成版本化 tracking map
    → 前端异步构建并在 packet boundary 应用
    → PGO 触发跳过 lag 检查，其他触发检查 lag
    → replay 建图期间产生的新点
    → 下一条 correction 报告地图已切换
    → 后端提交优化后的关键帧位姿
~~~

核心原则是：

~~~text
位姿优化结果不能脱离地图版本单独生效
地图不能脱离 correction/PGO 版本单独生效
关键帧不能在未稳定前永久写入最终地图
旧异步任务不能覆盖新状态
reanchor 判定基于局部相对形变，不是全局绝对修正量
终端边是驱动约束（影子 ESKF），帧间边是正则化（主 ESKF 里程计）
~~~

这样才能避免关键帧在 scan-to-map 前后发生位姿改变时，前端 ESKF、iVox、局部优化和全局地图之间出现历史一致性断裂。
