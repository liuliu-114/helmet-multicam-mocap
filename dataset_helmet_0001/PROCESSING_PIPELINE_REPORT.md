# `dataset_helmet_0001` 端到端处理流程与运行报告

> 项目：`/home/wy/mobile_mocap`  
> 数据集：`/home/wy/mobile_mocap/dataset_helmet_0001`  
> 正式输出：`/home/wy/mobile_mocap/dataset_helmet_0001/algorithm_output`  
> 唯一处理入口：`scripts/evaluate_dataset_metrics.py`  
> 分析与复跑日期：2026-07-12（Asia/Shanghai）

## 1. 怎么运行

生成当前正式输出的命令是：

```bash
cd /home/wy/mobile_mocap
/usr/bin/python3 scripts/evaluate_dataset_metrics.py dataset_helmet_0001
```

等价的完整参数写法：

```bash
/usr/bin/python3 scripts/evaluate_dataset_metrics.py \
  dataset_helmet_0001 \
  --match-radius 8 \
  --min-area 2 \
  --max-area 500 \
  --output dataset_helmet_0001/algorithm_output
```

调试时建议使用新目录，避免覆盖正式结果：

```bash
/usr/bin/python3 scripts/evaluate_dataset_metrics.py \
  dataset_helmet_0001 \
  --output /tmp/helmet_debug_output \
  > /tmp/helmet_metrics_stdout.json
```

脚本会把 `metrics.json` 的同一内容打印到标准输出。输出目录不存在时自动创建；脚本不会先清空已有目录，因此正式实验最好使用空目录。

## 2. 整条数据流

```text
camera.json ---------> 4 个投影矩阵 P = K * T_world_to_camera
rigid_bodies.json ---> H01-H06 的头盔局部坐标
scene_config.json ---> 选择 helmet_01、读取 FPS/时长

360 张 PNG
  -> BGR 转 HSV
  -> 两段红色阈值
  -> 3x3 闭运算
  -> 连通域和面积过滤
  -> 无身份的 2D 红点候选

gt_2d_frames.json + 2D 候选
  -> Hungarian 一对一分配
  -> 8 px 门限
  -> 带 H01-H06 身份的 2D 观察
  -> detections_2d.json

同帧、同 Marker、至少 2 台相机的 2D 观察
  -> 多视图线性 DLT
  -> 世界坐标 3D Marker
  -> 与 gt_3d_frames.json 比较
  -> reconstructed_markers_3d.json

同帧至少 3 个重建 Marker + 头盔局部点模板
  -> Kabsch/SVD 无尺度刚体拟合
  -> 头盔 R、t
  -> 与 object_pose_frames.json 比较
  -> estimated_object_pose.json

累计全部误差和覆盖率
  -> metrics.json

帧 0/45/89 的原图 + GT + 检测
  -> overlays/*.png（4 相机 x 3 帧 = 12 张）
```

这是一条离线评估管线，不是完全自主的生产跟踪管线。红色检测本身不读 GT，但检测后的 H01-H06 身份由 GT 2D 位置辅助分配。

## 3. 运行环境和参数

实际复跑环境：Python 3.8.10、OpenCV 4.2.0、NumPy 1.17.4、SciPy 1.3.3。

脚本依赖：

```text
argparse, json, pathlib
cv2, numpy
scipy.optimize.linear_sum_assignment
scipy.spatial.transform.Rotation
```

脚本兼容旧 SciPy：有 `Rotation.from_matrix` 时使用它，否则使用 SciPy 1.3.3 的 `Rotation.from_dcm`。

命令行参数：

| 参数 | 默认值 | 影响 |
|---|---:|---|
| `dataset` | `dataset_rigidbody_0001` | 数据集目录；本任务必须显式传头盔数据集 |
| `--match-radius` | 8.0 px | GT 与检测质心的最大接受距离 |
| `--min-area` | 2 px | 红色连通域最小面积 |
| `--max-area` | 500 px | 红色连通域最大面积 |
| `--output` | `<dataset>/algorithm_output` | 输出目录 |

## 4. 真正读取和不读取的文件

### 4.1 运行必需输入

| 输入 | 读取字段 | 用途 |
|---|---|---|
| `camera.json` | K、世界到 OpenCV 相机变换、相机位置 | 投影、三角化、距离统计 |
| `rigid_bodies.json` | 刚体 ID、H01-H06 局部坐标 | 位姿拟合模板 |
| `scene_config.json` | 刚体 ID、FPS、时长 | 选择刚体、结果摘要 |
| `gt_2d_frames.json` | 帧号、Marker ID、`pixel_uv` | 遍历帧、GT 基线、检测后 ID 分配、2D 误差 |
| `gt_3d_frames.json` | Marker 世界坐标 | 3D 误差 |
| `object_pose_frames.json` | `T_object_to_world` | 位置和姿态误差 |
| `images/Camera_XX/frame_XXXX.png` | 360 张实际图像 | 红色 Marker 检测 |

### 4.2 不会读取

- `visibility.json`：当前代码以 `gt_2d.pixel_uv is not None` 作为参与条件。
- `videos/*.mp4`、`videos/*.gif`：只读无损 PNG。
- 已有 `algorithm_output/*`：每次运行重新计算，不是输入。
- Markdown 文档：只供人阅读。

所以物理遮挡不会被自动过滤。本数据 2,160 个 GT 2D 点均有非空 `pixel_uv`，即使圆片被头盔挡住，也进入“理论可见”分母。

## 5. 函数地图

| 函数 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `load_json` | 路径 | Python 对象 | UTF-8 读取 |
| `dump_json` | 路径、对象 | JSON 文件 | UTF-8、2 空格缩进写出 |
| `summarize` | 数组 | count/mean/RMSE/std/P95/max | 统一统计 |
| `projection_from_camera` | 相机配置 | 3x4 P | `K @ T[:3,:]` |
| `project` | P、世界点 | 像素 uv | 重投影 |
| `detect_red_markers` | BGR 图像、面积范围 | 候选、mask | HSV 检测 |
| `match_detections` | GT、候选、门限 | 匹配、未匹配、误差 | Hungarian ID 分配 |
| `triangulate_multiview` | 多相机 uv、P | 世界 3D 点 | 线性 DLT |
| `best_rigid_transform` | 局部点、世界点 | R、t | Kabsch/SVD |
| `evaluate_observations` | 2D 观察、相机、GT | 指标、3D、位姿 | 共用评估核心 |
| `main` | CLI | 4 JSON、12 PNG、stdout | 组织全流程 |

## 6. 步骤 1：加载静态数据

`main()` 首先读取 6 个 JSON，并从 `scene_config.json` 得到 `rigid_body_id=helmet_01`。然后在 `rigid_bodies.json` 中找到对应刚体，构建：

```text
local_points = {
  H01: [-4.829,  9.003, 8.200] cm,
  H02: [ 4.342,  9.366, 8.051] cm,
  H03: [-12.141,-0.832, 3.304] cm,
  H04: [12.649,  2.225, 2.844] cm,
  H05: [ 4.776,-18.094, 3.448] cm,
  H06: [-4.365,-7.865, 9.008] cm
}
```

输入：`scene_config.json`、`rigid_bodies.json`。  
内存输出：刚体模板 `local_points`。  
意义：后面用它把重建 3D 点集合变成头盔 6DoF 位姿。

## 7. 步骤 2：构建四相机投影矩阵

对每台相机：

```python
P = K @ T_world_to_camera_opencv[:3, :]
```

P 为 3x4，接收 UE 世界坐标厘米，产生齐次像素。四台相机放入 `projections[camera_id]`。

输入：`camera.json/cameras[]`。  
内存输出：4 个 3x4 投影矩阵。  
后续消费者：GT 投影验证、DLT 三角化、重投影误差。

## 8. 步骤 3：按 90 个同步帧读 360 张 PNG

外层循环遍历 `gt_2d_frames.json/frames`；内层遍历该帧四台相机。路径由帧号构造：

```text
images/<camera_id>/frame_<frame_index:04d>.png
```

例如第 45 帧 Camera_03：

```text
dataset_helmet_0001/images/Camera_03/frame_0045.png
```

OpenCV 用 `IMREAD_COLOR` 读取为 720x1280x3 BGR。读取失败立即抛出异常，不会跳帧。

输入：GT 帧号和 PNG。  
内存输出：当前 BGR 图像。  
数量：90 帧 x 4 相机 = 360 张。

## 9. 步骤 4：建立理想 GT 观察分支

每个相机帧先把所有 `pixel_uv` 非空的 GT 转成：

```text
{H01: [u,v], H02: [u,v], ..., H06: [u,v]}
```

存入 `gt_observations`。该分支稍后执行：

```text
GT pixel_uv -> 四相机 DLT -> Kabsch
```

它不评价图像检测，只验证相机矩阵、坐标方向、DLT 和 Kabsch 实现是否正确。本数据输入总量为 90 x 4 x 6 = 2,160 个理想 2D 观察。

## 10. 步骤 5：HSV 红点检测

`detect_red_markers()` 执行：

1. BGR 转 HSV。
2. 低端红/橙阈值：H 0-25、S 45-255、V 100-255。
3. 高端红阈值：H 165-179、S 45-255、V 100-255。
4. 两张 mask 按位 OR。
5. 3x3 闭运算，填补小孔并连接近邻红像素。
6. `connectedComponentsWithStats` 提取连通域。
7. 跳过背景，只保留面积 2-500 px 的连通域。

每个候选为：

```json
{"pixel_uv": [centroid_u, centroid_v], "area_px": 连接域面积}
```

此时没有 H01-H06 身份，只知道“这里有红色区域”。全序列共 1,517 个候选：Camera_01/02/03/04 分别 372/443/341/361 个。

输入：当前 BGR 图像、面积参数。  
内存输出：无身份候选列表、二值 mask。  
注意：mask 只在函数内返回，当前主流程不保存 mask。

## 11. 步骤 6：Hungarian GT 辅助 ID 分配

`match_detections()` 先构造 GT 与候选之间的欧氏距离矩阵：

```text
cost[i,j] = ||gt_pixel[i] - detection_centroid[j]||
```

`linear_sum_assignment` 求全局总代价最小的一对一配对，避免一个候选被多个 Marker 占用。随后拒绝距离大于 8 px 的配对。

输出：

```text
matches   = {marker_id: detected_pixel_uv}
unmatched = 未使用 detection 的数组下标
errors    = 成功匹配的像素距离
```

边界必须说清：

- HSV 检测没有使用 GT；
- H01-H06 身份分配使用了 GT；
- 因此后续结果是“真实检测 + GT 辅助对应”的上限评估；
- 它没有验证部署时的自主跨相机 Marker 对应。

全序列 968 个成功匹配、549 个未匹配候选。H05 匹配 0 次，H06 仅 17 次。

## 12. 步骤 7：积累检测输出和绘制叠加图

每个相机帧形成：

```text
detections[]                        所有红色候选
matched_by_gt_for_evaluation[]      带 ID 的成功匹配
unmatched_detection_indices[]       未匹配候选下标
```

同时累积全局、逐相机、逐 Marker 的 GT 数、匹配数、未匹配数和中心误差。

仅帧 0、45、89绘制 overlay：

- 蓝圆/蓝字：GT 中心和 Marker ID；
- 绿十字：全部检测质心；
- 红线：成功匹配的 GT 到检测质心。

输入：原图、GT、候选、匹配。  
磁盘输出：12 张 `overlays/*.png`。  
内存输出：`detections_output`、`image_observations` 和统计数组。

## 13. 步骤 8：形成三套评估输入

图像循环结束后有：

1. `gt_observations`：理想像素和真实 ID；
2. `image_observations`：PNG 检测中心和 GT 辅助 ID；
3. `two_camera_observations`：从第 2 套只保留 Camera_01/02。

三套都调用同一个 `evaluate_observations()`，使用相同 DLT、Kabsch 和误差公式。差异只来自输入 2D 观察范围。

## 14. 步骤 9：按 ID 聚合多相机观察

对每帧每个 H01-H06，收集给出该 ID 的相机：

```text
views = {camera_id: pixel_uv}
```

少于 2 台相机则跳过，因为单视图无法确定 3D 深度。实际图像分支得到 359 个可三角化 Marker 时刻：

| 参与相机数 | 数量 |
|---|---:|
| 2 台 | 152 |
| 3 台 | 182 |
| 4 台 | 25 |

按 ID：H01 89 次，H02/H03/H04 各 90 次，H05/H06 为 0。

## 15. 步骤 10：多视图线性 DLT 三角化

对每个相机观察 `(u,v)` 和投影矩阵 P，堆叠两行：

```text
u * P[2] - P[0]
v * P[2] - P[1]
```

N 台相机得到 `2N x 4` 矩阵 A。SVD 求解 `A X_h = 0`，取 `Vh[-1]`，再齐次除法：

```text
X_world_cm = X_h[:3] / X_h[3]
```

输入：同帧同 ID 的 2-4 个像素、对应 P。  
输出：一个 UE 世界坐标厘米点、`camera_view_count`。  
评估：与 `gt_3d_frames.json` 比较得到毫米误差；重投影到参与相机得到像素误差。

当前 DLT 是无权重线性解，没有检测置信度、RANSAC、异常相机剔除或非线性优化。

## 16. 步骤 11：Kabsch/SVD 恢复头盔位姿

同一帧至少有 3 个已重建 Marker 时，建立对应点：

```text
p_local = rigid_bodies.json 中的局部坐标
p_world = DLT 重建世界坐标
```

求无尺度刚体变换：

```text
p_world ~= R * p_local + t
```

算法过程：两组点分别求质心并去中心，构造协方差，SVD 得到 `R=V U^T`；若 `det(R)<0` 则修正反射；最后 `t=world_center-R*local_center`。

输入：至少 3 个同 ID 局部/世界点。  
输出：3x3 `rotation_matrix`、`location_cm`、本帧 `marker_ids`。  
覆盖：90/90 帧；89 帧用 H01-H04，帧 5 只用 3 点。

## 17. 步骤 12：位置和姿态误差

位置：

```text
d_t_mm = (t_est_cm - T_object_to_world[:3,3]) * 10
position_error_mm = ||d_t_mm||
```

姿态：

```text
R_error = R_est * R_gt^T
orientation_error_deg = ||rotation_vector(R_error)||（度）
```

SO(3) 相对旋转避免了直接相减欧拉角的奇异性和角度缠绕。

统计包括 count、mean、RMSE、总体标准差、P95、max；另计算位置/小角度误差向量的系统偏差，以及去除平均偏差后的三维 RMS。后者只是运动序列精度代理，不是固定姿态重复性。

## 18. 步骤 13：三条结果分支

### 18.1 GT 几何基线

```text
GT pixel_uv -> 四相机 DLT -> Kabsch -> GT
```

恢复 540/540 个 Marker 时刻、90/90 个六点位姿，误差接近浮点舍入。用途是验证坐标和数学实现，不代表图像算法精度。

### 18.2 四相机 PNG 主分支

```text
PNG -> HSV -> GT 辅助 ID -> 四相机 DLT -> Kabsch -> GT
```

恢复 359 个 Marker 时刻、90 个位姿，是 `image_pipeline_four_camera` 主结果。

### 18.3 Camera_01/02 双相机对照

只保留 Camera_01/02 的相同检测，恢复 67 个 Marker 时刻，只有 1 帧有至少 3 个点并形成位姿。它证明该遮挡场景需要四相机覆盖。

## 19. 步骤 14：写最终输出

### `detections_2d.json`（502,552 字节）

```text
frames[90]
  frame_index
  cameras[4]
    detections[]: pixel_uv, area_px
    matched_by_gt_for_evaluation[]: marker_id, pixel_uv
    unmatched_detection_indices[]
```

### `reconstructed_markers_3d.json`（213,468 字节）

```text
gt_pixel_baseline[90]
image_pipeline[90]
  frame_index
  markers[]: marker_id, world_position_cm, camera_view_count
```

### `estimated_object_pose.json`（131,535 字节）

```text
gt_pixel_baseline[90]
image_pipeline[90]
  frame_index
  marker_ids
  location_cm
  rotation_matrix
  position_error_mm
  orientation_error_deg
```

双相机逐帧数据不写入该文件，只在 `metrics.json` 保留汇总。

### `metrics.json`（15,455 字节）

主要键：`protocol`、`dataset_summary`、`detection`、`gt_geometry_baseline`、`image_pipeline_four_camera`、`image_pipeline_project_native_two_camera_scope`、`claim_limits`。

### `overlays/*.png`

Camera_01-04 的帧 0/45/89，共 12 张。仅供人工诊断，不参与后续计算。

## 20. 第 0 帧完整穿透示例

图像检测：

| 相机 | 候选 | 成功匹配 | 未匹配 |
|---|---:|---:|---:|
| Camera_01 | 6 | 3 | 3 |
| Camera_02 | 5 | 4 | 1 |
| Camera_03 | 4 | 3 | 1 |
| Camera_04 | 3 | 3 | 0 |

跨相机聚合后：

| Marker | 相机数 | 重建世界位置 cm |
|---|---:|---|
| H01 | 4 | `[-49.9105,10.4487,158.5594]` |
| H02 | 3 | `[-41.5461,6.0857,158.2141]` |
| H03 | 2 | `[-60.9891,5.4271,153.5194]` |
| H04 | 3 | `[-38.4612,-4.8171,152.6798]` |

H05 没匹配；H06 只在 Camera_02 有观察，均无法三角化。Kabsch 使用 H01-H04：

```text
估计位置 = [-50.34145, 0.07952, 150.15473] cm
GT 位置  = [-50.00000, 0.00000, 150.00000] cm
位置误差  = 3.83218 mm
姿态误差  = 1.22628 deg
```

数量在这一帧中经历：18 个红色候选 -> 13 个带 ID 2D 匹配 -> 4 个多视图 3D 点 -> 1 个头盔位姿。

## 21. 全序列数量如何逐层收缩

```text
输入 PNG                         360 张
理论 GT 相机-Marker             2,160 个
HSV 连通域候选                  1,517 个
GT 辅助成功匹配                 968 个
未匹配候选                      549 个
至少双视图的 3D Marker          359 个
至少三点的头盔位姿              90 帧
```

968 个 2D 匹配不会产生 968 个 3D 点：一个 3D 点需要同帧同 ID 的至少两个相机观察；359 个 3D Marker 再按帧聚合成 90 个位姿。

## 22. 实际复跑与确定性验证

本次在 `/tmp` 新空目录使用相同参数完整复跑：

| 项目 | 结果 |
|---|---:|
| 墙钟时间 | 12.73 s |
| 用户 CPU | 22.87 s |
| 系统 CPU | 6.35 s |
| 峰值 RSS | 150,360 KiB（约 146.8 MiB） |
| 退出码 | 0 |

4 个 JSON 与正式输出逐字节相同，12 张 overlay 也全部逐字节相同。正式 JSON SHA-256：

| 文件 | SHA-256 |
|---|---|
| `detections_2d.json` | `7b2868129e2e0ad30f096bff52d853993a0272d750d762963f2fd820b74b66d8` |
| `reconstructed_markers_3d.json` | `516877af05b921de6e171229e224b29b52eb3ca01fb6342c7d8ad245c2d5c9f4` |
| `estimated_object_pose.json` | `57bfe6d9dcad1d25a999795c0c4f7f20c74707ceb3b9586bd548ca943dc3d4cf` |
| `metrics.json` | `0e7aacb5d2166b7871179dd6dbe479ddc21eded51684a28fdbab29678255495d` |

说明当前环境和参数下处理链是确定性可复现的。

## 23. 推荐的新运行验证流程

```bash
cd /home/wy/mobile_mocap

/usr/bin/python3 -m py_compile scripts/evaluate_dataset_metrics.py

OUT=/tmp/helmet_run_$(date +%Y%m%d_%H%M%S)
/usr/bin/python3 scripts/evaluate_dataset_metrics.py \
  dataset_helmet_0001 --output "$OUT"

find "$OUT" -type f -printf '%P\n' | sort

python3 - <<PY
import json
from pathlib import Path
root = Path("$OUT")
for path in root.glob("*.json"):
    json.loads(path.read_text())
    print("OK", path)
m = json.loads((root / "metrics.json").read_text())
p = m["image_pipeline_four_camera"]
print("pose frames:", p["pose_frames"])
print("position mean mm:", p["position_accuracy"]["error_mm"]["mean"])
print("orientation mean deg:", p["orientation_accuracy"]["geodesic_error_deg"]["mean"])
PY
```

## 24. 参数修改会传播到哪里

| 修改 | 直接影响 | 后续影响 |
|---|---|---|
| `min-area/max-area` | 候选保留 | 匹配、3D、位姿和全部图像指标 |
| `match-radius` | 匹配接受/拒绝 | 2D 数量、三角化/位姿覆盖和误差 |
| HSV 阈值 | 红色 mask | 所有图像分支输出 |
| 形态学核 | 连通域形状/质心 | 2D 中心、3D、位姿 |
| 相机 K/外参 | 投影矩阵 | 投影、三角化、3D/位姿结果 |
| Marker 局部坐标 | Kabsch 模板 | 位姿；不改变检测和 3D 点 |
| 相机子集 | 共同观察数和基线 | 3D/位姿覆盖与精度 |

每组参数应使用独立输出目录。`metrics.json` 记录面积和匹配门限，但不记录 Git commit、依赖版本或运行时间；正式实验最好额外保存这些信息。

## 25. 与真实部署流程的差别

当前离线流程：

```text
PNG + 已知同步 + GT 辅助 ID + 四相机 DLT + Kabsch
```

真实系统还需要：实时采集、相机同步、去畸变、无 GT 的跨相机对应、异常点剔除、可能的非线性优化、时间滤波和 ROS 发布。当前脚本主要回答“合成 PNG 的检测中心经过已知 ID 对应后，几何恢复能达到什么结果”，不能证明完全自主系统能可靠识别每个 Marker。

## 26. 最关键的十点

1. `gt_2d_frames.json` 同时承担帧遍历、GT 基线、ID 归属和 2D 评估。
2. 图像阶段只检测红色连通域，没有 Marker ID。
3. H01-H06 是检测后用 GT/Hungarian 加上的，因此是评估上限。
4. 同一 ID 至少两个相机观察才能产生 3D 点。
5. 同帧至少三个非退化 3D Marker 才能产生头盔位姿。
6. 当前 90 帧位姿主要由 H01-H04 得到，H05/H06 未进入实际 3D 位姿拟合。
7. `visibility.json` 不参与脚本，也没有物理遮挡真值。
8. GT 基线、四相机图像结果和双相机对照必须分开解释。
9. JSON 在全部帧结束后统一写出；中途崩溃可能只有部分 overlay。
10. 当前正式输出已由临时目录复跑逐字节验证，可按本报告命令复现。

