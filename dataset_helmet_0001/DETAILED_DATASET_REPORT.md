# `dataset_helmet_0001` 数据集详解与数据字典

> 分析对象：`/home/wy/mobile_mocap/dataset_helmet_0001`  
> 分析日期：2026-07-12（Asia/Shanghai）  
> 目的：让第一次接触该数据集的人能够理解每类文件、每个字段、坐标关系、时间关系、算法输出及使用边界。

## 1. 一句话理解这个数据集

这是一个由 Unreal Engine 5 生成的、四台同步理想针孔相机拍摄的头盔刚体运动序列。头盔上定义了 6 个红色圆片 Marker（H01-H06），序列包含 90 个同步时刻、每个时刻 4 张 1280x720 图像，并附带相机标定、头盔 6DoF 位姿、Marker 3D 真值、Marker 2D 投影真值和基础可见性真值。

它适合做：

- 多相机同步读取和数据管线联调；
- 红色 Marker 检测；
- 多视图三角化；
- 已知刚体几何下的 6DoF 位姿恢复；
- 2D、3D、位置和姿态误差评估。

它不等同于真实红外反光动捕数据，因为没有真实镜头畸变、传感器噪声、运动模糊、滚动快门、曝光波动，也没有物理遮挡标签。

## 2. 数据流全景

```text
rigid_bodies.json                     camera.json
  6 个 Marker 的头盔局部坐标             K、外参、坐标约定
          |                                  |
          +----------+-----------------------+
                     |
object_pose_frames.json (每帧头盔位姿)
                     |
                     +--> gt_3d_frames.json (每帧 Marker 世界坐标)
                                  |
                                  +--> gt_2d_frames.json (每相机投影)
                                  |          |
                                  |          +--> visibility.json
                                  |
images/Camera_XX/frame_XXXX.png ---+--> 图像检测/三角化/位姿估计
                                                 |
                                                 +--> algorithm_output/
```

最重要的同步键是 `frame_index`。相同帧号的四张 PNG、四类逐帧 JSON 记录代表同一仿真时刻。

## 3. 当前目录的完整构成

加入本报告后目录共 395 个文件、9 个目录，磁盘占用约 411 MiB；报告生成前为 394 个文件：

| 类型 | 数量 | 角色 |
|---|---:|---|
| 原始相机 PNG | 360 | 4 相机 x 90 帧，主要算法输入 |
| 算法叠加 PNG | 12 | 4 相机 x 帧 0/45/89，诊断输出 |
| MP4 | 4 | 每相机一个压缩视频，集成/预览输入 |
| GIF | 4 | 每相机一个快速预览动画 |
| JSON | 11 | 7 个数据/真值文件 + 4 个算法输出文件 |
| Markdown | 4（含本报告） | 使用指南、实验报告和数据解释 |

原始包在加入算法输出和报告前包含 360 PNG、4 MP4、4 GIF、7 JSON 和 1 份英文指南。`algorithm_output/` 及中文报告是后续分析产生的派生文件，不应混作采集真值。

### 3.1 顶层文件

| 文件 | 字节数（分析时） | 含义 |
|---|---:|---|
| `scene_config.json` | 1,844 | 整个序列的概要、轨迹、光照、距离范围 |
| `camera.json` | 14,368 | 4 台相机的内参、外参和坐标约定 |
| `rigid_bodies.json` | 4,139 | 头盔模型和 6 个 Marker 的局部几何 |
| `object_pose_frames.json` | 107,987 | 90 帧头盔原点位置和旋转真值 |
| `gt_3d_frames.json` | 216,673 | 90 帧 x 6 Marker 的局部/世界 3D 真值 |
| `gt_2d_frames.json` | 838,086 | 90 帧 x 4 相机 x 6 Marker 的相机坐标和像素真值 |
| `visibility.json` | 313,483 | 2,160 个相机-Marker 时刻的几何可见性 |
| `ALGORITHM_USAGE_GUIDE.md` | 7,667 | 数据生产者提供的英文使用说明 |
| `实验报告.md` | 25,774 | 已有完整算法实验和误差评估 |
| `实验结果简版.md` | 1,556 | 已有实验结果摘要 |
| `DETAILED_DATASET_REPORT.md` | 本报告 | 逐文件、逐字段的数据字典与解读 |

### 3.2 目录

```text
images/
  Camera_01/frame_0000.png ... frame_0089.png
  Camera_02/frame_0000.png ... frame_0089.png
  Camera_03/frame_0000.png ... frame_0089.png
  Camera_04/frame_0000.png ... frame_0089.png
videos/
  Camera_01.mp4/.gif ... Camera_04.mp4/.gif
algorithm_output/
  detections_2d.json
  reconstructed_markers_3d.json
  estimated_object_pose.json
  metrics.json
  overlays/Camera_XX_frame_0000/0045/0089.png
```

## 4. 时间轴和同步

| 项目 | 值 |
|---|---:|
| FPS | 30 |
| 帧数 | 90（编号 0-89） |
| 采样间隔 | 0.033333333333 s |
| 第一帧时间 | 0.0 s |
| 最后一帧时间 | 2.966666666667 s |
| 标称序列时长 | 3.0 s |

最后一帧位于 `89/30` 秒；3.0 秒表示覆盖到下一采样边界，而不是最后一帧时间戳。四个相机目录都连续具有 `frame_0000.png` 到 `frame_0089.png`，没有缺号。位姿、3D、2D、可见性 JSON 也都有 90 帧且帧号一致。

读取同步帧的正确方式是：

```python
for frame_index in range(90):
    paths = [
        f"images/Camera_{i:02d}/frame_{frame_index:04d}.png"
        for i in range(1, 5)
    ]
```

文件修改时间不是同步依据。生成时文件系统时间只有秒级显示，而且各相机写盘顺序不同；应始终使用 `frame_index`。

## 5. 坐标系、单位和矩阵规则

### 5.1 三种坐标系

- UE 世界坐标：`+X` 房间长度方向，`+Y` 房间宽度方向，`+Z` 向上。
- UE 相机局部坐标：`+X` 前，`+Y` 右，`+Z` 上。
- OpenCV 相机坐标：`+X` 图像右，`+Y` 图像下，`+Z` 相机前。

JSON 中 3D 长度单位统一为厘米，图像坐标单位是像素。输出毫米误差时要乘 10；转成米要除以 100。

### 5.2 矩阵约定

矩阵按 row-major 数组保存，但数学上作用于齐次列向量：

```text
p_camera = T_world_to_camera_opencv * [p_world; 1]
p_world  = T_camera_opencv_to_world * [p_camera; 1]
p_world  = T_object_to_world * [p_object; 1]
```

投影矩阵：

```text
P = K * T_world_to_camera_opencv[0:3, 0:4]
```

针孔投影：

```text
u = fx * Xc / Zc + cx
v = fy * Yc / Zc + cy
```

四元数顺序是 `[x, y, z, w]`。不要按 `[w, x, y, z]` 读取。欧拉角字段顺序是 `[pitch, yaw, roll]`；做算法和误差比较时优先使用四元数或旋转矩阵。

## 6. `scene_config.json`

这是最适合首先读取的“数据集封面”。字段解释如下：

| 字段 | 值/含义 |
|---|---|
| `version` | `0.6.1-profiled-rigidbody-capture`，生成格式版本 |
| `active_profile` | `helmet_dynamic`，启用头盔动态轨迹配置 |
| `capture_mode` | `dynamic`，不是静态重复测量 |
| `dataset_name` | `dataset_helmet_0001` |
| `units` | `cm` |
| `fps/frame_count/duration_sec` | 30 / 90 / 3.0 |
| `image_width/image_height` | 1280 / 720 |
| `horizontal_fov_deg_by_camera` | 四相机均为 70 度 |
| `camera_ids` | `Camera_01` 至 `Camera_04` |
| `rigid_body_id` | `helmet_01`，连接其他文件的主键 |
| `rigid_body_blueprint` | UE Blueprint 资产路径 |

`camera_to_rigid_body_distance` 给出相机到头盔原点而非到单个 Marker 的距离：全局 1.245170-1.933908 m，处于目标 0.2-2.3 m 内。各相机范围：

| 相机 | 最小 m | 最大 m |
|---|---:|---:|
| Camera_01 | 1.319091 | 1.933908 |
| Camera_02 | 1.245170 | 1.933908 |
| Camera_03 | 1.245170 | 1.933908 |
| Camera_04 | 1.319091 | 1.933908 |

`lighting` 表示房间灯光强度归一化为 100、固定曝光、曝光补偿 -1.5。它只描述 UE 渲染设置，不是现实相机的曝光时间或 ISO。

`trajectory` 描述头盔原点轨迹参数：

- X：-50 到 +50 cm；
- Y：正弦振幅 20 cm，实际范围约 -19.9969 到 +19.9969 cm；
- Z：基准 150 cm、振幅 10 cm，实际轨迹为 150 到约 159.9984 cm；
- Yaw：-30 到 +30 度；
- Pitch：约 -9.9984 到 +9.9984 度；
- Roll：约 -7.9988 到 +7.9988 度。

`visibility.physical_occlusion=false` 是全数据集最重要的限制之一：生成器没有做头盔网格遮挡射线测试。

## 7. `camera.json`

### 7.1 顶层字段

- `generated_from_current_level=true`：相机参数从 UE 当前关卡实例导出。
- `camera_system.scene_actor=CameraActor`。
- `camera_system.capture_actor=SceneCapture2D`。
- `projection=perspective`。
- `calibration_model=ideal pinhole with zero lens distortion`。
- `coordinate_conventions` 明文记录了上一节的坐标约定。

### 7.2 四台相机共同内参

```text
fx = fy = 914.0147243149534 px
cx = 640 px
cy = 360 px
K = [[914.0147243, 0, 640],
     [0, 914.0147243, 360],
     [0, 0, 1]]
horizontal FOV = 70 deg
vertical FOV = 42.9956614 deg
distortion = [0, 0, 0, 0, 0]
```

主点正好在图像中心，像素为方形（`fx=fy`），没有径向或切向畸变。不要对这些图像再套仓库 `config/camera_calibration/` 下的真实双目畸变参数；那套标定与本数据集不是同一相机。

### 7.3 相机布局

四台相机位于房间平面四角上方，Z 都是 220 cm，朝向中心区域并向下俯拍约 26.334 度：

| 相机 | 世界位置 cm | UE `[pitch,yaw,roll]` deg |
|---|---|---|
| Camera_01 | `[-100,-100,220]` | `[-26.334,45,~0]` |
| Camera_02 | `[100,-100,220]` | `[-26.334,135,~0]` |
| Camera_03 | `[-100,100,220]` | `[-26.334,-45,~0]` |
| Camera_04 | `[100,100,220]` | `[-26.334,-135,~0]` |

每个相机项还含：

- `ue_world_basis.forward/right/up`：相机在世界坐标中的三个单位基向量；
- `T_camera_ue_to_world`：UE 相机局部坐标到世界；
- `T_world_to_camera_opencv`：世界到 OpenCV 相机坐标，投影时最常用；
- `T_camera_opencv_to_world`：上一矩阵的逆，三角化结果或相机射线转回世界时使用。

数值核验表明，每台相机的两个 OpenCV 变换互逆，单位旋转正交，最大矩阵元素残差约 `2.84e-14`。

## 8. `rigid_bodies.json`

该文件定义“头盔是什么”和“6 个点在头盔自身坐标里在哪里”。

顶层信息：

- 格式版本与其他静态文件一致；
- `generated_from_spawned_blueprint_instance=true`；
- Marker 从带 `MocapMarker` 标签的 Blueprint 组件读取；
- 唯一刚体 ID 为 `helmet_01`；
- 头盔网格资产为 `helmet_project.helmet_project`，缩放 `[1,1,1]`。

Marker 局部坐标：

| ID | 组件 | 头盔局部 XYZ cm | 直观位置 |
|---|---|---|---|
| H01 | Marker_01 | `[-4.829, 9.003, 8.200]` | 较高、+Y 侧、-X |
| H02 | Marker_02 | `[4.342, 9.366, 8.051]` | 较高、+Y 侧、+X |
| H03 | Marker_03 | `[-12.141,-0.832,3.304]` | -X 外侧 |
| H04 | Marker_04 | `[12.649,2.225,2.844]` | +X 外侧 |
| H05 | Marker_05 | `[4.776,-18.094,3.448]` | 后侧较远，最易被遮挡 |
| H06 | Marker_06 | `[-4.365,-7.865,9.008]` | 后上侧，较易被遮挡 |

每个 Marker 的视觉组件都是 UE Cylinder，缩放 `[0.03,0.03,0.002]`。这是渲染几何，不应直接当成真实反光球半径。15 对点间距离为 9.180-29.139 cm；最小是 H01-H02，约 9.180 cm。

`position_cm` 永远不随帧变化。世界坐标应由该局部点和当帧 `T_object_to_world` 相乘得到。

## 9. `object_pose_frames.json`

顶层只有 `frames`，包含 90 个对象。每个对象字段：

| 字段 | 含义 |
|---|---|
| `frame_index` | 0-89，同步主键 |
| `time_sec` | `frame_index/30` |
| `rigid_body_id` | 始终为 `helmet_01` |
| `pose.location_cm` | 头盔局部原点在世界坐标的位置 |
| `pose.rotation_pitch_yaw_roll_deg` | 便于查看的欧拉角 |
| `pose.quaternion_xyzw` | 推荐使用的旋转四元数 |
| `pose.T_object_to_world` | 推荐使用的完整 4x4 刚体变换 |

位置范围：X `[-50,50]` cm，Y 约 `[-19.9969,19.9969]` cm，Z `[150,159.9984]` cm。四元数均为单位旋转，矩阵与四元数的最大旋转元素差约 `5e-16`。

注意：该文件给的是头盔坐标原点，不是“可见红点中心”或头盔几何包围盒中心。

## 10. `gt_3d_frames.json`

结构是 `frames[90].markers[6]`，共 540 条 Marker 时刻记录。每条含：

- `marker_id`：H01-H06；
- `rigid_body_id`：`helmet_01`；
- `local_position_cm`：从 `rigid_bodies.json` 复制的固定局部坐标；
- `world_position_cm`：应用当帧头盔位姿后的世界坐标。

核心约束：

```text
gt_3d.world_position_cm
  == object_pose.T_object_to_world * rigid_body.local_position_cm
```

实测最大差约 `3.22e-14 cm`，可视为浮点舍入误差。

各 Marker 世界范围反映“头盔原点轨迹 + 局部点旋转偏移”。例如 H05 的 X 范围比头盔原点更宽，因为它离原点较远且随头盔旋转。

## 11. `gt_2d_frames.json`

结构是 `frames[90].cameras[4].markers[6]`，共 2,160 条相机-Marker 记录。每条含：

- `marker_id`、`rigid_body_id`；
- `camera_xyz_cm=[Xc,Yc,Zc]`：Marker 在该相机 OpenCV 坐标系的位置；
- `pixel_uv=[u,v]`：理想针孔投影，原点位于左上角。

所有 `Zc` 都为正。各相机投影范围：

| 相机 | u 范围 px | v 范围 px | Zc 范围 cm |
|---|---:|---:|---:|
| Camera_01 | 332.096-993.709 | 235.716-524.471 | 110.025-199.688 |
| Camera_02 | 284.637-902.553 | 206.220-544.973 | 106.334-198.705 |
| Camera_03 | 285.180-872.567 | 202.713-502.543 | 112.211-205.990 |
| Camera_04 | 375.909-1036.491 | 229.364-511.634 | 113.384-202.510 |

`pixel_uv` 保留到约 4 位小数，所以用完整矩阵重新投影会有约 `1e-5` 到 `1e-4` 像素的舍入差；不是标定误差。实测平均重投影差约 `3.83e-05 px`，最大约 `7.01e-05 px`。

这里的 `pixel_uv` 是 Marker 组件理论中心投影，不保证与图像中被遮挡红色圆片的可见区域质心相同。

## 12. `visibility.json`

结构与 `gt_2d_frames.json` 不同：

```text
frames[f].cameras.Camera_01.H01.in_front
frames[f].cameras.Camera_01.H01.in_frame
frames[f].cameras.Camera_01.H01.occlusion_tested
```

相机下直接以 Marker ID 为键，不存在 `markers` 数组。三个布尔量含义：

- `in_front`：`Zc>0`；
- `in_frame`：理论 `u,v` 落在 `[0,1280) x [0,720)`；
- `occlusion_tested`：是否进行了头盔/场景射线遮挡测试。

2,160 条记录全部是 `(true, true, false)`。因此它只证明所有理论中心都在相机前且在画框内，绝不表示 6 个 Marker 在每张图中都肉眼可见。

目视检查明确看到：通常每个视角只完整显示 2-4 个红色圆片；H05/H06 经常位于头盔后侧或仅露出边缘。这是算法表观召回低于 50% 的主要解释。

## 13. `images/`

### 13.1 每个文件的身份

所有原始图像都遵循唯一映射：

```text
images/<camera_id>/frame_<frame_index:04d>.png
```

因此 360 个文件可以无歧义地理解为 4 个相机与 90 个时刻的笛卡尔积。不存在额外隐含命名字段。

| 目录 | 文件数 | 总字节 | 单文件最小/最大字节 |
|---|---:|---:|---:|
| Camera_01 | 90 | 97,259,512 | 846,093 / 1,168,381 |
| Camera_02 | 90 | 104,323,553 | 846,093 / 1,173,227 |
| Camera_03 | 90 | 105,503,652 | 1,168,767 / 1,173,227 |
| Camera_04 | 90 | 103,189,335 | 846,093 / 1,173,227 |

图像均为 1280x720、8-bit RGBA、非隔行 PNG。OpenCV 用 `IMREAD_COLOR` 读取时变成 720x1280x3 BGR，Alpha 被丢弃。已有审计确认 360 个文件 SHA-256 全不同，解码 BGR 哈希也全不同，没有重复帧或仅 Alpha 变化的伪运动。

PNG 是像素级评估的权威输入，因为无有损视频压缩。文件大小差异来自 PNG 压缩率随画面内容变化，并不代表分辨率或位深变化。

### 13.2 画面内容

画面是灰色房间、灰黑色头盔和高饱和红色圆片。四台相机从四个角度观察。帧 0、45、89 分别约对应轨迹起点、中点、终点，可用于快速了解运动覆盖，但正式评估必须用全部 90 帧。

## 14. `videos/`

### 14.1 MP4

四个 MP4 都是 H.264、1280x720、YUV420P、30 FPS、90 帧、3.0 秒：

| 文件 | 字节 | 平均码率约 kb/s |
|---|---:|---:|
| Camera_01.mp4 | 81,267 | 216.7 |
| Camera_02.mp4 | 79,291 | 211.4 |
| Camera_03.mp4 | 90,663 | 241.8 |
| Camera_04.mp4 | 88,252 | 235.3 |

它们适合验证视频解码、ROS 虚拟摄像头或快速观看，不适合与 GT 做亚像素检测误差，因为 H.264 和 YUV420 色度抽样会改变红色边缘。

### 14.2 GIF

四个 GIF 都有 90 帧、1280x720、BGRA，但 `ffprobe` 显示帧率为 `100/3`（约 33.333 FPS）、总时长 2.7 秒，而不是数据集的 30 FPS/3.0 秒。它们播放约快 11.1%，只能作为视觉预览，不能拿来恢复正确时间戳或速度。

## 15. `algorithm_output/detections_2d.json`

这是离线评估脚本从 360 张 PNG 检出的红色连通域，不是真值。结构：

```text
frames[f].frame_index
frames[f].cameras[camera_id].detections[]
frames[f].cameras[camera_id].matched_by_gt_for_evaluation[]
frames[f].cameras[camera_id].unmatched_detection_indices[]
```

`detections[]` 的核心字段来自连通域：质心 `pixel_uv`、像素面积、包围盒等。`matched_by_gt_for_evaluation` 是检测完成后，用 Hungarian 算法在 8 px 门限内与 GT 对应的结果；它使用了 GT 身份，因此不能当成完全自主 ID 关联输出。`unmatched_detection_indices` 指向同一相机帧的 `detections[]` 下标。

检测参数：红/橙 HSV 两段阈值、3x3 闭运算、连通域面积 2-500 px。总候选 1,517，其中匹配 968、未匹配 549：

| 相机 | 候选 | 匹配 | 未匹配 |
|---|---:|---:|---:|
| Camera_01 | 372 | 187 | 185 |
| Camera_02 | 443 | 241 | 202 |
| Camera_03 | 341 | 270 | 71 |
| Camera_04 | 361 | 270 | 91 |

“未匹配”不一定都是误检：被遮挡圆片只露出弧段时，可见区域质心可能离理论组件中心超过 8 px。

## 16. `algorithm_output/reconstructed_markers_3d.json`

有两个分支：

- `gt_pixel_baseline`：直接用理想 GT 2D 做四相机 DLT，用于验证几何实现；
- `image_pipeline`：用 PNG 检测并经 GT 辅助 ID 后做四相机 DLT，是实际图像结果。

每个 `markers[]` 含 `marker_id`、重建 `world_position_cm` 和 `camera_view_count`。GT 分支 90 帧均有 6 点，共 540 个 Marker 时刻；图像分支共 359 个 Marker 时刻，89 帧有 4 点、1 帧有 3 点。H05/H06 没有形成同帧至少双视图观测，所以图像分支实际只重建 H01-H04。

## 17. `algorithm_output/estimated_object_pose.json`

同样分 `gt_pixel_baseline` 和 `image_pipeline`。每个有效位姿包含：

- `frame_index`；
- `marker_ids`：本帧 Kabsch 拟合实际使用的点；
- `location_cm`：估计头盔原点世界坐标；
- `rotation_matrix`：3x3 对象到世界旋转；
- `position_error_mm`：相对 GT 的平移误差模长；
- `orientation_error_deg`：SO(3) 测地角。

两个分支都有 90 帧。GT 分支每帧使用 6 点；图像分支 89 帧用 H01-H04 四点，1 帧只用三点。因此“六 Marker 头盔”是模型定义，但当前图像位姿结果不是六点融合结果。

## 18. `algorithm_output/metrics.json`

该文件是 `scripts/evaluate_dataset_metrics.py` 的汇总输出，主要分区：

- `protocol`：输入、检测、匹配、三角化、位姿和姿态误差定义；
- `dataset_summary`：4 相机、90 帧、30 FPS、6 Marker、距离范围；
- `detection`：总体、逐相机、逐 Marker 的 2D 统计；
- `gt_geometry_baseline`：理想像素几何基线；
- `image_pipeline_four_camera`：四相机实际 PNG 主结果；
- `image_pipeline_project_native_two_camera_scope`：只用 Camera_01/02 的对照；
- `claim_limits`：必须保留的结论边界。

主结果：

| 指标 | 四相机 PNG 结果 |
|---|---:|
| 2D 匹配数 / 理论 in-frame 数 | 968 / 2,160 |
| 表观 Recall | 44.815%（受无物理遮挡标签影响） |
| 2D 中心 Mean / RMSE | 1.770 / 2.246 px |
| 重建 Marker 时刻 | 359 / 540 |
| Marker 3D Mean / RMSE | 3.385 / 3.756 mm |
| 有效位姿帧 | 90 / 90 |
| 位置 Mean / RMSE | 3.100 / 3.155 mm |
| 位置 P95 / Max | 4.152 / 4.704 mm |
| 姿态 Mean / RMSE | 1.102 / 1.133 deg |
| 姿态 P95 / Max | 1.639 / 1.846 deg |

位置偏差约 `[-2.098,-1.209,+1.616] mm`，模长 2.911 mm，说明位置误差主要是系统偏差。姿态小角度偏差模长约 0.994 度。H04 的 3D 平均误差 5.772 mm，明显高于 H01-H03，是位姿误差的重要来源。

GT 几何基线接近浮点误差，说明相机矩阵、坐标变换、DLT 和 Kabsch 实现内部一致。只用 Camera_01/02 时仅 1/90 帧形成三点位姿，证明四相机覆盖对该头盔遮挡场景是必要的。

## 19. `algorithm_output/overlays/`

12 张图是 4 相机 x 帧 0、45、89：

```text
Camera_01_frame_0000.png ... Camera_04_frame_0089.png
```

编码为 1280x720、8-bit RGB PNG。绘制语义由评估代码确定：

- 蓝色空心圆和文字：GT 理论 Marker 中心及 H01-H06 ID；
- 绿色十字：HSV 连通域检测质心；
- 红色线段：GT 中心到成功匹配检测质心的偏差。

这些图是诊断工具，不是新传感器数据。观察蓝点落在被头盔遮挡区域、绿色十字落在仅剩红色弧段质心的情况，可以理解 8 px 门限为何拒绝一些肉眼看起来“检测到了红色”的候选。

## 20. 三份已有文档分别怎么看

- `ALGORITHM_USAGE_GUIDE.md`：生成方的数据格式合同，优先用于理解坐标系和推荐流程。
- `实验报告.md`：已有的完整实跑、误差、ROS 链路和参数敏感性报告，优先用于引用实验数字。
- `实验结果简版.md`：面向快速汇报的摘要，不足以替代完整报告。
- 本报告：面向初学者的数据字典和文件关系说明。

当文档与 JSON 冲突时，以 JSON 和读取代码为准；当真值与肉眼可见性冲突时，要先检查 `physical_occlusion=false`，不要直接判定图像或 GT 错位。

## 21. 与原仓库 ROS 管线的关系

仓库原生系统是 ROS 2 双相机移动动捕，真实配置位于 `config/camera_calibration/`，默认刚体位于 `config/rigid_bodies/bodies.yaml`。它们与本数据集有关键差异：

- 原生标定约为 640x480 且有明显镜头畸变；本数据是 1280x720、零畸变；
- 原生刚体是三点 `scalene`；本数据是六点 `helmet_01`；
- 原生检测面向 NIR 反光亮点；本数据是 UE 渲染红色圆片；
- 原生三角化主要是双相机；离线评估使用四相机 DLT；
- 本数据没有真实设备时间戳、曝光、硬件触发或相机序列号。

因此不能把仓库默认 YAML 直接套到该数据集。应直接读取该数据集自己的 `camera.json` 和 `rigid_bodies.json`。

## 22. 推荐读取顺序

1. 读 `scene_config.json`，确认序列规模和单位。
2. 读 `camera.json`，构建相机字典与投影矩阵。
3. 读 `rigid_bodies.json`，构建 `marker_id -> local_position_cm`。
4. 按 `frame_index` 同步读取四张 PNG。
5. 开发检测时使用 `gt_2d_frames.json` 做离线评估，但不要把 GT 身份偷偷输入最终自主算法。
6. 三角化后与 `gt_3d_frames.json` 比较。
7. 位姿估计后与 `object_pose_frames.json` 比较。
8. 只用 `visibility.json` 筛选几何出画/相机后方点；物理遮挡必须另行标注或从图像判断。

## 23. 最小可靠加载示例

```python
import json
from pathlib import Path
import cv2
import numpy as np

root = Path("/home/wy/mobile_mocap/dataset_helmet_0001")
camera_data = json.loads((root / "camera.json").read_text())
rigid_data = json.loads((root / "rigid_bodies.json").read_text())
poses = json.loads((root / "object_pose_frames.json").read_text())["frames"]
gt3 = json.loads((root / "gt_3d_frames.json").read_text())["frames"]
gt2 = json.loads((root / "gt_2d_frames.json").read_text())["frames"]

cameras = {c["camera_id"]: c for c in camera_data["cameras"]}
local_points = {
    m["id"]: np.asarray(m["position_cm"], float)
    for m in rigid_data["rigid_bodies"][0]["markers"]
}

f = 0
images = {
    camera_id: cv2.imread(str(root / "images" / camera_id / f"frame_{f:04d}.png"))
    for camera_id in cameras
}

camera_id = "Camera_01"
K = np.asarray(cameras[camera_id]["intrinsics"]["K"], float)
T_wc = np.asarray(
    cameras[camera_id]["extrinsics"]["T_world_to_camera_opencv"], float
)
P = K @ T_wc[:3, :]
```

## 24. 使用时最容易犯的错误

1. 把厘米当成米，导致尺度差 100 倍。
2. 把四元数 `[x,y,z,w]` 当成 `[w,x,y,z]`。
3. 把 `T_world_to_camera` 方向用反。
4. 看到 `in_frame=true` 就认为 Marker 未被头盔遮挡。
5. 用 MP4/GIF 替代 PNG 做像素精度评估。
6. 用 GIF 的 2.7 秒时长推断真实速度。
7. 将 GT 辅助 Hungarian ID 匹配当成自主对应算法。
8. 声称当前结果融合了 6 点；实际图像位姿主要融合 H01-H04。
9. 将运动序列去偏 RMS 称为严格静态重复性。
10. 直接使用仓库默认双目标定和三点刚体 YAML。

## 25. 最终判断

该数据集在几何和时间层面高度自洽：相机正反变换互逆、位姿生成 3D、3D 投影生成 2D 的误差都接近数值舍入，四路图像和四类逐帧 JSON 完整连续。它非常适合验证多相机管线和初步算法精度。

真正限制算法解释力的不是 JSON 精度，而是仿真与标签定义：所有点都被标为几何 `in_frame`，但没有物理遮挡标签；红色圆片的理论组件中心与被遮挡后可见区域质心可能相差数十像素；Marker 无视觉编码，现有结果的 ID 归属依赖 GT；且渲染和相机模型过于理想。因此可以把它当作“结构正确、场景有遮挡的合成集成测试集”，不能单独作为真实移动动捕系统的最终验收集。
