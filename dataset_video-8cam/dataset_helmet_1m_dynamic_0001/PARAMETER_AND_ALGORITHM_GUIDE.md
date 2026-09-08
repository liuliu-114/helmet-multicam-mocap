# 1 m近场头盔动态数据集参数说明

## 1. 数据集概况

本数据集用于验证多相机条件下，头盔刚体在1 m以内工作距离中的Marker检测、三维定位和姿态解算精度。

| 参数 | 数值 |
| --- | --- |
| 数据集版本 | `0.7.0-nearfield-accuracy-capture` |
| 测试类型 | 动态序列（`nearfield_dynamic`） |
| 相机数量 | 8台 |
| 图像分辨率 | 1280 x 720 |
| 帧率 | 30 FPS |
| 每相机帧数 | 180帧 |
| 序列时长 | 6秒 |
| PNG总数 | 1440张 |
| Marker数量 | 6个（`H01`-`H06`） |
| 刚体原点实测距离 | 0.6931-0.8357 m |
| Marker实测距离 | 0.5137-0.9849 m |
| 完整入画率 | 1440/1440（100%） |
| 5%边缘余量通过率 | 1420/1440（98.61%） |

## 2. 文件用途

```text
dataset_helmet_1m_dynamic_0001/
|-- images/                    # 8台相机的逐帧PNG，算法首选输入
|-- videos/                    # 每台相机对应的MP4和GIF，便于播放检查
|-- camera.json                # 相机内参、外参和投影矩阵
|-- rigid_bodies.json          # 刚体及6个Marker的定义和初始几何关系
|-- object_pose_frames.json    # 每帧刚体真实位置和姿态
|-- gt_3d_frames.json          # 每帧Marker世界坐标真值
|-- gt_2d_frames.json          # 每帧、每相机Marker二维投影真值
|-- visibility.json            # Marker逐相机逐帧可见性和入画状态
|-- distance_frames.json       # 每帧距离、工作区间及画面边界检查结果
|-- scene_config.json          # 场景、采集、轨迹、照明和质量统计配置
`-- PARAMETER_AND_ALGORITHM_GUIDE.md
```

同一帧在各相机目录中的编号一致。例如，各目录中的`frame_000050.png`表示同一仿真时刻，应作为一组同步图像处理。

MP4经过视频编码，可能产生压缩误差。定量精度测试建议读取`images`中的PNG；MP4主要用于人工浏览、演示或算法流程联调。

## 3. 相机参数

所有相机使用相同内参：

| 参数 | 数值 |
| --- | --- |
| 水平视场角 | 85 deg |
| 垂直视场角 | 54.5364 deg |
| `fx` | 698.4374 px |
| `fy` | 698.4374 px |
| `cx` | 640 px |
| `cy` | 360 px |

相机布局分为两圈：

- `Camera_01`-`Camera_04`：约0.70 m半径，高度1.70 m，位于四个对角方向。
- `Camera_05`-`Camera_08`：约0.75 m半径，高度1.80 m，位于前、右、后、左方向。
- 相机均朝向头盔运动区域中心附近，目标点约为UE世界坐标`(0, 0, 150)` cm。

`camera.json`包含每台相机的：

- `intrinsics`：内参及图像尺寸。
- `location_cm`、`rotation_deg`：UE世界坐标中的相机位姿。
- `world_to_camera_matrix`：世界坐标到相机坐标的变换矩阵。
- `camera_to_world_matrix`：相机坐标到世界坐标的变换矩阵。
- `projection_matrix`：投影相关矩阵。

二维投影可按齐次坐标计算：

```text
X_camera = T_world_to_camera * X_world
p = K * X_camera
u = p_x / p_z
v = p_y / p_z
```

实现前应先用`gt_3d_frames.json`中的三维点投影，并与`gt_2d_frames.json`比较，以验证矩阵方向、单位和图像坐标方向是否正确。

## 4. 坐标和单位

- UE世界位置原始单位为厘米，导出的米制字段以`_m`结尾。
- 图像坐标单位为像素，原点位于图像左上角，`u`向右、`v`向下。
- 欧拉角单位为度；角度比较时必须处理`-180 deg/180 deg`环绕。
- 刚体位置误差应在同一坐标系和同一单位下计算。
- 姿态建议统一转换为旋转矩阵或四元数后再计算误差，避免直接相减欧拉角。

## 5. 刚体与运动轨迹

刚体ID为`helmet_01`，Blueprint资源为：

```text
/Game/Mocap/Blueprints/RigidBodies/BP_HelmetRigidBody
```

Marker ID为`H01`、`H02`、`H03`、`H04`、`H05`、`H06`。`rigid_bodies.json`记录Marker相对刚体原点的局部位置，可用于刚体拟合和姿态解算。

本序列采用小范围六自由度动态轨迹：

| 分量 | 范围或幅值 |
| --- | --- |
| X平移 | -3至3 cm |
| Y平移幅值 | 3 cm |
| Z中心高度 | 150 cm |
| Z平移幅值 | 2 cm |
| Yaw | -30至30 deg |
| Pitch幅值 | 10 deg |
| Roll幅值 | 8 deg |

## 6. 真值文件使用方法

### `gt_2d_frames.json`

用于验证Marker检测和ID匹配。按帧号、相机名和Marker ID读取真实像素坐标，与算法检测坐标比较。

单点二维误差：

```text
e_2d = sqrt((u_pred-u_gt)^2 + (v_pred-v_gt)^2)
```

建议输出检测率、ID正确率、平均像素误差、RMSE、P95和最大误差。

### `gt_3d_frames.json`

记录每帧各Marker的世界三维坐标，用于验证多相机三角化结果。

```text
e_3d = ||X_pred-X_gt||_2
```

建议统一换算为毫米，并分别统计每个Marker和全部Marker的平均误差、RMSE、P95及最大误差。

### `object_pose_frames.json`

记录每帧头盔刚体的真实位姿。算法从多个Marker恢复刚体位姿后，可与此文件比较。

位置误差：

```text
e_t = ||t_pred-t_gt||_2
```

姿态误差建议用相对旋转角：

```text
R_delta = R_pred * transpose(R_gt)
e_R = acos(clamp((trace(R_delta)-1)/2, -1, 1))
```

将`e_R`由弧度换算为度后统计平均值、RMSE、P95和最大值。

### `visibility.json`

用于区分Marker未检测的原因。评估检测率时，应主要统计真值标记为可见且在画面内的点，避免把本来不可见的Marker计为漏检。

### `distance_frames.json`

用于1 m工作范围验证，包含相机到刚体原点和Marker的真实距离，以及是否完整入画、是否满足5%边缘余量等状态。该文件用于距离分段统计和质量核验，不应作为算法定位输入。

## 7. 推荐算法流程

1. 从8个`images/Camera_xx`目录同步读取同一帧PNG。
2. 检测红色Marker中心，并为检测结果分配`H01`-`H06`身份。
3. 使用`camera.json`内外参进行多视角三角化，得到Marker三维坐标。
4. 使用`rigid_bodies.json`中的局部Marker模板拟合刚体位置和姿态。
5. 用`gt_2d_frames.json`验证二维检测，用`gt_3d_frames.json`验证三维点。
6. 用`object_pose_frames.json`验证刚体位置和姿态。
7. 按`distance_frames.json`中的距离区间分组统计精度，重点报告1 m以内结果。

为保证评价独立性，算法运行阶段只应使用图像、`camera.json`和刚体局部Marker模板。其他`gt_*`、逐帧位姿、距离和可见性文件只在评测阶段读取。

## 8. 数据验证

在项目根目录运行：

```powershell
python validate_dataset_v070.py --dataset output\dataset_helmet_1m_dynamic_0001
```

本数据集当前验证结果：8台相机各180帧，共1440张PNG；6个Marker；刚体原点和Marker最大距离均不超过1 m，验证通过。

## 9. 已知条件

- `physical_occlusion`为`false`，当前可见性主要反映画面范围和投影状态，不等同于完整的真实遮挡判定。
- 数据来自UE仿真，尚未包含真实相机的镜头畸变、传感器噪声、运动模糊、曝光波动和时间同步误差。
- 本数据集适合验证算法几何链路与1 m近场精度；向真实系统迁移时，应补充畸变模型、噪声模型及实拍标定数据测试。
