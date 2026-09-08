# BP_HelmetRigidBody_01动态数据集验证报告

验证日期：2026-07-16

## 结论

本数据集的文件完整性、帧同步、相机参数、刚体位姿、Marker局部/世界坐标、二维投影和距离数据均验证通过，可用于算法定位和姿态评测。

## 数据规模

| 项目 | 结果 |
|---|---:|
| Blueprint | `/Game/Mocap/Blueprints/RigidBodies/BP_HelmetRigidBody_01` |
| 相机 | 8台 |
| 每相机帧数 | 180帧 |
| PNG总数 | 1440张 |
| 帧率/时长 | 30 FPS / 6秒 |
| Marker | 6个，`H01～H06` |
| 刚体原点距离 | 0.6931～0.8363 m |
| Marker距离 | 0.5680～0.9015 m |

## 全量几何审计

共重新计算180 × 8 × 6 = 8640个相机-Marker观测。

| 检查项 | 最大误差 |
|---|---:|
| 相机正逆矩阵误差 | `1.42e-14` |
| Marker刚体变换误差 | `6.4e-15 cm` |
| 世界坐标到相机坐标误差 | `4.88e-14 cm` |
| 二维重投影误差 | `0.000070 px` |
| 刚体原点距离重算误差 | `2.22e-16 m` |
| Marker距离重算误差 | `2.22e-16 m` |

基础验证和全量几何审计均通过。

## Marker定义

本Blueprint取消了Marker父组件，直接使用`MarkerVisual_01～06`的组件Pivot作为真值锚点。

| ID | 组件 | 局部坐标/cm |
|---|---|---|
| H01 | `MarkerVisual_01` | `[-7.6784, 5.7964, 8.3000]` |
| H02 | `MarkerVisual_02` | `[6.4675, 7.0916, 8.5360]` |
| H03 | `MarkerVisual_03` | `[-10.9177, -3.2835, 5.4300]` |
| H04 | `MarkerVisual_04` | `[11.5900, 1.0000, 5.0294]` |
| H05 | `MarkerVisual_05` | `[7.3500, -9.3271, 4.7000]` |
| H06 | `MarkerVisual_06` | `[-3.9164, -10.9428, 4.8500]` |

组件未设置`MocapMarker`标签，本次按名称可靠映射为`H01～H06`。该发现方式已准确记录在`rigid_bodies.json`：

```json
"discovery_mode": "component_name_fallback",
"component_name_pattern": "MarkerVisual_01-MarkerVisual_06",
"ground_truth_anchor": "component_pivot"
```

## 需要算法侧注意

1. 六个Marker的缩放均为`[0.03, 0.04, 0.002]`。按基础圆柱尺寸计算，贴片约为3 cm × 4 cm、厚2 mm，是椭圆而不是直径3 cm的正圆。
2. 新排布的X/Y/Z跨度约为22.51/18.03/3.84 cm，最小点间距约8.70 cm。
3. 新排布的中心化坐标奇异值约为`[20.68, 17.15, 1.78] cm`，最小/最大比值为0.086；旧排布该比值约0.241。因此新排布更接近平面，虽然可以正常解算六自由度位姿，但深度和平面外旋转可能对检测噪声更敏感。
4. `gt_2d_frames.json`记录组件Pivot的理想针孔投影，不是红色像素区域的分割质心。
5. `visibility.json`中`physical_occlusion=false`；`in_frame=true`不代表Marker没有被头盔遮挡，不应直接用于严格计算检测召回率。

## 复核命令

```powershell
python validate_dataset_v070.py --dataset output\dataset_helmet_layout01_1m_dynamic_0001
python audit_dataset_geometry_v071.py --dataset output\dataset_helmet_layout01_1m_dynamic_0001
```
