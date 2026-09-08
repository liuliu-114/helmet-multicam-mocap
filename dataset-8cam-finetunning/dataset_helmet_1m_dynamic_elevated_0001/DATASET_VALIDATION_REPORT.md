# 数据集验证报告

## 验证结论

`dataset_helmet_1m_dynamic_elevated_0001`的数据结构和几何真值验证通过，可用于Marker三维定位和头盔刚体位姿精度评测。

验证日期：2026-07-15。

## 数据完整性

| 项目 | 结果 |
|---|---:|
| 相机 | 8台 |
| 每相机PNG | 180张 |
| PNG总数 | 1440张 |
| JSON文件 | 8个，均可正常解析 |
| 非有限数值（NaN/Infinity） | 0 |
| Marker | 6个，ID为`H01～H06` |
| 相机到刚体原点距离 | 0.6931～0.8363 m |
| 相机到Marker距离 | 0.5137～0.9562 m |
| 1 m距离门禁 | 通过 |

## 全量几何一致性检查

对180帧、8台相机、6个Marker，共8640个相机-Marker观测进行了重新计算。

| 检查项 | 最大误差 |
|---|---:|
| 相机正逆变换矩阵乘积相对单位阵 | `1.42e-14` |
| Marker局部坐标经刚体位姿变换后的世界坐标 | `7.94e-15 cm` |
| 世界坐标经相机外参变换后的相机坐标 | `4.28e-14 cm` |
| 使用相机内参重新投影得到的二维坐标 | `0.000071 px` |
| 相机到刚体原点距离重算误差 | `2.22e-16 m` |
| 相机到Marker距离重算误差 | `2.22e-16 m` |

同时确认：

- 五个逐帧JSON的`frame_index`、`time_sec`、`pose_id`和测试标识逐帧一致。
- 时间戳符合`time_sec = frame_index / 30`。
- `rigid_bodies.json`局部Marker结构与`gt_3d_frames.json`一致。
- `Camera_01～04`保持原近场位置。
- `Camera_05～08`实际位置为半径60 cm、高度205 cm，与`elevated_outer_overhead`布局一致。
- `scene_config.json`记录的距离范围与逐帧重算结果一致。

## 算法使用规则

算法运行阶段可使用：

- `images/`：算法图像输入，精度测试优先使用PNG。
- `camera.json`：相机内参和外参。
- `rigid_bodies.json`：刚体Marker局部几何模板。

算法输出后才可用于评测：

- `gt_2d_frames.json`：二维理想投影真值。
- `gt_3d_frames.json`：Marker三维世界坐标真值。
- `object_pose_frames.json`：刚体位置和姿态真值。
- `distance_frames.json`：距离分段和画面范围统计。
- `visibility.json`：投影可见性辅助信息，使用限制见下文。

不得将逐帧真值、距离或可见性字段作为定位算法输入。

## `visibility.json`的重要限制

当前数据的：

```json
"physical_occlusion": false
```

因此`visibility.json`中的：

- `in_front=true`仅表示Marker位于相机前方。
- `in_frame=true`仅表示Marker中心投影落在图像范围内。
- 它们不表示Marker红色贴片一定没有被头盔实体遮挡。

实际图像抽查确认，部分Marker在特定视角下会被头盔遮挡或只露出边缘。这不是2D/3D坐标错误，而是当前版本没有输出物理遮挡真值。

此外，`gt_2d_frames.json`中的`pixel_uv`是Marker组件中心的理想针孔投影，不是从渲染图像分割得到的红色像素质心。贴片斜视或部分遮挡时，算法检测到的红色区域质心可能与该理想中心存在系统偏差；评价报告中应将其视为检测与渲染模型误差，而不是相机矩阵错误。

使用影响：

- 对成功检测并正确匹配的Marker计算二维误差、三维误差、刚体位置误差和姿态误差，不受此限制。
- 不应直接以`in_frame=true`作为检测召回率或漏检率的分母，否则会把被头盔遮挡的Marker错误统计为漏检。
- 若需要严格评价检测率，后续数据版本应增加UE物理射线遮挡检测或单独的渲染可见性标注。

## 推荐评测方式

1. 算法仅从PNG检测实际可见的红色Marker。
2. 使用相机参数进行多视角三角化和刚体拟合。
3. 对已成功关联的Marker，与`gt_2d_frames.json`和`gt_3d_frames.json`比较。
4. 对成功输出的刚体位姿，与`object_pose_frames.json`比较。
5. 单独报告成功帧率、有效Marker数量、位置误差和姿态误差。
6. 当前版本不要直接使用`visibility.json`计算物理可见Marker召回率。

## 复核命令

基础完整性和距离检查：

```powershell
python validate_dataset_v070.py --dataset output\dataset_helmet_1m_dynamic_elevated_0001
```

全量几何一致性检查：

```powershell
python audit_dataset_geometry_v071.py --dataset output\dataset_helmet_1m_dynamic_elevated_0001
```

两项检查当前均已通过。
