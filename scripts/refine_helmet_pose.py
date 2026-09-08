#!/usr/bin/env python3
"""Refine helmet poses by jointly minimizing multi-camera reprojection error.

The existing DLT + Kabsch pose is retained as the initializer and fallback.
Marker identities come from the dataset's existing GT-assisted detection output;
this script evaluates pose refinement, not autonomous marker association.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def load_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def dump_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


def rotation_from_matrix(matrix):
    if hasattr(Rotation, "from_matrix"):
        return Rotation.from_matrix(matrix)
    return Rotation.from_dcm(matrix)


def rotation_as_matrix(rotation):
    if hasattr(rotation, "as_matrix"):
        return rotation.as_matrix()
    return rotation.as_dcm()


def summarize(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return None
    return {
        "count": int(values.size),
        "mean": float(values.mean()),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "std": float(values.std(ddof=0)),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


def project(projection, world_point):
    homogeneous = projection @ np.append(np.asarray(world_point, dtype=float), 1.0)
    if homogeneous[2] <= 0:
        return None
    return homogeneous[:2] / homogeneous[2]


def compose_pose(delta_pose, initial_rotation, initial_translation):
    delta_rotation = rotation_as_matrix(Rotation.from_rotvec(delta_pose[:3]))
    rotation = delta_rotation @ initial_rotation
    translation = initial_translation + delta_pose[3:]
    return rotation, translation


def build_observations(frame):
    observations = []
    for camera_id, camera_frame in frame["cameras"].items():
        for marker in camera_frame["matched_by_gt_for_evaluation"]:
            observations.append(
                {
                    "camera_id": camera_id,
                    "marker_id": marker["marker_id"],
                    "pixel_uv": np.asarray(marker["pixel_uv"], dtype=float),
                }
            )
    return observations


def pose_residuals(
    delta_pose,
    initial_rotation,
    initial_translation,
    observations,
    projections,
    local_points,
):
    rotation, translation = compose_pose(
        delta_pose, initial_rotation, initial_translation
    )
    residuals = []
    for observation in observations:
        world_point = rotation @ local_points[observation["marker_id"]] + translation
        predicted = project(projections[observation["camera_id"]], world_point)
        if predicted is None:
            residuals.extend((1e4, 1e4))
        else:
            residuals.extend(predicted - observation["pixel_uv"])
    return np.asarray(residuals, dtype=float)


def point_residuals(rotation, translation, observations, projections, local_points):
    values = []
    by_marker = {marker_id: [] for marker_id in local_points}
    for observation in observations:
        marker_id = observation["marker_id"]
        world_point = rotation @ local_points[marker_id] + translation
        predicted = project(projections[observation["camera_id"]], world_point)
        error = (
            float("inf")
            if predicted is None
            else float(np.linalg.norm(predicted - observation["pixel_uv"]))
        )
        values.append(error)
        by_marker[marker_id].append(error)
    return values, by_marker


def robust_cost(residuals, loss, f_scale):
    """Match scipy.least_squares' robust objective for acceptance checks."""
    scaled_squared = (np.asarray(residuals, dtype=float) / f_scale) ** 2
    if loss == "linear":
        rho = scaled_squared
    elif loss == "soft_l1":
        rho = 2.0 * (np.sqrt(1.0 + scaled_squared) - 1.0)
    elif loss == "huber":
        rho = np.where(
            scaled_squared <= 1.0,
            scaled_squared,
            2.0 * np.sqrt(scaled_squared) - 1.0,
        )
    elif loss == "cauchy":
        rho = np.log1p(scaled_squared)
    else:
        raise ValueError(f"Unsupported loss: {loss}")
    return float(0.5 * f_scale**2 * np.sum(rho))


def refine_pose(
    initial_rotation,
    initial_translation,
    observations,
    projections,
    local_points,
    loss,
    f_scale,
    max_nfev,
):
    residual_function = lambda delta: pose_residuals(
        delta,
        initial_rotation,
        initial_translation,
        observations,
        projections,
        local_points,
    )
    initial_delta = np.zeros(6, dtype=float)
    initial_residuals = residual_function(initial_delta)
    result = least_squares(
        residual_function,
        initial_delta,
        loss=loss,
        f_scale=f_scale,
        x_scale=np.asarray([0.01, 0.01, 0.01, 1.0, 1.0, 1.0]),
        max_nfev=max_nfev,
    )
    refined_rotation, refined_translation = compose_pose(
        result.x, initial_rotation, initial_translation
    )
    final_residuals = residual_function(result.x)

    initial_plain_cost = float(np.sum(initial_residuals**2))
    final_plain_cost = float(np.sum(final_residuals**2))
    initial_robust_cost = robust_cost(initial_residuals, loss, f_scale)
    final_robust_cost = robust_cost(final_residuals, loss, f_scale)
    accepted = bool(
        result.success
        and np.all(np.isfinite(result.x))
        and final_robust_cost <= initial_robust_cost
    )
    if not accepted:
        refined_rotation = initial_rotation
        refined_translation = initial_translation
        final_residuals = initial_residuals

    return refined_rotation, refined_translation, {
        "accepted": accepted,
        "optimizer_success": bool(result.success),
        "optimizer_status": int(result.status),
        "optimizer_message": str(result.message),
        "function_evaluations": int(result.nfev),
        "initial_plain_squared_cost": initial_plain_cost,
        "final_plain_squared_cost": float(np.sum(final_residuals**2)),
        "initial_robust_cost": initial_robust_cost,
        "final_robust_cost": robust_cost(final_residuals, loss, f_scale),
        "delta_rotation_deg": np.degrees(result.x[:3]).tolist(),
        "delta_translation_cm": result.x[3:].tolist(),
    }


def pose_error(rotation, translation, ground_truth_transform):
    position_vector_mm = (translation - ground_truth_transform[:3, 3]) * 10.0
    error_rotation = rotation @ ground_truth_transform[:3, :3].T
    orientation_vector_deg = np.degrees(
        rotation_from_matrix(error_rotation).as_rotvec()
    )
    return {
        "position_vector_mm": position_vector_mm,
        "position_error_mm": float(np.linalg.norm(position_vector_mm)),
        "orientation_vector_deg": orientation_vector_deg,
        "orientation_error_deg": float(np.linalg.norm(orientation_vector_deg)),
    }


def pose_to_json(rotation, translation):
    return {
        "location_cm": translation.tolist(),
        "rotation_matrix": rotation.tolist(),
        "quaternion_xyzw": rotation_from_matrix(rotation).as_quat().tolist(),
    }


def aggregate_pose_errors(frame_results, key):
    position_errors = [frame[key]["position_error_mm"] for frame in frame_results]
    orientation_errors = [
        frame[key]["orientation_error_deg"] for frame in frame_results
    ]
    position_vectors = np.asarray(
        [frame[key]["position_vector_mm"] for frame in frame_results], dtype=float
    )
    orientation_vectors = np.asarray(
        [frame[key]["orientation_vector_deg"] for frame in frame_results], dtype=float
    )
    return {
        "position_error_mm": summarize(position_errors),
        "position_bias_xyz_mm": position_vectors.mean(axis=0).tolist(),
        "position_bias_norm_mm": float(np.linalg.norm(position_vectors.mean(axis=0))),
        "orientation_error_deg": summarize(orientation_errors),
        "orientation_bias_xyz_deg": orientation_vectors.mean(axis=0).tolist(),
        "orientation_bias_norm_deg": float(
            np.linalg.norm(orientation_vectors.mean(axis=0))
        ),
    }


def percent_reduction(before, after):
    return float((before - after) / before * 100.0)


def write_csv(path, frame_results):
    fields = [
        "frame_index",
        "observation_count",
        "camera_count",
        "marker_count",
        "accepted",
        "initial_position_error_mm",
        "refined_position_error_mm",
        "initial_orientation_error_deg",
        "refined_orientation_error_deg",
        "initial_reprojection_mean_px",
        "refined_reprojection_mean_px",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame in frame_results:
            writer.writerow(
                {
                    "frame_index": frame["frame_index"],
                    "observation_count": frame["observation_count"],
                    "camera_count": frame["camera_count"],
                    "marker_count": frame["marker_count"],
                    "accepted": frame["optimization"]["accepted"],
                    "initial_position_error_mm": frame["initial_error"][
                        "position_error_mm"
                    ],
                    "refined_position_error_mm": frame["refined_error"][
                        "position_error_mm"
                    ],
                    "initial_orientation_error_deg": frame["initial_error"][
                        "orientation_error_deg"
                    ],
                    "refined_orientation_error_deg": frame["refined_error"][
                        "orientation_error_deg"
                    ],
                    "initial_reprojection_mean_px": frame["initial_reprojection_px"][
                        "mean"
                    ],
                    "refined_reprojection_mean_px": frame["refined_reprojection_px"][
                        "mean"
                    ],
                }
            )


def write_report(path, metrics):
    initial = metrics["initial_dlt_kabsch"]
    refined = metrics["refined_pose"]
    improvement = metrics["improvement_percent"]
    source = metrics.get("source_pipeline")
    source_summary = ""
    if source:
        detection = source["detection"]
        multicamera = source["multicamera"]
        protocol = source.get("protocol", {})
        source_summary = f"""
## 基础流程结果

- 图像输入：{protocol.get('image_source', '未记录')}
- 连通域面积范围：{protocol.get('detector_component_area_px', '未记录')} px²
- 二维检测匹配：{detection['matched_detection_count']} / {detection['gt_in_frame_count']}，表观召回率 {detection['recall_against_in_frame_gt'] * 100:.2f}%
- 二维中心 RMSE：{detection['center_error_px']['rmse']:.6f} px
- 多相机 3D Marker 平均误差：{multicamera['marker_3d_error_mm']['mean']:.6f} mm
- 多相机 3D Marker RMSE：{multicamera['marker_3d_error_mm']['rmse']:.6f} mm
- 基础位姿有效帧：{multicamera['pose_frames']}
"""
    parameter_summary = ""
    comparison = metrics.get("parameter_comparison")
    if comparison:
        parameter_summary = f"""
## 近场面积门限适配

| 最大面积 | 匹配数 | 表观召回率 | 中心 RMSE |
|---:|---:|---:|---:|
| {comparison['legacy']['max_area_px']} px² | {comparison['legacy']['matched_detection_count']} | {comparison['legacy']['recall'] * 100:.2f}% | {comparison['legacy']['center_rmse_px']:.6f} px |
| {comparison['selected']['max_area_px']} px² | {comparison['selected']['matched_detection_count']} | {comparison['selected']['recall'] * 100:.2f}% | {comparison['selected']['center_rmse_px']:.6f} px |

近场 Marker 面积增大，旧的 {comparison['legacy']['max_area_px']} px² 上限会误删真实红点，因此正式结果采用 {comparison['selected']['max_area_px']} px²。
"""
    text = f"""# 头盔 {metrics['camera_count']} 相机 6DoF 联合优化结果

## 运行范围

- 数据集：`{metrics['dataset']}`
- 相机数量：{metrics['camera_count']}
- 帧数：{metrics['frames']}
- 优化成功并采用：{metrics['accepted_frames']} 帧
- 鲁棒损失：`{metrics['optimizer']['loss']}`
- Huber/Cauchy 尺度：{metrics['optimizer']['f_scale_px']} px
- Marker ID：沿用现有 GT 辅助匹配，仅验证位姿优化
{source_summary}
{parameter_summary}

## 误差对比

| 指标 | 原 DLT + Kabsch | 6DoF 联合优化 | 改善 |
|---|---:|---:|---:|
| 位置平均误差 | {initial['position_error_mm']['mean']:.6f} mm | {refined['position_error_mm']['mean']:.6f} mm | {improvement['position_mean']:.2f}% |
| 位置 RMSE | {initial['position_error_mm']['rmse']:.6f} mm | {refined['position_error_mm']['rmse']:.6f} mm | {improvement['position_rmse']:.2f}% |
| 姿态平均误差 | {initial['orientation_error_deg']['mean']:.6f}° | {refined['orientation_error_deg']['mean']:.6f}° | {improvement['orientation_mean']:.2f}% |
| 姿态 RMSE | {initial['orientation_error_deg']['rmse']:.6f}° | {refined['orientation_error_deg']['rmse']:.6f}° | {improvement['orientation_rmse']:.2f}% |
| 二维重投影平均误差 | {metrics['initial_reprojection_error_px']['mean']:.6f} px | {metrics['refined_reprojection_error_px']['mean']:.6f} px | {improvement['reprojection_mean']:.2f}% |
| 二维重投影 RMSE | {metrics['initial_reprojection_error_px']['rmse']:.6f} px | {metrics['refined_reprojection_error_px']['rmse']:.6f} px | {improvement['reprojection_rmse']:.2f}% |

逐帧比较中，位置误差改善 {metrics['frame_outcomes']['position_improved']} 帧，姿态误差改善 {metrics['frame_outcomes']['orientation_improved']} 帧，两项同时改善 {metrics['frame_outcomes']['both_improved']} 帧。

## 说明

联合优化没有替换原流程，而是使用原 DLT + Kabsch 位姿作为初值，再让同一个头盔位姿同时解释 {metrics['camera_count']} 台相机中的全部有效二维 Marker。异常观测通过鲁棒损失降权。若某帧优化失败或鲁棒目标变差，则自动回退到原位姿。Huber 会允许少数大残差来换取总体鲁棒结果，因此重投影 RMSE 不一定与平均值同步下降。
"""
    path.write_text(text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="dataset_helmet_0001")
    parser.add_argument("--output", default="output-lkf")
    parser.add_argument(
        "--algorithm-output",
        default=None,
        help="directory containing detections_2d.json and estimated_object_pose.json",
    )
    parser.add_argument("--loss", choices=("linear", "huber", "soft_l1", "cauchy"), default="huber")
    parser.add_argument("--f-scale", type=float, default=1.0)
    parser.add_argument("--max-nfev", type=int, default=150)
    args = parser.parse_args()

    dataset = Path(args.dataset)
    has_physical_occlusion_labels = (
        dataset / "observation_quality_frames.json"
    ).exists()
    output = Path(args.output)
    algorithm_output = (
        Path(args.algorithm_output)
        if args.algorithm_output
        else dataset / "algorithm_output"
    )
    output.mkdir(parents=True, exist_ok=True)

    camera_data = load_json(dataset / "camera.json")
    projections = {
        camera["camera_id"]: np.asarray(camera["intrinsics"]["K"], dtype=float)
        @ np.asarray(
            camera["extrinsics"]["T_world_to_camera_opencv"], dtype=float
        )[:3, :]
        for camera in camera_data["cameras"]
    }
    scene = load_json(dataset / "scene_config.json")
    rigid_bodies = load_json(dataset / "rigid_bodies.json")["rigid_bodies"]
    rigid_body = next(
        body
        for body in rigid_bodies
        if body["rigid_body_id"] == scene["rigid_body_id"]
    )
    local_points = {
        marker["id"]: np.asarray(marker["position_cm"], dtype=float)
        for marker in rigid_body["markers"]
    }

    detection_frames = load_json(algorithm_output / "detections_2d.json")["frames"]
    initial_poses = load_json(algorithm_output / "estimated_object_pose.json")[
        "image_pipeline"
    ]
    ground_truth_poses = load_json(dataset / "object_pose_frames.json")["frames"]
    initial_by_frame = {frame["frame_index"]: frame for frame in initial_poses}
    gt_by_frame = {frame["frame_index"]: frame for frame in ground_truth_poses}
    source_metrics_path = algorithm_output / "metrics.json"
    source_pipeline = None
    if source_metrics_path.exists():
        source_metrics = load_json(source_metrics_path)
        multicamera_metrics = source_metrics.get("image_pipeline_multicamera")
        if multicamera_metrics is None:
            multicamera_metrics = source_metrics.get("image_pipeline_four_camera")
        if multicamera_metrics is not None:
            source_pipeline = {
                "protocol": source_metrics.get("protocol", {}),
                "dataset_summary": source_metrics["dataset_summary"],
                "detection": source_metrics["detection"],
                "multicamera": multicamera_metrics,
            }
    comparison_path = algorithm_output.parent / "baseline_max_area_500" / "metrics.json"
    parameter_comparison = None
    if source_pipeline is not None and comparison_path.exists():
        legacy_metrics = load_json(comparison_path)
        legacy_detection = legacy_metrics["detection"]
        selected_detection = source_pipeline["detection"]
        parameter_comparison = {
            "legacy": {
                "max_area_px": legacy_metrics["protocol"][
                    "detector_component_area_px"
                ][1],
                "matched_detection_count": legacy_detection[
                    "matched_detection_count"
                ],
                "recall": legacy_detection["recall_against_in_frame_gt"],
                "center_rmse_px": legacy_detection["center_error_px"]["rmse"],
            },
            "selected": {
                "max_area_px": source_pipeline["protocol"][
                    "detector_component_area_px"
                ][1],
                "matched_detection_count": selected_detection[
                    "matched_detection_count"
                ],
                "recall": selected_detection["recall_against_in_frame_gt"],
                "center_rmse_px": selected_detection["center_error_px"]["rmse"],
            },
        }

    frame_results = []
    all_initial_reprojection = []
    all_refined_reprojection = []
    marker_initial_reprojection = {marker_id: [] for marker_id in local_points}
    marker_refined_reprojection = {marker_id: [] for marker_id in local_points}

    for detection_frame in detection_frames:
        frame_index = detection_frame["frame_index"]
        observations = build_observations(detection_frame)
        initial_pose = initial_by_frame[frame_index]
        initial_rotation = np.asarray(initial_pose["rotation_matrix"], dtype=float)
        initial_translation = np.asarray(initial_pose["location_cm"], dtype=float)

        refined_rotation, refined_translation, diagnostics = refine_pose(
            initial_rotation,
            initial_translation,
            observations,
            projections,
            local_points,
            args.loss,
            args.f_scale,
            args.max_nfev,
        )

        initial_reprojection, initial_by_marker = point_residuals(
            initial_rotation,
            initial_translation,
            observations,
            projections,
            local_points,
        )
        refined_reprojection, refined_by_marker = point_residuals(
            refined_rotation,
            refined_translation,
            observations,
            projections,
            local_points,
        )
        all_initial_reprojection.extend(initial_reprojection)
        all_refined_reprojection.extend(refined_reprojection)
        for marker_id in local_points:
            marker_initial_reprojection[marker_id].extend(initial_by_marker[marker_id])
            marker_refined_reprojection[marker_id].extend(refined_by_marker[marker_id])

        ground_truth_transform = np.asarray(
            gt_by_frame[frame_index]["pose"]["T_object_to_world"], dtype=float
        )
        initial_error = pose_error(
            initial_rotation, initial_translation, ground_truth_transform
        )
        refined_error = pose_error(
            refined_rotation, refined_translation, ground_truth_transform
        )
        frame_results.append(
            {
                "frame_index": frame_index,
                "observation_count": len(observations),
                "camera_count": len({item["camera_id"] for item in observations}),
                "marker_count": len({item["marker_id"] for item in observations}),
                "marker_ids": sorted({item["marker_id"] for item in observations}),
                "initial_pose": pose_to_json(initial_rotation, initial_translation),
                "refined_pose": pose_to_json(refined_rotation, refined_translation),
                "initial_error": {
                    "position_vector_mm": initial_error["position_vector_mm"].tolist(),
                    "position_error_mm": initial_error["position_error_mm"],
                    "orientation_vector_deg": initial_error[
                        "orientation_vector_deg"
                    ].tolist(),
                    "orientation_error_deg": initial_error["orientation_error_deg"],
                },
                "refined_error": {
                    "position_vector_mm": refined_error["position_vector_mm"].tolist(),
                    "position_error_mm": refined_error["position_error_mm"],
                    "orientation_vector_deg": refined_error[
                        "orientation_vector_deg"
                    ].tolist(),
                    "orientation_error_deg": refined_error["orientation_error_deg"],
                },
                "initial_reprojection_px": summarize(initial_reprojection),
                "refined_reprojection_px": summarize(refined_reprojection),
                "optimization": diagnostics,
            }
        )

    initial_metrics = aggregate_pose_errors(frame_results, "initial_error")
    refined_metrics = aggregate_pose_errors(frame_results, "refined_error")
    initial_reprojection_metrics = summarize(all_initial_reprojection)
    refined_reprojection_metrics = summarize(all_refined_reprojection)
    frame_outcomes = {
        "position_improved": sum(
            frame["refined_error"]["position_error_mm"]
            < frame["initial_error"]["position_error_mm"]
            for frame in frame_results
        ),
        "orientation_improved": sum(
            frame["refined_error"]["orientation_error_deg"]
            < frame["initial_error"]["orientation_error_deg"]
            for frame in frame_results
        ),
        "both_improved": sum(
            frame["refined_error"]["position_error_mm"]
            < frame["initial_error"]["position_error_mm"]
            and frame["refined_error"]["orientation_error_deg"]
            < frame["initial_error"]["orientation_error_deg"]
            for frame in frame_results
        ),
    }
    metrics = {
        "dataset": str(dataset),
        "algorithm_output": str(algorithm_output),
        "camera_count": len(camera_data["cameras"]),
        "frames": len(frame_results),
        "accepted_frames": sum(
            frame["optimization"]["accepted"] for frame in frame_results
        ),
        "optimizer": {
            "method": "six-DoF joint multi-camera reprojection refinement",
            "initializer": "existing multi-view DLT + Kabsch pose",
            "rotation_update": "left-multiplied SO(3) rotation vector increment",
            "loss": args.loss,
            "f_scale_px": args.f_scale,
            "max_nfev": args.max_nfev,
            "identity_assignment": "existing GT-assisted Hungarian matches",
            "units": {"translation": "cm", "rotation": "radian", "residual": "pixel"},
        },
        "initial_dlt_kabsch": initial_metrics,
        "refined_pose": refined_metrics,
        "initial_reprojection_error_px": initial_reprojection_metrics,
        "refined_reprojection_error_px": refined_reprojection_metrics,
        "source_pipeline": source_pipeline,
        "parameter_comparison": parameter_comparison,
        "frame_outcomes": frame_outcomes,
        "reprojection_by_marker_px": {
            marker_id: {
                "initial": summarize(marker_initial_reprojection[marker_id]),
                "refined": summarize(marker_refined_reprojection[marker_id]),
            }
            for marker_id in local_points
        },
        "improvement_percent": {
            "position_mean": percent_reduction(
                initial_metrics["position_error_mm"]["mean"],
                refined_metrics["position_error_mm"]["mean"],
            ),
            "position_rmse": percent_reduction(
                initial_metrics["position_error_mm"]["rmse"],
                refined_metrics["position_error_mm"]["rmse"],
            ),
            "orientation_mean": percent_reduction(
                initial_metrics["orientation_error_deg"]["mean"],
                refined_metrics["orientation_error_deg"]["mean"],
            ),
            "orientation_rmse": percent_reduction(
                initial_metrics["orientation_error_deg"]["rmse"],
                refined_metrics["orientation_error_deg"]["rmse"],
            ),
            "reprojection_mean": percent_reduction(
                initial_reprojection_metrics["mean"],
                refined_reprojection_metrics["mean"],
            ),
            "reprojection_rmse": percent_reduction(
                initial_reprojection_metrics["rmse"],
                refined_reprojection_metrics["rmse"],
            ),
        },
        "limitations": [
            "Marker identities still use GT-assisted Hungarian matching.",
            f"The same {len(frame_results)}-frame sequence was used for this refinement evaluation.",
            (
                "Physical-occlusion labels are available but are not used by the optimizer."
                if has_physical_occlusion_labels
                else "Physical occlusion labels are unavailable in the dataset."
            ),
        ],
    }

    dump_json(output / "metrics.json", metrics)
    dump_json(output / "refined_object_pose.json", {"frames": frame_results})
    write_csv(output / "pose_errors.csv", frame_results)
    write_report(output / "RESULTS.md", metrics)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
