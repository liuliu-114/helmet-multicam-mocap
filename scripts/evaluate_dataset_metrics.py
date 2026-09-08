#!/usr/bin/python3
"""Evaluate the rigid-body dataset from GT pixels and from the actual PNG images.

The image pipeline deliberately keeps GT out of marker detection. GT is used only
after detection to assign the unencoded red blobs to marker IDs for evaluation.
This is the dataset guide's "GT-assisted matching" protocol, not an autonomous
cross-camera identity algorithm.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


def load_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def dump_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


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


def rotation_from_matrix(matrix):
    if hasattr(Rotation, "from_matrix"):
        return Rotation.from_matrix(matrix)
    return Rotation.from_dcm(matrix)


def projection_from_camera(camera):
    k = np.asarray(camera["intrinsics"]["K"], dtype=float)
    world_to_camera = np.asarray(
        camera["extrinsics"]["T_world_to_camera_opencv"], dtype=float
    )
    return k @ world_to_camera[:3, :]


def project(projection, xyz):
    homogeneous = projection @ np.append(np.asarray(xyz, dtype=float), 1.0)
    return homogeneous[:2] / homogeneous[2]


def triangulate_multiview(observations, projections):
    rows = []
    for camera_id, uv in observations.items():
        projection = projections[camera_id]
        u, v = uv
        rows.extend((u * projection[2] - projection[0], v * projection[2] - projection[1]))
    _, _, vh = np.linalg.svd(np.asarray(rows))
    homogeneous = vh[-1]
    return homogeneous[:3] / homogeneous[3]


def best_rigid_transform(local_points, world_points):
    local_points = np.asarray(local_points, dtype=float)
    world_points = np.asarray(world_points, dtype=float)
    local_center = local_points.mean(axis=0)
    world_center = world_points.mean(axis=0)
    covariance = (local_points - local_center).T @ (world_points - world_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = world_center - rotation @ local_center
    return rotation, translation


def estimate_component_center(
    image, labels, stats, centers, component_index, method="centroid"
):
    """Estimate a component center and return diagnostics for offline analysis."""
    centroid = np.asarray(centers[component_index], dtype=float)
    diagnostics = {
        "center_method_used": "centroid",
        "centroid_pixel_uv": centroid.tolist(),
    }
    if method == "centroid":
        return centroid, diagnostics

    left = int(stats[component_index, cv2.CC_STAT_LEFT])
    top = int(stats[component_index, cv2.CC_STAT_TOP])
    width = int(stats[component_index, cv2.CC_STAT_WIDTH])
    height = int(stats[component_index, cv2.CC_STAT_HEIGHT])
    area = int(stats[component_index, cv2.CC_STAT_AREA])
    component = (labels[top : top + height, left : left + width] == component_index)

    if method == "red_weighted":
        yy, xx = np.nonzero(component)
        pixels = image[top : top + height, left : left + width][component].astype(float)
        blue, green, red = pixels[:, 0], pixels[:, 1], pixels[:, 2]
        weights = np.maximum(red - 0.5 * (green + blue), 1.0)
        weighted = np.asarray(
            [left + np.average(xx, weights=weights), top + np.average(yy, weights=weights)],
            dtype=float,
        )
        diagnostics["center_method_used"] = "red_weighted"
        diagnostics["center_shift_from_centroid_px"] = float(
            np.linalg.norm(weighted - centroid)
        )
        return weighted, diagnostics

    component_mask = component.astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    contour = max(contours, key=cv2.contourArea) if contours else None
    if contour is None or len(contour) < 5 or area < 12:
        diagnostics["ellipse_rejection_reason"] = "insufficient_contour"
        return centroid, diagnostics

    try:
        fit = cv2.fitEllipseAMS(contour)
    except cv2.error:
        diagnostics["ellipse_rejection_reason"] = "opencv_fit_failure"
        return centroid, diagnostics

    (center_x, center_y), (axis_a, axis_b), angle_deg = fit
    ellipse_center = np.asarray([left + center_x, top + center_y], dtype=float)
    major_axis = float(max(axis_a, axis_b))
    minor_axis = float(min(axis_a, axis_b))
    axis_ratio = minor_axis / major_axis if major_axis > 0 else 0.0
    ellipse_area = np.pi * float(axis_a) * float(axis_b) / 4.0
    fill_ratio = area / ellipse_area if ellipse_area > 0 else float("inf")
    center_shift = float(np.linalg.norm(ellipse_center - centroid))
    diagnostics.update(
        {
            "ellipse_center_pixel_uv": ellipse_center.tolist(),
            "ellipse_axes_px": [float(axis_a), float(axis_b)],
            "ellipse_angle_deg": float(angle_deg),
            "ellipse_axis_ratio": float(axis_ratio),
            "ellipse_fill_ratio": float(fill_ratio),
            "center_shift_from_centroid_px": center_shift,
        }
    )

    if method == "ellipse":
        diagnostics["center_method_used"] = "ellipse"
        return ellipse_center, diagnostics

    valid_hybrid_fit = (
        np.all(np.isfinite(ellipse_center))
        and major_axis >= 3.0
        and axis_ratio >= 0.20
        and 0.45 <= fill_ratio <= 1.35
        and -0.5 <= center_x <= width - 0.5
        and -0.5 <= center_y <= height - 0.5
        and center_shift <= max(1.5, 0.15 * major_axis)
    )
    if valid_hybrid_fit:
        diagnostics["center_method_used"] = "ellipse_hybrid"
        return ellipse_center, diagnostics

    diagnostics["ellipse_rejection_reason"] = "quality_gate"
    return centroid, diagnostics


def detect_red_markers(
    image, min_area_px=2, max_area_px=500, center_method="red_weighted"
):
    """Return independent red/orange components with configurable centers."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 45, 100), (25, 255, 255))
    high_red = cv2.inRange(hsv, (165, 45, 100), (179, 255, 255))
    mask = cv2.bitwise_or(low_red, high_red)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    detections = []
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if min_area_px <= area <= max_area_px:
            center, diagnostics = estimate_component_center(
                image, labels, stats, centers, index, method=center_method
            )
            detections.append(
                {
                    "pixel_uv": [float(center[0]), float(center[1])],
                    "area_px": area,
                    **diagnostics,
                }
            )
    return detections, mask


def match_detections(gt_markers, detections, max_distance_px):
    visible_gt = [marker for marker in gt_markers if marker["pixel_uv"] is not None]
    if not visible_gt or not detections:
        return {}, list(range(len(detections))), []

    gt_pixels = np.asarray([marker["pixel_uv"] for marker in visible_gt], dtype=float)
    detected_pixels = np.asarray([item["pixel_uv"] for item in detections], dtype=float)
    costs = np.linalg.norm(gt_pixels[:, None, :] - detected_pixels[None, :, :], axis=2)
    gt_indices, detection_indices = linear_sum_assignment(costs)

    matches = {}
    errors = []
    used_detections = set()
    for gt_index, detection_index in zip(gt_indices, detection_indices):
        distance = float(costs[gt_index, detection_index])
        if distance > max_distance_px:
            continue
        marker_id = visible_gt[gt_index]["marker_id"]
        matches[marker_id] = detections[detection_index]["pixel_uv"]
        errors.append(distance)
        used_detections.add(int(detection_index))
    unmatched = [index for index in range(len(detections)) if index not in used_detections]
    return matches, unmatched, errors


class DatasetFrameReader:
    """Read synchronized frames from PNG directories or sequential MP4 files."""

    def __init__(self, dataset, camera_ids, input_source="auto"):
        self.dataset = dataset
        self.camera_ids = list(camera_ids)
        self.captures = {}
        self.next_video_frame = {camera_id: 0 for camera_id in self.camera_ids}
        png_available = all(
            any((dataset / "images" / camera_id).glob("frame_*.png"))
            for camera_id in self.camera_ids
        )
        if input_source == "png" and not png_available:
            raise RuntimeError("PNG input requested, but one or more camera image directories are missing")
        self.use_png = png_available and input_source != "video"
        if self.use_png:
            self.description = "lossless PNG"
            return

        self.description = "H.264 MP4 decoded sequentially with OpenCV"
        for camera_id in self.camera_ids:
            video_path = dataset / "videos" / f"{camera_id}.mp4"
            capture = cv2.VideoCapture(str(video_path))
            if not capture.isOpened():
                self.close()
                raise RuntimeError(f"Cannot open {video_path}")
            self.captures[camera_id] = capture

    def read(self, camera_id, frame_index):
        if self.use_png:
            candidates = (
                self.dataset
                / "images"
                / camera_id
                / f"frame_{frame_index:04d}.png",
                self.dataset
                / "images"
                / camera_id
                / f"frame_{frame_index:06d}.png",
            )
            image_path = next((path for path in candidates if path.exists()), candidates[0])
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"Cannot read {image_path}")
            return image

        expected = self.next_video_frame[camera_id]
        if frame_index != expected:
            raise RuntimeError(
                f"Non-sequential frame request for {camera_id}: expected {expected}, got {frame_index}"
            )
        ok, image = self.captures[camera_id].read()
        if not ok or image is None:
            raise RuntimeError(
                f"Cannot decode {camera_id} frame {frame_index} from MP4"
            )
        self.next_video_frame[camera_id] += 1
        return image

    def close(self):
        for capture in self.captures.values():
            capture.release()
        self.captures.clear()


def evaluate_observations(
    name, frames_observations, projections, gt3d_frames, pose_frames, local_points
):
    marker_errors_mm = []
    reprojection_errors_px = []
    position_vectors_mm = []
    orientation_vectors_deg = []
    marker_errors_by_id_mm = {marker_id: [] for marker_id in local_points}
    pose_marker_counts = []
    reconstructed_output = []
    pose_output = []

    for observations, gt3d_frame, pose_frame in zip(
        frames_observations, gt3d_frames, pose_frames
    ):
        gt_world = {
            marker["marker_id"]: np.asarray(marker["world_position_cm"], dtype=float)
            for marker in gt3d_frame["markers"]
        }
        reconstructed = {}
        marker_views = {}
        for marker_id in sorted(gt_world):
            views = {
                camera_id: camera_markers[marker_id]
                for camera_id, camera_markers in observations.items()
                if marker_id in camera_markers
            }
            if len(views) < 2:
                continue
            xyz = triangulate_multiview(views, projections)
            reconstructed[marker_id] = xyz
            marker_views[marker_id] = len(views)
            marker_error_mm = float(np.linalg.norm(xyz - gt_world[marker_id]) * 10.0)
            marker_errors_mm.append(marker_error_mm)
            marker_errors_by_id_mm[marker_id].append(marker_error_mm)
            for camera_id, uv in views.items():
                reprojection_errors_px.append(
                    float(np.linalg.norm(project(projections[camera_id], xyz) - uv))
                )

        reconstructed_output.append(
            {
                "frame_index": gt3d_frame["frame_index"],
                "markers": [
                    {
                        "marker_id": marker_id,
                        "world_position_cm": reconstructed[marker_id].tolist(),
                        "camera_view_count": marker_views[marker_id],
                    }
                    for marker_id in sorted(reconstructed)
                ],
            }
        )

        if len(reconstructed) < 3:
            continue
        marker_ids = sorted(reconstructed)
        pose_marker_counts.append(len(marker_ids))
        rotation, translation = best_rigid_transform(
            [local_points[marker_id] for marker_id in marker_ids],
            [reconstructed[marker_id] for marker_id in marker_ids],
        )
        gt_transform = np.asarray(pose_frame["pose"]["T_object_to_world"], dtype=float)
        position_vector_mm = (translation - gt_transform[:3, 3]) * 10.0
        error_rotation = rotation @ gt_transform[:3, :3].T
        orientation_vector_deg = np.degrees(rotation_from_matrix(error_rotation).as_rotvec())
        position_vectors_mm.append(position_vector_mm)
        orientation_vectors_deg.append(orientation_vector_deg)
        pose_output.append(
            {
                "frame_index": pose_frame["frame_index"],
                "marker_ids": marker_ids,
                "location_cm": translation.tolist(),
                "rotation_matrix": rotation.tolist(),
                "position_error_mm": float(np.linalg.norm(position_vector_mm)),
                "orientation_error_deg": float(np.linalg.norm(orientation_vector_deg)),
            }
        )

    position_vectors_mm = np.asarray(position_vectors_mm, dtype=float).reshape(-1, 3)
    orientation_vectors_deg = np.asarray(orientation_vectors_deg, dtype=float).reshape(-1, 3)
    position_norms = np.linalg.norm(position_vectors_mm, axis=1)
    orientation_norms = np.linalg.norm(orientation_vectors_deg, axis=1)
    position_bias = (
        position_vectors_mm.mean(axis=0) if len(position_vectors_mm) else None
    )
    orientation_bias = (
        orientation_vectors_deg.mean(axis=0)
        if len(orientation_vectors_deg)
        else None
    )
    position_precision = (
        np.sqrt(np.mean(np.sum((position_vectors_mm - position_bias) ** 2, axis=1)))
        if len(position_vectors_mm)
        else None
    )
    orientation_precision = (
        np.sqrt(np.mean(np.sum((orientation_vectors_deg - orientation_bias) ** 2, axis=1)))
        if len(orientation_vectors_deg)
        else None
    )

    metrics = {
        "name": name,
        "triangulated_marker_observations": len(marker_errors_mm),
        "pose_frames": len(position_vectors_mm),
        "marker_3d_error_mm": summarize(marker_errors_mm),
        "marker_3d_error_by_id_mm": {
            marker_id: summarize(errors)
            for marker_id, errors in marker_errors_by_id_mm.items()
        },
        "pose_marker_count": {
            "mean": float(np.mean(pose_marker_counts)) if pose_marker_counts else None,
            "min": int(min(pose_marker_counts)) if pose_marker_counts else None,
            "max": int(max(pose_marker_counts)) if pose_marker_counts else None,
            "histogram": {
                str(count): pose_marker_counts.count(count)
                for count in sorted(set(pose_marker_counts))
            },
        },
        "triangulation_reprojection_error_px": summarize(reprojection_errors_px),
        "position_accuracy": {
            "error_mm": summarize(position_norms),
            "bias_xyz_mm": position_bias.tolist() if position_bias is not None else None,
            "bias_norm_mm": (
                float(np.linalg.norm(position_bias))
                if position_bias is not None
                else None
            ),
            "mean_error_percent_of_2m": float(position_norms.mean() / 20.0)
            if len(position_norms)
            else None,
        },
        "position_precision": {
            "axis_std_mm": position_vectors_mm.std(axis=0).tolist()
            if len(position_vectors_mm)
            else None,
            "centered_3d_rms_mm": float(position_precision)
            if position_precision is not None
            else None,
        },
        "orientation_accuracy": {
            "geodesic_error_deg": summarize(orientation_norms),
            "small_angle_bias_xyz_deg": (
                orientation_bias.tolist() if orientation_bias is not None else None
            ),
            "small_angle_bias_norm_deg": (
                float(np.linalg.norm(orientation_bias))
                if orientation_bias is not None
                else None
            ),
        },
        "orientation_precision": {
            "small_angle_axis_std_deg": orientation_vectors_deg.std(axis=0).tolist()
            if len(orientation_vectors_deg)
            else None,
            "centered_3d_rms_deg": float(orientation_precision)
            if orientation_precision is not None
            else None,
        },
    }
    return metrics, reconstructed_output, pose_output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", nargs="?", default="dataset_rigidbody_0001")
    parser.add_argument("--match-radius", type=float, default=8.0)
    parser.add_argument("--min-area", type=int, default=2)
    parser.add_argument("--max-area", type=int, default=500)
    parser.add_argument(
        "--center-method",
        choices=("centroid", "red_weighted", "ellipse", "ellipse_hybrid"),
        default="red_weighted",
    )
    parser.add_argument(
        "--input-source",
        choices=("auto", "png", "video"),
        default="auto",
        help="frame source; auto keeps the existing PNG-first behavior",
    )
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    dataset = Path(args.dataset)
    output_dir = Path(args.output) if args.output else dataset / "algorithm_output"
    camera_data = load_json(dataset / "camera.json")
    gt2d_frames = load_json(dataset / "gt_2d_frames.json")["frames"]
    gt3d_frames = load_json(dataset / "gt_3d_frames.json")["frames"]
    pose_frames = load_json(dataset / "object_pose_frames.json")["frames"]
    rigid_bodies = load_json(dataset / "rigid_bodies.json")["rigid_bodies"]
    scene = load_json(dataset / "scene_config.json")
    observation_quality_path = dataset / "observation_quality_frames.json"
    has_physical_occlusion_labels = observation_quality_path.exists()

    rigid_body_id = scene["rigid_body_id"]
    rigid_body = next(
        body for body in rigid_bodies if body["rigid_body_id"] == rigid_body_id
    )
    local_points = {
        marker["id"]: np.asarray(marker["position_cm"], dtype=float)
        for marker in rigid_body["markers"]
    }
    cameras = {camera["camera_id"]: camera for camera in camera_data["cameras"]}
    projections = {
        camera_id: projection_from_camera(camera) for camera_id, camera in cameras.items()
    }

    gt_observations = []
    image_observations = []
    detections_output = []
    center_errors_px = []
    total_gt_in_frame = 0
    false_positive_count = 0
    image_shapes = set()
    per_camera_detection = {
        camera_id: {"gt": 0, "matched": 0, "unmatched": 0, "errors": []}
        for camera_id in cameras
    }
    per_marker_detection = {
        marker_id: {"gt": 0, "matched": 0, "errors": []}
        for marker_id in local_points
    }
    frame_reader = DatasetFrameReader(dataset, cameras, args.input_source)
    overlay_indices = {0, len(gt2d_frames) // 2, len(gt2d_frames) - 1}

    for gt2d_frame in gt2d_frames:
        frame_index = gt2d_frame["frame_index"]
        gt_frame_observations = {}
        image_frame_observations = {}
        frame_detection_output = {"frame_index": frame_index, "cameras": {}}
        for camera_id, camera_frame in gt2d_frame["cameras"].items():
            gt_markers = camera_frame["markers"]
            gt_frame_observations[camera_id] = {
                marker["marker_id"]: marker["pixel_uv"]
                for marker in gt_markers
                if marker["pixel_uv"] is not None
            }
            total_gt_in_frame += len(gt_frame_observations[camera_id])
            per_camera_detection[camera_id]["gt"] += len(
                gt_frame_observations[camera_id]
            )
            for marker in gt_markers:
                if marker["pixel_uv"] is not None:
                    per_marker_detection[marker["marker_id"]]["gt"] += 1

            image = frame_reader.read(camera_id, frame_index)
            image_shapes.add(tuple(image.shape))
            detections, _ = detect_red_markers(
                image,
                min_area_px=args.min_area,
                max_area_px=args.max_area,
                center_method=args.center_method,
            )
            matches, unmatched, errors = match_detections(
                gt_markers, detections, args.match_radius
            )
            image_frame_observations[camera_id] = matches
            center_errors_px.extend(errors)
            false_positive_count += len(unmatched)
            per_camera_detection[camera_id]["matched"] += len(errors)
            per_camera_detection[camera_id]["unmatched"] += len(unmatched)
            per_camera_detection[camera_id]["errors"].extend(errors)
            gt_pixels_by_id = {
                marker["marker_id"]: np.asarray(marker["pixel_uv"], dtype=float)
                for marker in gt_markers
                if marker["pixel_uv"] is not None
            }
            for marker_id, pixel in matches.items():
                error = float(
                    np.linalg.norm(np.asarray(pixel, dtype=float) - gt_pixels_by_id[marker_id])
                )
                per_marker_detection[marker_id]["matched"] += 1
                per_marker_detection[marker_id]["errors"].append(error)
            frame_detection_output["cameras"][camera_id] = {
                "detections": detections,
                "matched_by_gt_for_evaluation": [
                    {"marker_id": marker_id, "pixel_uv": pixel}
                    for marker_id, pixel in sorted(matches.items())
                ],
                "unmatched_detection_indices": unmatched,
            }

            if frame_index in overlay_indices:
                overlay = image.copy()
                gt_by_id = {
                    marker["marker_id"]: marker["pixel_uv"]
                    for marker in gt_markers
                    if marker["pixel_uv"] is not None
                }
                for marker_id, uv in gt_by_id.items():
                    center = tuple(int(round(value)) for value in uv)
                    cv2.circle(overlay, center, 5, (255, 0, 0), 1)
                    cv2.putText(
                        overlay,
                        marker_id,
                        (center[0] + 5, center[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.35,
                        (255, 0, 0),
                        1,
                        cv2.LINE_AA,
                    )
                for detection in detections:
                    center = tuple(
                        int(round(value)) for value in detection["pixel_uv"]
                    )
                    cv2.drawMarker(
                        overlay,
                        center,
                        (0, 255, 0),
                        cv2.MARKER_CROSS,
                        7,
                        1,
                    )
                for marker_id, uv in matches.items():
                    cv2.line(
                        overlay,
                        tuple(int(round(value)) for value in gt_by_id[marker_id]),
                        tuple(int(round(value)) for value in uv),
                        (0, 0, 255),
                        1,
                    )
                overlay_dir = output_dir / "overlays"
                overlay_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(
                    str(overlay_dir / f"{camera_id}_frame_{frame_index:04d}.png"),
                    overlay,
                )
        gt_observations.append(gt_frame_observations)
        image_observations.append(image_frame_observations)
        detections_output.append(frame_detection_output)
    frame_reader.close()

    camera_count = len(cameras)
    gt_metrics, gt_reconstructed, gt_poses = evaluate_observations(
        f"GT pixels -> {camera_count}-camera DLT -> Kabsch",
        gt_observations,
        projections,
        gt3d_frames,
        pose_frames,
        local_points,
    )
    image_metrics, image_reconstructed, image_poses = evaluate_observations(
        f"{frame_reader.description} red detection -> GT-assisted ID -> {camera_count}-camera DLT -> Kabsch",
        image_observations,
        projections,
        gt3d_frames,
        pose_frames,
        local_points,
    )
    first_two = list(cameras)[:2]
    two_camera_observations = [
        {camera_id: frame[camera_id] for camera_id in first_two} for frame in image_observations
    ]
    two_camera_metrics, _, _ = evaluate_observations(
        f"{frame_reader.description} red detection -> GT-assisted ID -> Camera_01/02 DLT -> Kabsch",
        two_camera_observations,
        projections,
        gt3d_frames,
        pose_frames,
        local_points,
    )

    camera_locations = [
        np.asarray(camera["extrinsics"]["ue_world_position_cm"], dtype=float)
        for camera in cameras.values()
    ]
    object_locations = [
        np.asarray(frame["pose"]["location_cm"], dtype=float) for frame in pose_frames
    ]
    tracking_distances_cm = [
        np.linalg.norm(location - camera_location)
        for location in object_locations
        for camera_location in camera_locations
    ]
    metrics = {
        "dataset": str(dataset),
        "protocol": {
            "image_source": frame_reader.description,
            "detector": "HSV red/orange threshold + 3x3 close + connected components",
            "detector_component_area_px": [args.min_area, args.max_area],
            "detector_center_method": args.center_method,
            "identity_assignment": (
                f"Hungarian against GT pixel_uv, {args.match_radius:g} px gate"
            ),
            "triangulation": "multi-view linear DLT in UE world coordinates (cm)",
            "pose": "Kabsch/SVD, no scale",
            "orientation_error": "SO(3) geodesic angle",
        },
        "dataset_summary": {
            "cameras": len(cameras),
            "frames": len(gt2d_frames),
            "fps": scene["fps"],
            "duration_sec": scene["duration_sec"],
            "markers": len(local_points),
            "image_shapes_bgr": [list(shape) for shape in sorted(image_shapes)],
            "tracking_distance_object_to_camera_cm": {
                "min": float(min(tracking_distances_cm)),
                "max": float(max(tracking_distances_cm)),
            },
        },
        "detection": {
            "gt_in_frame_count": total_gt_in_frame,
            "matched_detection_count": len(center_errors_px),
            "recall_against_in_frame_gt": len(center_errors_px) / total_gt_in_frame,
            "unmatched_detection_count": false_positive_count,
            "center_error_px": summarize(center_errors_px),
            "per_camera": {
                camera_id: {
                    "gt_in_frame_count": values["gt"],
                    "matched_detection_count": values["matched"],
                    "recall_against_in_frame_gt": values["matched"] / values["gt"],
                    "unmatched_detection_count": values["unmatched"],
                    "center_error_px": summarize(values["errors"]),
                }
                for camera_id, values in per_camera_detection.items()
            },
            "per_marker": {
                marker_id: {
                    "gt_in_frame_count": values["gt"],
                    "matched_detection_count": values["matched"],
                    "recall_against_in_frame_gt": values["matched"] / values["gt"],
                    "center_error_px": summarize(values["errors"]),
                }
                for marker_id, values in per_marker_detection.items()
            },
            "visibility_caveat": (
                "Dataset provides physical-occlusion labels, but this recall uses all "
                "in-frame GT projections as its denominator."
                if has_physical_occlusion_labels
                else "GT in_frame does not test physical occlusion."
            ),
        },
        "gt_geometry_baseline": gt_metrics,
        "image_pipeline_multicamera": image_metrics,
        "image_pipeline_project_native_two_camera_scope": two_camera_metrics,
        "claim_limits": [
            "GT is used to assign IDs after detection because markers are visually unencoded.",
            (
                "Physical-occlusion labels exist, but the reported in-frame recall does not "
                "exclude occluded projections."
                if has_physical_occlusion_labels
                else "The dataset has no physical-occlusion labels, so in-frame recall is a lower bound."
            ),
            "The project ROS detector targets NIR retroreflectors; this dataset requires the red detector.",
            f"The project ROS triangulator implements two cameras only; {camera_count}-camera DLT is an offline extension.",
            "Precision values are centered residual scatter over a moving sequence, not ISO-style repeatability at repeated static poses.",
            "MP4 input may add compression error relative to the dataset's intended lossless PNG evaluation source."
            if not frame_reader.use_png
            else "Lossless PNG input avoids video compression error.",
        ],
    }

    dump_json(output_dir / "detections_2d.json", {"frames": detections_output})
    dump_json(
        output_dir / "reconstructed_markers_3d.json",
        {"gt_pixel_baseline": gt_reconstructed, "image_pipeline": image_reconstructed},
    )
    dump_json(
        output_dir / "estimated_object_pose.json",
        {"gt_pixel_baseline": gt_poses, "image_pipeline": image_poses},
    )
    dump_json(output_dir / "metrics.json", metrics)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
