#!/usr/bin/env python3
"""Re-evaluate cached marker detections over several component-area thresholds."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from evaluate_dataset_metrics import (
    dump_json,
    evaluate_observations,
    load_json,
    match_detections,
    projection_from_camera,
    summarize,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("detections", type=Path)
    parser.add_argument("--thresholds", default="2,5,10,15,20,25,30,35,40,45,50,60,75,100")
    parser.add_argument("--max-area", type=int, default=1000)
    parser.add_argument("--match-radius", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--materialize-dir",
        type=Path,
        help="optionally write cached candidate inputs for pose refinement",
    )
    args = parser.parse_args()

    camera_data = load_json(args.dataset / "camera.json")
    cameras = {item["camera_id"]: item for item in camera_data["cameras"]}
    projections = {
        camera_id: projection_from_camera(camera)
        for camera_id, camera in cameras.items()
    }
    gt2d_frames = load_json(args.dataset / "gt_2d_frames.json")["frames"]
    gt3d_frames = load_json(args.dataset / "gt_3d_frames.json")["frames"]
    pose_frames = load_json(args.dataset / "object_pose_frames.json")["frames"]
    scene = load_json(args.dataset / "scene_config.json")
    rigid_bodies = load_json(args.dataset / "rigid_bodies.json")["rigid_bodies"]
    rigid_body = next(
        body for body in rigid_bodies
        if body["rigid_body_id"] == scene["rigid_body_id"]
    )
    local_points = {
        marker["id"]: np.asarray(marker["position_cm"], dtype=float)
        for marker in rigid_body["markers"]
    }
    cached_frames = load_json(args.detections)["frames"]
    total_gt = sum(
        marker["pixel_uv"] is not None
        for frame in gt2d_frames
        for camera in frame["cameras"].values()
        for marker in camera["markers"]
    )

    rows = []
    for minimum in [int(value) for value in args.thresholds.split(",")]:
        observations = []
        materialized_frames = []
        center_errors = []
        unmatched_count = 0
        for gt_frame, cached_frame in zip(gt2d_frames, cached_frames):
            frame_observations = {}
            materialized_frame = {
                "frame_index": gt_frame["frame_index"],
                "cameras": {},
            }
            for camera_id, camera_frame in gt_frame["cameras"].items():
                detections = [
                    item
                    for item in cached_frame["cameras"][camera_id]["detections"]
                    if minimum <= item["area_px"] <= args.max_area
                ]
                matches, unmatched, errors = match_detections(
                    camera_frame["markers"], detections, args.match_radius
                )
                frame_observations[camera_id] = matches
                materialized_frame["cameras"][camera_id] = {
                    "detections": detections,
                    "matched_by_gt_for_evaluation": [
                        {"marker_id": marker_id, "pixel_uv": pixel}
                        for marker_id, pixel in sorted(matches.items())
                    ],
                    "unmatched_detection_indices": unmatched,
                }
                center_errors.extend(errors)
                unmatched_count += len(unmatched)
            observations.append(frame_observations)
            materialized_frames.append(materialized_frame)
        metrics, reconstructed, poses = evaluate_observations(
            f"cached detections, area {minimum}-{args.max_area}",
            observations,
            projections,
            gt3d_frames,
            pose_frames,
            local_points,
        )
        rows.append(
            {
                "min_area": minimum,
                "max_area": args.max_area,
                "matched": len(center_errors),
                "recall": len(center_errors) / total_gt,
                "unmatched": unmatched_count,
                "center_error_px": summarize(center_errors),
                "marker_3d_error_mm": metrics["marker_3d_error_mm"],
                "pose_frames": metrics["pose_frames"],
                "pose_marker_count": metrics["pose_marker_count"],
                "position_error_mm": metrics["position_accuracy"]["error_mm"],
                "orientation_error_deg": metrics["orientation_accuracy"]["geodesic_error_deg"],
            }
        )
        if args.materialize_dir:
            candidate_dir = args.materialize_dir / f"min_area_{minimum}"
            dump_json(
                candidate_dir / "detections_2d.json",
                {"frames": materialized_frames},
            )
            dump_json(
                candidate_dir / "reconstructed_markers_3d.json",
                {"image_pipeline": reconstructed},
            )
            dump_json(
                candidate_dir / "estimated_object_pose.json",
                {"image_pipeline": poses},
            )

    text = json.dumps({"threshold_scan": rows}, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
