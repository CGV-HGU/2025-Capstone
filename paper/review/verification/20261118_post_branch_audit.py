"""Read-only audit: real source functions with synthetic inputs, plus existing CSVs.

Does not import ROS, load a model, publish ROS messages, or access a robot.
"""
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from collections import Counter, deque
import statistics
import sys
import time

import cv2
import numpy as np

root = Path(sys.argv[1]).resolve()


def read_rows(relative):
    with (root / relative).open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def original_function(relative, name, class_name=None, namespace=None):
    path = root / relative
    tree = ast.parse(path.read_text())
    nodes = tree.body
    if class_name:
        nodes = next(n for n in nodes if isinstance(n, ast.ClassDef) and n.name == class_name).body
    node = next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == name)
    scope = {'np': np, 'cv2': cv2, 'time': time}
    scope.update(namespace or {})
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), scope)
    return scope[name]


floor_path = 'src/freespace_detection/freespace_detection/floor_detector.py'
scan_path = 'src/fake_lidar_with_tf/fake_lidar_with_tf/fake_lidar_with_tf.py'
benchmark_path = 'experiments/benchmark_depth_baselines.py'
out = {'scope': 'Local source/CSV audit; synthetic function probes; no model inference or robot execution.'}
out['source_sha256'] = {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                        for name in [floor_path, scan_path, benchmark_path, 'experiments/run_dual_bag_benchmark.py']}

# Exercise the actual image_callback, using injected model outputs.
class Tensor:
    def __init__(self, data):
        self.data = data

    def cpu(self):
        return self

    def numpy(self):
        return self.data


callback = original_function(floor_path, 'image_callback', 'RoiChecker',
                             {'CvBridgeError': RuntimeError, 'Bool': lambda **kw: NS(**kw)})
floor = np.ones((256, 320), dtype=np.uint8)
floor[145:235, 130:190] = 0
person = np.zeros_like(floor)
person[145:235, 130:190] = 1


def process_supplied_masks(masks):
    captured = []
    result = NS(masks=NS(data=[Tensor(mask) for mask in masks]),
                names={0: 'person', 1: 'floor'},
                boxes=NS(cls=np.array([1] if len(masks) == 1 else [1, 0])))
    state = NS(last_time=0.0, inference_interval=0.1,
               bridge=NS(imgmsg_to_cv2=lambda *_: np.zeros((480, 640, 3), dtype=np.uint8)),
               target_w=320, target_h=256, conf_threshold=0.15,
               model=lambda *_, **__: [result],
               publish_channel_distances=lambda mask: captured.append(mask.copy()),
               roi_slice=(slice(201, 252), slice(70, 251)), roi_area=9000,
               in_roi_history=deque(maxlen=5), true_since=None, cleared_once=False,
               costmap_obstacle_duration=15.0, roi_pub=NS(publish=lambda _: None),
               enable_gui=False)
    callback(state, None)
    return int(np.count_nonzero(captured[0][145:235, 130:190] == 0))


out['synthetic_class_union'] = {
    'input': 'A 90x60 non-floor rectangle; person mask fills the same rectangle.',
    'remaining_nonfloor_pixels_floor_only': process_supplied_masks([floor]),
    'remaining_nonfloor_pixels_floor_plus_person': process_supplied_masks([floor, person]),
    'limitation': 'Supplied masks only; does not measure real model/person detection accuracy.'}

# Exercise real temporal filtering, preserving state between calls.
temporal = original_function(floor_path, 'apply_temporal_filter', 'RoiChecker')
state = NS(alpha=0.6, obs_persist_frames=2, clear_persist_frames=2,
           filtered_dist=np.full(1, np.inf), obs_counter=np.zeros(1, dtype=int),
           inf_counter=np.zeros(1, dtype=int))
sequence = [np.inf, 3.0, 3.0, np.inf, np.inf]
filtered = [float(temporal(state, np.array([value]))[0]) for value in sequence]
finite_or_none = lambda values: [v if math.isfinite(v) else None for v in values]
out['synthetic_temporal_filter'] = {'input_m': finite_or_none(sequence),
                                    'output_m': finite_or_none(filtered),
                                    'null_means': 'positive infinity',
                                    'timing_note': 'One extra processing sample before confirming onset; not a measured millisecond latency.'}

# Exercise the timer callback with no intervening channel update.
def transform():
    return NS(header=NS(), transform=NS(translation=NS(), rotation=NS()))


timer = original_function(scan_path, 'timer_callback', 'FakeLidarWithTF',
                          {'TransformStamped': transform, 'LaserScan': NS, 'Header': NS})
published = []
clock = NS(seconds=0.0)
state = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: clock.seconds)),
           tf_broadcaster=NS(sendTransform=lambda _: None),
           scan_pub=NS(publish=lambda msg: published.append(msg)),
           angle_min=-math.radians(35), angle_max=math.radians(35),
           angle_increment=math.radians(0.5), range_min=0.1, range_max=10.0,
           num_readings=141, channel_distances=np.full(141, 3.0))
for seconds in [0.0, 5.0]:
    clock.seconds = seconds
    timer(state)
out['synthetic_stale_scan'] = {
    'input_update_count': 1,
    'timer_times_s': [msg.header.stamp for msg in published],
    'output_ranges_equal': published[0].ranges == published[1].ranges,
    'limitation': 'Direct timer invocation with stub messages/clock; no ROS runtime or robot.'}

# Verify the static evaluation's no-detection behavior without loading any model.
ranging = original_function(benchmark_path, 'evaluate_ranging_accuracy')
estimate, error, mae = ranging(NS(infer=lambda _: (np.full(141, np.inf), None)), None, 2.5)
out['synthetic_no_detection_ranging'] = {'valid_input_ranges': 0, 'ground_truth_m': 2.5,
                                         'reported_estimate_m': estimate, 'reported_error_m': error,
                                         'reported_mae_m': mae}

inventory = read_rows('experiments/logs/pc_extracted/rosbag_inventory_summary.csv')
out['inventory_csv'] = {'bags': len(inventory),
                        'groups': dict(Counter(row['source_group'] for row in inventory)),
                        'with_camera': sum(int(row['cnt_cam'] or 0) > 0 for row in inventory),
                        'camera_messages': sum(int(row['cnt_cam'] or 0) for row in inventory),
                        'status': dict(Counter(row['status'] for row in inventory))}
runs = read_rows('experiments/logs/pc_extracted/minimal_A_scen1_20runs_metrics.csv')
out['twenty_run_csv'] = {'rows': len(runs), 'camera_messages': sum(int(row['cnt_cam'] or 0) for row in runs),
                         'plan_messages': sum(int(row['cnt_plan'] or 0) for row in runs),
                         'endpoint_x_at_least_9_5m': sum(float(row['end_x_m']) >= 9.5 for row in runs),
                         'has_action_or_success_outcome': any('result' in key or 'success' in key for key in runs[0])}

rows = []
for filename in ['benchmark_run01.csv', 'benchmark_run02.csv']:
    rows.extend({k: float(v) for k, v in row.items()}
                for row in read_rows('experiments/logs/' + filename))
# Match run_dual_bag_benchmark.py's existing selection exactly.
selected = [row for row in rows if row['robot_x'] < 3.8 and
            2.18 <= row['gt_distance'] <= 5.0 and math.isfinite(row['vl_dist'])]
metrics = {'input_rows': len(rows), 'selected_rows': len(selected),
           'selection': 'robot_x < 3.8 and 2.18 <= gt_distance <= 5.0 and finite(vl_dist)',
           'caution': 'Reproduces legacy CSV aggregation, not independent ground truth validation.'}
for name in ['vl', 'midas', 'da']:
    errors = [abs(row[name + '_dist'] - row['gt_distance']) for row in selected
              if math.isfinite(row[name + '_dist'])]
    latency = [row[name + '_latency_ms'] for row in rows]
    metrics[name] = {'mae_m': statistics.mean(errors),
                     'latency_ms_mean': statistics.mean(latency),
                     'latency_ms_population_sd': statistics.pstdev(latency)}
out['legacy_runner_reaggregation'] = metrics

lutdir = root / 'src/freespace_detection/scripts'
col = np.load(lutdir / 'col_to_ch_lut.npy')
dist = np.load(lutdir / 'distance_lut_2d.npy')
out['committed_luts'] = {'declared_channels': 141, 'mapped_channels': len(np.unique(col)),
                         'mapped_min_channel': int(col.min()), 'mapped_max_channel': int(col.max()),
                         'minimum_finite_2d_range_m': float(dist[np.isfinite(dist)].min()),
                         'caution': 'A LUT support bound, not measured obstacle clearance.'}

def wilson(k, n):
    z = statistics.NormalDist().inv_cdf(0.975)
    p = k / n
    denominator = 1 + z*z/n
    center = (p + z*z/(2*n)) / denominator
    delta = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / denominator
    return [center - delta, center + delta]


out['illustrative_wilson_intervals'] = {'9_of_10': wilson(9, 10), '10_of_10': wilson(10, 10),
                                        '18_of_20': wilson(18, 20),
                                        'note': 'Statistical illustrations, not new experimental outcomes.'}
print(json.dumps(out, ensure_ascii=False, indent=2, allow_nan=False))
