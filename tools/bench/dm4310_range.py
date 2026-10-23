#!/usr/bin/env python3
"""Offline validation of two disabled, stationary dual-DM4310 endpoint captures.

No device access. Produces a candidate only, never a motion authorization.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from statistics import median


def endpoint(rows, velocity_range=30):
    if velocity_range not in (30, 50):
        raise ValueError('速度编码范围需明确为已支持的30或50 rad/s')
    if len(rows) != 40:
        raise ValueError('端点必须包含完整的双轴20轮采样（40帧）')
    axes = {1: [], 2: []}
    last_time = -1.0
    for index, row in enumerate(rows):
        motor = row['motor_id']
        if motor != index % 2 + 1 or row['feedback_can_id'] != motor + 3:
            raise ValueError('ID、路由或采样顺序不匹配')
        t, q, v = (row[k] for k in ('t_s', 'position_rad', 'velocity_rad_s'))
        if not all(math.isfinite(x) for x in (t, q, v)) or t <= last_time:
            raise ValueError('非有限值或时间顺序错误')
        if row['status'] != 0 or abs(v) > 0.05 or abs(q) > 12.5:
            raise ValueError('电机未失能、未停稳或位置超出编码区间')
        raw = bytes.fromhex(row['raw'])
        if len(raw) != 8 or raw[0] != motor:
            raise ValueError('原始帧身份/状态不匹配')
        raw_q = ((raw[1] << 8) | raw[2]) * 25 / 65535 - 12.5
        raw_v = ((raw[3] << 4) | (raw[4] >> 4)) * (2*velocity_range) / 4095 - velocity_range
        if abs(q - raw_q) > 1e-9 or abs(v - raw_v) > 1e-9:
            raise ValueError('数值与普通DM4310原始帧换算不匹配')
        axes[motor].append((t, q))
        last_time = t
    result = {}
    for motor, samples in axes.items():
        times, positions = zip(*samples)
        if not 1.7 <= times[-1] - times[0] <= 3:
            raise ValueError('端点采样窗口长度异常')
        if max(b-a for a, b in zip(times, times[1:])) > 0.2:
            raise ValueError('反馈间隔过大')
        if max(positions) - min(positions) > 0.005:
            raise ValueError('端点位置不稳定')
        result[motor] = {'median_rad': median(positions),
                         'observed_min_rad': min(positions),
                         'observed_max_rad': max(positions)}
    return result


def candidate(first, second, margin_deg, velocity_range=30):
    if not math.isfinite(margin_deg) or margin_deg <= 0:
        raise ValueError('必须明确给出正的内缩余量（度）')
    margin = math.radians(margin_deg)
    # Keep the complete boundary guard inside the observed endpoints as well.
    if margin <= 0.02:
        raise ValueError('内缩余量必须大于底层方向保护余量0.02rad（约1.15度）')
    a, b = endpoint(first, velocity_range), endpoint(second, velocity_range)
    axes = []
    for motor in (1, 2):
        low, high = sorted((a[motor], b[motor]), key=lambda x: x['median_rad'])
        # Conservative stable-window edges, rather than median-only limits.
        lower = low['observed_max_rad'] + margin
        upper = high['observed_min_rad'] - margin
        if upper - lower <= 0.04:
            raise ValueError(f'{motor}号内缩后的范围不足以容纳两端0.02rad保护区')
        axes.append({'motor_id': motor, 'command_can_id': motor,
                     'feedback_can_id': motor+3,
                     'endpoint_a': a[motor], 'endpoint_b': b[motor],
                     'position_min_rad': lower, 'position_max_rad': upper,
                     'position_guard_margin_rad': 0.02,
                     'joint_scale': 1.0, 'joint_offset_rad': 0.0})
    return {'schema': 'dm4310-shaft-range-candidate-v1',
            'created_at': datetime.now(timezone.utc).isoformat(),
            'model': 'DM4310', 'coordinate': 'motor_feedback_rad',
            'encoding_ranges_assumed': [12.5, float(velocity_range), 10.0],
            'calibration_status': 'valid',
            'mechanical_inset_deg': margin_deg,
            'motion_authorized': False, 'deployment_verified': False,
            'axes': axes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('endpoint_a', type=Path)
    parser.add_argument('endpoint_b', type=Path)
    parser.add_argument('--margin-deg', type=float, required=True)
    parser.add_argument('--velocity-range-rad-s', type=int, choices=(30,50), default=30,
                        help='select actual encoding VMAX; not the motion speed limit')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        rows = [[json.loads(line) for line in path.read_text().splitlines()]
                for path in (args.endpoint_a, args.endpoint_b)]
        output = candidate(*rows, args.margin_deg, args.velocity_range_rad_s)
        # Exclusive creation: invalid captures/cancelled attempts cannot replace limits.
        with args.output.open('x') as stream:
            stream.write(json.dumps(output, indent=2, allow_nan=False) + '\n')
        print(json.dumps(output, indent=2, ensure_ascii=False))
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f'拒绝生成限位: {error}\n')


if __name__ == '__main__':
    main()
