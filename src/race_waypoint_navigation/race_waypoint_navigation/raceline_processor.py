#!/usr/bin/env python3
import argparse
import csv
import math
import os


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_points(path):
    points = []
    with open(os.path.expanduser(path), newline='', encoding='utf-8') as csv_file:
        sample = csv_file.read(512)
        csv_file.seek(0)
        has_header = bool(sample) and 'x' in sample.splitlines()[0].lower()
        reader = csv.DictReader(csv_file) if has_header else csv.reader(csv_file)
        for row in reader:
            try:
                if has_header:
                    points.append((float(row['x']), float(row['y'])))
                else:
                    points.append((float(row[0]), float(row[1])))
            except (KeyError, TypeError, ValueError, IndexError):
                continue
    return remove_duplicate_points(points)


def remove_duplicate_points(points, min_distance=1e-4):
    filtered = []
    for point in points:
        if not filtered or distance(point, filtered[-1]) >= min_distance:
            filtered.append(point)
    return filtered


def smooth_closed_path(points, iterations, weight):
    if len(points) < 3:
        return points
    smoothed = [(float(x), float(y)) for x, y in points]
    for _ in range(iterations):
        updated = []
        for index, point in enumerate(smoothed):
            previous_point = smoothed[(index - 1) % len(smoothed)]
            next_point = smoothed[(index + 1) % len(smoothed)]
            x = point[0] * (1.0 - weight) + (previous_point[0] + next_point[0]) * 0.5 * weight
            y = point[1] * (1.0 - weight) + (previous_point[1] + next_point[1]) * 0.5 * weight
            updated.append((x, y))
        smoothed = updated
    return smoothed


def cumulative_distances(points, closed):
    distances = [0.0]
    count = len(points)
    end = count if closed else count - 1
    for index in range(end):
        segment = distance(points[index], points[(index + 1) % count])
        distances.append(distances[-1] + segment)
    return distances


def interpolate_loop(points, spacing, closed):
    if len(points) < 2:
        return points
    cumulative = cumulative_distances(points, closed)
    total_length = cumulative[-1]
    if total_length <= 0.0:
        return points

    output = []
    sample_count = max(2, int(total_length / spacing))
    max_samples = sample_count if closed else sample_count + 1
    segment_index = 0
    for sample_index in range(max_samples):
        target_distance = min(sample_index * spacing, total_length)
        while (
            segment_index + 1 < len(cumulative)
            and cumulative[segment_index + 1] < target_distance
        ):
            segment_index += 1

        start = points[segment_index % len(points)]
        end = points[(segment_index + 1) % len(points)]
        segment_start = cumulative[segment_index]
        segment_end = cumulative[min(segment_index + 1, len(cumulative) - 1)]
        segment_length = max(segment_end - segment_start, 1e-9)
        ratio = (target_distance - segment_start) / segment_length
        output.append((
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        ))
    return output


def compute_yaw(points, index, closed):
    if closed:
        previous_point = points[(index - 1) % len(points)]
        next_point = points[(index + 1) % len(points)]
    else:
        previous_point = points[max(index - 1, 0)]
        next_point = points[min(index + 1, len(points) - 1)]
    return math.atan2(next_point[1] - previous_point[1], next_point[0] - previous_point[0])


def triangle_curvature(a, b, c):
    ab = distance(a, b)
    bc = distance(b, c)
    ca = distance(c, a)
    area2 = abs(
        (b[0] - a[0]) * (c[1] - a[1])
        - (b[1] - a[1]) * (c[0] - a[0])
    )
    denominator = ab * bc * ca
    if denominator < 1e-9:
        return 0.0
    return 2.0 * area2 / denominator


def compute_curvature(points, index, closed):
    if closed:
        return triangle_curvature(
            points[(index - 1) % len(points)],
            points[index],
            points[(index + 1) % len(points)],
        )
    if index == 0 or index == len(points) - 1:
        return 0.0
    return triangle_curvature(points[index - 1], points[index], points[index + 1])


def speed_from_curvature(curvature, min_speed, max_speed, lateral_accel):
    if curvature < 1e-6:
        return max_speed
    speed = math.sqrt(max(lateral_accel, 0.01) / curvature)
    return max(min_speed, min(max_speed, speed))


def apply_braking_margin(speeds, points, max_decel, closed):
    if len(speeds) < 2 or max_decel <= 0.0:
        return speeds
    adjusted = list(speeds)
    passes = len(speeds) * (2 if closed else 1)
    for offset in range(passes):
        index = (len(speeds) - 2 - offset) % len(speeds) if closed else len(speeds) - 2 - offset
        if index < 0:
            break
        next_index = (index + 1) % len(speeds)
        segment = distance(points[index], points[next_index])
        allowed = math.sqrt(adjusted[next_index] ** 2 + 2.0 * max_decel * segment)
        adjusted[index] = min(adjusted[index], allowed)
    return adjusted


def write_raceline(path, rows):
    os.makedirs(os.path.dirname(os.path.expanduser(path)), exist_ok=True)
    with open(os.path.expanduser(path), 'w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['x', 'y', 'yaw', 'curvature', 'target_speed', 'lookahead'])
        writer.writerows(rows)


def process(args):
    points = load_points(args.input)
    if len(points) < 3:
        raise RuntimeError('input path must contain at least 3 valid points')

    closed = not args.open_path
    if closed and distance(points[0], points[-1]) < args.spacing * 1.5:
        points = points[:-1]
    points = smooth_closed_path(points, args.smooth_iterations, args.smooth_weight)
    points = interpolate_loop(points, args.spacing, closed)

    curvatures = [
        compute_curvature(points, index, closed)
        for index in range(len(points))
    ]
    speeds = [
        speed_from_curvature(
            curvature,
            args.min_speed,
            args.max_speed,
            args.max_lateral_accel,
        )
        for curvature in curvatures
    ]
    speeds = apply_braking_margin(speeds, points, args.max_decel, closed)

    rows = []
    for index, point in enumerate(points):
        yaw = compute_yaw(points, index, closed)
        speed = speeds[index]
        lookahead = max(
            args.min_lookahead,
            min(args.max_lookahead, speed * args.lookahead_gain),
        )
        rows.append([
            f'{point[0]:.6f}',
            f'{point[1]:.6f}',
            f'{yaw:.6f}',
            f'{curvatures[index]:.6f}',
            f'{speed:.6f}',
            f'{lookahead:.6f}',
        ])

    write_raceline(args.output, rows)
    print(f'Wrote {len(rows)} raceline points to {os.path.expanduser(args.output)}')


def main():
    parser = argparse.ArgumentParser(
        description='Convert a raw waypoint CSV into a smoothed race line.'
    )
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--spacing', type=float, default=0.15)
    parser.add_argument('--smooth-iterations', type=int, default=8)
    parser.add_argument('--smooth-weight', type=float, default=0.25)
    parser.add_argument('--min-speed', type=float, default=0.25)
    parser.add_argument('--max-speed', type=float, default=1.2)
    parser.add_argument('--max-lateral-accel', type=float, default=1.4)
    parser.add_argument('--max-decel', type=float, default=1.6)
    parser.add_argument('--min-lookahead', type=float, default=0.45)
    parser.add_argument('--max-lookahead', type=float, default=1.4)
    parser.add_argument('--lookahead-gain', type=float, default=0.8)
    parser.add_argument('--open-path', action='store_true')
    process(parser.parse_args())


if __name__ == '__main__':
    main()
