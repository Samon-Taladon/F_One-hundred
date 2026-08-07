#!/usr/bin/env python3
import argparse
import csv
import math
import os

import cv2
import numpy as np
import yaml


def load_map(map_yaml):
    map_yaml = os.path.expanduser(map_yaml)
    with open(map_yaml, 'r', encoding='utf-8') as yaml_file:
        metadata = yaml.safe_load(yaml_file)

    image_path = metadata['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)

    image = cv2.imread(os.path.expanduser(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f'failed to read map image: {image_path}')

    resolution = float(metadata['resolution'])
    origin = metadata['origin']
    if len(origin) < 2:
        raise RuntimeError('map origin must contain at least x and y')

    return image, resolution, float(origin[0]), float(origin[1]), metadata


def make_free_mask(image, metadata):
    negate = int(metadata.get('negate', 0))
    mode = str(metadata.get('mode', 'trinary')).lower()

    if mode == 'trinary':
        if negate:
            return image <= 5
        return image >= 250

    pixels = image.astype(np.float32) / 255.0
    if negate:
        occupancy = pixels
    else:
        occupancy = 1.0 - pixels
    free_thresh = float(metadata.get('free_thresh', 0.25))
    return occupancy <= free_thresh


def largest_component(mask):
    components, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8),
        connectivity=8,
    )
    if components <= 1:
        raise RuntimeError('map has no free-space component')
    largest_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == largest_label


def apply_clearance(mask, resolution, robot_radius, safety_margin):
    clearance_px = int(math.ceil((robot_radius + safety_margin) / resolution))
    if clearance_px <= 0:
        return mask
    kernel_size = clearance_px * 2 + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernel_size, kernel_size),
    )
    return cv2.erode(mask.astype(np.uint8), kernel, iterations=1).astype(bool)


def pixel_to_map(col, row, height, resolution, origin_x, origin_y):
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - row - 0.5) * resolution
    return x, y


def map_to_pixel(x, y, height, resolution, origin_x, origin_y):
    col = int(round((x - origin_x) / resolution - 0.5))
    row = int(round(height - (y - origin_y) / resolution - 0.5))
    return col, row


def load_reference_points(path):
    points = []
    with open(os.path.expanduser(path), newline='', encoding='utf-8') as csv_file:
        sample = csv_file.read(512)
        csv_file.seek(0)
        has_header = bool(sample) and 'x' in sample.splitlines()[0].lower()
        reader = csv.DictReader(csv_file) if has_header else csv.reader(csv_file)
        for row in reader:
            try:
                if has_header:
                    point = (float(row['x']), float(row['y']))
                else:
                    point = (float(row[0]), float(row[1]))
            except (KeyError, TypeError, ValueError, IndexError):
                continue
            if not points or math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) > 1e-4:
                points.append(point)
    return points


def sample_ray(mask, center_col, center_row, angle, max_radius):
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    samples = []
    height, width = mask.shape
    for radius in range(max_radius):
        col = int(round(center_col + cos_a * radius))
        row = int(round(center_row - sin_a * radius))
        if col < 0 or col >= width or row < 0 or row >= height:
            break
        samples.append((col, row, bool(mask[row, col])))
    return samples


def longest_free_segment(samples, min_segment_px):
    best = None
    start = None
    for index, (_, _, is_free) in enumerate(samples):
        if is_free and start is None:
            start = index
        if (not is_free or index == len(samples) - 1) and start is not None:
            end = index if is_free else index - 1
            if end - start + 1 >= min_segment_px:
                if best is None or end - start > best[1] - best[0]:
                    best = (start, end)
            start = None
    return best


def free_segments(samples, min_segment_px):
    segments = []
    start = None
    for index, (_, _, is_free) in enumerate(samples):
        if is_free and start is None:
            start = index
        if (not is_free or index == len(samples) - 1) and start is not None:
            end = index if is_free else index - 1
            if end - start + 1 >= min_segment_px:
                segments.append((start, end))
            start = None
    return segments


def closest_free_segment(samples, min_segment_px, center_index):
    segments = free_segments(samples, min_segment_px)
    if not segments:
        return None
    for start, end in segments:
        if start <= center_index <= end:
            return start, end
    return min(
        segments,
        key=lambda segment: abs((segment[0] + segment[1]) * 0.5 - center_index),
    )


def sample_cross_section(mask, center_col, center_row, normal_x, normal_y, radius_px):
    samples = []
    height, width = mask.shape
    for offset in range(-radius_px, radius_px + 1):
        col = int(round(center_col + normal_x * offset))
        row = int(round(center_row - normal_y * offset))
        if col < 0 or col >= width or row < 0 or row >= height:
            samples.append((col, row, False))
            continue
        samples.append((col, row, bool(mask[row, col])))
    return samples


def reference_centerline_points(
    free,
    reference_points,
    resolution,
    origin_x,
    origin_y,
    min_segment_px,
    search_width,
):
    height, _ = free.shape
    radius_px = max(min_segment_px, int(round(search_width / resolution)))
    center_index = radius_px
    points = []

    for index, point in enumerate(reference_points):
        previous_point = reference_points[(index - 1) % len(reference_points)]
        next_point = reference_points[(index + 1) % len(reference_points)]
        tangent_x = next_point[0] - previous_point[0]
        tangent_y = next_point[1] - previous_point[1]
        tangent_length = math.hypot(tangent_x, tangent_y)
        if tangent_length < 1e-6:
            continue

        normal_x = -tangent_y / tangent_length
        normal_y = tangent_x / tangent_length
        center_col, center_row = map_to_pixel(
            point[0],
            point[1],
            height,
            resolution,
            origin_x,
            origin_y,
        )
        samples = sample_cross_section(
            free,
            center_col,
            center_row,
            normal_x,
            normal_y,
            radius_px,
        )
        segment = closest_free_segment(samples, min_segment_px, center_index)
        if segment is None:
            continue
        midpoint = (segment[0] + segment[1]) // 2
        col, row, _ = samples[midpoint]
        points.append(pixel_to_map(col, row, height, resolution, origin_x, origin_y))

    return points


def smooth_closed_points(points, iterations=2):
    if len(points) < 4:
        return points
    smoothed = list(points)
    for _ in range(iterations):
        updated = []
        for index, point in enumerate(smoothed):
            previous_point = smoothed[(index - 1) % len(smoothed)]
            next_point = smoothed[(index + 1) % len(smoothed)]
            updated.append((
                point[0] * 0.5 + (previous_point[0] + next_point[0]) * 0.25,
                point[1] * 0.5 + (previous_point[1] + next_point[1]) * 0.25,
            ))
        smoothed = updated
    return smoothed


def resample_closed_path(points, spacing):
    if len(points) < 3:
        return points

    distances = [0.0]
    for index in range(len(points)):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % len(points)]
        distances.append(distances[-1] + math.hypot(x1 - x0, y1 - y0))

    total_length = distances[-1]
    if total_length <= 0.0:
        return points

    result = []
    sample_count = max(3, int(total_length / spacing))
    segment_index = 0
    for sample_index in range(sample_count):
        target = sample_index * total_length / sample_count
        while segment_index + 1 < len(distances) and distances[segment_index + 1] < target:
            segment_index += 1

        start = points[segment_index % len(points)]
        end = points[(segment_index + 1) % len(points)]
        segment_length = max(distances[segment_index + 1] - distances[segment_index], 1e-9)
        ratio = (target - distances[segment_index]) / segment_length
        result.append((
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        ))
    return result


def compute_yaw(points, index):
    previous_point = points[(index - 1) % len(points)]
    next_point = points[(index + 1) % len(points)]
    return math.atan2(
        next_point[1] - previous_point[1],
        next_point[0] - previous_point[0],
    )


def generate_centerline(args):
    image, resolution, origin_x, origin_y, metadata = load_map(args.map)
    raw_free = make_free_mask(image, metadata)
    free = largest_component(raw_free)
    free = apply_clearance(free, resolution, args.robot_radius, args.safety_margin)
    free = largest_component(free)

    height, width = free.shape
    min_segment_px = max(2, int(round(args.min_corridor_width / resolution)))
    points = []

    if args.reference_path:
        reference_points = load_reference_points(args.reference_path)
        if len(reference_points) < 3:
            raise RuntimeError('reference path must contain at least 3 valid points')
        points = reference_centerline_points(
            free,
            reference_points,
            resolution,
            origin_x,
            origin_y,
            min_segment_px,
            args.reference_search_width,
        )
    else:
        if args.center_x is None or args.center_y is None:
            ys, xs = np.nonzero(free)
            center_col = float(xs.mean())
            center_row = float(ys.mean())
        else:
            center_col, center_row = map_to_pixel(
                args.center_x,
                args.center_y,
                height,
                resolution,
                origin_x,
                origin_y,
            )

        max_radius = int(math.hypot(width, height))
        angular_samples = max(24, int(args.angular_samples))

        for index in range(angular_samples):
            angle = 2.0 * math.pi * index / angular_samples
            samples = sample_ray(free, center_col, center_row, angle, max_radius)
            segment = longest_free_segment(samples, min_segment_px)
            if segment is None:
                continue
            midpoint = (segment[0] + segment[1]) // 2
            col, row, _ = samples[midpoint]
            points.append(pixel_to_map(col, row, height, resolution, origin_x, origin_y))

    if len(points) < 3:
        raise RuntimeError(
            'failed to generate a closed centerline; check map, clearance, or center'
        )

    points = smooth_closed_points(points, args.smooth_iterations)
    points = resample_closed_path(points, args.spacing)

    os.makedirs(os.path.dirname(os.path.expanduser(args.output)), exist_ok=True)
    with open(os.path.expanduser(args.output), 'w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['x', 'y', 'yaw'])
        for index, (x, y) in enumerate(points):
            writer.writerow([
                f'{x:.6f}',
                f'{y:.6f}',
                f'{compute_yaw(points, index):.6f}',
            ])

    print(f'Wrote {len(points)} auto waypoints to {os.path.expanduser(args.output)}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate closed centerline waypoints from an occupancy map.'
    )
    parser.add_argument('--map', required=True, help='Path to map YAML file')
    parser.add_argument('--output', required=True, help='Output raw waypoint CSV')
    parser.add_argument('--spacing', type=float, default=0.15)
    parser.add_argument('--robot-radius', type=float, default=0.18)
    parser.add_argument('--safety-margin', type=float, default=0.10)
    parser.add_argument('--min-corridor-width', type=float, default=0.25)
    parser.add_argument('--angular-samples', type=int, default=720)
    parser.add_argument('--smooth-iterations', type=int, default=2)
    parser.add_argument('--center-x', type=float)
    parser.add_argument('--center-y', type=float)
    parser.add_argument(
        '--reference-path',
        help='Recorded waypoint CSV used to preserve track order while centering on the map corridor',
    )
    parser.add_argument('--reference-search-width', type=float, default=2.0)
    generate_centerline(parser.parse_args())


if __name__ == '__main__':
    main()
