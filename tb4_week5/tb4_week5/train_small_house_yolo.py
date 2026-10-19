#!/usr/bin/env python3
"""Prepare the labelled small-house images and fine-tune YOLO11n."""

import argparse
import os
from pathlib import Path
import shutil
import sys


DEFAULT_BASE_MODEL = Path(
    '/home/thiendac/ros2_ws/src/TuongTacNguoi_Robot/'
    'turtlebot4_ws/yolo11n.pt')


def project_root():
    """Locate the source package when used through a symlink install."""
    module_path = Path(__file__).resolve()
    candidate = module_path.parents[1]
    if (candidate / 'config' / 'small_house_annotations.yaml').is_file():
        return candidate

    workspace_candidate = (
        Path.home() / 'ros2_ws' / 'src' / 'TuongTacNguoi_Robot' /
        'tb4_week5')
    if (workspace_candidate / 'config' /
            'small_house_annotations.yaml').is_file():
        return workspace_candidate

    raise FileNotFoundError(
        'Khong tim thay thu muc source tb4_week5/config.')


def parse_args():
    root = project_root()
    parser = argparse.ArgumentParser(
        description='Tao dataset va train YOLO cho small house.')
    parser.add_argument(
        '--world-models-dir', type=Path,
        default=root.parent / 'aws-robomaker-small-house-world' / 'models',
        help='Thu muc model Small House, dung texture Portrait lam picture.')
    parser.add_argument(
        '--annotations', type=Path,
        default=root / 'config' / 'small_house_annotations.yaml')
    parser.add_argument('--dataset-dir', type=Path,
                        default=root / 'datasets' / 'small_house')
    parser.add_argument('--base-model', type=Path,
                        default=DEFAULT_BASE_MODEL)
    parser.add_argument('--output-model', type=Path,
                        default=root / 'models' / 'small_house_best.pt')
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--batch', type=int, default=2)
    parser.add_argument('--device', default='cpu')
    parser.add_argument(
        '--prepare-only', action='store_true',
        help='Chi tao/kiem tra dataset, khong train.')
    return parser.parse_args()


def load_manifest(path):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            'Thieu PyYAML. Cai bang: sudo apt install python3-yaml') from exc

    with path.open('r', encoding='utf-8') as stream:
        manifest = yaml.safe_load(stream)
    if not isinstance(manifest, dict):
        raise ValueError(f'Annotation YAML khong hop le: {path}')
    return manifest


def class_mapping(manifest):
    raw_names = manifest.get('names', {})
    names = {int(index): str(name) for index, name in raw_names.items()}
    if sorted(names) != list(range(len(names))):
        raise ValueError('Class id trong names phai lien tiep tu 0.')
    return names, {name: index for index, name in names.items()}


def reset_generated_dirs(dataset_dir):
    # Only generated dataset leaves are replaced; map assets are never deleted.
    for relative in (
            'images/train', 'images/val', 'labels/train', 'labels/val',
            'preview'):
        target = dataset_dir / relative
        if target.exists():
            shutil.rmtree(target)
        target.mkdir(parents=True, exist_ok=True)


def yolo_box(class_id, xyxy, width, height):
    x1, y1, x2, y2 = xyxy
    center_x = ((x1 + x2) / 2.0) / width
    center_y = ((y1 + y2) / 2.0) / height
    box_width = (x2 - x1) / width
    box_height = (y2 - y1) / height
    return (
        f'{class_id} {center_x:.8f} {center_y:.8f} '
        f'{box_width:.8f} {box_height:.8f}')


def draw_preview(image, boxes, output_path):
    import cv2

    colors = [
        (0, 165, 255), (255, 80, 80), (80, 220, 80),
        (255, 0, 255), (0, 220, 220), (220, 120, 0),
        (255, 255, 0), (180, 80, 255), (0, 100, 255),
    ]
    for class_id, class_name, (x1, y1, x2, y2) in boxes:
        color = colors[class_id % len(colors)]
        start = (round(x1), round(y1))
        end = (round(x2), round(y2))
        cv2.rectangle(image, start, end, color, 2)
        cv2.putText(
            image, class_name, (start[0], max(18, start[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    if not cv2.imwrite(str(output_path), image):
        raise OSError(f'Khong ghi duoc preview: {output_path}')


def write_sample(dataset_dir, split, output_name, image, label_lines,
                 preview_boxes):
    """Write one image, its YOLO label and an annotated preview."""
    import cv2

    output_path = dataset_dir / 'images' / split / output_name
    suffix = output_path.suffix.lower()
    if suffix in ('.jpg', '.jpeg'):
        success = cv2.imwrite(
            str(output_path), image, [cv2.IMWRITE_JPEG_QUALITY, 92])
    else:
        success = cv2.imwrite(str(output_path), image)
    if not success:
        raise OSError(f'Khong ghi duoc anh: {output_path}')

    label_path = (
        dataset_dir / 'labels' / split /
        (Path(output_name).stem + '.txt'))
    label_path.write_text(
        '\n'.join(label_lines) + ('\n' if label_lines else ''),
        encoding='utf-8')
    draw_preview(
        image.copy(), preview_boxes,
        dataset_dir / 'preview' /
        f'{split}_{Path(output_name).stem}.jpg')


def add_picture_texture_samples(models_dir, dataset_dir, name_to_id,
                                split_counts, class_counts):
    """Create framed `picture` samples from every map Portrait texture.

    Several textures contain human faces. The complete framed image is always
    labelled `picture`, and the dataset deliberately has no `person` class.
    """
    import cv2
    import numpy as np

    if 'picture' not in name_to_id:
        return
    texture_paths = sorted(models_dir.rglob('*Portrait*.jpg'))
    if not texture_paths:
        raise FileNotFoundError(
            f'Khong tim thay texture Portrait trong: {models_dir}')

    rng = np.random.default_rng(20261019)
    class_id = name_to_id['picture']
    for texture_index, texture_path in enumerate(texture_paths):
        texture = cv2.imread(str(texture_path), cv2.IMREAD_COLOR)
        if texture is None:
            raise ValueError(f'Khong doc duoc texture: {texture_path}')
        texture_height, texture_width = texture.shape[:2]

        for variant in range(2):
            canvas_width, canvas_height = 640, 480
            wall_color = rng.integers(185, 241, size=3, dtype=np.uint8)
            canvas = np.full(
                (canvas_height, canvas_width, 3), wall_color, dtype=np.uint8)
            noise = rng.normal(0.0, 3.0, canvas.shape).astype(np.int16)
            canvas = np.clip(
                canvas.astype(np.int16) + noise, 0, 255).astype(np.uint8)

            max_width = int(rng.integers(220, 401))
            max_height = int(rng.integers(190, 351))
            scale = min(
                max_width / texture_width,
                max_height / texture_height)
            shown_width = max(80, round(texture_width * scale))
            shown_height = max(80, round(texture_height * scale))
            shown = cv2.resize(
                texture, (shown_width, shown_height),
                interpolation=cv2.INTER_AREA)

            frame_width = int(rng.integers(8, 19))
            outer_width = shown_width + 2 * frame_width
            outer_height = shown_height + 2 * frame_width
            x1 = int(rng.integers(20, canvas_width - outer_width - 19))
            y1 = int(rng.integers(20, canvas_height - outer_height - 19))
            x2, y2 = x1 + outer_width, y1 + outer_height
            frame_color = (
                (35, 45, 55) if variant == 0 else (45, 75, 105))
            cv2.rectangle(canvas, (x1, y1), (x2, y2), frame_color, -1)
            canvas[
                y1 + frame_width:y2 - frame_width,
                x1 + frame_width:x2 - frame_width] = shown

            xyxy = (float(x1), float(y1), float(x2), float(y2))
            label_lines = [yolo_box(
                class_id, xyxy, canvas_width, canvas_height)]
            preview_boxes = [(class_id, 'picture', xyxy)]
            split = (
                'val' if variant == 1 and texture_index % 5 == 0
                else 'train')
            output_name = (
                f'picture_texture_{texture_index:02d}_{variant}.jpg')
            write_sample(
                dataset_dir, split, output_name, canvas, label_lines,
                preview_boxes)
            split_counts[split] += 1
            class_counts['picture'] += 1


def add_asset_texture_samples(models_dir, dataset_dir, assets, name_to_id,
                              split_counts, class_counts):
    """Create labelled samples only from textures shipped with the world.

    This path deliberately does not read source screenshots. Each map asset is
    placed at several sizes and positions on varied backgrounds; the rectangle
    used for placement is also the exact YOLO bounding box.
    """
    import cv2
    import numpy as np

    rng = np.random.default_rng(20261019)
    found_classes = set()
    for asset in assets:
        class_name = str(asset['class'])
        if class_name not in name_to_id:
            raise ValueError(f'Class asset khong co trong names: {class_name}')
        patterns = asset.get('patterns', [])
        texture_paths = []
        for pattern in patterns:
            texture_paths.extend(sorted(models_dir.rglob(str(pattern))))
        unique_paths = []
        for path in texture_paths:
            if path.is_file() and path not in unique_paths:
                unique_paths.append(path)
        if not unique_paths:
            raise FileNotFoundError(
                f'Khong tim thay texture cho {class_name}: {patterns}')
        found_classes.add(class_name)
        class_id = name_to_id[class_name]

        for path_index, texture_path in enumerate(unique_paths):
            texture = cv2.imread(str(texture_path), cv2.IMREAD_UNCHANGED)
            if texture is None:
                raise ValueError(f'Khong doc duoc texture: {texture_path}')
            if texture.ndim == 2:
                texture = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR)
            if texture.shape[2] == 4:
                bgr = texture[:, :, :3]
                alpha = texture[:, :, 3]
            else:
                bgr = texture
                alpha = None

            for variant in range(6):
                canvas_width, canvas_height = 640, 480
                wall_color = rng.integers(175, 241, size=3, dtype=np.uint8)
                canvas = np.full(
                    (canvas_height, canvas_width, 3), wall_color,
                    dtype=np.uint8)
                noise = rng.normal(0.0, 4.0, canvas.shape).astype(np.int16)
                canvas = np.clip(
                    canvas.astype(np.int16) + noise, 0, 255).astype(np.uint8)

                max_width = int(rng.integers(150, 331))
                max_height = int(rng.integers(130, 351))
                scale = min(
                    max_width / bgr.shape[1], max_height / bgr.shape[0])
                shown_width = max(80, round(bgr.shape[1] * scale))
                shown_height = max(80, round(bgr.shape[0] * scale))
                shown = cv2.resize(
                    bgr, (shown_width, shown_height),
                    interpolation=cv2.INTER_AREA)
                shown_alpha = None
                if alpha is not None:
                    shown_alpha = cv2.resize(
                        alpha, (shown_width, shown_height),
                        interpolation=cv2.INTER_AREA)

                x1 = int(rng.integers(18, canvas_width - shown_width - 18))
                y1 = int(rng.integers(18, canvas_height - shown_height - 18))
                x2, y2 = x1 + shown_width, y1 + shown_height
                if shown_alpha is not None and int(shown_alpha.min()) < 250:
                    mask = shown_alpha.astype(np.float32)[:, :, None] / 255.0
                    target = canvas[y1:y2, x1:x2].astype(np.float32)
                    source = shown.astype(np.float32)
                    canvas[y1:y2, x1:x2] = np.clip(
                        source * mask + target * (1.0 - mask), 0, 255
                    ).astype(np.uint8)
                else:
                    canvas[y1:y2, x1:x2] = shown
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (45, 45, 45), 2)

                xyxy = (float(x1), float(y1), float(x2), float(y2))
                label_lines = [yolo_box(
                    class_id, xyxy, canvas_width, canvas_height)]
                preview_boxes = [(class_id, class_name, xyxy)]
                split = (
                    'val' if path_index == 0 and variant == 0 else 'train')
                output_name = (
                    f'asset_{class_name}_{path_index:02d}_{variant:02d}.jpg')
                write_sample(
                    dataset_dir, split, output_name, canvas,
                    label_lines, preview_boxes)
                split_counts[split] += 1
                class_counts[class_name] += 1

    missing = set(name_to_id) - found_classes - {'picture'}
    if missing:
        raise ValueError(
            'Chua co asset texture cho class: ' + ', '.join(sorted(missing)))


def write_dataset_yaml(dataset_dir, names):
    import yaml

    data = {
        'path': str(dataset_dir.resolve()),
        'train': 'images/train',
        'val': 'images/val',
        'names': names,
    }
    yaml_path = dataset_dir / 'small_house.yaml'
    with yaml_path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
    return yaml_path


def prepare_dataset(annotations_path, dataset_dir, world_models_dir):
    manifest = load_manifest(annotations_path)
    names, name_to_id = class_mapping(manifest)
    assets = manifest.get('assets', [])

    if assets:
        reset_generated_dirs(dataset_dir)
        split_counts = {'train': 0, 'val': 0}
        class_counts = {name: 0 for name in name_to_id}
        add_asset_texture_samples(
            world_models_dir, dataset_dir, assets, name_to_id,
            split_counts, class_counts)
        add_picture_texture_samples(
            world_models_dir, dataset_dir, name_to_id, split_counts,
            class_counts)
        yaml_path = write_dataset_yaml(dataset_dir, names)
        print(f'Dataset: {yaml_path}')
        print(
            f"Anh train={split_counts['train']}, val={split_counts['val']}")
        print('So bounding box: ' + ', '.join(
            f'{name}={count}' for name, count in class_counts.items()))
        print(f'Preview annotation: {dataset_dir / "preview"}')
        return yaml_path

    raise ValueError(
        'Dataset nay la asset-only: manifest phai co muc assets; '
        'khong chap nhan train tu screenshot.')


def train(args, dataset_yaml):
    if not args.base_model.is_file():
        raise FileNotFoundError(f'Khong tim thay base model: {args.base_model}')

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            'Thieu ultralytics. Chay: unset PYTHONNOUSERSITE && '
            'python3 -m pip install --user ultralytics') from exc

    root = project_root()
    run_root = root / 'runs'
    run_name = 'small_house_yolo'
    run_dir = run_root / run_name
    if run_dir.exists():
        shutil.rmtree(run_dir)
    model = YOLO(str(args.base_model))
    results = model.train(
        data=str(dataset_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=0,
        project=str(run_root),
        name=run_name,
        exist_ok=False,
        pretrained=True,
        patience=max(20, args.epochs // 3),
        cache=False,
        degrees=8.0,
        translate=0.12,
        scale=0.30,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.55,
        hsv_v=0.35,
        mosaic=0.35,
        close_mosaic=max(5, args.epochs // 10),
        plots=True,
        verbose=True,
    )

    best_model = Path(results.save_dir) / 'weights' / 'best.pt'
    if not best_model.is_file():
        raise FileNotFoundError(f'Khong tim thay best.pt: {best_model}')
    args.output_model.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_model, args.output_model)
    print(f'HOAN TAT - custom model: {args.output_model}')


def main():
    # Avoid Ultralytics writing settings/caches into an unexpected filesystem.
    os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/tb4_week5_ultralytics')
    args = parse_args()
    try:
        dataset_yaml = prepare_dataset(
            args.annotations, args.dataset_dir, args.world_models_dir)
        if not args.prepare_only:
            train(args, dataset_yaml)
    except Exception as exc:  # Show one concise actionable error in terminal.
        print(f'LOI: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
