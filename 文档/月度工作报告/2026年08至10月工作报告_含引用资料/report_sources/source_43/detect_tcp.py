"""
YOLOv5 检测 + TCP 上报
- 从摄像头/视频/图片进行推理
- 统计每帧各类别数量
- 通过 TCP 发送给上位机，JSON 格式（每行一个 JSON，\n 分隔）

TCP 数据包格式：
  {"frame_id": 42, "timestamp": 1718000000.123, "counts": {"ls_board": 1, "antibody_kit": 0, "tip_box": 2, "96_board": 1}, "total": 4}\n
"""
import sys
import os
import json
import socket
import time
import argparse
import threading
import pathlib
import math
from pathlib import Path
from unittest.mock import patch

BASE_DIR = Path(__file__).resolve().parent
# The release includes all dependencies and must never install/download at startup.
os.environ['YOLO_OFFLINE'] = 'true'
os.environ['YOLO_AUTOINSTALL'] = 'false'
os.environ['YOLOv5_AUTOINSTALL'] = 'false'
os.environ.setdefault('GIT_PYTHON_REFRESH', 'quiet')
os.environ.setdefault('MPLBACKEND', 'Agg')
CACHE_DIR = (Path(os.environ['LOCALAPPDATA']) / 'StationTCP'
             if os.name == 'nt' and os.environ.get('LOCALAPPDATA') else BASE_DIR / 'cache')
os.environ.setdefault('YOLO_CONFIG_DIR', str(CACHE_DIR / 'ultralytics'))
os.environ.setdefault('MPLCONFIGDIR', str(CACHE_DIR / 'matplotlib'))
Path(os.environ['YOLO_CONFIG_DIR']).mkdir(parents=True, exist_ok=True)
Path(os.environ['MPLCONFIGDIR']).mkdir(parents=True, exist_ok=True)

import cv2
import torch
import numpy as np

YOLOV5_PATH = str(BASE_DIR / 'vendor' / 'yolov5')
sys.path.insert(0, YOLOV5_PATH)

WEIGHT_PATH = str(BASE_DIR / 'weight' / 'last.pt')


# ── TCP 发送器（后台线程，自动重连） ─────────────────────────────────────────

class TCPSender:
    def __init__(self, host: str, port: int, reconnect_interval: float = 3.0):
        self.host = host
        self.port = port
        self.reconnect_interval = reconnect_interval
        self._sock = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._keep_alive, daemon=True)
        self._thread.start()

    def _keep_alive(self):
        while not self._stop.is_set():
            if self._sock is None:
                s = None
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0)
                    s.connect((self.host, self.port))
                    with self._lock:
                        if self._stop.is_set():
                            s.close()
                            break
                        self._sock = s
                    print(f"[TCP] 已连接 {self.host}:{self.port}")
                except Exception as e:
                    if s is not None:
                        s.close()
                    print(f"[TCP] 连接失败: {e}，{self.reconnect_interval}s 后重试")
                    self._stop.wait(self.reconnect_interval)
            else:
                self._stop.wait(0.2)

    def send(self, data: dict) -> bool:
        payload = (json.dumps(data, ensure_ascii=False) + '\n').encode('utf-8')
        with self._lock:
            sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendall(payload)
            return True
        except Exception as e:
            print(f"[TCP] 发送失败: {e}，触发重连")
            with self._lock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
            return False

    def stop(self):
        self._stop.set()
        with self._lock:
            if self._sock:
                self._sock.close()
                self._sock = None
        self._thread.join(timeout=2.0)


# ── 模型加载 ─────────────────────────────────────────────────────────────────

def load_model(weight: str, device: str):
    if not Path(weight).is_file():
        raise FileNotFoundError(f'模型文件不存在: {weight}')
    if not (Path(YOLOV5_PATH) / 'models' / 'common.py').is_file():
        raise FileNotFoundError(f'缺少包内 YOLOv5 源码: {YOLOV5_PATH}')
    from models.common import DetectMultiBackend
    from utils.torch_utils import select_device
    dev = select_device(device)
    # The original trusted checkpoint was saved on Linux and contains PosixPath.
    # Limit the compatibility alias to deserialization instead of changing pathlib globally.
    if os.name == 'nt':
        with patch.object(pathlib, 'PosixPath', pathlib.WindowsPath):
            model = DetectMultiBackend(weight, device=dev)
    else:
        model = DetectMultiBackend(weight, device=dev)
    model.eval()
    return model, dev


# ── 推理（单帧） ─────────────────────────────────────────────────────────────

def infer(model, dev, frame_bgr, imgsz: int, conf_thres: float, iou_thres: float):
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes

    stride = int(model.stride)
    img, ratio, (dw, dh) = letterbox(frame_bgr, imgsz, stride=stride, auto=True)
    img = img[:, :, ::-1].transpose(2, 0, 1)          # BGR→RGB, HWC→CHW
    img = np.ascontiguousarray(img)
    img_t = torch.from_numpy(img).to(dev).float() / 255.0
    if img_t.ndim == 3:
        img_t = img_t.unsqueeze(0)

    with torch.no_grad():
        pred = model(img_t, augment=False, visualize=False)

    pred = non_max_suppression(pred, conf_thres, iou_thres)

    # 把框坐标映射回原图
    results = []
    for det in pred:
        if det is not None and len(det):
            det[:, :4] = scale_boxes(img_t.shape[2:], det[:, :4], frame_bgr.shape).round()
            for *xyxy, conf, cls in det:
                results.append({
                    'x1': int(xyxy[0]), 'y1': int(xyxy[1]),
                    'x2': int(xyxy[2]), 'y2': int(xyxy[3]),
                    'conf': round(float(conf), 3),
                    'cls': int(cls),
                    'name': model.names[int(cls)],
                })
    return results


# ── 统计 & 绘图 ──────────────────────────────────────────────────────────────

def count_classes(detections: list, class_names: list) -> dict:
    counts = {n: 0 for n in class_names}
    for d in detections:
        name = d['name']
        if name in counts:
            counts[name] += 1
    return counts


def draw(frame, detections, counts):
    for d in detections:
        cv2.rectangle(frame, (d['x1'], d['y1']), (d['x2'], d['y2']), (0, 255, 0), 2)
        cv2.putText(frame, f"{d['name']} {d['conf']:.2f}",
                    (d['x1'], d['y1'] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    y = 24
    for name, cnt in counts.items():
        cv2.putText(frame, f"{name}: {cnt}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
        y += 26
    total = sum(counts.values())
    cv2.putText(frame, f"total: {total}", (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
    return frame


# ── 主循环 ───────────────────────────────────────────────────────────────────

def run(args):
    device = args.device
    torch.set_num_threads(args.threads)
    print(f"[INFO] 设备: {device}")

    model, dev = load_model(args.weight, device)
    class_names = list(model.names.values()) if isinstance(model.names, dict) else list(model.names)
    print(f"[INFO] 类别: {class_names}")

    src = int(args.source) if args.source.isdigit() else args.source
    backend = {'auto': cv2.CAP_ANY, 'dshow': cv2.CAP_DSHOW,
               'msmf': cv2.CAP_MSMF}[args.camera_backend]
    cap = cv2.VideoCapture(src, backend) if isinstance(src, int) else cv2.VideoCapture(src)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'无法打开摄像头/视频: {args.source}。请检查 config.json、设备连接及 Windows 相机权限。')

    sender = TCPSender(args.host, args.port) if not args.no_tcp else None

    frame_id = 0
    send_interval = 1.0 / args.send_fps
    last_send = 0.0
    failed_reads = 0

    print("[INFO] 开始推理，按 q 退出")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                if isinstance(src, str):
                    break
                failed_reads += 1
                if failed_reads >= 30:
                    raise RuntimeError('摄像头连续读取失败，请重新连接设备后启动。')
                time.sleep(0.1)
                continue

            failed_reads = 0
            frame_id += 1
            detections = infer(model, dev, frame, args.imgsz, args.conf, args.iou)
            counts = count_classes(detections, class_names)
            total = sum(counts.values())

            now = time.monotonic()
            if sender and (now - last_send) >= send_interval:
                packet = {
                    'frame_id': frame_id,
                    'timestamp': round(time.time(), 3),
                    'counts': counts,
                    'total': total,
                }
                ok = sender.send(packet)
                if ok:
                    print(f"[TCP] frame={frame_id} {counts} total={total}")
                last_send = now

            if not args.no_display:
                vis = draw(frame.copy(), detections, counts)
                cv2.imshow('YOLOv5 Detection', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if args.max_frames and frame_id >= args.max_frames:
                break
    finally:
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        if sender:
            sender.stop()
        print("[INFO] 退出")


# ── CLI ──────────────────────────────────────────────────────────────────────

def parse_args(argv=None):
    pre = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    pre.add_argument('--config', default=str(BASE_DIR / 'config.json'))
    config_arg, _ = pre.parse_known_args(argv)
    config_path = Path(config_arg.config).resolve()
    defaults = {}
    if config_path.is_file():
        with config_path.open(encoding='utf-8-sig') as f:
            config = json.load(f)
        if not isinstance(config, dict):
            pre.error('config.json 必须是 JSON 对象')
        allowed = {'source', 'weight', 'device', 'imgsz', 'conf', 'iou', 'host',
                   'port', 'send_fps', 'display', 'tcp', 'camera_backend', 'threads'}
        unknown = set(config) - allowed
        if unknown:
            pre.error(f'配置中有未知字段: {sorted(unknown)}')
        defaults = dict(config)
        for key in ('display', 'tcp'):
            if key in defaults:
                if not isinstance(defaults[key], bool):
                    pre.error(f'{key} 必须是 true 或 false')
                defaults['no_' + key] = not defaults.pop(key)
        if 'source' in defaults:
            defaults['source'] = str(defaults['source'])
            source_path = Path(defaults['source'])
            if not defaults['source'].isdigit() and '://' not in defaults['source'] and not source_path.is_absolute():
                defaults['source'] = str(config_path.parent / source_path)
        if 'weight' in defaults:
            weight = Path(defaults['weight'])
            defaults['weight'] = str(weight if weight.is_absolute() else config_path.parent / weight)
    elif any(a == '--config' or a.startswith('--config=') for a in (argv if argv is not None else sys.argv[1:])):
        pre.error(f'配置文件不存在: {config_path}')
    p = argparse.ArgumentParser(description='YOLOv5 检测 + TCP 上报', allow_abbrev=False)
    p.add_argument('--config', default=str(config_path), help='配置文件路径')
    p.add_argument('--source',      default='0',
                   help='输入源：摄像头编号(0/1…) 或 视频/图片路径')
    p.add_argument('--weight',      default=WEIGHT_PATH,
                   help='模型权重路径')
    p.add_argument('--imgsz',       type=int,   default=640)
    p.add_argument('--conf',        type=float, default=0.25, help='置信度阈值')
    p.add_argument('--iou',         type=float, default=0.45, help='NMS IOU 阈值')
    p.add_argument('--host',        default='127.0.0.1', help='上位机 IP')
    p.add_argument('--port',        type=int,   default=9000, help='上位机 TCP 端口')
    p.add_argument('--send-fps',    type=float, default=10.0, help='TCP 发送频率(帧/秒)')
    p.add_argument('--device', default='cpu', help='默认 CPU，离线包不需要 CUDA')
    p.add_argument('--threads', type=int, default=4, help='CPU 推理线程数')
    p.add_argument('--camera-backend', choices=['auto', 'dshow', 'msmf'], default='auto')
    p.add_argument('--max-frames', type=int, default=0, help='处理指定帧数后退出，0 为持续运行')
    p.add_argument('--no-tcp',      action='store_true', help='禁用 TCP（本地调试）')
    p.add_argument('--tcp', dest='no_tcp', action='store_false', default=argparse.SUPPRESS)
    p.add_argument('--no-display',  action='store_true', help='无头模式，不显示画面')
    p.add_argument('--display', dest='no_display', action='store_false', default=argparse.SUPPRESS)
    p.set_defaults(**defaults)
    args = p.parse_args(argv)
    if not isinstance(args.host, str) or not args.host.strip():
        p.error('host 必须是有效的 IP 地址或主机名')
    try:
        integers = all(type(getattr(args, k)) is int for k in ('port', 'imgsz', 'threads', 'max_frames'))
        numbers = all(type(getattr(args, k)) in (int, float) and math.isfinite(getattr(args, k))
                      for k in ('conf', 'iou', 'send_fps'))
        valid = (integers and numbers and 0 < args.conf < 1 and 0 < args.iou < 1 and args.send_fps > 0
                 and 1 <= args.port <= 65535 and args.imgsz > 0 and args.imgsz % 32 == 0
                 and args.threads > 0 and args.max_frames >= 0)
    except TypeError:
        valid = False
    if not valid:
        p.error('检查配置数值：conf/iou 在 0~1 内，端口 1~65535，imgsz 为 32 的正整数倍，send_fps/threads > 0')
    if args.camera_backend not in ('auto', 'dshow', 'msmf'):
        p.error('camera_backend 必须为 auto、dshow 或 msmf')
    if not isinstance(args.device, str):
        p.error('device 必须是字符串，离线包使用 cpu')
    return args


if __name__ == '__main__':
    try:
        run(parse_args())
    except KeyboardInterrupt:
        print('\n[INFO] 已停止')
    except Exception as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        raise SystemExit(1)
