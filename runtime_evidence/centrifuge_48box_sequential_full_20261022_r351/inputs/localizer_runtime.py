import hashlib
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from h1_2_navigation import mid360_square_localizer as original
try:
    from .exact_rate_localizer import ExactRateLocalizer
except ImportError:
    from exact_rate_localizer import ExactRateLocalizer


ROOT = Path(__file__).resolve().parent.parent
ORIGINAL_SHA256 = '65c49409765cf9a0194ca2a9476d881b466d2dcf516ceb34deef9694d913a199'


class PipelineLog:
    def __init__(self, path):
        self.stream = Path(path).open('x', buffering=1)
        self.lock = threading.Lock()

    def record(self, event, **payload):
        with self.lock:
            self.stream.write(json.dumps(dict(event=event, wall=time.monotonic(),
                                             **payload), allow_nan=False) + '\n')

    def close(self):
        self.stream.close()


def source_stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.e-9


class InstrumentedLocalizer(ExactRateLocalizer):
    def __init__(self, recorder):
        self.recorder = recorder
        self.active_stamp = None
        super().__init__()
        lookup = self.transform_buffer.lookup_transform

        def traced_lookup(target, source, stamp, *args, **kwargs):
            started = time.monotonic()
            self.recorder.record('tf_enter', target=target, source=source,
                                 stamp=stamp.nanoseconds * 1.e-9)
            try:
                result = lookup(target, source, stamp, *args, **kwargs)
            except Exception as error:
                self.recorder.record('tf_error', elapsed=time.monotonic() - started,
                                     error=str(error))
                raise
            self.recorder.record('tf_exit', elapsed=time.monotonic() - started)
            return result

        self.transform_buffer.lookup_transform = traced_lookup
        self.recorder.record('configured', localization_rate=self.localization_rate,
                             transform_timeout=self.transform_timeout,
                             publish_map_to_odom=self.publish_map_to_odom,
                             original_sha256=ORIGINAL_SHA256,
                             rate_gate="integer_nanoseconds_no_source_stamp_rewrite")

    def _point_cloud_callback(self, message):
        started = time.monotonic()
        self.active_stamp = source_stamp(message)
        self.recorder.record('cloud_enter', stamp=self.active_stamp)
        try:
            return super()._point_cloud_callback(message)
        finally:
            self.recorder.record('cloud_exit', stamp=self.active_stamp,
                                 elapsed=time.monotonic() - started)
            self.active_stamp = None

    def _publish_pose(self, source, inlier_rms):
        result = super()._publish_pose(source, inlier_rms)
        self.recorder.record('pose_published', stamp=source_stamp(source),
                             inlier_rms=float(inlier_rms), map_to_odom_published=result)
        return result

    def _publish_status(self, valid, reason, point_count, *args, **kwargs):
        result = super()._publish_status(valid, reason, point_count, *args, **kwargs)
        self.recorder.record('status_published', stamp=self.active_stamp,
                             valid=bool(valid), reason=str(reason), point_count=int(point_count))
        return result


def main():
    assert hashlib.sha256(Path(original.__file__).read_bytes()).hexdigest() == ORIGINAL_SHA256
    recorder = PipelineLog(ROOT / 'run/localizer_pipeline.jsonl')
    rclpy.init()
    node = InstrumentedLocalizer(recorder)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        recorder.close()


if __name__ == '__main__':
    main()
