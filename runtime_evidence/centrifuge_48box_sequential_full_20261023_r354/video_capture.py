import os
from pathlib import Path

import psutil


def capture_command(window, path, frames=None):
    identifier = int(window, 16)
    if identifier <= 0:
        raise ValueError('A specific Gazebo window is required')
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    command = ['gst-launch-1.0', '-e', 'ximagesrc', 'display-name=:0',
               'xid=' + str(identifier), 'use-damage=false', 'show-pointer=false']
    command += ['endx=1199', 'endy=899']
    if frames is not None:
        if not isinstance(frames, int) or frames <= 0:
            raise ValueError('Positive frame count required')
        command.append('num-buffers=' + str(frames))
    command += ['!', 'video/x-raw,framerate=12/1', '!', 'videoconvert', '!', 'videoscale', '!',
                'video/x-raw,format=I420,width=1200,height=900', '!', 'queue',
                'max-size-buffers=0', 'max-size-bytes=0', 'max-size-time=2000000000', '!',
                'x264enc', 'speed-preset=veryfast', 'pass=qual', 'quantizer=21',
                'key-int-max=24', '!', 'video/x-h264,stream-format=avc,alignment=au', '!', 'mp4mux', 'fragment-duration=2000',
                '!', 'filesink', 'location=' + str(path)]
    return command


def recorder_alive(entry):
    try:
        process = psutil.Process(entry['pid'])
        return (process.is_running() and process.status() != psutil.STATUS_ZOMBIE and
                abs(process.create_time() - entry['created']) < .01 and
                process.cmdline() == entry['command'])
    except (psutil.Error, KeyError, TypeError, ValueError):
        return False


def capture_environment():
    return dict(os.environ, DISPLAY=':0')
