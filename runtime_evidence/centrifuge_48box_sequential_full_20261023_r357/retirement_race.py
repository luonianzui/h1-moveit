import signal

import psutil


def active_process(process):
    try:
        return process.is_running() and process.status() != psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        return False


def describe_and_interrupt(process):
    try:
        record = dict(pid=process.pid, created=process.create_time(),
                      command=[value for value in process.cmdline() if value])
        if active_process(process):
            process.send_signal(signal.SIGINT)
        return record
    except psutil.NoSuchProcess:
        return dict(pid=process.pid, disappeared_before_sigint=True)
