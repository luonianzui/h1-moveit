import signal
import time


def wait_for_planner(process, spin, timeout_s=60):
    deadline = time.monotonic() + timeout_s
    try:
        while process.poll() is None:
            spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Alignment worker timeout')
        if process.returncode != 0:
            raise RuntimeError('Alignment worker failed: exit ' + str(process.returncode))
    except BaseException:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        raise

