import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest

from interrupt_control import verify_interrupt_delivery


def test_inherited_ignored_sigint_is_restored_in_real_subprocess(tmp_path):
    root = Path(__file__).resolve().parent
    script = ("import os,signal,sys,time; sys.path.insert(0," + repr(str(root)) + "); "
              "from interrupt_control import establish_interrupt_handler; "
              "establish_interrupt_handler(); print(os.getpid(),flush=True);\n"
              "try:\n time.sleep(30)\n"
              "except KeyboardInterrupt:\n print('STOP_AND_HOLD_HANDLER',flush=True)\n")
    process = subprocess.Popen(['bash', '-c', 'trap "" INT; exec "$@"', 'fixture', sys.executable, '-c', script],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert int(process.stdout.readline()) == process.pid
        verify_interrupt_delivery(process.pid)
        process.send_signal(signal.SIGINT)
        output, error = process.communicate(timeout=10)
        assert process.returncode == 0 and 'STOP_AND_HOLD_HANDLER' in output and not error
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)


def test_live_parent_signal_configuration_is_readable():
    verify_interrupt_delivery(os.getpid())


def test_missing_process_is_not_accepted():
    with pytest.raises(FileNotFoundError):
        verify_interrupt_delivery(99999999)
