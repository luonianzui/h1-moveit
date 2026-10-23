from pathlib import Path
import signal


def establish_interrupt_handler():
    signal.signal(signal.SIGINT, signal.default_int_handler)
    if signal.getsignal(signal.SIGINT) is not signal.default_int_handler:
        raise RuntimeError('SIGINT must enter the existing stop-and-hold exception path')


def verify_interrupt_delivery(pid):
    contents = (Path('/proc') / str(pid) / 'status').read_text()
    mask = int(next(line.split()[1] for line in contents.splitlines() if line.startswith('SigIgn:')), 16)
    if mask & (1 << (signal.SIGINT - 1)):
        raise RuntimeError('Certified executor ignores SIGINT; no motion allowed')
