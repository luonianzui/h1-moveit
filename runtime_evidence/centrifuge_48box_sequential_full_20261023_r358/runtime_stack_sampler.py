from collections import Counter
import json
import math
import sys
import threading
import time


class RuntimeStackSampler:
    def __init__(self, output, phase, interval_s=.02):
        if not math.isfinite(interval_s) or interval_s < .02:
            raise ValueError('Sampling must be bounded at 50Hz or lower')
        self.output = output
        self.phase = phase
        self.interval_s = interval_s
        self.stop_event = threading.Event()
        self.counts = Counter()
        self.samples = 0
        self.sample_cpu_s = 0.
        self.errors = []
        self.thread = None

    def start(self):
        if self.thread is not None or self.output.exists():
            raise RuntimeError('Sampler identity already used')
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self.collect, name='runtime-stack-sampler', daemon=True)
        self.thread.start()

    def collect(self):
        while not self.stop_event.wait(self.interval_s):
            began = time.thread_time()
            try:
                names = {thread.ident: thread.name for thread in threading.enumerate()}
                phase = str(self.phase())
                frames = sys._current_frames()
                for identifier, frame in frames.items():
                    if identifier == threading.get_ident():
                        continue
                    chain = []
                    while frame is not None and len(chain) < 18:
                        chain.append(f'{frame.f_code.co_filename}:{frame.f_lineno}:{frame.f_code.co_name}')
                        frame = frame.f_back
                    self.counts[(phase, names.get(identifier, 'unknown'), tuple(chain))] += 1
                del frames
                self.samples += 1
            except Exception as error:
                if len(self.errors) < 10:
                    self.errors.append(repr(error))
            self.sample_cpu_s += time.thread_time() - began

    def stop(self):
        if self.thread is None:
            raise RuntimeError('Sampler not started')
        self.stop_event.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('Sampler did not terminate')
        result = dict(status='OBSERVATION_NOT_PHYSICS_OR_CPU_CAUSAL_PROOF',
                      interval_wall_s=self.interval_s, wall_s=time.monotonic() - self.started,
                      sampler_thread_cpu_s=self.sample_cpu_s, samples=self.samples,
                      errors=self.errors, stacks=[dict(phase=phase, thread=thread, stack=list(stack), count=count)
                          for (phase, thread, stack), count in self.counts.most_common()])
        with self.output.open('x') as stream:
            json.dump(result, stream, indent=2)
        return result
