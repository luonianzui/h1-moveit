import json
from pathlib import Path
import time

import psutil


ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r356/run'


def main():
    identity = json.loads((RUN/'startup_review_freeze.json').read_text())['server']
    process = psutil.Process(identity['pid'])
    assert abs(process.create_time() - identity['created']) < .01
    previous = {}
    deadline = time.monotonic() + 1800
    samples = 0
    with (ROOT/'server_thread_samples.jsonl').open('x') as stream:
        while not (RUN/'process_exit_status.json').exists() and time.monotonic() < deadline:
            try:
                stamp = time.monotonic()
                rows = []
                progress = json.loads((RUN/'trace_progress.json').read_text())
                for thread in process.threads():
                    total = thread.user_time + thread.system_time
                    old = previous.get(thread.id)
                    previous[thread.id] = (stamp, total)
                    if old is None or total < old[1]:
                        continue
                    try:
                        name = Path(f'/proc/{process.pid}/task/{thread.id}/comm').read_text().strip()
                    except FileNotFoundError:
                        name = 'exited'
                    rows.append(dict(tid=thread.id, name=name,
                                     cores=(total-old[1])/(stamp-old[0])))
                stream.write(json.dumps(dict(monotonic=stamp, phase=progress['phase'],
                    sim=progress['sim'], threads=rows, read_only=True))+'\n')
                stream.flush()
                samples += 1
            except (psutil.NoSuchProcess, FileNotFoundError, json.JSONDecodeError):
                pass
            time.sleep(1)
    with (ROOT/'server_thread_observer_result.json').open('x') as stream:
        json.dump(dict(status='READ_ONLY_OBSERVER_FINISHED', samples=samples,
                       no_signals_or_robot_commands=True,
                       thread_cpu_not_function_level_causal_profile=True),stream,indent=2)


if __name__ == '__main__':
    main()
