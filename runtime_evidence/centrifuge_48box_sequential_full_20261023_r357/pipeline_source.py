CHANGES = (
    ('from preload_observation import PreloadObservationMixin\n',
     'from preload_observation import PreloadObservationMixin\nfrom planning_pipeline import PlanningPipelineMixin\nfrom joined_stages import joined_timing\n'),
    ('class SpeedPriorityCycle(RuntimePerformanceMixin,',
     'class SpeedPriorityCycle(PlanningPipelineMixin, RuntimePerformanceMixin,'),
    ('        return common_timing(actual, samples, rate)\n',
     "        if getattr(self, 'joined_stage_segments', None):\n            return joined_timing(actual, self.joined_stage_segments, rate)\n        return common_timing(actual, samples, rate)\n"),
    ('    def record(self, kind, payload):\n',
     "    def record(self, kind, payload):\n        if kind == 'command' and getattr(self, 'joined_stage_segments', None):\n            payload = dict(payload, joined_segments=copy.deepcopy(self.joined_stage_segments))\n"),
    ("            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'speed_worker.py'), str(source), str(output)],\n                stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)\n            wait_for_planner(process, self.spin, timeout_s=120)\n",
     "            receipt = self.planning_worker.execute(source, output, self.spin)\n            stream.write(json.dumps(dict(reused_planner_receipt=receipt)) + '\\n')\n"),
)


def restore_pipeline_source(name, source):
    if name == 'preload_observation.py':
        from unilateral_source import restore_unilateral_observation
        return restore_unilateral_observation(source)
    if name == 'preload_floor_frame.py':
        from floor_time_source import restore_floor_time
        return restore_floor_time(source)
    if name in ('sequential_cycle.py', 'sequential_worker.py'):
        if source.count('from bounded_release_alignment import ') != 1:
            raise ValueError('Expected exact bounded alignment import seam')
        source = source.replace('from bounded_release_alignment import ', 'from release_alignment import ', 1)
    if name == 'speed_worker.py':
        source = source.replace("SOURCE_NAMES = ('bounded_release_alignment.py', 'release_alignment.py', ", "SOURCE_NAMES = (", 1)
    if name == 'speed_retrieval_geometry.py':
        if source.count('1.12475 if') != 1:
            raise ValueError('Expected exact right intermediate transfer height delta')
        source = source.replace('1.12475 if', '1.13475 if', 1)
    if name == 'sequential_speed_cycle.py':
        from interval_source import restore_interval_observation
        source = restore_interval_observation(source)
        for previous, current in reversed(CHANGES):
            if source.count(current) != 1:
                raise ValueError('Expected exact declared pipeline source seam')
            source = source.replace(current, previous, 1)
    return source
