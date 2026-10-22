def finish_before_retirement(inspect_terminal, finish_video, retire_scene, recorder_alive):
    steps = {}
    for name, operation in (('inspection', inspect_terminal), ('video', finish_video)):
        try:
            operation()
            steps[name] = dict(status='PASS')
        except Exception as error:
            steps[name] = dict(status='FAIL', error=repr(error))
    try:
        live = recorder_alive()
        steps['recorder_exit'] = dict(status='FAIL' if live else 'PASS', alive=bool(live))
    except Exception as error:
        live = True
        steps['recorder_exit'] = dict(status='FAIL', error=repr(error), alive_unknown=True)
    if live:
        steps['retirement'] = dict(status='HELD', reason='Recorder has not verifiably exited; preserve scene')
    else:
        try:
            retire_scene()
            steps['retirement'] = dict(status='PASS')
        except Exception as error:
            steps['retirement'] = dict(status='FAIL', error=repr(error))
    return dict(status='PASS' if all(step['status'] == 'PASS' for step in steps.values()) else 'FAIL',
                steps=steps, inspection_failure_does_not_skip_video_sigint=True,
                scene_retirement_requires_recorder_exited=True)
