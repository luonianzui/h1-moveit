import copy

from speed_policy import common_timing, motion_limits


PAIRS = {('load_lift', 'load_transfer'), ('retrieve_outboard', 'retrieve_transfer')}


def stage_groups(stages):
    groups = []
    index = 0
    while index < len(stages):
        group = stages[index:index+2]
        if len(group) == 2 and tuple(stage['phase'] for stage in group) in PAIRS:
            if group[0]['samples'][-1] != group[1]['samples'][0]:
                raise ValueError('Joined stages require the exact common qualified waypoint')
            groups.append(group)
            index += 2
        else:
            groups.append(stages[index:index+1])
            index += 1
    return groups


def joined_samples(segments):
    if len(segments) != 2 or tuple(stage['phase'] for stage in segments) not in PAIRS:
        raise ValueError('Only declared free loaded segment pairs may join')
    if not all(stage['samples'] for stage in segments) or segments[0]['samples'][-1] != segments[1]['samples'][0]:
        raise ValueError('Missing or discontinuous original dense path')
    return copy.deepcopy(segments[0]['samples'] + segments[1]['samples'])


def joined_phase(segments):
    joined_samples(segments)
    return segments[0]['phase'] + '_and_' + segments[1]['phase']


def joined_timing(actual, segments, rate):
    joined_samples(segments)
    if any(motion_limits(stage['phase'], True)[0] != rate for stage in segments):
        raise ValueError('Different motion limits cannot share a joined command')
    first = common_timing(actual, segments[0]['samples'], rate)
    second = common_timing(segments[0]['samples'][-1], segments[1]['samples'], rate)
    output = copy.deepcopy(first)
    output['points'].extend(second['points'][1:])
    output['times'].extend(first['duration_sim_s'] + stamp - second['times'][0] for stamp in second['times'][1:])
    output['duration_sim_s'] = output['times'][-1]
    output['joined_segments'] = copy.deepcopy(segments)
    return output
