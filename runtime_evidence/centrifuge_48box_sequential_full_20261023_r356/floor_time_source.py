PREVIOUS = '0 <= now-contact_stamp <= .001'
CURRENT = '0 <= round(now * 1e9) - round(contact_stamp * 1e9) <= 1_000_000'


def restore_floor_time(source):
    if source.count(CURRENT) != 1 or PREVIOUS in source:
        raise ValueError('Only exact single floor-age nanosecond comparison is declared')
    return source.replace(CURRENT, PREVIOUS, 1)
