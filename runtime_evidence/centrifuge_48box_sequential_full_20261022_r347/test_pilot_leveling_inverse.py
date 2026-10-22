def undo_pilot_leveling(source):
    replacements = [
        ('from pilot_leveling import leveled_retrieval_pilot_target\n', 'from pilot_centering import centered_pilot_target\n'),
        ("        target = leveled_retrieval_pilot_target(body, request['poses'], request['payload'])\n",
         "        target = centered_pilot_target(body, request['poses'], request['payload'])\n")]
    for current, original in replacements:
        assert source.count(current) == 1
        source = source.replace(current, original, 1)
    return source
