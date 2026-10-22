from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent
original = EVIDENCE / 'probe_actual_guided_retreat.py'
source = original.read_text().replace('actual_guided_retreat_plan.json', 'actual_guided_retreat_plan_checked.json')
source = source.replace('actual_guided_retreat_audit.json', 'actual_guided_retreat_audit_checked.json')
exec(compile(source, str(original), 'exec'), dict(__name__='__main__', __file__=str(original)))
