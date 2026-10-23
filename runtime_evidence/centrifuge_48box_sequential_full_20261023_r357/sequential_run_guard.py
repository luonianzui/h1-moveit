from pathlib import Path


ROOT = Path(__file__).resolve().parent
path = ROOT / 'run_guard.py'
source = path.read_text()
assert source.count("'execute_cycle.py'") == 2
source = source.replace("'execute_cycle.py'", "'execute_sequential.py'")
exec(compile(source, str(path), 'exec'), dict(__name__='__main__', __file__=str(path)))
