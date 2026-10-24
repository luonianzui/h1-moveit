import hashlib
import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
SOURCES = {
    'observe': ('observe_cpu.py', '1b8d2d34cd51abb331e9f588532bed752e8c8d7827a4252c876e1e33cc9f76b3'),
    'summarize': ('summarize_profile.py', '6f3c6ed2b9c4539cf02e23412b20a451290967a54d594c8b3d1dafe496099d07'),
}
name, checksum = SOURCES[sys.argv[1]]
source = ROOT.parent / 'centrifuge_48box_sequential_review_20261020_r18' / name
assert hashlib.sha256(source.read_bytes()).hexdigest() == checksum
specification = importlib.util.spec_from_file_location('sealed_readonly_observer', source)
module = importlib.util.module_from_spec(specification)
specification.loader.exec_module(module)
module.ROOT = ROOT
module.RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261024_r359/run'
module.main()
