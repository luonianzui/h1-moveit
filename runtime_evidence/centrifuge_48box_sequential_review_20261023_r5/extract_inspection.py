import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r358/run'
source = RUN / 'full_cycle.mp4'
metadata = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-show_format', '-show_streams', '-of', 'json', str(source)]))
duration = float(metadata['format']['duration'])
records = []
for fraction in (.20, .35, .50, .72, .90, .99):
    second = duration * fraction
    destination = ROOT / f'inspection_{int(fraction * 100):02d}.png'
    subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error', '-n',
        '-ss', str(second), '-i', str(source), '-frames:v', '1', str(destination)], check=True)
    records.append(dict(path=str(destination), source_wall_s=second,
        sha256=hashlib.sha256(destination.read_bytes()).hexdigest()))
with (ROOT / 'inspection_sources.json').open('x') as stream:
    json.dump(dict(video_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), frames=records,
        actual_tool_inspection=False, status='EXTRACTED_FOR_PENDING_VISUAL_INSPECTION'), stream, indent=2)
print(json.dumps(records))
