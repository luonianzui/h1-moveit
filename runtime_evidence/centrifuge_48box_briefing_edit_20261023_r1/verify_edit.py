import hashlib
import json
from pathlib import Path
import struct
import subprocess

ROOT = Path(__file__).resolve().parent / 'revision2'
manifest = json.loads((ROOT / 'edit_manifest.json').read_text())
output = Path(manifest['output'])
metadata = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-show_format', '-show_streams',
    '-count_frames', '-of', 'json', str(output)]))
streams = metadata['streams']
assert len(streams) == 1 and streams[0]['codec_name'] == 'h264'
assert (streams[0]['width'], streams[0]['height'], streams[0]['avg_frame_rate']) == (1200, 900, '24/1')
expected_duration = sum(item['intended_duration_s'] for item in manifest['segments']) + 5
duration = float(metadata['format']['duration'])
assert abs(duration-expected_duration) < .5
with (ROOT / 'full_decode.log').open('xb') as stream:
    decode = subprocess.run(['ffmpeg', '-v', 'error', '-xerror', '-nostdin', '-threads', '1',
        '-i', str(output), '-f', 'null', '-'], stdout=stream, stderr=subprocess.STDOUT, check=True)
atoms = []
with output.open('rb') as stream:
    while header := stream.read(8):
        size, name = struct.unpack('>I4s', header)
        if size == 1:
            size = struct.unpack('>Q', stream.read(8))[0]
            stream.seek(size-16, 1)
        elif size == 0:
            break
        else:
            stream.seek(size-8, 1)
        atoms.append(name.decode('ascii'))
assert atoms.index('moov') < atoms.index('mdat')
frames = []
for second in (1, 23, 44, 60, 65, 76, 97, 139, 153):
    path = ROOT / f'preview_{second:03d}.png'
    subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-n', '-ss', str(second), '-i', str(output),
        '-frames:v', '1', '-threads', '1', str(path)], check=True)
    frames.append(dict(time_s=second, path=str(path)))
with Path(manifest['source']).open('rb') as stream:
    assert hashlib.file_digest(stream, 'sha256').hexdigest() == manifest['source_sha256']
result = dict(status='PASS_TECHNICAL_QA_AWAITING_VISUAL_REVIEW', metadata=metadata,
    full_decode_returncode=decode.returncode, duration_s=duration, size_bytes=output.stat().st_size,
    mp4_atoms=atoms, faststart=True, source_unchanged=True, previews=frames)
with (ROOT / 'technical_qa.json').open('x') as stream:
    json.dump(result, stream, indent=2, ensure_ascii=False)
print(json.dumps(dict(duration_s=duration, size_bytes=output.stat().st_size,
    frames=streams[0]['nb_read_frames'], full_decode=decode.returncode, faststart=True)))
