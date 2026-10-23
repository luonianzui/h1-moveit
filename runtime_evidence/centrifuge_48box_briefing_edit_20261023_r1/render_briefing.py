import hashlib
import json
from pathlib import Path
import subprocess

ARTIFACT = Path(__file__).resolve().parent
ROOT = ARTIFACT / 'revision2'
SOURCE = ARTIFACT.parent / 'centrifuge_48box_sequential_full_20261023_r356/run/full_cycle.mp4'
FINAL = SOURCE.parent / 'final_overhead.png'
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
OUTPUT = ROOT / 'H1-2_离心机双盒流程_汇报版.mp4'
SEGMENTS = [
    (71.0, 82.0, 2., '01  操作准备 · 打开盖子'),
    (82.0, 155.2, 3., '02  左手装盒 · 抬肘搬运并放入摇篮'),
    (155.2, 215.5, 3., '03  右手装盒 · 完成双盒装篮'),
    (215.5, 241.0, 2., '04  按键操作 · 关盖并启动'),
    (241.0, 246.0, 1., '05  离心运行 · 转子旋转'),
    (299.0, 308.6, 2., '06  按键停转 · 等待转子停止'),
    (321.0, 338.0, 2., '07  按键开盖 · 准备取回'),
    (338.0, 439.0, 3., '08  左手取盒 · 返回桌面'),
    (439.0, 546.2, 3., '09  右手取盒 · 返回桌面'),
]

def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def execute(command, log):
    with log.open('xb') as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)

def overlay(label):
    return (
        "drawbox=x=0:y=0:w=iw:h=56:color=0x142438:t=fill,"
        f"drawtext=fontfile={FONT}:text='H1-2 离心机双盒操作演示':fontsize=29:fontcolor=white:x=26:y=12,"
        f"drawtext=fontfile={FONT}:text='Gazebo 仿真':fontsize=21:fontcolor=0x8ED4E6:x=w-tw-26:y=17,"
        "drawbox=x=0:y=842:w=iw:h=58:color=0x142438:t=fill,"
        f"drawtext=fontfile={FONT}:text='{label}':fontsize=27:fontcolor=white:x=26:y=855"
    )

def main():
    provenance = json.loads((ARTIFACT / 'source_timeline.json').read_text())
    original_hash = digest(SOURCE)
    assert original_hash == provenance['source_sha256']
    assert not OUTPUT.exists()
    segments = []
    timeline = []
    cursor = 0.
    encoder = ['-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
               '-pix_fmt', 'yuv420p', '-threads', '2', '-movflags', '+faststart']
    for index, (start, end, speed, label) in enumerate(SEGMENTS):
        path = ROOT / f'part_{index:02d}.mp4'
        duration = (end-start)/speed
        command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n',
            '-threads', '1', '-ss', str(start), '-t', str(end-start), '-i', str(SOURCE),
            '-vf', f'setpts=(PTS-STARTPTS)/{speed},fps=24,'+overlay(label), *encoder, str(path)]
        execute(command, ROOT / f'render_{index:02d}.log')
        segments.append(path)
        timeline.append(dict(source_start_s=start, source_end_s=end, playback_speed=speed,
            label=label, intended_output_start_s=cursor, intended_duration_s=duration, argv=command))
        cursor += duration
        print('RENDERED', index, label, flush=True)
    end_path = ROOT / 'part_09.mp4'
    command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n',
        '-loop', '1', '-framerate', '24', '-i', str(FINAL), '-t', '5',
        '-vf', overlay('完成  双盒已取回 · 空篮 · 空爪 · 转子停止'), *encoder, str(end_path)]
    execute(command, ROOT / 'render_09.log')
    segments.append(end_path)
    with (ROOT / 'concat.txt').open('x') as stream:
        stream.write(''.join(f"file '{path.name}'\n" for path in segments))
    execute(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n', '-f', 'concat',
        '-safe', '1', '-i', str(ROOT / 'concat.txt'), '-c', 'copy', '-movflags', '+faststart',
        '-metadata', 'title=H1-2 离心机双盒操作演示', '-metadata', 'comment=Edited Gazebo demonstration',
        str(OUTPUT)], ROOT / 'concat.log')
    assert digest(SOURCE) == original_hash
    with (ROOT / 'edit_manifest.json').open('x') as stream:
        json.dump(dict(status='RENDERED_AWAITING_DECODE_AND_VISUAL_QA', source=str(SOURCE),
            source_sha256=original_hash, source_unchanged=True, output=str(OUTPUT),
            output_sha256=digest(OUTPUT), segments=timeline, end_still=str(FINAL),
            end_still_sha256=digest(FINAL), end_still_seconds=5,
            source_task_wall_s=provenance['source_task_wall_s'],
            editorial_shortening_not_real_time_performance_evidence=True,
            no_synthetic_action_frames=True, no_music_or_narration_added=True), stream, indent=2, ensure_ascii=False)
    print('RENDER_COMPLETE', OUTPUT, flush=True)

if __name__ == '__main__':
    main()
