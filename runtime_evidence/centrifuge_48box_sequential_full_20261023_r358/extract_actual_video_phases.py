import json
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw


EVIDENCE = Path(__file__).resolve().parent
PHASES = ('machine_preparation_open_press', 'paired_load_approach', 'paired_load_grasp',
          'paired_load_static_grip_hold', 'paired_load_forward_outside_housing_table_transfer',
          'paired_load_forward_inside_basket_raise', 'paired_load_drop_settling', 'machine_close_press',
          'machine_running_steady', 'machine_stop_press', 'machine_final_open_press',
          'paired_retrieval_approach', 'paired_retrieval_grasp', 'paired_retrieval_lift',
          'paired_retrieval_static_grip_hold', 'paired_retrieval_transfer',
          'paired_retrieval_setdown_fine', 'paired_retrieval_release_on_table', 'terminal_table_hold')


def main():
    folder = EVIDENCE / 'run'
    profile = json.loads((folder / 'actual_timing_profile.json').read_text())
    capture = json.loads((folder / 'full_cycle.capture.json').read_text())
    recorder_start = json.loads((folder / 'video_pid.json').read_text())['created']
    duration = capture['frames'] / 12
    intervals = {row['phase']: row for row in profile['phase_intervals']}
    records = []
    for index, phase in enumerate(PHASES + ('original_movie_last_frame',), 1):
        if phase == 'original_movie_last_frame':
            offset = duration - .2
        else:
            interval = intervals[phase]
            offset = (interval['start_wall'] + interval['end_wall']) / 2 - recorder_start
        offset = min(max(offset, .5), duration - .2)
        path = folder / f'recorded_actual_phase_{index:02d}.png'
        assert not path.exists()
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n',
                        '-ss', str(offset), '-i', str(folder / 'full_cycle.mp4'),
                        '-frames:v', '1', str(path)], check=True, timeout=60)
        records.append(dict(phase=phase, path=str(path), approximate_wall_offset_s=offset))
    sheet = Image.new('RGB', (1800, ((len(records) + 2) // 3) * 475), 'white')
    draw = ImageDraw.Draw(sheet)
    for index, record in enumerate(records):
        column = index % 3
        row = index // 3
        with Image.open(record['path']) as frame:
            sheet.paste(frame.resize((600, 450)), (column * 600, row * 475 + 25))
        draw.text((column * 600 + 5, row * 475 + 5),
                  f"{index + 1:02d} {record['phase']}", fill='black')
    sheet_path = folder / 'actual_phase_contact_sheet.png'
    assert not sheet_path.exists()
    sheet.save(sheet_path)
    with (folder / 'actual_video_phase_context.json').open('x') as stream:
        json.dump(dict(status='EXTRACTED_NOT_YET_REVIEWED', frames=records,
                       original_movie_only=True, exact_sim_frame_sync_not_claimed=True,
                       terminal_hold_or_full_task_pass_not_fabricated=True), stream, indent=2)
    print('ORIGINAL_PHASE_FRAMES', len(records), str(sheet_path), flush=True)


if __name__ == '__main__':
    main()
