"""Edit a verified screen recording into a captioned 1-3 minute submission MP4."""
import bisect
import csv
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
CAPTURE=(Path(sys.argv[1]).resolve() if len(sys.argv)>1 else
         sorted(p for p in (ROOT/'submission_capture').iterdir() if p.is_dir())[-1])
META=json.loads((CAPTURE/'recording.json').read_text())
VERIFY=Path(META['verification_directory'])
OUT=CAPTURE/'edited'
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def run(args):
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y',*args],check=True)


def card(name,text,duration,font_size=43):
    txt=OUT/(name+'.txt');txt.write_text(text,encoding='utf-8')
    file=OUT/(name+'.mp4')
    vf=f'drawtext=fontfile={FONT}:textfile={txt}:expansion=none:x=70:y=90:fontsize={font_size}:line_spacing=26:fontcolor=white'
    run(['-f','lavfi','-i',f'color=c=0x17372D:s=1280x720:r=30:d={duration}',
         '-vf',vf,'-an','-c:v','libx264','-preset','medium','-crf','20','-threads','2',
         '-pix_fmt','yuv420p',str(file)])
    return file


def timestamp(seconds):
    n=max(0,round(seconds*100));h,n=divmod(n,360000);m,n=divmod(n,6000);s,c=divmod(n,100)
    return f'{h}:{m:02}:{s:02}.{c:02}'


def main():
    OUT.mkdir(exist_ok=True)
    events=META['events'];base=META['capture_start_monotonic']
    started=next(e for e in events if e['event']=='started')
    origin=started['wall_monotonic_s']-started['elapsed']
    trim=max(0.,started['wall_monotonic_s']-base-1.)
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration',
        '-of','json',str(CAPTURE/'demo_realtime.mp4')],text=True))
    raw_duration=float(probe['format']['duration'])
    rows=list(csv.DictReader(next(VERIFY.glob('*_precision.csv')).open()))
    times=[origin+float(r['elapsed_s']) for r in rows]
    ev_times=[e['wall_monotonic_s'] for e in events]
    ass=['''[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 720
WrapStyle: 2

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Meter,Noto Sans CJK JP,23,&H00FFFFFF,&H00FFFFFF,&H0017372D,&H0017372D,0,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
''']
    phase_names={'idle':'巡回・次の鉢へ移動','approach':'鉢へ低速で接近','settle':'停止して給水準備',
                 'pour':'測定角度で給水を調整','upright':'じょうろを直立へ戻す','return':'巡回経路へ復帰'}
    t=trim
    while t<raw_duration:
        now=base+t
        previous=events[:bisect.bisect_right(ev_times,now)]
        decision=next((e for e in reversed(previous) if e['event']=='ai_decision'),{})
        reached=next((e for e in reversed(previous) if e['event']=='waypoint_reached'),{})
        lap=reached.get('lap',1);waypoint=reached.get('waypoint',1)
        r=rows[max(0,bisect.bisect_right(times,now)-1)]
        active=r['phase']!='idle' and r['waypoint']!=''
        phase=phase_names.get(r['phase'],'巡回')
        if not active and decision.get('action')=='skip':phase='AI判断: 給水を見送り'
        if previous and previous[-1]['event']=='finished':phase='2周を完了・停止'
        inputs=decision.get('inputs',{})
        soil=inputs.get('soil_moisture_pct');wilt=inputs.get('leaf_wilt_pct')
        soil='--' if soil is None else f'{soil:.1f}'
        wilt='--' if wilt is None else f'{wilt:.1f}'
        target=decision.get('target_ml',0.)
        amount=float(r['session_ml']) if active else 0.
        lines=['AI給水 巡回デモ',f'{lap}/2周目  地点{waypoint}','',phase,'',
            f'模擬土壌水分 {soil}',f'模擬しおれ度 {wilt}',f'AI目標 {target:.1f} mL','',
            f'測定角度 {float(r["angle_deg"]):.1f} 度',f'指令角度 {float(r["command_deg"]):.1f} 度',
            f'仮想流量 {float(r["flow_ml_s"]):.1f}', 'mL / Sim秒','',
            f'今回の給水 {amount:.1f} mL' if active else '今回の給水 --',
            '（水量は仮想モデル）','', '1周目: 鉢1・3へ給水','2周目: 状態を再判断']
        txt=r'{\pos(952,22)\q2}'+r'\N'.join(lines)
        ass.append(f'Dialogue: 0,{timestamp((t-trim)/2)},{timestamp((min(t+.5,raw_duration)-trim)/2)},Meter,,0,0,0,,{txt}\n')
        t+=.5
    (OUT/'meters.ass').write_text(''.join(ass),encoding='utf-8-sig')
    intro1=card('intro1','植物の状態に応じて水やりする\n自律巡回ロボット\n\nPhysical AI応用1 最終課題\nomnicampus: nobima123',6,46)
    intro2=card('intro2','4個のプランターを2周巡回\n模擬センサーからAIが目標給水量を推定\nじょうろの測定角度で仮想給水を調整\n\n実行映像は2倍速 / 字幕付き・音声なし',6,39)
    vf=(f'trim=start={trim:.6f},setpts=(PTS-STARTPTS)/2,fps=30,'
        f'drawbox=x=932:y=0:w=348:h=720:color=0x17372D:t=fill,ass={OUT/"meters.ass"}')
    body=OUT/'body.mp4'
    run(['-i',str(CAPTURE/'demo_realtime.mp4'),'-vf',vf,'-an','-c:v','libx264',
         '-preset','medium','-crf','20','-threads','2','-pix_fmt','yuv420p',str(body)])
    a,b=META['verification']['dose_results']
    end=card('results',f'結果: 2周の巡回を完了\n鉢1   180.00 / {a["delivered_ml"]:.2f} 仮想mL\n'
        f'鉢3   152.20 / {b["delivered_ml"]:.2f} 仮想mL\n（AI目標 / 給水の積算量）\n'
        '2周目は4鉢すべて給水見送り\n\n土・葉・水量は仮想モデル。実植物への精度は未評価',14,35)
    listing=OUT/'concat.txt'
    listing.write_text(''.join(f"file '{p}'\n" for p in [intro1,intro2,body,end]))
    final=OUT/'PAI_final_nobima123.mp4'
    run(['-f','concat','-safe','0','-i',str(listing),'-c','copy','-movflags','+faststart',
        '-metadata','title=PAI最終課題_nobima123','-metadata','artist=nobima123',str(final)])
    result=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries',
        'format=duration,size:stream=codec_name,width,height,r_frame_rate','-of','json',str(final)],text=True))
    duration=float(result['format']['duration'])
    assert 60<=duration<=180
    (OUT/'movie_info.json').write_text(json.dumps(dict(raw_trim_start=trim,playback_speed=2,
        intro_seconds=12,result_seconds=14,**result),indent=2)+'\n')
    print(json.dumps(dict(final=str(final),**result)),flush=True)


if __name__=='__main__':main()
