#!/usr/bin/env python3
"""Validate and publish completed H-preserving research MATs; never controls simulations."""
import argparse
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import io
import itertools
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import shutil

from public_export import public_export

from validate_results import validate_file

ROOT = Path(__file__).resolve().parents[1]
PRIVATE = ROOT/'results/hmc_standalone'
OUT = ROOT/'results/hmc_standalone_public'
FROZEN = ROOT/'validation/hmc_standalone/frozen_code'
STATE = ROOT/'validation/hmc_standalone/run/publisher_status.json'
MARKER = 'HMCIndependent-Publisher: true'
COMBOS = list(itertools.product([50,60,70,80,90,100], [.9,.8,.7,.6,.5], [0,.5,1,1.5,2,2.5,3]))
EXPECTED = {f'hmc_{s:g}kph_mu{mu:g}_delta{a:+g}.mat':(s,mu,a)
            for s,mu,a in COMBOS}
OVERVIEWS = {f'overview_final_{s:g}kph_mu{mu:g}_trim.png'.replace(f'mu{mu:g}',f'mu{mu:g}'.replace('.','p')):(s,mu)
             for s,mu in itertools.product([50,60,70,80,90,100],[.9,.8,.7,.6,.5])}
MODEL_FILES = ['hmc_build_native.m','hmc_classify.m','hmc_generate.m',
    'hmc_handling_reference.m','hmc_mcode_simulate.m','hmc_native.m','hmc_options.m',
    'hmc_tire_force.m','hmc_vehicle_model.m','run_hmc_standalone.m',
    'native/hmc_native.cpp','native/hmc_tire.hpp','native/hmc_vehicle.hpp']
SOURCES = {'hmc_standalone/'+name for name in MODEL_FILES+[
    'README_KO.md','IMPORTER_README.md','hmc_import_data.py','run_full_210.py',
    'plot_results.py','validate_results.py','publish_results.py','compare_reference.py','public_export.py']}
SOURCES.update({'validation/hmc_standalone/baseline_comparison.json',
                'validation/hmc_standalone/current_validation.json',
                'validation/hmc_standalone/dt_comparison.json'})
OPTIONAL_SOURCE_PACKAGES = set()
GENERATED = {'README.md','publication.json','publication_progress.json',
             'publication_summary.csv','figure_index.json'}
CACHE = {}
FIG_CACHE = {}


def utc():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def log(message):
    print(utc()+' '+str(message),flush=True)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def readjson(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def atomic(path, text):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp = path.with_name(path.name+f'.publisher.{os.getpid()}.tmp')
    temp.write_text(text)
    temp.replace(path)


def writejson(path, data):
    atomic(path,json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def require(condition,message):
    if not condition:
        raise ValueError(message)


def set_state(phase,**extra):
    value=readjson(STATE)
    value.update(phase=phase,pid=os.getpid(),updated_utc=utc(),**extra)
    writejson(STATE,value)


def source_hashes():
    hashes=readjson(FROZEN/'source_fingerprint.json')
    require(set(hashes)==set(MODEL_FILES+['native/hmc_native','data/damper.json','data/vehicle.json','data/tires.json','data/source_manifest.json','data/hmc_data.mat']),'Missing frozen source manifest')
    for name,value in hashes.items():
        require(sha(FROZEN/name)==value,'Frozen source changed: '+name)
        require(sha(ROOT/'hmc_standalone'/name)==value,'Public source differs from production: '+name)
    return hashes


def scan():
    hashes=source_hashes()
    items=[]
    errors={}
    for name,condition in EXPECTED.items():
        path=PRIVATE/name
        if not path.is_file():
            continue
        stamp=(path.stat().st_size,path.stat().st_mtime_ns,tuple(sorted(hashes.items())))
        try:
            cached=CACHE.get(name)
            if cached and cached[0]==stamp:
                # Atomic final MATs are immutable; unchanged byte size/mtime
                # reuses the independently checked SHA and full-data validation.
                item=cached[1]
            else:
                checked=validate_file(path,hashes)
                exported=public_export(path,OUT/name,hashes)
                item=validate_file(OUT/name)
                item['input_sha256']=checked['sha256']
                item['private_model_data_excluded']=exported['private_model_data_excluded']
                require(tuple(item[k] for k in ['speed_kmh','mu_road','front_steer_deg'])==condition,
                        'Filename/condition mismatch')
                require(item.get('all_checks_passed') is True,'Data validator did not pass')
                CACHE[name]=(stamp,item)
            items.append(item)
        except Exception as exc:
            errors[name]=str(exc)
    return items,errors,hashes


def image_record(path):
    stamp=(path.stat().st_size,path.stat().st_mtime_ns)
    cached=FIG_CACHE.get(path.name)
    if cached and cached[0]==stamp:
        return cached[1]
    with path.open('rb') as stream:
        require(stream.read(8)==b'\x89PNG\r\n\x1a\n','Not a PNG: '+path.name)
    require(stamp[0]>100,'Empty PNG: '+path.name)
    record=dict(filename=path.name,bytes=stamp[0],sha256=sha(path))
    FIG_CACHE[path.name]=(stamp,record)
    return record


def figures(items):
    index=readjson(PRIVATE/'figures/plot_index.json')
    plotter=ROOT/'hmc_standalone/plot_results.py'
    if index.get('plotter_sha256')!=sha(plotter):
        return [],[]
    cases=[]
    valid={item['filename']:item for item in items}
    records=index.get('records',{})
    ready=set()
    for name,item in valid.items():
        record=records.get(name,{})
        image=Path(name).stem+'.png'
        path=PRIVATE/'figures'/image
        if (record.get('source_sha256')==item['input_sha256'] and record.get('source_mat')==name
                and record.get('figure')==image and path.is_file()):
            target=OUT/'figures'/image
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(path,target)
            picture=image_record(target)
            picture.update(source_mat=name,source_sha256=item['sha256'])
            cases.append(picture)
            ready.add(name)
    overviews=[]
    for image,(speed,mu) in OVERVIEWS.items():
        members=[name for name,(s,m,a) in EXPECTED.items() if s==speed and m==mu]
        path=PRIVATE/'figures'/image
        if set(members)<=ready and image in index.get('overview_figures',[]) and path.is_file():
            target=OUT/'figures'/image
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(path,target)
            picture=image_record(target)
            picture.update(source_mats=members)
            overviews.append(picture)
    return cases,overviews


def allowed_path(path):
    if path in SOURCES or path in OPTIONAL_SOURCE_PACKAGES:
        return True
    prefix='results/hmc_standalone_public/'
    if not path.startswith(prefix):
        return False
    rel=path[len(prefix):]
    if rel in GENERATED or rel in EXPECTED:
        return True
    if rel.endswith('_summary.csv') and rel[:-12]+'.mat' in EXPECTED:
        return True
    if rel.startswith('figures/') and rel.count('/')==1:
        filename=rel[8:]
        return filename in OVERVIEWS or filename[:-4]+'.mat' in EXPECTED if filename.endswith('.png') else False
    return False


def git(*args):
    result=subprocess.run(['git','-C',str(ROOT),*args],text=True,capture_output=True,timeout=180)
    if result.returncode:
        raise RuntimeError('git '+args[0]+': '+result.stderr.strip())
    return result.stdout.strip()


def sync_git():
    require(git('symbolic-ref','--short','HEAD')=='main','Unexpected Git branch; publisher paused')
    require(not git('diff','--cached','--name-only'),'Existing staged work; publisher paused')
    git('fetch','--quiet','origin','main')
    local,remote=git('rev-parse','HEAD'),git('rev-parse','FETCH_HEAD')
    if local==remote:
        return
    base=git('merge-base',local,remote)
    if base==local:
        require(not git('diff','--name-only'),'Tracked changes prevent a safe fast-forward')
        git('merge','--ff-only','FETCH_HEAD')
        return
    require(base==remote,'Local and remote main diverged; publisher paused')
    for commit in git('rev-list',remote+'..'+local).splitlines():
        require(MARKER in git('show','-s','--format=%B',commit),'Unknown unpushed commit; publisher paused')
        paths=git('diff-tree','--no-commit-id','--name-only','-r',commit).splitlines()
        require(paths and all(allowed_path(path) for path in paths),'Unapproved path in pending publisher commit')
    git('push','origin','HEAD:main')


def sanitized_progress(items,producer,phase):
    current=producer.get('current')
    if current:
        fields=['scenario_index','speed_kmh','mu_road','front_steer_deg',
                'completed_wear_cases','current_wear_case','wear_total']
        current={key:current[key] for key in fields if key in current}
    return dict(completed=len(items),total=210,producer_phase=phase,current=current,
                producer_completed_scenarios=producer.get('completed_scenarios',0),
                grid_size=[61,121],model_scope='Independent approximation, not CarSim/TNO engine',dt_s=.001,slice_mode='trim',initial_roll_mode='per-case computed trim phi_eq',
                main_mask='final',outside_H='not_simulated',roll_dynamics_retained=True,
                preliminary_estimate_hours=producer.get('preliminary_estimate_hours'),
                horizons_s=[5,10],observed_utc=utc())


def write_public_files(items,errors,hashes,cases,overviews,producer):
    phase=producer.get('phase','unknown')
    progress=sanitized_progress(items,producer,phase)
    writejson(OUT/'publication_progress.json',progress)
    writejson(OUT/'publication.json',dict(version='HMC_INDEPENDENT_1',
              validated_scenarios=len(items),total_scenarios=210,producer_phase=phase,
              validation_errors=errors,source_sha256=hashes,scenarios=items,
              condition_figures=len(cases),overview_figures=len(overviews),updated_utc=utc()))
    writejson(OUT/'figure_index.json',dict(condition_figures=cases,overview_figures=overviews,
              plotter_sha256=sha(ROOT/'hmc_standalone/plot_results.py'),updated_utc=utc()))
    table=io.StringIO()
    fields=['filename','speed_kmh','mu_road','front_steer_deg','wear_case','initial_roll_rad',
            'reference_count','recovered_5s','recovered_10s','final_5s','final_10s',
            'guarded_5s','guarded_10s','sampled_area',
            'accepted_cells','center_points','trim_usable','trim_in_reference',
            'grid_truncated','reference_grid_truncated']
    writer=csv.DictWriter(table,fieldnames=fields);writer.writeheader()
    for item in items:
        for case in item['case_summaries']:
            row={key:item[key] for key in fields[:4]}
            for key in ['wear_case','initial_roll_rad','reference_count','sampled_area','accepted_cells',
                        'center_points','trim_usable','trim_in_reference','grid_truncated','reference_grid_truncated']:
                row[key]=case[key]
            for name in ['recovered','final','guarded']:
                row[name+'_5s'],row[name+'_10s']=case[name+'_counts']
            writer.writerow(row)
    atomic(OUT/'publication_summary.csv',table.getvalue())
    ready={picture['source_mat']:picture['filename'] for picture in cases}
    lines=['# HMC 독립 모델: H 유지 복귀 영역','',
           f'**검증 후 업로드: {len(items)}/210조건 · 조건별 마모 5종.** 계산 상태: `{phase}`.','',
           '[현재 진행 상황](publication_progress.json) · [전체 수치 요약](publication_summary.csv) · '
           '[실행·판정 설명](../../hmc_standalone/README_KO.md)','',
           '**CarSim/TNO 실행 결과가 아닌 HMC 데이터를 이용해 구현한 독립 근사 모델 결과입니다.** '
           '사용자 제공 실행 Echo·MF6.2 마모별 타이어5종·CDC950mA 선도·속도PI제어를 반영한34상태 모델입니다. '
           '내부 MF-Swift 벨트·다물체 엔진 전체의 동일성은 보장하지 않습니다.','',
           '제공된 정상타이어50km/h·mu=.5·0deg 한 조건에서 H227점 동일, final CarSim200점/독립201점으로 '
           '226/227개 판정이 일치했고 guarded_final196점은 좌표까지 일치했습니다. '
           '다른210조건 전체의 CarSim일치성을 검증했다는 뜻은 아닙니다. '
           '**Pitch·roll 시간응답은 아직 CarSim시계열과 미검증입니다.** '
           '정상pitch는CarSim.00777deg와독립모델약0deg로차이가있습니다.','',
           '초기 격자는 vy−3:.1:3, r−1.5:.025:1.5,61×121입니다. '
           'CarSim과 같은 .001초 시간간격을 사용하며 기준조건에서 .0005초 결과와 모든 마스크·셀 판정이 일치했습니다. '
           '주 결과 `masks.final`은 H안 초기점 중 물리제약·H유지·정상선회점 복귀를 통과한 점입니다. '
           '`guarded_final`은 별도 yaw/ay정책을 추가합니다. H밖 초기점은 미계산입니다.','',
           'MAT 변수는 `RESEARCH2D`, `CASE{1}`~`{5}`는 타이어 마모입니다. '
           '10초 final의 네 꼭짓점과 실제 중심점 시뮬레이션이 통과한 셀 면적을 저장합니다. '
           '영역 이동·볼록껍질 채우기·점 수에 맞춘 피팅을 하지 않습니다. '
           '유한시간 표본 판정이며 연속영역·불변집합·점근 안정성의 증명이 아닙니다.','',
           '공개MAT에는 계산결과만 포함하며, 차량·타이어 원자료와 내장된 입력데이터 메타데이터를 제거했습니다. '
           '원자료의 체크섬만 보존합니다. 코드 재실행에는 사용자가 가진 로컬 입력데이터가 필요합니다.','',
           '| 속도 km/h | μ | 앞 휠 조향각 ° | MAT | 2D 그림 | 10초 final 점 수, 마모 1~5 | 격자 접촉 |',
           '|---|---|---|---|---|---|---|']
    for item in items:
        name=item['filename'];picture=ready.get(name)
        plot=f'[그림](figures/{picture})' if picture else '생성 중'
        counts=', '.join(str(case['final_counts'][1]) for case in item['case_summaries'])
        touch=', '.join(str(case['wear_case']) for case in item['case_summaries'] if case['grid_truncated'] or case['reference_grid_truncated']) or '없음'
        lines.append(f"| {item['speed_kmh']:g} | {item['mu_road']:g} | {item['front_steer_deg']:g} | [MAT]({name}) | {plot} | {counts} | {touch} |")
    if overviews:
        lines+=['','## 조향각 7종 비교','']
        for image in overviews:
            speed,mu=OVERVIEWS[image['filename']]
            lines.append(f"- [{speed:g} km/h · μ={mu:g}](figures/{image['filename']})")
    if errors:lines+=['','검증 실패 파일은 업로드에서 제외했습니다. `publication.json`에 상세 오류를 기록합니다.']
    atomic(OUT/'README.md','\n'.join(lines)+'\n')


def publish(items,errors,hashes,cases,overviews,producer):
    sync_git()
    write_public_files(items,errors,hashes,cases,overviews,producer)
    paths=sorted(SOURCES)
    paths += sorted(path for path in OPTIONAL_SOURCE_PACKAGES if (ROOT/path).is_file())
    paths += [str((OUT/name).relative_to(ROOT)) for name in GENERATED]
    for item in items:
        paths.append(str((OUT/item['filename']).relative_to(ROOT)))
        # The MATLAB producer writes per-case CSVs after the atomic final MAT.
        # Publish the independently reconstructed publication_summary.csv instead.
    paths += [str((OUT/'figures'/image['filename']).relative_to(ROOT)) for image in cases+overviews]
    require(all(allowed_path(path) and (ROOT/path).is_file() for path in paths),'Unapproved or missing publication path')
    git('add','-f','--',*paths)
    staged=git('diff','--cached','--name-only').splitlines()
    require(all(allowed_path(path) for path in staged),'Unexpected staged path; no commit was made')
    if staged:
        body=f'Publish validated independent HMC stability results ({len(items)}/210)\n\n{MARKER}'
        git('commit','-m',body)
        git('push','origin','HEAD:main')
    log(f'PUBLISHED {len(items)}/210; figures {len(cases)}/210 + {len(overviews)}/30')
    return git('rev-parse','HEAD')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval',type=float,default=15)
    parser.add_argument('--progress-interval',type=float,default=60)
    parser.add_argument('--scan-only',action='store_true')
    args=parser.parse_args()
    if args.scan_only:
        items,errors,hashes=scan()
        cases,overviews=figures(items)
        print(json.dumps(dict(validated_count=len(items),validation_errors=errors,
                    source_sha256=hashes,condition_figures=len(cases),overview_figures=len(overviews),
                    scenarios=items),indent=2,allow_nan=False))
        return 1 if errors else 0
    STATE.parent.mkdir(parents=True,exist_ok=True)
    lock=(STATE.parent/'publisher.lock').open('w')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        log('Publisher already running')
        return 2
    lock.write(str(os.getpid()));lock.flush()
    last_data=None;last_progress=None;last_figures=None;last_publish=0
    while True:
        try:
            items,errors,hashes=scan()
            cases,overviews=figures(items)
            producer=readjson(PRIVATE/'progress.json')
            phase=producer.get('phase','unknown')
            terminal=phase in {'completed','failed','cancelled','stopped'}
            data_key=tuple((item['filename'],item['sha256']) for item in items)
            progress_key=json.dumps({k:producer.get(k) for k in ['phase','completed_scenarios','current']},sort_keys=True)
            figure_key=tuple((image['filename'],image['sha256']) for image in cases+overviews)
            changed=(data_key!=last_data or terminal or
                     ((progress_key!=last_progress or figure_key!=last_figures) and time.time()-last_publish>=args.progress_interval))
            complete=phase=='completed' and len(items)==210 and not errors and len(cases)==210 and len(overviews)==30
            if changed and (items or terminal or last_data is None):
                commit=publish(items,errors,hashes,cases,overviews,producer)
                last_data,last_progress,last_figures=data_key,progress_key,figure_key
                last_publish=time.time()
                set_state('completed' if complete else 'watching',published_count=len(items),
                          condition_figures=len(cases),overview_figures=len(overviews),
                          ready_condition_figures=len(cases),ready_overview_figures=len(overviews),
                          validation_errors=errors,last_commit=commit,last_published_utc=utc(),errors=[])
            else:
                set_state('watching',validated_count=len(items),validation_errors=errors,
                          ready_condition_figures=len(cases),ready_overview_figures=len(overviews),errors=[])
            if complete:
                return 0
            if phase in {'failed','cancelled','stopped'}:
                set_state('partial_stopped',producer_phase=phase,published_count=len(items),validation_errors=errors)
                return 1
            if phase=='completed' and errors:
                set_state('blocked_validation',published_count=len(items),validation_errors=errors)
                return 1
        except Exception as exc:
            set_state('retrying',errors=[str(exc)])
            log('RETRY '+str(exc))
        time.sleep(max(1,args.interval))


if __name__=='__main__':
    sys.exit(main())
