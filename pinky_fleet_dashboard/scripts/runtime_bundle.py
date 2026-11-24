"""로봇 쪽 스크립트·설정을 하나의 작고 검증된 묶음(runtime bundle)으로 만들어 SSH 명령 한 줄로 올린다.

실행 위치: PC 관제(app.py가 install_command를 import). 만든 명령은 로봇에서 돈다.
사용처: app._nav2_runtime_install_command → 주행준비(app.py의 _ssh_run).

핵심: 소스마다 셸 인자를 하나씩 늘려 가는 대신, 작고 검증된 묶음 하나를 보낸다.
- 파일들을 {로봇 쪽 상대 경로: 내용} JSON으로 묶고 gzip → base64 한 문자열로 만든다.
- 로봇에서는 sha256으로 내용을 확인한 뒤 /home/pinky/.cache/pinky-fleet/ 아래에 .tmp로 쓰고 os.replace로
  바꿔 끼운다(쓰다 끊겨도 반쯤 쓴 파일이 남지 않는다). 그다음 install_nav2_runtime.py를 바로 실행한다.
- 주의: 명령 전체가 SSH 인자 하나(로봇에서는 bash -c 인자 하나)로 넘어가므로 Linux 인자 한 개 한도
  131072 바이트(MAX_ARG_STRLEN)를 넘으면 주행준비가 실패한다. 묶이는 스크립트가 커지면 크기를 확인한다.
"""
import base64,gzip,hashlib,json,shlex
from pathlib import Path


def install_command(root,config,profile):
    """로봇에서 묶음을 풀고 Nav2 실행 설정을 만드는 셸 명령 문자열을 돌려준다.

    root: pinky_fleet_dashboard 폴더, config: 로봇별 runtime_config.json 내용(차체 치수·구역·navigation 설정),
    profile: 그 로봇의 IR 보정값(ir_profile.json).
    """
    # 로봇 쪽 경로(키) ← 저장소 경로(값). 설치기와 경로 주행기는 pinky-fleet/ 바로 아래, 나머지 노드는 nav2/ 아래에 둔다.
    sources={'install_nav2_runtime.py':'scripts/install_nav2_runtime.py','navigate_route.py':'scripts/navigate_route.py'}
    for name in ['ultrasonic_safety','velocity_guard','line_following','mission_behaviors','motion_gate','traffic','control_journal','status_lamp','status_display','fusion_grid']:
        sources['nav2/'+name+'.py']='scripts/'+name+'.py'
    sources.update({
        'nav2/keepout_mask.pgm':'config/keepout/keepout_mask.pgm',
        'nav2/keepout_mask.yaml':'config/keepout/keepout_mask.yaml',
    })
    # 금지 구역 마스크도 함께 보낸다(map-proto 지도일 때 Nav2 keepout filter가 쓴다).
    files={name:(Path(root)/source).read_text(encoding='utf-8') for name,source in sources.items()}
    files['nav2/runtime_config.json']=json.dumps(config,allow_nan=False)
    files['nav2/ir_profile.json']=json.dumps(profile,allow_nan=False)
    # 압축 전 JSON의 sha256을 같이 보내, 로봇이 푼 내용이 PC가 만든 것과 같은지 확인하게 한다.
    raw=json.dumps(files).encode();encoded=base64.b64encode(gzip.compress(raw)).decode();digest=hashlib.sha256(raw).hexdigest()
    # 로봇에서 python3 -c로 도는 작은 설치 프로그램. PAYLOAD·DIGEST 자리에 위 값이 들어간다.
    # 경로가 root 밖을 가리키면(예: '../') 거부한다.
    program="""import base64,gzip,hashlib,json,os
from pathlib import Path
raw=gzip.decompress(base64.b64decode(PAYLOAD))
if hashlib.sha256(raw).hexdigest()!=DIGEST:raise RuntimeError('Runtime bundle hash mismatch')
root=Path('/home/pinky/.cache/pinky-fleet');root.mkdir(parents=True,exist_ok=True)
for name,text in json.loads(raw).items():
 p=root/name
 if not p.resolve().is_relative_to(root.resolve()):raise ValueError('Invalid runtime path')
 p.parent.mkdir(parents=True,exist_ok=True);t=p.with_suffix(p.suffix+'.tmp');t.write_text(text);os.replace(t,p)
""".replace('PAYLOAD',repr(encoded)).replace('DIGEST',repr(digest))
    return 'python3 -c '+shlex.quote(program)+' && python3 /home/pinky/.cache/pinky-fleet/install_nav2_runtime.py'
