#!/usr/bin/env python3
"""PC 관제를 systemd 사용자 서비스(pinky-fleet.service)로 등록한다. 활성화(enable)는 따로 명시해야 하는 단계다.

실행 위치: 오프라인 도구 (사람이 PC에서 직접 실행: /usr/bin/python3 scripts/install_user_service.py [--start]).
사용처: OPERATIONS.md의 "사용자 서비스 재설치".

핵심: ~/.config/systemd/user/pinky-fleet.service를 쓰고 daemon-reload만 한다. --start를 주면 enable --now로
등록과 시작을 함께 한다. 서비스는 run.sh를 실행하고 실패 시 3초 뒤 다시 시작한다. SSH 비밀번호는 서비스 파일에
넣지 않는다(재시작 뒤 웹에서 다시 입력).
"""
import argparse,subprocess
from pathlib import Path

def main():
    """서비스 파일을 쓰고 systemd에 다시 읽힌다. --start면 바로 켠다."""
    # root는 이 파일의 두 단계 위(pinky_fleet_dashboard). 서비스 파일의 ROOT 자리에 들어간다.
    parser=argparse.ArgumentParser();parser.add_argument('--start',action='store_true');args=parser.parse_args()
    root=Path(__file__).resolve().parent.parent
    unit=Path.home()/'.config/systemd/user/pinky-fleet.service';unit.parent.mkdir(parents=True,exist_ok=True)
    unit.write_text('''[Unit]
Description=Pinky fleet dashboard
After=network.target

[Service]
Type=simple
WorkingDirectory=ROOT
ExecStart=/bin/bash "ROOT/run.sh"
Restart=on-failure
RestartSec=3
TimeoutStopSec=15
KillMode=control-group
Environment=PYTHONUNBUFFERED=1
StandardOutput=journal
StandardError=journal
UMask=0077

[Install]
WantedBy=default.target
'''.replace('ROOT',str(root).replace('%','%%')))
    # systemd 유닛 파일에서 '%'는 지정자(specifier) 기호라, 경로의 '%'는 위에서 '%%'로 바꿨다.
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    if args.start:subprocess.run(['systemctl','--user','enable','--now','pinky-fleet.service'],check=True)
    print(unit)
if __name__=='__main__':main()
