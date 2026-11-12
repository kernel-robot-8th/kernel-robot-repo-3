#!/usr/bin/env python3
"""Install the user service definition; enabling it is a separate explicit step."""
import argparse,subprocess
from pathlib import Path

def main():
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
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    if args.start:subprocess.run(['systemctl','--user','enable','--now','pinky-fleet.service'],check=True)
    print(unit)
if __name__=='__main__':main()
