"""Small, verified runtime bundle instead of a growing shell argument per source."""
import base64,gzip,hashlib,json,shlex
from pathlib import Path


def install_command(root,config,profile):
    sources={'install_nav2_runtime.py':'scripts/install_nav2_runtime.py','navigate_route.py':'scripts/navigate_route.py'}
    for name in ['ultrasonic_safety','velocity_guard','line_following','mission_behaviors','motion_gate','traffic','control_journal']:
        sources['nav2/'+name+'.py']='scripts/'+name+'.py'
    sources.update({
        'nav2/keepout_mask.pgm':'config/keepout/keepout_mask.pgm',
        'nav2/keepout_mask.yaml':'config/keepout/keepout_mask.yaml',
    })
    files={name:(Path(root)/source).read_text(encoding='utf-8') for name,source in sources.items()}
    files['nav2/runtime_config.json']=json.dumps(config,allow_nan=False)
    files['nav2/ir_profile.json']=json.dumps(profile,allow_nan=False)
    raw=json.dumps(files).encode();encoded=base64.b64encode(gzip.compress(raw)).decode();digest=hashlib.sha256(raw).hexdigest()
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
