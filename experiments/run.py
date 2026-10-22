"""Run one paper configuration on prepared sensor inputs."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import re
import subprocess
import numpy as np
from evaluation.status import classify_navigation

ROOT = Path(__file__).resolve().parents[1]

def render_config(sequence, profile, data_root):
    text = (ROOT / 'configs' / f'{sequence.lower()}.yaml').read_text()
    profiles = json.loads((ROOT / 'configs/profiles.json').read_text())
    for key, value in profiles[profile].items():
        token = str(value).lower() if isinstance(value, bool) else str(value)
        text, count = re.subn(r'^' + re.escape(key) + r':.*$', key + ': ' + token, text, flags=re.M)
        if count != 1:
            raise ValueError(f'Expected one configuration key: {key}')
    return text.replace('${DATA_ROOT}', Path(data_root).resolve().as_posix())

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', choices=['BY2','BY2H','BY2O'], required=True)
    parser.add_argument('--profile', default='full', choices=list(json.loads((ROOT/'configs/profiles.json').read_text())))
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--binary', type=Path, default=ROOT/'build/legsa_gins')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = args.output/'config.yaml'
    config.write_text(render_config(args.sequence,args.profile,args.data_root))
    result = subprocess.run([str(args.binary.resolve()),'--config',str(config.resolve()),'--output-dir',str(args.output.resolve())])
    summary_path=args.output/'summary.json'
    summary=json.loads(summary_path.read_text()) if summary_path.exists() else {'status':'FAILED'}
    native=args.output/'KF_GINS_Navresult.nav'
    if native.exists():
        bound=classify_navigation(np.loadtxt(native))
        if bound!='COMPLETED':summary['status']=bound
        elif result.returncode==3:
            summary['status']='ALGORITHM_FAILURE_ALL_YAW_REJECTED' if summary['heading_rejected'] else 'ALGORITHM_FAILURE_NO_VALID_HEADING_INPUT'
    elif result.returncode==4:summary['status']='ALGORITHM_FAILURE_DIVERGED'
    summary.update(sequence=args.sequence,profile=args.profile,exit_code=result.returncode)
    summary_path.write_text(json.dumps(summary,indent=2)+'\n')
    raise SystemExit(result.returncode or (0 if summary['status']=='COMPLETED' else 4))

if __name__ == '__main__':
    main()
