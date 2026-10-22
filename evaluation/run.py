"""Evaluate a navigation run at the antenna midpoint used in the manuscript."""
import argparse
from io import StringIO
import json
from pathlib import Path
import subprocess
import sys
import pandas as pd
from .midpoint import transform_nav,write_transformed_nav

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sequence',choices=['BY2','BY2H','BY2O'],required=True)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();spec=json.loads((ROOT/'configs/evaluation.json').read_text())[a.sequence]
    a.output.mkdir(parents=True,exist_ok=False)
    native=a.run/'KF_GINS_Navresult.nav'
    text='\n'.join(line for line in native.read_text().splitlines() if line.strip() and not line.lstrip().startswith(('#','%')))
    nav=pd.read_csv(StringIO(text),sep=r'\s+',engine='python',header=None).to_numpy(float)
    midpoint=a.output/'midpoint.nav'
    write_transformed_nav(native,midpoint,transform_nav(nav,spec['baseline_median_m']))
    command=[sys.executable,str(ROOT/'evaluation/evaluate.py'),'--trace',str(a.reference.resolve()),
        '--nav',str(midpoint.resolve()),'--std',str((a.run/'KF_GINS_STD.txt').resolve()),
        '--outdir',str(a.output.resolve()),'--base_time',str(spec['base_time']),'--yaw_truth_mode','enu']
    raise SystemExit(subprocess.run(command).returncode)

if __name__=='__main__':main()
