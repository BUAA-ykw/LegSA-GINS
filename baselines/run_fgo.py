"""Run a factor-graph comparison using prepared observation arrays."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--method',choices=['oisam','tc','gnc'],required=True)
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--ahrs',type=Path)
    p.add_argument('--sequence',choices=['BY2','BY2H','BY2O'],default='BY2')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();config=json.loads(a.config.read_text())
    with np.load(a.input,allow_pickle=False) as z:data=dict(z)
    a.output.mkdir(parents=True,exist_ok=False)
    if a.method=='oisam':
        from .fgo.oisam import run_inputs,STATE_COLUMNS
        from .fgo.inputs import SequenceInput,select_nodes
        if 'imu_intervals' not in data:raise ValueError('OiSAM requires exact IMU start/end intervals')
        inputs=SequenceInput(a.sequence,data['gnss'],data['imu'],
            select_nodes(data['gnss'],float(data['end_s']),config['maximum_gnss_node_offset_s']),{},data['imu_intervals'])
        with (a.output/'states.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(STATE_COLUMNS)
            result=run_inputs(inputs,config,writer.writerow,lambda event:None)
        (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    else:
        if a.method=='tc':
            if a.ahrs is None:p.error('--ahrs is required for the TC comparison')
            from .fgo.wen_tc import solve
            with np.load(a.ahrs,allow_pickle=False) as z:ahrs=dict(z)
            result=solve(data,ahrs,config)
        else:
            from .fgo.gnc import solve
            result=solve(data,config)
        arrays={k:v for k,v in result.items() if isinstance(v,np.ndarray)}
        np.savez_compressed(a.output/'solution.npz',**arrays)
        scalar={k:v for k,v in result.items() if isinstance(v,(str,bool,int,float,type(None)))}
        (a.output/'summary.json').write_text(json.dumps(scalar,indent=2)+'\n')

if __name__=='__main__':main()
