"""Summarize matched case-level errors while retaining all terminal outcomes."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd

METRICS=('horizontal_rmse_m','vertical_rmse_m','roll_rmse_deg','pitch_rmse_deg','yaw_rmse_deg')

def paired_summary(full,control):
    merged=full.merge(control,on='case_id',how='outer',validate='one_to_one',suffixes=('_full','_control'),indicator=True)
    eligible=(merged['_merge']=='both')&(merged.status_full=='COMPLETED')&(merged.status_control=='COMPLETED')
    out={'full_outcomes':{str(k):int(v) for k,v in full.status.value_counts(dropna=False).items()},
         'control_outcomes':{str(k):int(v) for k,v in control.status.value_counts(dropna=False).items()},
         'paired_completed_cases':int(eligible.sum()),'paired_differences':{}}
    for metric in METRICS:
        if metric+'_full' not in merged or metric+'_control' not in merged:continue
        a=merged.loc[eligible,metric+'_full'].to_numpy(float);b=merged.loc[eligible,metric+'_control'].to_numpy(float)
        if not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('Completed cases contain unavailable '+metric)
        delta=a-b
        out['paired_differences'][metric]={'count':len(delta),'median_full_minus_control':float(np.median(delta)) if len(delta) else None,
            'improved_cases':int((delta<0).sum()),'worsened_cases':int((delta>0).sum()),'equal_cases':int((delta==0).sum())}
    return out

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('full',type=Path);p.add_argument('control',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();result=paired_summary(pd.read_csv(a.full),pd.read_csv(a.control));a.output.write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
