"""Convert the recorded contact/IMU prefix to the contact estimator's CSV input."""
import argparse
import csv
from pathlib import Path
from .contact import stream_h5_prefix

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('robot_log',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    records,_=stream_h5_prefix(args.robot_log)
    with args.output.open('x',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['timestamp_ns',*['gyro_'+x for x in 'xyz'],*['accel_'+x for x in 'xyz'],
            *[f'foot_{leg}_{x}' for leg in ['FL','FR','RL','RR'] for x in 'xyz'],
            'contact_mask','add_mask','remove_mask'])
        for row in records:
            writer.writerow([row.timestamp_ns,*row.gyro_body,*row.accel_body,
                *[v for foot in row.foot_position_body_canonical for v in foot],
                row.contact_mask,row.add_mask,row.remove_mask])

if __name__=='__main__':main()
