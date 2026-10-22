"""Whole-run solution bounds used to classify the manuscript experiments."""
import numpy as np

def classify_navigation(nav):
    nav=np.asarray(nav,float)
    if nav.ndim!=2 or nav.shape[1]!=11 or not len(nav):
        raise ValueError('Expected a nonempty 11-column navigation array')
    finite=np.isfinite(nav).all(axis=1)
    with np.errstate(invalid='ignore',over='ignore'):
        lat,lon=np.deg2rad(nav[:,2]),np.deg2rad(nav[:,3]);height=nav[:,4]
        sl,cl,so,co=np.sin(lat),np.cos(lat),np.sin(lon),np.cos(lon)
        a,e2=6378137.,6.69437999014e-3
        radius=a/np.sqrt(1-e2*sl*sl)
        xyz=np.column_stack(((radius+height)*cl*co,(radius+height)*cl*so,(radius*(1-e2)+height)*sl))
        rotation=np.array([[-sl[0]*co[0],-sl[0]*so[0],cl[0]],[-so[0],co[0],0.],[-cl[0]*co[0],-cl[0]*so[0],-sl[0]]])
        ned=(xyz-xyz[0])@rotation.T
        position=np.hypot.reduce(ned,axis=1)
        speed=np.hypot.reduce(nav[:,5:8],axis=1)
        height_change=np.abs(nav[:,4]-nav[0,4])
    bad=~finite|(np.abs(nav[:,2])>90)|(np.abs(nav[:,3])>180)
    for values,limit in [(position,1e4),(speed,50.),(height_change,1e3)]:
        bad|=~np.isfinite(values)|(values>limit)
    return 'ALGORITHM_FAILURE_DIVERGED' if bad.any() else 'COMPLETED'
