"""Plot the recorded physical joint feedback and virtual dose, not desired commands alone."""
import csv
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder=Path(sys.argv[1])
trace=next(folder.glob('*_precision.csv'))
rows=list(csv.DictReader(trace.open()))
fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
for col,waypoint in enumerate(('1','3')):
    relevant=[r for r in rows if r['waypoint']==waypoint and r['phase'] in ('pour','upright')]
    if not relevant: continue
    t0=float(relevant[0]['elapsed_s'])
    t=[float(r['elapsed_s'])-t0 for r in relevant]
    axes[0,col].plot(t,[float(r['angle_deg']) for r in relevant],label='Measured Gazebo joint',lw=2)
    axes[0,col].plot(t,[float(r['command_deg']) for r in relevant],'--',label='Angle command',lw=1.5)
    axes[0,col].axhline(20,color='gray',ls=':',label='Virtual flow threshold')
    axes[0,col].set(title=f'Planter {waypoint}: angle feedback',ylabel='Angle (degrees)')
    axes[1,col].plot(t,[float(r['session_ml']) for r in relevant],label='Delivered (virtual flow integral)',lw=2)
    axes[1,col].axhline(float(relevant[0]['target_ml']),color='#b45309',ls='--',label='AI target volume')
    axes[1,col].set(xlabel='Wall seconds since pouring started',ylabel='Virtual volume (mL)')
    for row in range(2): axes[row,col].grid(alpha=.25); axes[row,col].legend(fontsize=8,loc='best')
fig.suptitle('Closed-loop watering: measured angle and simulated delivered volume',fontsize=14)
fig.savefig(folder/'feedback.png',dpi=160)
print(folder/'feedback.png')
