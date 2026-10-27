"""Small learned watering-volume model for simulated soil and leaf sensors."""
import json
import math
from pathlib import Path

FEATURES=('soil_moisture_pct','leaf_wilt_pct','temperature_c','humidity_pct')
CENTERS=(50.,50.,25.,50.)
SCALES=(50.,50.,20.,50.)
LIMITS=((0,100),(0,100),(5,45),(10,100))


def values(observation):
    row=[]
    for key,(low,high) in zip(FEATURES,LIMITS):
        v=observation.get(key)
        if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError(f'Invalid simulated sensor: {key}')
        row.append(float(v))
    return row


class DoseModel:
    def __init__(self,path=None):
        path=path or Path(__file__).resolve().parent/'ai_model'/'dose_model.json'
        m=json.loads(Path(path).read_text())
        if m['features']!=list(FEATURES) or m['algorithm']!='relu_mlp_regression' or m['training_source']!='synthetic':
            raise ValueError('Invalid dose model')
        self.m=m

    def predict(self,observation):
        row=values(observation)
        x=[(v-c)/s for v,c,s in zip(row,CENTERS,SCALES)]
        hidden=[max(0.,b+sum(w*v for w,v in zip(weights,x)))
                for weights,b in zip(self.m['hidden_weights'],self.m['hidden_bias'])]
        amount=180*(self.m['output_bias']+sum(w*v for w,v in zip(self.m['output_weights'],hidden)))
        amount=round(max(0.,min(180.,amount)),1)
        # Explicit saturation/uncertainty guards; the volume inside the guards is learned.
        if row[0]>=55:
            return dict(action='skip',target_ml=0.,reason='soil_already_wet')
        if amount<5:
            return dict(action='skip',target_ml=0.,reason='predicted_volume_below_minimum')
        return dict(action='water',target_ml=amount,reason='learned_soil_and_leaf_volume')
