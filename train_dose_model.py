"""Train the offline volume model on documented artificial labels (NumPy installed)."""
import json
from pathlib import Path
import numpy as np
from dose_model import FEATURES,CENTERS,SCALES

def main():
    rng=np.random.default_rng(20261026)
    raw=rng.uniform([0,0,5,10],[100,100,45,100],size=(1700,4))
    # Educational rule only. No agronomic calibration is claimed.
    y=np.clip(4*(55-raw[:,0])+.45*raw[:,1]+.6*(raw[:,2]-25)-.1*(raw[:,3]-50),0,180)/180
    x=(raw-np.array(CENTERS))/np.array(SCALES)
    train=x[:1200]; target=y[:1200,None]
    params=[rng.normal(0,.25,(4,16)),np.zeros(16),rng.normal(0,.15,(16,1)),np.zeros(1)]
    means=[np.zeros_like(p) for p in params]; squares=[np.zeros_like(p) for p in params]
    for step in range(1,2201):
        w,b,v,c=params
        z=train@w+b; h=np.maximum(z,0); error=(h@v+c-target)*2/len(train)
        dh=(error@v.T)*(z>0)
        grads=[train.T@dh,dh.sum(0),h.T@error,error.sum(0)]
        for i,g in enumerate(grads):
            means[i]=.9*means[i]+.1*g; squares[i]=.999*squares[i]+.001*g*g
            params[i]-=.012*(means[i]/(1-.9**step))/(np.sqrt(squares[i]/(1-.999**step))+1e-8)
    w,b,v,c=params
    def metric(a,z):
        prediction=np.clip((np.maximum(a@w+b,0)@v+c).ravel(),0,1)*180
        actual=z*180
        return dict(n=len(z),mae_ml=float(np.mean(np.abs(prediction-actual))),
                    rmse_ml=float(np.sqrt(np.mean((prediction-actual)**2))))
    report=dict(validation=metric(x[1200:1400],y[1200:1400]),test=metric(x[1400:],y[1400:]),
                training_samples=1200,seed=20261026,
                limitation='Artificial soil/leaf rule, not real plant watering accuracy.')
    model=dict(algorithm='relu_mlp_regression',training_source='synthetic',features=list(FEATURES),
               hidden_weights=w.T.tolist(),hidden_bias=b.tolist(),output_weights=v.ravel().tolist(),
               output_bias=float(c[0]),teaching_rule='clip(4*(55-soil)+.45*wilt+.6*(temp-25)-.1*(humidity-50),0,180)',
               centers=list(CENTERS),scales=list(SCALES))
    folder=Path(__file__).resolve().parent/'ai_model'
    (folder/'dose_model.json').write_text(json.dumps(model,indent=2)+'\n')
    (folder/'dose_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savetxt(folder/'synthetic_dose_data.csv',np.c_[raw,y*180],delimiter=',',
               header=','.join([*FEATURES,'synthetic_target_ml']),comments='')
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
