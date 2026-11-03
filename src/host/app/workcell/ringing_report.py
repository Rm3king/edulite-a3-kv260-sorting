"""Offline tracking-error and ringing report of one chained wrist session (board result.json.gz).

The commanded reference is rebuilt from the submitted septic segments and aligned to the feedback by the
delay that minimises the tracking error; the ringing figure is the largest spectral line of the tracking
error between 2.5 and 30 Hz after removing the slow lag shape. No hardware access.
"""
import argparse,collections,gzip,json
import numpy as np

DIRECTIONS=np.array([-1,1,-1,1,-1,1,1.])


def reference(segments,t,motor):
    edges=np.cumsum([0]+[s['plan']['duration_s'] for s in segments]);out=np.empty_like(t)
    for i,x in enumerate(t):
        j=max(min(np.searchsorted(edges,x,side='right')-1,len(segments)-1),0);u=np.clip((x-edges[j])/segments[j]['plan']['duration_s'],0,1)
        out[i]=np.polynomial.polynomial.polyval(u,segments[j]['plan']['coefficients_rad'][motor-1])
    return DIRECTIONS[motor-1]*out


def tasks_of(result):
    submissions=result['segment_submissions'];index=0;label='departure';nut=0;tasks=[]
    for event in result['events']:
        name=event['event']
        if name=='CHAIN_NEXT_NUT':nut=event['nut'];label='transfer_empty'
        if name=='ONLINE_GRASP_TASK':label=event['name']
        if name=='PATH_START':
            count=event['segments'];tasks.append((event['time'],f'n{nut}:{label}',submissions[index:index+count]));index+=count
    return tasks


def analyse(result,motors=(2,3,4),minimum_s=1.5):
    series=collections.defaultdict(list)
    for f in result['feedback']:series[f['motor']].append((f['time'],f['position']))
    series={m:np.array(v) for m,v in series.items()};rows=[]
    for t0,name,segments in tasks_of(result):
        total=sum(s['plan']['duration_s'] for s in segments)
        if total<minimum_s or name.endswith(('_close','_open')):continue
        for motor in motors:
            a=series[motor];s=a[(a[:,0]>=t0-.2)&(a[:,0]<=t0+total+.6)];best=None
            for delay in np.arange(0.,.3,.005):
                tr=s[:,0]-t0-delay;keep=(tr>=0)&(tr<=total)
                if keep.sum()<100:continue
                e=s[keep,1]-reference(segments,tr[keep],motor);e=e-e.mean()
                if best is None or e.std()<best[0]:best=(e.std(),delay,e)
            if best is None:continue
            _,delay,e=best;p=(e-np.convolve(e,np.ones(81)/81,'same'))[40:-40]
            spectrum=np.abs(np.fft.rfft(p*np.hanning(len(p))))*2/len(p);f=np.fft.rfftfreq(len(p),.005);band=(f>=2.5)&(f<=30)
            rows.append(dict(task=name,joint=motor,delay_ms=delay*1000,rms_mrad=e.std()*1000,peak_hz=float(f[band][spectrum[band].argmax()]),
                             ring_mrad=float(spectrum[band].max()*1000),max_mrad=float(np.abs(e).max()*1000)))
    return rows


def summary(rows):
    out={}
    for joint in (2,3,4):
        mine=[r for r in rows if r['joint']==joint]
        if mine:out[f'J{joint}']=dict(mean_ring_mrad=float(np.mean([r['ring_mrad'] for r in mine])),max_ring_mrad=float(max(r['ring_mrad'] for r in mine)),
                                      mean_rms_mrad=float(np.mean([r['rms_mrad'] for r in mine])),max_error_mrad=float(max(r['max_mrad'] for r in mine)))
    for name in ('transfer_empty','lift'):
        mine=[r['ring_mrad'] for r in rows if r['joint']==2 and r['task'].endswith(name)]
        if mine:out[f'J2_{name}_ring_mrad']=float(np.mean(mine))
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('result');ap.add_argument('--json',action='store_true');a=ap.parse_args()
    result=json.load(gzip.open(a.result));rows=analyse(result);info=summary(rows)
    spd=result.get('csp_experiment')
    if a.json:print(json.dumps(dict(experiment=spd,summary=info,rows=rows)));return
    print('experiment:',spd or 'none (parameters as found)')
    for r in rows:print('%-26s J%d  delay %3.0f ms  rms %5.2f  ring %5.2f mrad @ %4.1f Hz  max %5.2f'%(r['task'],r['joint'],r['delay_ms'],r['rms_mrad'],r['ring_mrad'],r['peak_hz'],r['max_mrad']))
    print(json.dumps(info,indent=1))


if __name__=='__main__':main()
