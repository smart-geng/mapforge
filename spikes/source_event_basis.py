"""Align long basis spans with actual branch events, never each source point."""
import copy

import numpy as np
from scipy.interpolate import BSpline
from spikes.source_contact_fit import SourceFamily


def event_aligned_model(original):
    model=copy.deepcopy(original)
    events=np.unique([float(group[0][1]) for group in model.endpoint_groups if len(group)>2])
    families=[];cursor=0;layouts=[]
    for f in model.families:
        a,b=float(f.knots[0]),float(f.knots[-1])
        inner=events[(events>a+1e-8)&(events<b-1e-8)]
        required=np.r_[a,inner,b]
        if np.min(np.diff(required))<model.min_span-1e-7:
            raise ValueError('source event would violate long-span budget; no short fallback')
        breaks=np.unique(np.concatenate([np.linspace(lo,hi,max(1,int((hi-lo)/model.min_span))+1)
            for lo,hi in zip(required[:-1],required[1:])]))
        knots=np.r_[[a]*(model.degree+1),np.repeat(breaks[1:-1],model.degree-2),[b]*(model.degree+1)]
        size=len(knots)-model.degree-1
        families.append(SourceFamily(f.features,knots,BSpline(knots,np.eye(size),model.degree,extrapolate=False),slice(cursor,cursor+size)))
        cursor+=size;layouts.append(dict(features=f.features,breaks_m=breaks.tolist(),required_internal_events_m=inner.tolist()))
    if cursor>original.nvar:raise ValueError('event placement may not silently increase geometry dimension')
    model.families=families;model.nvar=cursor;model._build_constraints()
    return model,dict(method='same-degree C2 long basis aligned with original branch contacts',
        event_stations_m=events.tolist(),families=layouts,previous_variables=original.nvar,
        variables=cursor,min_span_m=min(float(np.min(np.diff(np.unique(f.knots)))) for f in families),
        source_tolerance_changed=False,source_record_cuts_are_not_events=True,
        direct_xodr_compilation_implemented=False)
