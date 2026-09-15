"""Separate physical-edge continuations from immutable lane travel links.

At a gradual split, the parent's outer edge can continue as the branch's
outer edge; the continuing lane's separator is then the newly appearing
curve. This chart convention preserves the outer collapsed edge. It only
applies to zero-width events supported by source evidence, not abrupt forks.
"""
import json
from mapforge.validate.smoothness import lane_edges_kinematics_at


def source_tip_evidence(lane, rec, role):
    """Compare the source endpoint in the correct s/traffic direction."""
    meta=lane.find("userData[@code='mapforge.provenance/v1']")
    direction=json.loads(meta.get('value','{}')).get('travel_direction') if meta is not None else None
    if rec is None or direction not in ('with_s','against_s'): return False
    source_start=(role=='predecessor') != (direction=='against_s')
    prefix='s' if source_start else 'e'
    return getattr(rec,prefix+'_width_known',False) and getattr(rec,prefix+'_width_mm')==0


def build_graph(road, side, tip_evidence):
    sections = road.findall('lanes/laneSection')
    starts = [float(s.get('s')) for s in sections]
    ends = starts[1:]+[float(road.get('length'))]
    ids = [[l.get('id') for l in sorted(s.findall(side+'/lane'),key=lambda l:abs(int(l.get('id'))))]
           for s in sections]
    occ = {(si,l.get('id')):l for si,s in enumerate(sections) for l in s.findall(side+'/lane')}
    successors, ties, events = {}, {}, []

    def width(si,lid,at_start):
        s = starts[si]+1e-7 if at_start else ends[si]-1e-7
        edges = lane_edges_kinematics_at(road,s,side)
        k = ids[si].index(lid)
        return abs(edges[k+1][0]-edges[k][0])

    for si in range(len(sections)-1):
        old, new = ids[si], ids[si+1]
        mapping = {}
        for lid in old:
            links = occ[si,lid].findall('link/successor')
            if len(links)>1: raise ValueError('ambiguous lane linkage at boundary event')
            if not links: continue  # Missing link is not permission to guess the same ID.
            nxt=links[0].get('id')
            if nxt not in new: raise ValueError('lane successor missing from next section')
            if nxt in mapping.values(): raise ValueError('many-to-one physical boundary association is unsupported by this prototype')
            pred=occ[si+1,nxt].find('link/predecessor')
            if pred is not None and pred.get('id')!=lid:
                raise ValueError('non-reciprocal lane linkage at boundary event')
            mapping[lid]=nxt
        born = [lid for lid in new if lid not in mapping.values()]
        dying = [lid for lid in old if lid not in mapping]
        # Process inside-out for both events. Consecutive
        # zero-width lanes form a collapsed edge group, not extra geometry.
        for lid in dying:
            if width(si,lid,False)>.01 or not tip_evidence(si,lid,'successor'): continue
            k=old.index(lid); inner=old[k-1] if k else '0'
            if inner=='0':
                ties[si,lid,'successor']=(si,'0')
                continue
            if inner not in mapping:
                raise ValueError('unresolved collapsed death group')
            target=mapping.pop(inner); mapping[lid]=target
            ties[si,inner,'successor']=(si,lid)
            events.append({'kind':'death','station_m':ends[si],
                'lane_continuation':[inner,target],'edge_continuation':[lid,target],
                'ending_separator':inner,'collapsed_onto':lid})
        for lid in born:
            if width(si+1,lid,True)>.01 or not tip_evidence(si+1,lid,'predecessor'): continue
            k=new.index(lid); inner=new[k-1] if k else '0'
            if inner=='0':
                ties[si+1,lid,'predecessor']=(si+1,'0')
                continue
            parent=next((a for a,b in mapping.items() if b==inner),None)
            if parent is None: raise ValueError('unresolved collapsed birth group')
            mapping[parent]=lid
            ties[si+1,inner,'predecessor']=(si+1,lid)
            events.append({'kind':'birth','station_m':starts[si+1],
                'lane_continuation':[parent,inner],'edge_continuation':[parent,lid],
                'new_separator':inner,'collapsed_onto':lid})
        successors.update({(si,a):(si+1,b) for a,b in mapping.items()})
    pred=set(successors.values()); chains=[]
    for start in occ:
        if start in pred: continue
        chain=[]; current=start
        while current is not None:
            chain.append(current); current=successors.get(current)
        chains.append(chain)
    flat=[key for chain in chains for key in chain]
    if len(flat)!=len(occ) or len(set(flat))!=len(occ):
        raise ValueError('physical boundary occurrence duplicated or omitted')
    return occ,successors,chains,ties,events
