"""Record-level writer/readback test for the approved parent layout.

This emits an in-memory diagnostic XML fragment, NOT an OpenDRIVE document or
map candidate. Full seven-road compilation/transaction remains a separate gate.
"""
import xml.etree.ElementTree as ET
import numpy as np

from mapforge.repair_web.coupled_event_sources import polynomial_shift, polynomial_absmax
from mapforge.repair_web.model import digest


def check_parent_records(contract, coefficients):
    before=digest(contract.reference)
    parent=contract.parent; x=np.asarray(coefficients,float)
    contract.frames(x)  # The same position/birth/upstream constraints.
    layout=contract.written_layout(); fragment=ET.Element('diagnosticRecordsNotOpenDRIVE')
    for row in layout['records']:
        lane=-row['lane']; a=row['start_m']; b=row['end_m']
        c=(parent.power(lane-1,a)-parent.power(lane,a))@x
        ET.SubElement(fragment,'width',section=str(row['section']),lane=str(row['lane']),
            sOffset=format(a-float(parent.road.findall('lanes/laneSection')[row['section']].get('s')),'.17g'),
            start=format(a,'.17g'),end=format(b,'.17g'),**{key:format(float(value),'.17g') for key,value in zip('abcd',c)})
    for a,b in zip(parent.scope.knots,parent.scope.knots[1:]):
        c=parent.power(0,a)@x
        ET.SubElement(fragment,'laneOffset',s=format(a,'.17g'),end=format(b,'.17g'),
            **{key:format(float(value),'.17g') for key,value in zip('abcd',c)})
    decoded=ET.fromstring(ET.tostring(fragment)); worst=0.; rows=[]
    for width in decoded.findall('width'):
        a,b=float(width.get('start')),float(width.get('end')); lane=-int(width.get('lane'))
        offset=max((e for e in decoded.findall('laneOffset') if float(e.get('s'))<=a),key=lambda e:float(e.get('s')))
        decoded_offset=polynomial_shift([float(offset.get(k)) for k in 'abcd'],a-float(offset.get('s')))
        edge=decoded_offset.copy()
        for j in range(1,lane+1):
            matches=[w for w in decoded.findall('width') if w.get('section')==width.get('section') and int(w.get('lane'))==-j
                     and float(w.get('start'))<=a<float(w.get('end'))]
            if len(matches)!=1: raise ValueError('Ambiguous/missing actual stacked width interval')
            w=matches[0]; edge-=polynomial_shift([float(w.get(k)) for k in 'abcd'],a-float(w.get('start')))
        error=polynomial_absmax(edge-parent.power(lane,a)@x,b-a); worst=max(worst,error)
        rows.append(dict(section=int(width.get('section')),lane=-lane,start_m=a,end_m=b,maximum_stack_readback_error_m=error))
    short=[]
    for row in layout['short_semantic_records']:
        a=row['start_m']; edge=-row['lane']; independent=row['boundary_span_start_m']
        current=(parent.power(edge-1,a)-parent.power(edge,a))@x
        previous=(parent.power(edge-1,independent)-parent.power(edge,independent))@x
        delta=current-polynomial_shift(previous,a-independent)
        short.append(dict(section=row['section'],lane=row['lane'],interval_m=[a,row['end_m']],
            same_long_polynomial_error_m=polynomial_absmax(delta,row['length_m']),added_shape_freedom=False))
    return dict(status='RECORD_FRAGMENT_READBACK_NOT_MAP_COMPILE',width_records=len(rows),
        lane_offset_records=len(decoded.findall('laneOffset')),maximum_stack_readback_error_m=worst,
        short_semantic_reexpressions=short,actual_xodr_written=False,xsd_or_esmini_checked=False,
        full_writer_admitted=False,reference_bytes_unchanged=digest(contract.reference)==before)
