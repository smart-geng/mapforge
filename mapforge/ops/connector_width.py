"""Connector cross-section sizes come from the final linked lanes, not MAP defaults."""
import math


def centred_width_records(start_width, end_width, length):
    """Fixed three-span monotone C2 transition with zero first/second end jets.

    This fixes endpoint width mismatch, not skew end-cap or edge-jet continuity.
    Width and laneOffset=width/2 cancel identically at the driving centerline.
    """
    a, z, length = map(float, (start_width, end_width, length))
    if not all(math.isfinite(x) for x in (a,z,length)) or min(a,z,length)<=0:
        raise ValueError('connector requires positive finite endpoint widths/length')
    delta=z-a
    if abs(delta)<=1e-10:return [(0.,a,0.,0.,0.)]
    # Integral of a nonnegative quadratic bell. No free per-sample controls:
    # all three cubics are determined by endpoint sizes and the whole length.
    return [(0.,a,0.,0.,4.5*delta/length**3),
            (length/3,a+delta/6,1.5*delta/length,4.5*delta/length**2,-9*delta/length**3),
            (2*length/3,a+5*delta/6,1.5*delta/length,-4.5*delta/length**2,4.5*delta/length**3)]


def written_lane_width(road, lane_id, contact):
    """Actual endpoint width in the writer model, including absolute borders."""
    from mapforge.ops.map_to_xodr import _written_jet
    if contact not in ('start','end') or lane_id==0:
        raise ValueError('invalid linked lane endpoint')
    s=0. if contact=='start' else road.length
    section=road.sections[0 if contact=='start' else -1]
    inner=_written_jet(road.offsets,s)[0]
    sign=1 if lane_id>0 else -1
    for lane in sorted(section.left if lane_id>0 else section.right,key=lambda x:abs(x.lane_id)):
        outer=(_written_jet(lane.borders,s-section.s)[0] if lane.borders else
               inner+sign*_written_jet(lane.widths,s-section.s)[0])
        if lane.lane_id==lane_id:
            width=sign*(outer-inner)
            if not math.isfinite(width) or width<=0:
                raise ValueError('linked lane has nonpositive endpoint width')
            return width
        inner=outer
    raise ValueError(f'road {road.road_id}: linked lane {lane_id} absent')
