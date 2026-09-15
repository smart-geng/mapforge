"""Original-chain bindings for shared junction cuts, not exterior crop rules.

Ordinary source support and dependent movement support overlap deliberately
until the final ownership/readback compiler certifies the whole union. Never
discard an exterior end merely because another direction starts later.
"""
import numpy as np
from mapforge.ops.port_dependencies import PortDependencies, revision
from mapforge.ops.reconstruction_scope import _lane_at, _source_ids
from mapforge.ops.source_cubic_export import source_chains


class SourcePortDomain:
    def __init__(self, model, root):
        if revision(root) != model.scope['base_revision']:
            raise ValueError('source port graph differs from original bound baseline')
        graph = PortDependencies(root)
        graph.validate_junction_table()
        chains = source_chains(model)
        self.bindings = {}; self.dependents = {}
        for port in sorted(graph.incident):
            if port.road != model.road:
                continue
            ids = set(_source_ids(_lane_at(graph, port)))
            matches = [c for c in chains if ids <= set(c['source_ids'])]
            if len(matches) != 1:
                raise ValueError('explicit source port has no unique original topology chain')
            self.bindings[port] = tuple(matches[0]['source_ids'])
            self.dependents[port] = tuple(sorted(graph.incident[port], key=int))
        if not self.bindings:
            raise ValueError('source parent has no original junction port')
        first = self.evaluate(model, [0., 0.], verify_features=False)
        self.features = first['features']

    def evaluate(self, model, offsets, *, verify_features=True):
        offsets = np.asarray(offsets, float)
        if offsets.shape != (2,) or not np.isfinite(offsets).all():
            raise ValueError('two finite original-relative junction cut offsets required')
        chains = {tuple(c['source_ids']): c for c in source_chains(model)}
        if any(ids not in chains for ids in self.bindings.values()):
            raise ValueError('source traffic chain changed under continuous evaluation')
        # Bounds with no incident ports are inventory only, NOT common
        # transverse exterior cuts and not an excuse to drop oblique tails.
        cuts = np.array([min(c['a'] for c in chains.values()), max(c['b'] for c in chains.values())])
        for i, cp in enumerate(('start', 'end')):
            attached = [chains[ids] for p, ids in self.bindings.items() if p.contact == cp]
            if attached:
                cuts[i] = (max(c['a'] for c in attached) if cp == 'start' else min(c['b'] for c in attached)) + offsets[i]
            elif offsets[i] != 0:
                raise ValueError('unused exterior cut is not a mutable junction parameter')
        features = {}; tails = []; stations = {}
        for port, ids in self.bindings.items():
            chain = chains[ids]; s = float(cuts[0 if port.contact == 'start' else 1])
            domains = [d for d in chain['domains'] if d['a']-1e-8 <= s <= d['b']+1e-8]
            pairs = {(model.owner[d['left']], model.owner[d['right']]) for d in domains}
            if len(pairs) != 1:
                raise ValueError('shared mouth left its original physical source chain support')
            d = domains[0]; keys = (d['left'], d['right'])
            # Original side order, not nearest fitted edges or lane-ID sign.
            sign = float(np.interp(s, model.raw[keys[0]][:,0], model.raw[keys[0]][:,1]) -
                         np.interp(s, model.raw[keys[1]][:,0], model.raw[keys[1]][:,1]))
            if abs(sign) < 1e-7:
                raise ValueError('zero-width source cannot be an active junction movement port')
            ordered = keys if sign > 0 else keys[::-1]
            if verify_features:
                if tuple(model.owner[k] for k in ordered) != tuple(model.owner[k] for k in self.features[port]):
                    raise ValueError('moving cut crossed its registered physical boundary family')
                ordered = self.features[port]
            features[port] = ordered; stations[port] = s
            span = [chain['a'], s] if port.contact == 'start' else [s, chain['b']]
            tails.append(dict(port=dict(road=port.road, contact=port.contact, lane=port.lane),
                source_ids=list(ids), full_chain_s=[chain['a'],chain['b']], junction_tail_s=span,
                incident_connectors=list(self.dependents[port]), source_vertices_removed=0,
                coverage_accepted=False))
        return dict(cuts=cuts, stations=stations, features=features, tails=tails,
            original_chain_domains=[dict(source_ids=list(ids), a=c['a'], b=c['b'], direction=c['direction'])
                                    for ids,c in chains.items()],
            external_common_cut_imposed=False, aggregate_ownership_accepted=False,
            final_width_interval_validation_required=True, export_allowed=False)
