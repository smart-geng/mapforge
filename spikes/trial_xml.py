"""Preserve road identity/order and record real XSD results for isolated trials."""
from pathlib import Path
from lxml import etree

ROOT=Path(__file__).resolve().parents[1]


def replace_road(root,old,new):
    if old.tag!='road' or new.tag!='road' or old.get('id')!=new.get('id'):
        raise ValueError('trial replacement must preserve road identity')
    root[list(root).index(old)]=new


def order_root(root):
    # OpenDRIVE_1.5M.xsd lines 74-97. Keep all additional/unknown elements;
    # unknown content is still subject to XSD, not dropped to obtain PASS.
    rank={tag:i for i,tag in enumerate(('header','road','controller','junction','junctionGroup','station'))}
    before=list(root);root[:]=sorted(before,key=lambda e:rank.get(e.tag,6))
    return list(root)!=before


def write_trial(tree,target):
    changed=order_root(tree.getroot())
    target.parent.mkdir(parents=True,exist_ok=True)
    tree.write(target,encoding='utf-8',xml_declaration=True)
    schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
    passed=schema.validate(etree.parse(str(target)))
    return {'schema':'OpenDRIVE_1.5M.xsd','status':'PASS' if passed else 'FAIL',
            'root_order_repaired':changed,'errors':[str(e) for e in schema.error_log]}
