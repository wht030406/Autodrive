"""Build a reproducible, two-lane, three-signal CARLA/SUMO road."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'generated'
OUT.mkdir(exist_ok=True)

nodes = ET.Element('nodes')
for name, x, y, typ in [('west',0,0,'priority'),('j1',70,0,'traffic_light'),('j2',140,0,'traffic_light'),('j3',210,0,'traffic_light'),('east',280,0,'priority')]:
    ET.SubElement(nodes,'node',id=name,x=str(x),y=str(y),type=typ)
for n,x in enumerate((70,140,210),1):
    for side,y in [('n',35),('s',-35)]:
        ET.SubElement(nodes,'node',id=f'{side}{n}',x=str(x),y=str(y),type='dead_end')
ET.ElementTree(nodes).write(OUT/'three_lights.nod.xml',encoding='utf-8',xml_declaration=True)

edges = ET.Element('edges')
for left,right in zip(('west','j1','j2','j3'),('j1','j2','j3','east')):
    for start,end in ((left,right),(right,left)):
        ET.SubElement(edges,'edge',id=f'{start}_{end}',attrib={'from':start,'to':end,'numLanes':'2','speed':'13.89','priority':'3'})
for n in (1,2,3):
    for side in ('n','s'):
        for start,end in ((side+str(n),'j'+str(n)),('j'+str(n),side+str(n))):
            ET.SubElement(edges,'edge',id=f'{start}_{end}',attrib={'from':start,'to':end,'numLanes':'1','speed':'8.33','priority':'1'})
ET.ElementTree(edges).write(OUT/'three_lights.edg.xml',encoding='utf-8',xml_declaration=True)

cmd=['/home/wht/sumo-1.22.0/bin/netconvert','--node-files',str(OUT/'three_lights.nod.xml'),'--edge-files',str(OUT/'three_lights.edg.xml'),'--output-file',str(OUT/'three_lights.net.xml'),'--opendrive-output',str(OUT/'three_lights.xodr'),'--tls.default-type','static','--tls.cycle.time','60','--no-turnarounds','true']
subprocess.run(cmd,check=True)
# Shift identical 60 s programs so the three junctions present different phases.
net_path = OUT/'three_lights.net.xml'
net_tree = ET.parse(net_path)
# netconvert's OpenDRIVE export uses normalized coordinates. Preserve those
# same coordinates for CoBridge's SUMO-to-CARLA conversion.
location = net_tree.getroot().find('location')
location.set('netOffset', '0.00,0.00')
location.set('origBoundary', location.get('convBoundary'))
for light in net_tree.getroot().findall('tlLogic'):
    light.set('offset', str({'j1': 0, 'j2': 20, 'j3': 40}[light.get('id')]))
net_tree.write(net_path, encoding='utf-8', xml_declaration=True)
config=ET.Element('configuration')
ET.SubElement(ET.SubElement(config,'input'),'net-file',value='three_lights.net.xml')
ET.ElementTree(config).write(OUT/'three_lights.sumocfg',encoding='utf-8',xml_declaration=True)
net=ET.parse(OUT/'three_lights.net.xml').getroot()
lights=net.findall('tlLogic')
assert len(lights)==3, f'expected 3 traffic lights, found {len(lights)}'
print('signals:',[(x.get('id'),x.get('offset'),len(x.findall('phase'))) for x in lights])
# CARLA 0.9.16 cannot build the full 56-road turn-connector mesh reliably.
# Keep all outside roads and the straight links through each junction. This
# preserves three physical crossroads and the exact two-lane SUMO corridor.
def build_carla_junctions():
    tree=ET.parse(OUT/'three_lights.xodr')
    root=tree.getroot()
    root.find('header').set('date','1970-01-01')
    keep=set()
    for road in root.findall('road'):
        length=float(road.get('length'))
        if (road.get('junction')=='-1' or abs(length-14.4)<.01
                or abs(length-20.8)<.01):
            keep.add(road.get('id'))
    for road in list(root.findall('road')):
        if road.get('id') not in keep:
            root.remove(road)
    for junction in root.findall('junction'):
        for connection in list(junction.findall('connection')):
            if connection.get('connectingRoad') not in keep:
                junction.remove(connection)
    for controller in list(root.findall('controller')):
        root.remove(controller)
    for road in root.findall('road'):
        signals=road.find('signals')
        if signals is not None:
            road.remove(signals)
    eastbound=[]
    for road in root.findall('road'):
        if road.get('junction')!='-1':
            continue
        geometry=road.find('planView/geometry')
        if geometry is None or abs(float(geometry.get('hdg')))>1e-4:
            continue
        stop=float(geometry.get('x'))+float(road.get('length'))
        if any(abs(stop-target)<.1 for target in (62.8,132.8,202.8)):
            eastbound.append((stop,road))
    assert len(eastbound)==3, 'expected one incoming eastbound road per junction'
    for number,(stop,road) in enumerate(sorted(eastbound),1):
        signals=ET.SubElement(road,'signals')
        signal=ET.SubElement(signals,'signal',id=f'v2i_j{number}',
            name='Signal_3Light_Post01',s=str(float(road.get('length'))-.2),
            t='-4.8',orientation='+',dynamic='yes',country='OpenDRIVE',
            type='1000001',subtype='-1',zOffset='0',hOffset='0',
            pitch='0',roll='0',height='1.16',width='0.52',value='-1')
        ET.SubElement(signal,'validity',fromLane='-2',toLane='-1')
    assert len(root.findall('road'))==32
    assert len(root.findall('junction'))==3
    assert len(root.findall('.//signal'))==3
    tree.write(OUT/'three_lights.xodr',encoding='utf-8',xml_declaration=True)


build_carla_junctions()
print('OpenDRIVE:',OUT/'three_lights.xodr')
