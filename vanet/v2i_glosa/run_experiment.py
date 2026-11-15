"""Run the three-light paired A-driving/V2I comparison on real simulators."""
import argparse
import csv
from contextlib import closing
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import carla
from foundation.cobridge.control import ControlClient
from foundation.sumo.client import SumoClient
from vanet.v2i_glosa.glosa import STOP_LINES, horizontal_speed_kmh, red_safety_cap, speed_advice
from vanet.v2i_glosa.radio import V2IRadio

ROOT=Path(__file__).resolve().parents[2]
MAP=Path(__file__).resolve().parent/'generated'
NET=MAP/'three_lights.net.xml'
XODR=MAP/'three_lights.xodr'
CONFIG=MAP/'three_lights.sumocfg'
PYTHON=Path(sys.executable)
SIGNALS=('j1','j2','j3')
LINK_INDEX=11
SAMPLE_INTERVAL=.05


def launch(label,command,output,env,cwd=ROOT):
    log=(output/(label+'.log')).open('w')
    process=subprocess.Popen(list(map(str,command)),cwd=cwd,env=env,
                             stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    return process,log


def wait_until(test,processes,seconds=35):
    deadline=time.monotonic()+seconds
    next_report=time.monotonic()+30 if seconds>60 else float('inf')
    while time.monotonic()<deadline:
        for label,(process,log) in processes.items():
            if process.poll() is not None:
                log.flush()
                raise RuntimeError(f'{label} exited {process.returncode}: '+Path(log.name).read_text()[-1800:])
        try:
            result=test()
            if result:return result
        except (OSError,ConnectionError,TimeoutError,RuntimeError):
            pass
        if time.monotonic()>=next_report:
            print('[V2I] Waiting for CARLA cold start...',flush=True)
            next_report+=30
        time.sleep(.05)
    raise TimeoutError('Timed out waiting for '+str(test))


def close_processes(processes):
    # Close dependent clients before the shared bridge and SUMO so that each
    # owner can confirm cleanup while its control socket is still available.
    for label,(process,log) in reversed(list(processes.items())):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL)
            process.wait()
        log.close()


def advice_write(path,actor_id,speed,packet=None,age=None,wireless_speed=None):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(actor_id=actor_id,speed_kmh=None if speed is None else max(0.,speed),
                                         packet_sequence=packet,packet_age_s=age,wireless_speed_kmh=wireless_speed,
                                         expires_wall=time.monotonic()+.7)))
    temporary.replace(path)


def signal_snapshot(sumo):
    t=float(sumo.connection.simulation.getTime())
    states=[];details=[]
    for ident,stop in zip(SIGNALS,STOP_LINES):
        all_states=sumo.connection.trafficlight.getRedYellowGreenState(ident)
        state=all_states[LINK_INDEX]
        phase=int(sumo.connection.trafficlight.getPhase(ident))
        remaining=max(0.,float(sumo.connection.trafficlight.getNextSwitch(ident))-t)
        states.append(state)
        details.append(dict(id=ident,stop_x=stop,phase=phase,remaining=remaining))
    return t,states,details


def reset_signal_phases(sumo):
    """Restore the offset=0/20/40 fixed programs at the moment both cars start."""
    for ident,phase,remaining in (('j1',0,27.),('j2',2,17.),('j3',0,7.)):
        sumo.connection.trafficlight.setPhase(ident,phase)
        sumo.connection.trafficlight.setPhaseDuration(ident,remaining)


def sync_lights(world,states):
    actors=list(world.get_actors().filter('traffic.traffic_light*'))
    if len(actors)!=3:raise RuntimeError(f'Expected 3 CARLA lights, found {len(actors)}')
    for actor in actors:
        nearest=min(range(3),key=lambda n:abs(actor.get_location().x-(STOP_LINES[n]+.25)))
        state=states[nearest].lower()
        desired=carla.TrafficLightState.Green if state=='g' else (carla.TrafficLightState.Yellow if state=='y' else carla.TrafficLightState.Red)
        if actor.get_state()!=desired:actor.set_state(desired)


def vehicle_sample(actor):
    location=actor.get_location()
    velocity=actor.get_velocity()
    return (location.x,location.y,location.z,
            horizontal_speed_kmh(velocity),abs(velocity.z)*3.6)


def write_chart(rows,output):
    """Portable SVG chart with no plotting dependency."""
    from html import escape
    labels=[('stops','Stops','count'),('waiting_s','Waiting','s'),
            ('travel_s','Three-light travel','s'),('hard_brakes','Hard brakes','count')]
    groups=('A baseline','V2I assisted')
    colors=('#3170b5','#e39922')
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="960" height="620" viewBox="0 0 960 620">',
           '<rect width="960" height="620" fill="white"/>',
           '<text x="480" y="35" text-anchor="middle" font-size="22" font-family="sans-serif">Three-light V2I comparison</text>']
    for index,(key,title,unit) in enumerate(labels):
        left=35+(index%2)*470
        top=65+(index//2)*270
        values=[sum(float(r[key]) for r in rows if r['group']==group)/sum(r['group']==group for r in rows)
                for group in groups]
        maximum=max(max(values)*1.2,1.)
        parts.append(f'<rect x="{left}" y="{top}" width="445" height="240" fill="#f8fafc" stroke="#d0d7de"/>')
        parts.append(f'<text x="{left+18}" y="{top+30}" font-size="18" font-family="sans-serif">{escape(title)} ({unit})</text>')
        for number,(value,group,color) in enumerate(zip(values,groups,colors)):
            y=top+65+number*80
            width=270*value/maximum
            parts.append(f'<text x="{left+18}" y="{y+16}" font-size="13" font-family="sans-serif">{escape(group)}</text>')
            parts.append(f'<rect x="{left+135}" y="{y}" width="{width:.1f}" height="25" fill="{color}"/>')
            parts.append(f'<text x="{left+145+width:.1f}" y="{y+18}" font-size="14" font-family="sans-serif">{value:.2f}</text>')
    parts.append(f'<text x="480" y="605" text-anchor="middle" font-size="12" font-family="sans-serif" fill="#555">{len(rows)//2} paired runs; raw data in comparison.csv.</text></svg>')
    path=output/'comparison.svg'
    path.write_text(chr(10).join(parts),encoding='utf-8')
    return path


def run_round(output,assignment,seed,gui=False,rviz=False,npc_count=0,timeout_sim=110):
    output.mkdir(parents=True,exist_ok=True)
    processes={}
    env=dict(os.environ)
    env['SUMO_HOME']='/home/wht/sumo-1.22.0'
    env['PATH']='/home/wht/sumo-1.22.0/bin:'+env.get('PATH','')
    env['AUTODRIVE_PYTHON']=str(PYTHON)
    client=carla.Client('127.0.0.1',2000);client.set_timeout(5)
    own_carla=False
    collision_sensors=[]
    try:
        try:client.get_world()
        except RuntimeError:
            print('[V2I] Starting CARLA; first WSL cold launch may take several minutes.',flush=True)
            processes['carla']=launch('carla',['script','-q','-e','-c','/home/wht/autodrive-a-setup/run_carla_local.sh','/dev/null'],output,
                {key:env[key] for key in ('HOME','USER','DISPLAY','WAYLAND_DISPLAY','XDG_RUNTIME_DIR','XDG_DATA_DIRS') if key in env} | {'PATH':'/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'},
                cwd='/home/wht/carla-0.9.16')
            own_carla=True
            # Unreal loads its first world slowly in WSL. Avoid RPC probes during that boot.
            for elapsed in range(150):
                if processes['carla'][0].poll() is not None:
                    raise RuntimeError('CARLA exited during cold start')
                if elapsed and elapsed % 30 == 0:
                    print(f'[V2I] CARLA booting ({elapsed}s)...', flush=True)
                time.sleep(1)
            def reconnect_carla():
                nonlocal client
                client=carla.Client('127.0.0.1',2000)
                client.set_timeout(10)
                return client.get_world()
            wait_until(reconnect_carla,processes,seconds=240)
        client.set_timeout(35)
        world=client.generate_opendrive_world(XODR.read_text())
        if len(world.get_actors().filter('traffic.traffic_light*'))!=3:
            raise RuntimeError('CARLA map does not have three lights')
        processes['sumo']=launch('sumo',[PYTHON,'foundation/sumo/init/sumo_init.py',CONFIG,
                                          '--seed',str(seed)]
                                 +([] if gui else ['--headless']),output,env)
        def sumo_ready():
            with closing(SumoClient(timeout=2)) as probe:return probe.status()
        wait_until(sumo_ready,processes)
        processes['bridge']=launch('bridge',[PYTHON,'foundation/cobridge/cobridge.py','--v2i-external-lights'],output,env)
        def bridge_ready():
            with closing(ControlClient(timeout=2)) as probe:
                status=probe.command('status')
                return status if status['carla_connected'] and status['sumo_connected'] else None
        wait_until(bridge_ready,processes)
        comm_lane=assignment
        baseline_lane=1-assignment
        ys=(-30.2,-33.4)
        advice_paths=[output/'comm_advice.json',output/'baseline_safety.json']
        for n,(lane,path) in enumerate(((comm_lane,advice_paths[0]),(baseline_lane,advice_paths[1]))):
            args=[PYTHON,'modules/lincoln/add_lincoln.py','--pose','18',str(ys[lane]),'1.2','0',
                  '--sensors','none','--headless','--tm-seed',str(seed),'--v2i-advice-file',path]
            if n==1 or not rviz:args.append('--no-rviz')
            processes[f'lincoln{n}']=launch(f'lincoln{n}',args,output,env)
            wait_until(lambda:len(world.get_actors().filter('vehicle.lincoln.mkz_2020'))>=n+1,processes,seconds=45)
        with closing(ControlClient(timeout=8)) as bridge, closing(SumoClient(timeout=8)) as sumo:
            status=wait_until(lambda:([r for r in bridge.command('status')['lincolns'] if r['state']=='synced']
                                      if len([r for r in bridge.command('status')['lincolns'] if r['state']=='synced'])==2 else None),
                              processes,seconds=50)
            records={r['sumo_id']:r for r in status}
            assert set(records)=={'lincoln0','lincoln1'},records
            actors={name:world.get_actor(records[name]['actor_id']) for name in records}
            stable_since=[None]
            def both_settled():
                t=float(sumo.connection.simulation.getTime())
                readings=[vehicle_sample(actor) for actor in actors.values()]
                if all(horizontal<.5 and vertical<.5 for _,_,_,horizontal,vertical in readings):
                    if stable_since[0] is None:stable_since[0]=t
                    return t-stable_since[0]>=.5
                stable_since[0]=None
                return False
            wait_until(both_settled,processes,seconds=35)
            for name,actor in actors.items():
                x,y,_,horizontal,vertical=vehicle_sample(actor)
                if abs(x-18.)>.5 or abs(y-ys[comm_lane if name=='lincoln0' else baseline_lane])>.5 or horizontal>=.5 or vertical>=.5:
                    raise RuntimeError(f'{name} not settled at its start pose')
            metrics={name:dict(last_speed=None,last_x=18.,stopped=False,last_stop_x=-1e6,stops=0,waiting_s=0.,hard_brakes=0,hard_history=[],last_hard_at=-1e6,
                               crossings=[None]*3,started=None,finished=None,arrived=False,red_violations=0,collisions=0,
                               first_sample_time=None,initial_x=None,initial_y=None,initial_speed_kmh=None,initial_vertical_kmh=None,
                               start_states=None,max_lateral_error_m=0.)
                     for name in records}
            if npc_count:
                processes['npc']=launch('npc',[PYTHON,'modules/npcvehicules/npcvehicules.py',
                    '-n',str(npc_count),'--seed',str(seed)],output,env)
            for name,actor in actors.items():
                blueprint=world.get_blueprint_library().find('sensor.other.collision')
                sensor=world.spawn_actor(blueprint,carla.Transform(),attach_to=actor)
                sensor.listen(lambda event,vehicle=name: metrics[vehicle].__setitem__(
                    'collisions',metrics[vehicle]['collisions']+1))
                collision_sensors.append(sensor)
            samples=[];latest_packet=None;sequence=0;last_sent=-1e6;last_t=None
            world.freeze_all_traffic_lights(True)
            try:
                with V2IRadio(output/'radio',NET,seed=seed) as radio:
                    # Hold both cars while their independent route workers plan.
                    for name,path in (('lincoln0',advice_paths[0]),('lincoln1',advice_paths[1])):
                        advice_write(path,actors[name].id,0.)
                    for name,lane in (('lincoln0',comm_lane),('lincoln1',baseline_lane)):
                        record=records[name]
                        bridge.command('lincoln.navigate',sumo_id=name,
                                       owner_key=f"lincoln:{record['world_id']}:{record['actor_id']}",
                                       network_file=str(NET.resolve()),x=260.,y=-ys[lane])
                    def both_ready():
                        for name,path in (('lincoln0',advice_paths[0]),('lincoln1',advice_paths[1])):
                            advice_write(path,actors[name].id,0.)
                        statuses={r['sumo_id']:r.get('navigation',{}).get('state')
                                  for r in bridge.command('status')['lincolns']}
                        return all(statuses.get(name)=='navigating' for name in actors)
                    wait_until(both_ready,processes,seconds=45)
                    for name,actor in actors.items():
                        x,y,_,horizontal,vertical=vehicle_sample(actor)
                        if abs(x-18.)>.5 or horizontal>=1. or vertical>=1.:
                            raise RuntimeError(f'{name} moved while waiting for paired release')
                    reset_signal_phases(sumo)
                    phase_origin_time=float(sumo.connection.simulation.getTime())
                    for path in advice_paths:path.unlink(missing_ok=True)
                    started=time.monotonic()
                    while True:
                        for label,(process,log) in processes.items():
                            if process.poll() is not None:
                                log.flush();raise RuntimeError(f'{label} exited: '+Path(log.name).read_text()[-1100:])
                        t,states,details=signal_snapshot(sumo)
                        sync_lights(world,states)
                        nodes=[dict(id='rsu0',role='rsu',x=140,y=-20,z=6)]
                        poses={}
                        for name in ('lincoln0','lincoln1'):
                            actor=actors[name]
                            x,y,z,speed,vertical=vehicle_sample(actor)
                            poses[name]=(x,y,z,speed,vertical)
                            nodes.append(dict(id=name,role='lincoln',x=x,y=y,z=.6))
                        payload=None
                        if t-last_sent>=1.-1e-6:
                            sequence+=1;last_sent=t
                            payload=dict(type='v2i.spat',source='rsu0',sequence=sequence,
                                         sumo_time=t,signals=details)
                        delivered=radio.advance(nodes,payload)
                        for delivery in delivered:
                            message=delivery['payload']
                            if message.get('type')=='v2i.spat' and message.get('source')=='rsu0':
                                if latest_packet is None or message['sequence']>latest_packet['sequence']:
                                    latest_packet=message
                        for name,path in (('lincoln0',advice_paths[0]),('lincoln1',advice_paths[1])):
                            x,y,z,speed,vertical=poses[name]
                            safety=red_safety_cap(states,x)
                            assist=(speed_advice(latest_packet,t,x,speed) if name=='lincoln0' else None)
                            cap=min(v for v in (safety,assist) if v is not None) if safety is not None or assist is not None else None
                            packet=latest_packet if name=='lincoln0' and latest_packet and 0<=t-latest_packet['sumo_time']<=2. else None
                            advice_write(path,actors[name].id,cap,
                                         packet=packet['sequence'] if packet else None,
                                         age=t-packet['sumo_time'] if packet else None,
                                         wireless_speed=assist)
                            m=metrics[name]
                            if m['first_sample_time'] is None:
                                m.update(first_sample_time=t,initial_x=x,initial_y=y,
                                         initial_speed_kmh=speed,initial_vertical_kmh=vertical)
                            expected_y=ys[comm_lane if name=='lincoln0' else baseline_lane]
                            m['max_lateral_error_m']=max(m['max_lateral_error_m'],abs(y-expected_y))
                            dt=0 if last_t is None else max(0.,min(.5,t-last_t))
                            if speed>2. and x>18.5 and m['started'] is None:
                                m['started']=t
                                m['start_states']=''.join(states)
                            if m['started'] is not None and m['finished'] is None:
                                standing=speed<1. and x<STOP_LINES[-1]+5.
                                if standing:m['waiting_s']+=dt
                                if standing and not m['stopped'] and x>25. and x-m['last_stop_x']>5.:
                                    m['stops']+=1
                                    m['last_stop_x']=x
                                m['stopped']=standing
                                history=m['hard_history']
                                history.append((t,speed))
                                while len(history)>1 and t-history[1][0]>=.45:
                                    history.pop(0)
                                if (25.<x<STOP_LINES[-1]+5. and t-history[0][0]>=.45
                                        and (history[0][1]-speed)/3.6/(t-history[0][0])>4.
                                        and t-m['last_hard_at']>1.5):
                                    m['hard_brakes']+=1
                                    m['last_hard_at']=t
                            for i,stop in enumerate(STOP_LINES):
                                if m['crossings'][i] is None and m['last_x']<stop<=x:
                                    m['crossings'][i]=t
                                    if states[i].lower()!='g':m['red_violations']+=1
                            if x>=STOP_LINES[-1]+5. and m['finished'] is None:m['finished']=t
                            m['last_speed'],m['last_x']=speed,x
                            samples.append(dict(time=t,vehicle=name,group='V2I assisted' if name=='lincoln0' else 'A baseline',
                                                x=x,y=y,z=z,speed_kmh=speed,vertical_kmh=vertical,
                                                states=''.join(states),safety_cap=safety,
                                                wireless_cap=assist,applied_cap=cap,packet_sequence=latest_packet['sequence'] if name=='lincoln0' and latest_packet else None))
                        last_t=t
                        states_by_name={r['sumo_id']:r.get('navigation',{}).get('state')
                                        for r in bridge.command('status')['lincolns']}
                        for name,m in metrics.items():
                            m['arrived']=states_by_name.get(name)=='arrived'
                            if states_by_name.get(name) in ('error','unreachable','cancelled'):
                                raise RuntimeError(f'{name} navigation {states_by_name[name]}')
                        if all(m['arrived'] for m in metrics.values()):break
                        if t>timeout_sim or time.monotonic()-started>timeout_sim*3:
                            raise TimeoutError('Driving comparison did not finish; see partial samples')
                        time.sleep(SAMPLE_INTERVAL)
                    wireless=dict(sent=radio.sent,received=radio.received,
                                  delivery_rate=radio.received/radio.sent if radio.sent else 0,
                                  mean_delay_ms=1000*sum(radio.delays)/len(radio.delays) if radio.delays else None)
            finally:
                world.freeze_all_traffic_lights(False)
                for path in advice_paths:path.unlink(missing_ok=True)
            rows=[]
            for name,m in metrics.items():
                rows.append(dict(round=output.name,assignment=assignment,group='V2I assisted' if name=='lincoln0' else 'A baseline',
                                 lane=comm_lane if name=='lincoln0' else baseline_lane,
                                 stops=m['stops'],waiting_s=round(m['waiting_s'],2),
                                 travel_s=round(m['finished']-m['started'],2),hard_brakes=m['hard_brakes'],
                                 phase_origin_time=phase_origin_time,first_sample_time=m['first_sample_time'],
                                 initial_x=m['initial_x'],initial_y=m['initial_y'],
                                 initial_speed_kmh=m['initial_speed_kmh'],initial_vertical_kmh=m['initial_vertical_kmh'],
                                 start_time=m['started'],start_states=m['start_states'],
                                 max_lateral_error_m=round(m['max_lateral_error_m'],3),
                                 red_violations=m['red_violations'],collisions=m['collisions'],arrived=m['arrived'],crossings=m['crossings'],
                                 **(wireless if name=='lincoln0' else dict(sent=0,received=0,delivery_rate=None,mean_delay_ms=None))))
            (output/'summary.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False))
            with (output/'samples.csv').open('w',newline='') as file:
                writer=csv.DictWriter(file,fieldnames=samples[0].keys());writer.writeheader();writer.writerows(samples)
            return rows
    finally:
        for sensor in collision_sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except RuntimeError:
                pass
        close_processes(processes)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rounds',type=int,default=1,help='每种车道分配的重复次数；正式对照为 3')
    parser.add_argument('--only-lane',type=int,choices=(0,1),help='只运行一个车道分配，供排错演示')
    parser.add_argument('--gui',action='store_true',help='显示 SUMO GUI')
    parser.add_argument('--rviz',action='store_true',help='通信车显示 RViz')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--npc-count',type=int,default=0,help='可选少量 NPC 演示；正式双车对照保持 0')
    args=parser.parse_args()
    if args.rounds<1:parser.error('--rounds must be positive')
    if not 0<=args.npc_count<=10:parser.error('--npc-count must be 0..10')
    output=args.output or ROOT/'vanet/results'/('v2i_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    output=output.resolve();output.mkdir(parents=True,exist_ok=True)
    rows=[]
    for lane in ((args.only_lane,) if args.only_lane is not None else (0,1)):
        for repeat in range(args.rounds):
            round_output=output/f'lane{lane}_repeat{repeat+1}'
            print(f'[V2I] Running {round_output.name}',flush=True)
            rows.extend(run_round(round_output,lane,seed=575+repeat,gui=args.gui,rviz=args.rviz,npc_count=args.npc_count))
            print(json.dumps(rows[-2:],ensure_ascii=False),flush=True)
    with (output/'comparison.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    chart=write_chart(rows,output)
    print('Results:',output,'Chart:',chart,flush=True)

if __name__=='__main__':main()
