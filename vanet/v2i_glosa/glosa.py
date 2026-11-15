"""Deterministic speed advice from a received SPaT packet only."""
import math

PHASE_DURATIONS = (27., 3., 27., 3.)
STOP_LINES = (62.8, 132.8, 202.8)
NOMINAL_KMH = 50.0


def horizontal_speed_kmh(velocity):
    """Road progress excludes falling motion immediately after CARLA spawn."""
    return math.hypot(velocity.x,velocity.y)*3.6


def phase_at(phase, remaining, elapsed):
    phase=int(phase)
    remaining=float(remaining)-max(0.,elapsed)
    while remaining <= 0.:
        phase=(phase+1)%4
        remaining+=PHASE_DURATIONS[phase]
    return phase,remaining


def time_until_green(phase, remaining):
    if phase==0:return 0.
    result=remaining
    for step in range(1,4):
        upcoming=(phase+step)%4
        if upcoming==0:return result
        result+=PHASE_DURATIONS[upcoming]
    raise AssertionError('fixed signal cycle has no green')


def speed_advice(received_packet, current_sim_time, x, current_kmh,
                 max_age=2.0, max_deceleration=2.0, interval=.05):
    """None means fallback to A. Packet is an actual Veins delivery payload."""
    if received_packet is None or received_packet.get('type')!='v2i.spat':return None
    sent=float(received_packet['sumo_time'])
    age=current_sim_time-sent
    if not 0<=age<=max_age:return None
    best=NOMINAL_KMH
    for signal in received_packet['signals']:
        distance=float(signal['stop_x'])-x
        if distance<=3. or distance>145.:continue
        phase,remaining=phase_at(signal['phase'],signal['remaining'],age)
        arrival=distance/(NOMINAL_KMH/3.6)
        arrival_phase,_=phase_at(phase,remaining,arrival)
        if arrival_phase==0:
            continue
        green=time_until_green(phase,remaining)
        if green<=0.:
            green=remaining+sum(PHASE_DURATIONS[1:])
        target=3.6*distance/(green+2.0)
        best=min(best,max(5.,target))
        break
    if best>=NOMINAL_KMH:return None
    return min(NOMINAL_KMH,max(best,current_kmh-max_deceleration*interval*3.6))


def red_safety_cap(states, x):
    """Same current-signal safety layer for both cars; future phases are never read."""
    for stop_x,state in zip(STOP_LINES,states):
        distance=stop_x-x
        if 0<distance<35. and state.lower() not in ('g',):
            return min(NOMINAL_KMH,3.6*math.sqrt(2*2.5*max(0.,distance-4.)))
    return None
