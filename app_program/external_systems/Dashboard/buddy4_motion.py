"""Version-1 B4 motion topic: 73 information bits in ten little-endian bytes.

Events are sticky since robot RESET, not a lossless chronological event log.
Positive turn rate is encoder-estimated left yaw. Height stays on EDR16.
"""
MOTION_TOPIC = 'Robopico/sensors/buddy4/motion/v1'
FIELDS = (('sampled_ms',32,0,0xffffffff),('turn_rate_dps',14,-6000,6000),
          ('motion',3,0,5),('flags',2,0,3),('events',6,0,63),
          ('revision',8,0,255),('humps',8,0,255))
CLASSES = ('Unknown','Stationary','Moving','Accelerating','Decelerating','Turning')
EVENTS = ('Stationary','Accelerating','Turning','Climbing hump','Descending hump','Impact candidate')

def decode(payload):
    if len(payload) != 10:
        raise ValueError('B4 motion payload must be ten bytes')
    word = int.from_bytes(payload,'little')
    if word >> 73:
        raise ValueError('B4 motion padding')
    result = {}
    offset = 0
    for name,bits,minimum,maximum in FIELDS:
        value = ((word >> offset) & ((1 << bits)-1)) + minimum
        if value > maximum:
            raise ValueError(name)
        result[name] = value
        offset += bits
    flags = result['flags']
    if (flags & 2 and not flags & 1 or
        not flags & 2 and result['turn_rate_dps'] != 0 or
        not flags & 1 and result['motion'] != 0):
        raise ValueError('Contradictory B4 motion validity')
    return result

def decode_messages(payload):
    v = decode(payload)
    root = 'Robopico/sensors/terrain/'
    return [(root+'motion_class',CLASSES[v['motion']]),
            (root+'turn_rate_dps',v['turn_rate_dps'] if v['flags'] & 2 else None),
            (root+'observed_events',', '.join(name for bit,name in enumerate(EVENTS) if v['events'] & (1 << bit)) or 'None'),
            (root+'event_revision',v['revision']), (root+'hump_count',v['humps']),
            (root+'motion_valid',bool(v['flags'] & 1 and v['motion'] != 0)),
            (root+'impact_latched',bool(v['events'] & 32))]
