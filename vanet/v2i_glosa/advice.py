"""Read one fresh V2I speed ceiling written after a real radio delivery."""
import json
import math
import time


def read_speed_advice(path, actor_id, now=None):
    if path is None:
        return None
    try:
        data = json.loads(path.read_text())
        now = time.monotonic() if now is None else now
        speed = data.get('speed_kmh')
        expiry = data.get('expires_wall')
        if (type(data.get('actor_id')) is int and data['actor_id'] == actor_id
                and type(speed) in (int, float) and math.isfinite(speed) and 0 <= speed <= 120
                and type(expiry) in (int, float) and math.isfinite(expiry)
                and now <= expiry and expiry - now <= 3.0):
            return float(speed)
    except (OSError, ValueError, TypeError, KeyError):
        pass
    return None


def read_display_status(path, actor_id, now=None):
    """Return a fresh packet's display state without changing driving control."""
    if path is None:
        return None
    try:
        data=json.loads(path.read_text())
        now=time.monotonic() if now is None else now
        if (data.get('actor_id')==actor_id and
                isinstance(data.get('expires_wall'),(int,float)) and
                now<=data['expires_wall']<=now+3.):
            return data
    except (OSError,ValueError,TypeError):
        pass
    return None
