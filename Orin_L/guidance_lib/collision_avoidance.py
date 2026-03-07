"""3D collision avoidance filter — additive repulsive velocity blend.

Runs every tick AFTER guidance computes (vn, ve, vd) and BEFORE OutputSafety.
For each neighbor within threat_radius, adds a repulsive push in 3D NED space.

Priority rule: higher drone_id yields more (asymmetric so drones don't mirror).
"""

import math

# Flat-earth meters-per-degree
_M_PER_DEG = 111320.0


def collision_filter(
    vn: float, ve: float, vd: float,
    my_lat: float, my_lon: float, my_alt: float,
    my_id: int,
    neighbors: list,
    threat_radius: float = 10.0,
    max_repulsion: float = 2.0,
) -> tuple:
    """Blend repulsive velocity from all nearby neighbors (3D NED).

    Args:
        vn, ve, vd: Guidance output velocity (NED m/s).
        my_lat, my_lon, my_alt: Own GPS position.
        my_id: Own drone_id (for priority ordering).
        neighbors: List of dicts with {drone_id, lat, lon, alt, ...}.
        threat_radius: Start pushing when closer than this (meters).
        max_repulsion: Max repulsive speed per neighbor (m/s).

    Returns:
        (vn, ve, vd, flags) — modified velocity + info flags.
    """
    push_n = 0.0
    push_e = 0.0
    push_d = 0.0
    nearest = float('inf')
    ca_count = 0

    cos_lat = math.cos(math.radians(my_lat)) if my_lat != 0 else 1.0

    for nbr in neighbors:
        nid = nbr.get('drone_id', 0)
        if nid == my_id:
            continue

        nlat = nbr.get('lat', 0.0)
        nlon = nbr.get('lon', 0.0)
        nalt = nbr.get('alt', 0.0)

        if nlat == 0.0 and nlon == 0.0:
            continue

        # 3D NED vector AWAY from neighbor (positive = away)
        dn = (my_lat - nlat) * _M_PER_DEG
        de = (my_lon - nlon) * _M_PER_DEG * cos_lat
        dd = -(my_alt - nalt)  # NED: positive down

        dist = math.sqrt(dn * dn + de * de + dd * dd)

        if dist >= threat_radius:
            continue

        if dist < nearest:
            nearest = dist

        # Clamp to avoid division by zero
        dist = max(dist, 0.01)

        # Unit vector away from neighbor
        inv_dist = 1.0 / dist
        un = dn * inv_dist
        ue = de * inv_dist
        ud = dd * inv_dist

        # Linear repulsion: full strength at 0m, zero at threat_radius
        strength = max_repulsion * (1.0 - dist / threat_radius)

        # Priority: higher drone_id yields more
        if nid < my_id:
            strength *= 0.7   # I yield more to lower-ID neighbor
        else:
            strength *= 0.3   # Lower-ID neighbor yields more to me

        push_n += strength * un
        push_e += strength * ue
        push_d += strength * ud
        ca_count += 1

    flags = {}
    if ca_count > 0:
        flags['COLLISION_AVOIDANCE'] = True
        flags['ca_nearest'] = round(nearest, 2)
        flags['ca_count'] = ca_count

    return (vn + push_n, ve + push_e, vd + push_d, flags)
