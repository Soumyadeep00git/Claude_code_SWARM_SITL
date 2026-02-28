"""Target computation — leader position + NED offset + feedforward."""

import math

_METERS_PER_DEG_LAT = 111_320.0


class TargetComputer:
    """Computes desired follower position from leader state + offset.

    Separates target computation from guidance so it can be tested
    and tuned independently.
    """

    def compute(self,
                leader_lat: float, leader_lon: float, leader_alt: float,
                leader_vn: float, leader_ve: float,
                offset_n: float, offset_e: float, offset_d: float = 0.0,
                ff_gain: float = 0.0, dt: float = 0.1,
                ) -> tuple[float, float, float]:
        """Return (target_lat, target_lon, target_alt).

        Args:
            leader_lat, leader_lon, leader_alt: Leader GPS position.
            leader_vn, leader_ve: Leader velocity NED (m/s).
            offset_n, offset_e, offset_d: Formation offset in NED meters.
                Negative north = behind leader, positive east = right.
            ff_gain: Feedforward gain (0=reactive, 1=full prediction).
            dt: Control loop period in seconds.
        """
        cos_lat = math.cos(math.radians(leader_lat))
        if cos_lat < 1e-10:
            cos_lat = 1e-10

        # Base target = leader + offset
        target_lat = leader_lat + offset_n / _METERS_PER_DEG_LAT
        target_lon = leader_lon + offset_e / (_METERS_PER_DEG_LAT * cos_lat)
        target_alt = leader_alt - offset_d

        # Feedforward: shift target ahead by leader velocity * dt * gain
        if ff_gain > 0.01 and abs(leader_lat) > 1e-6:
            target_lat += ff_gain * leader_vn * dt / _METERS_PER_DEG_LAT
            target_lon += ff_gain * leader_ve * dt / (_METERS_PER_DEG_LAT * cos_lat)

        return target_lat, target_lon, target_alt
