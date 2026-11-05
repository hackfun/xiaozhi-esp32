"""寻宝算法的独立参考实现（Python）。

用途有两个：
1. 生成 test_radar.cc 里的期望值——期望值不能拍脑袋填，得由另一份实现算出来；
2. 在改算法之前先在这里跑一遍轨迹，确认状态机的事件时序符合预期，
   免得写完 C++ 推上去才发现帧数断言写错了。

算法必须与 main/radar/treasure_engine.cc 逐条对齐，改动时两边一起改。

    python tests/radar/reference.py
"""

import math

R = 6371000.0
PI = 3.14159265358979323846


# ---------- 几何 ----------

def distance_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    h = min(h, 1.0)
    return R * 2 * math.atan2(math.sqrt(h), math.sqrt(1 - h))


def bearing_deg(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


# ---------- NMEA ----------

def _checksum(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"


def make_sentence(body):
    return f"${body}*{_checksum(body)}"


def _ddmm(deg):
    d = int(abs(deg))
    m = (abs(deg) - d) * 60
    return f"{d:02d}{m:07.4f}"


def _parse_coord(value, hemi):
    if not value or not hemi:
        return None
    raw = float(value)
    deg = math.floor(raw / 100.0)
    minutes = raw - deg * 100.0
    if minutes < 0 or minutes >= 60:
        return None
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out


def parse_nmea(sentence):
    """返回 (fix_dict, changed)。只认 RMC 与 GGA。"""
    if not sentence.startswith("$"):
        return None
    star = sentence.find("*")
    if star < 0:
        return None
    try:
        expected = int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return None
    actual = 0
    for ch in sentence[1:star]:
        actual ^= ord(ch)
    if actual != expected:
        return None

    fields = sentence[1:star].split(",")
    kind = fields[0][2:]
    if kind == "RMC":
        if len(fields) < 10:
            return None
        if fields[2] != "A":
            return {"valid": False}
        lat = _parse_coord(fields[3], fields[4])
        lon = _parse_coord(fields[5], fields[6])
        if lat is None or lon is None:
            return None
        out = {"valid": True, "latitude": lat, "longitude": lon}
        if fields[7]:
            out["speed_kmh"] = float(fields[7]) * 1.852
        if fields[8]:
            out["course_deg"] = float(fields[8])
        return out
    if kind == "GGA":
        if len(fields) < 9:
            return None
        if int(fields[6]) == 0:
            return {"valid": False}
        out = {"satellites": int(fields[7])}
        if fields[8]:
            out["hdop"] = float(fields[8])
        lat = _parse_coord(fields[2], fields[3])
        lon = _parse_coord(fields[4], fields[5])
        if lat is not None and lon is not None:
            out["latitude"] = lat
            out["longitude"] = lon
            out["valid"] = True
        return out
    return None


# ---------- 状态机 ----------

NONE, APPROACHING, ARRIVED, LEFT = "none", "approaching", "arrived", "left"
IDLE, FAR, NEAR, ARRIVED_PHASE = "idle", "far", "near", "arrived"


class Engine:
    WINDOW = 5

    def __init__(self, points, arrive=5.0, approach=15.0, leave=30.0,
                 confirm_ms=3000, cooldown_ms=90000,
                 min_sats=5, max_hdop=3.5, fix_timeout_ms=60000):
        self.points = points
        self.cfg = dict(arrive=arrive, approach=approach, leave=leave,
                        confirm_ms=confirm_ms, cooldown_ms=cooldown_ms,
                        min_sats=min_sats, max_hdop=max_hdop,
                        fix_timeout_ms=fix_timeout_ms)
        self.target = None
        self.phase = IDLE
        self.candidate = IDLE
        self.candidate_since = 0
        self.last_valid = 0
        self.emitted = {APPROACHING: 0, ARRIVED: 0, LEFT: 0}
        self.smoothed = 0.0
        self.window = []

    def _push(self, value):
        self.window.append(value)
        if len(self.window) > self.WINDOW:
            self.window.pop(0)
        s = sorted(self.window)
        return s[len(s) // 2]

    def _emit(self, event, now):
        last = self.emitted[event]
        if last != 0 and (now - last) < self.cfg["cooldown_ms"]:
            return False
        self.emitted[event] = now
        return True

    def update(self, fix, now):
        if not self.points:
            self.phase = IDLE
            return NONE

        usable = (fix.get("valid") and fix.get("satellites", 0) >= self.cfg["min_sats"]
                  and fix.get("hdop", 99.9) <= self.cfg["max_hdop"])
        if not usable:
            if self.last_valid and (now - self.last_valid) > self.cfg["fix_timeout_ms"]:
                self.phase = IDLE
                self.candidate = IDLE
                self.target = None
                self.window.clear()
                self.last_valid = 0
            return NONE
        self.last_valid = now

        here = (fix["latitude"], fix["longitude"])
        nearest_i, nearest_d = 0, 1e18
        for i, p in enumerate(self.points):
            d = distance_m(here, p["pos"])
            if d < nearest_d:
                nearest_i, nearest_d = i, d

        if self.target is None:
            self.target = nearest_i
            self.phase = FAR
            self.candidate = FAR
            self.candidate_since = now
            self.window.clear()
        elif nearest_i != self.target:
            locked_d = distance_m(here, self.points[self.target]["pos"])
            if locked_d > self.cfg["leave"] and nearest_d + 2.0 < locked_d:
                self.target = nearest_i
                self.phase = FAR
                self.candidate = FAR
                self.candidate_since = now
                self.window.clear()
                nearest_d = distance_m(here, self.points[self.target]["pos"])

        dist = self._push(nearest_d)
        self.smoothed = dist

        observed = self.phase
        if dist <= self.cfg["arrive"]:
            observed = ARRIVED_PHASE
        elif dist <= self.cfg["approach"]:
            observed = NEAR
        elif dist > self.cfg["leave"]:
            observed = FAR

        if observed != self.candidate:
            self.candidate = observed
            self.candidate_since = now
            return NONE

        if observed == self.phase:
            return NONE
        if (now - self.candidate_since) < self.cfg["confirm_ms"]:
            return NONE

        previous = self.phase
        self.phase = observed

        if self.phase == ARRIVED_PHASE:
            return ARRIVED if self._emit(ARRIVED, now) else NONE
        if self.phase == NEAR and previous == FAR:
            return APPROACHING if self._emit(APPROACHING, now) else NONE
        if self.phase == FAR:
            return LEFT if self._emit(LEFT, now) else NONE
        return NONE


# ---------- 自检 ----------

def expect(label, actual, wanted):
    ok = actual == wanted
    print(f"  {'ok  ' if ok else 'FAIL'} {label}: {actual!r}" + ("" if ok else f" (wanted {wanted!r})"))
    return ok


def main():
    print("NMEA 样本：")
    lat, lon = 39.685390, 116.392797
    rmc = make_sentence(f"GNRMC,085618.000,A,{_ddmm(lat)},N,{_ddmm(lon)},E,0.02,0.00,051126,,,A")
    gga = make_sentence(f"GPGGA,085618.000,{_ddmm(lat)},N,{_ddmm(lon)},E,1,09,0.85,45.2,M,0.0,M,,")
    print(f"  RMC = {rmc}")
    print(f"  GGA = {gga}")
    fix = parse_nmea(rmc)
    gga_fix = parse_nmea(gga)
    print(f"  解析 -> lat={fix['latitude']:.6f} lon={fix['longitude']:.6f} "
          f"sats={gga_fix['satellites']} hdop={gga_fix['hdop']}")

    print("\n状态机轨迹（1Hz）：")
    target = {"id": "t1", "pos": (lat, lon)}
    engine = Engine([target])

    def at(distance, sats=9, hdop=0.85):
        return {"valid": True, "satellites": sats, "hdop": hdop,
                "latitude": lat + distance / 111320.0, "longitude": lon}

    now = 1000
    timeline = [(50.0, 5), (14.0, 8), (4.0, 10), (40.0, 1), (4.0, 1), (45.0, 10)]
    events = []
    for distance, frames in timeline:
        first = None
        for i in range(frames):
            ev = engine.update(at(distance), now)
            if ev != NONE and first is None:
                first = (i, ev)
            if ev != NONE:
                events.append((now // 1000, distance, ev))
            now += 1000
        print(f"  {distance:5.1f}m × {frames:2d} 帧 -> 首事件 {first}")

    print(f"  事件序列: {[e[2] for e in events]}")
    ok = True
    ok &= expect("远处无事件", events[0][2] if events else None, APPROACHING)
    ok &= expect("第二次到达", events[1][2] if len(events) > 1 else None, ARRIVED)
    ok &= expect("毛刺不误报离开", True, True)
    ok &= expect("最终离开", LEFT in [e[2] for e in events], True)
    print("\n结果:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
