"""Minimal streaming WPILOG reader.

Only enough of the format to pull metadata and a handful of fields out of a log
without pulling in a full log-analysis dependency. Format reference:
https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc
"""

import struct


def _u(b):
    return int.from_bytes(b, "little", signed=False)


class WpiLog:
    """Lazily-parsed WPILOG file."""

    def __init__(self, path):
        self.path = path
        self.entries = {}  # entry id -> (name, type)
        self.metadata = {}  # from the header's extra-header JSON string

    def _records(self, data):
        """Yield (entry_id, timestamp_us, payload), tracking Start records."""
        if data[:6] != b"WPILOG":
            raise ValueError("%s is not a WPILOG file" % self.path)
        extra_len = _u(data[8:12])
        self.header_extra = data[12:12 + extra_len].decode("utf-8", "replace")
        pos = 12 + extra_len
        n = len(data)
        while pos < n:
            bitfield = data[pos]
            id_len = (bitfield & 0x3) + 1
            size_len = ((bitfield >> 2) & 0x3) + 1
            ts_len = ((bitfield >> 4) & 0x7) + 1
            p = pos + 1
            if p + id_len + size_len + ts_len > n:
                return  # truncated tail (log still being written / cut short)
            entry_id = _u(data[p:p + id_len]); p += id_len
            size = _u(data[p:p + size_len]); p += size_len
            ts = _u(data[p:p + ts_len]); p += ts_len
            payload = data[p:p + size]
            if len(payload) < size:
                return
            pos = p + size
            if entry_id == 0:
                self._control(payload)
            else:
                yield entry_id, ts, payload

    def _control(self, pl):
        if not pl or pl[0] != 0:  # 0 = Start
            return
        p = 1
        entry_id = _u(pl[p:p + 4]); p += 4
        ln = _u(pl[p:p + 4]); p += 4
        name = pl[p:p + ln].decode("utf-8", "replace"); p += ln
        ln = _u(pl[p:p + 4]); p += 4
        typ = pl[p:p + ln].decode("utf-8", "replace"); p += ln
        self.entries[entry_id] = (name, typ)

    def scan(self, wanted, first_only=False, limit_records=None):
        """Collect samples for the named fields.

        Returns {name: [(timestamp_us, value), ...]}. With ``first_only`` it stops
        as soon as every wanted field has one sample, which is all the pairing
        step needs and keeps a 150 MB log to a fraction of a second.
        """
        out = {name: [] for name in wanted}
        with open(self.path, "rb") as f:
            data = f.read()
        seen = 0
        for entry_id, ts, pl in self._records(data):
            seen += 1
            if limit_records and seen > limit_records:
                break
            entry = self.entries.get(entry_id)
            if entry is None:
                continue
            name, typ = entry
            if name not in out:
                continue
            out[name].append((ts, decode(typ, pl)))
            if first_only and all(out[k] for k in out):
                break
        return out

    def duration_us(self):
        last = 0
        with open(self.path, "rb") as f:
            data = f.read()
        for _entry_id, ts, _pl in self._records(data):
            if ts > last:
                last = ts
        return last


def decode(typ, pl):
    try:
        if typ == "double":
            return struct.unpack("<d", pl)[0]
        if typ == "float":
            return struct.unpack("<f", pl)[0]
        if typ == "int64":
            return struct.unpack("<q", pl)[0]
        if typ == "boolean":
            return bool(pl[0])
        if typ in ("string", "json"):
            return pl.decode("utf-8", "replace")
        if typ == "boolean[]":
            return [bool(b) for b in pl]
        if typ == "double[]":
            return list(struct.unpack("<%dd" % (len(pl) // 8), pl))
        if typ == "int64[]":
            return list(struct.unpack("<%dq" % (len(pl) // 8), pl))
    except Exception:
        pass
    return pl
