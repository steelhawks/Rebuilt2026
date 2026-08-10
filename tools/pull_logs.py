#!/usr/bin/env python3
"""Pull the roboRIO and Orange Pi logs for the same run, and pair them up.

Why this exists
---------------
The RIO and the Pi each write their own AdvantageKit log, and nothing about the
files themselves says they belong together:

  * The Pi has no RTC. Its clock is whatever the image shipped with, so its
    filename lies - the 2026-08-07 session produced ``akit_26-06-06_07-14-14``.
  * Both logs start their timestamps at their own zero, so you cannot line up
    events between them by eye.
  * Recovering the pairing afterwards does not work. Cross-correlating the solve
    times shared over the link across the 2026-08-07 pair returned no alignment
    at all, which is how we learned the two files were from different runs.

So the RIO now mints a random session id at boot (``LogSession``), records it as
log metadata, and ships it to the Pi over the link. The Pi logs it every cycle
alongside the RIO's clock. This script reads both sides, groups files by session
id, and lays them out so the pairing is obvious on disk.

Output layout
-------------
    <out>/26-08-07_18-11-27__a3f1c2d4/
        26-08-07_18-11-27__a3f1c2d4_rio.wpilog
        26-08-07_18-11-27__a3f1c2d4_pi.wpilog
        manifest.json

``manifest.json`` carries the session id, both builds' git SHAs, and the clock
offset - add ``clock_offset_sec`` to a Pi timestamp to get RIO time.

Usage
-----
    tools/pull_logs.py --list                 # what is on each host
    tools/pull_logs.py                        # fetch + pair everything new
    tools/pull_logs.py --session a3f1c2d4     # just one session (prefix ok)
    tools/pull_logs.py --out ~/frclogs --keep-unpaired

Downloads are cached under ``<out>/.cache`` and skipped if already present at
the same size, so re-running is cheap.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wpilog import WpiLog  # noqa: E402

# Where each side writes its logs. RIO path matches the WPILOGWriter in
# Robot.java; the Pi path is WorkingDirectory + "logs" from poselink.service.
RIO_DEFAULT = ("admin", "10.26.1.2", "/home/lvuser/logs")
PI_DEFAULT = ("photon", "10.26.1.11", "/home/photon/poselink/logs")

# Where the session id shows up on each side. AdvantageKit puts recordMetadata
# under /RealMetadata and recordOutput under /RealOutputs.
RIO_SESSION_KEY = "/RealMetadata/SessionId"
PI_SESSION_KEY = "/RealOutputs/PoseLinkPi/SessionId"
PI_RIO_CLOCK_KEY = "/RealOutputs/PoseLinkPi/RioTimestamp"

NO_SESSION = "0" * 16


def run(cmd, **kw):
    return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw)


def remote_listing(user, host, path, timeout):
    """[(filename, size_bytes)] for *.wpilog on a host, or [] if unreachable."""
    target = "%s@%s" % (user, host)
    try:
        proc = run([
            "ssh", "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=%d" % timeout,
            "-o", "StrictHostKeyChecking=accept-new",
            target, "ls -l %s/*.wpilog 2>/dev/null || true" % path,
        ])
    except subprocess.CalledProcessError as e:
        print("  ! %s unreachable: %s" % (target, e.stderr.strip()), file=sys.stderr)
        return []
    except FileNotFoundError:
        sys.exit("!! ssh not found on PATH")

    out = []
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            size = int(parts[4])
        except ValueError:
            continue
        out.append((os.path.basename(parts[-1]), size))
    return out


def fetch(user, host, path, name, dest, timeout):
    """scp one log into the cache unless an identical-size copy is there."""
    if os.path.exists(dest):
        return dest
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    tmp = dest + ".part"
    print("    downloading %s" % name)
    try:
        run([
            "scp", "-q", "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=%d" % timeout,
            "-o", "StrictHostKeyChecking=accept-new",
            "%s@%s:%s/%s" % (user, host, path, name), tmp,
        ])
    except subprocess.CalledProcessError as e:
        print("    ! failed: %s" % e.stderr.strip(), file=sys.stderr)
        if os.path.exists(tmp):
            os.remove(tmp)
        return None
    os.replace(tmp, dest)
    return dest


def read_rio(path):
    """Session id + build metadata from a RIO log. Metadata sits at the front."""
    log = WpiLog(path)
    keys = [RIO_SESSION_KEY, "/RealMetadata/GitSHA", "/RealMetadata/GitBranch",
            "/RealMetadata/GitDirty", "/RealMetadata/Robot"]
    got = log.scan(keys, first_only=True)
    session = got[RIO_SESSION_KEY][0][1] if got[RIO_SESSION_KEY] else None
    return {
        "session": session,
        "git_sha": _first(got, "/RealMetadata/GitSHA"),
        "git_branch": _first(got, "/RealMetadata/GitBranch"),
        "git_dirty": _first(got, "/RealMetadata/GitDirty"),
        "robot": _first(got, "/RealMetadata/Robot"),
    }


def read_pi(path):
    """Per-session clock fits from a Pi log. One entry per RIO session it covers.

    A Pi process outlives RIO restarts: it keeps writing a single file while the
    RIO opens a fresh log and mints a new session id on every code start. So one
    Pi log can span several RIO sessions. Reading only the first non-zero id (what
    this used to do) handed the whole file to whichever session happened to be
    running when the Pi came up, and left every later RIO log with no Pi side at
    all - even though its data was sitting in that same file.

    The clock fit has to be per segment for the same reason. It is a median of
    (rio - pi) over the whole file, and FPGA time only survives a code redeploy -
    a RIO *reboot* restarts it at zero. A file spanning one would otherwise get a
    median sitting between two unrelated clock regimes, wrong for both halves,
    and a drift computed straight across the discontinuity.
    """
    log = WpiLog(path)
    got = log.scan([PI_SESSION_KEY, PI_RIO_CLOCK_KEY, "/RealMetadata/GitSHA"])

    # (pi_seconds, session_id) at each change. AdvantageKit only writes changes,
    # so this is already the edge list; the all-zero prologue before the first
    # odometry packet is not a session.
    spans = []
    for ts, value in got[PI_SESSION_KEY]:
        if not isinstance(value, str) or value == NO_SESSION:
            continue
        if spans and spans[-1][1] == value:
            continue
        spans.append((ts / 1e6, value))

    # The Pi logs the RIO's odometry sample time, so (rio - pi) converts this
    # log's timeline onto the RIO's.
    clock = [(ts / 1e6, v) for ts, v in got[PI_RIO_CLOCK_KEY]
             if isinstance(v, float) and v > 0.0]

    sessions = {}
    for i, (start, sid) in enumerate(spans):
        end = spans[i + 1][0] if i + 1 < len(spans) else float("inf")
        sessions[sid] = _clock_fit([p for p in clock if start <= p[0] < end])

    return {"git_sha": _first(got, "/RealMetadata/GitSHA"), "sessions": sessions}


def _clock_fit(pairs):
    """Median offset and drift from one session's (pi_seconds, rio_seconds)."""
    offset = drift = None
    if len(pairs) >= 2:
        offsets = sorted(rio - pi for pi, rio in pairs)
        offset = offsets[len(offsets) // 2]
        span = pairs[-1][0] - pairs[0][0]
        if span > 30.0:
            drift = ((pairs[-1][1] - pairs[-1][0]) - (pairs[0][1] - pairs[0][0])) / span
    return {
        "clock_offset_sec": offset,
        "clock_drift_sec_per_sec": drift,
        "rio_clock_samples": len(pairs),
        # Where in this Pi file the session lives, for trimming in AdvantageScope.
        "pi_span_sec": [pairs[0][0], pairs[-1][0]] if pairs else None,
    }


def _first(got, key):
    return got[key][0][1] if got.get(key) else None


def stamp_from_name(name):
    """'akit_26-08-07_18-11-27.wpilog' -> '26-08-07_18-11-27'."""
    base = os.path.splitext(name)[0]
    if base.startswith("akit_"):
        base = base[len("akit_"):]
    return base


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="matchlogs/paired",
                    help="output directory (default: matchlogs/paired)")
    ap.add_argument("--rio-user", default=RIO_DEFAULT[0])
    ap.add_argument("--rio-host", default=RIO_DEFAULT[1])
    ap.add_argument("--rio-path", default=RIO_DEFAULT[2])
    ap.add_argument("--pi-user", default=PI_DEFAULT[0])
    ap.add_argument("--pi-host", default=PI_DEFAULT[1])
    ap.add_argument("--pi-path", default=PI_DEFAULT[2])
    ap.add_argument("--session", help="only this session id (prefix match)")
    ap.add_argument("--list", action="store_true",
                    help="show what is on each host, download nothing")
    ap.add_argument("--no-fetch", action="store_true",
                    help="pair what is already cached, do not touch the network")
    ap.add_argument("--keep-unpaired", action="store_true",
                    help="also lay out sessions that only have one side")
    ap.add_argument("--timeout", type=int, default=10, help="ssh connect timeout")
    args = ap.parse_args()

    sides = {
        "rio": (args.rio_user, args.rio_host, args.rio_path),
        "pi": (args.pi_user, args.pi_host, args.pi_path),
    }

    print("==> Listing remote logs")
    listings = {}
    for side, (user, host, path) in sides.items():
        files = remote_listing(user, host, path, args.timeout)
        listings[side] = files
        print("  %-3s %s@%s:%s  %d log(s)" % (side, user, host, path, len(files)))
        if args.list:
            for name, size in sorted(files):
                print("        %-40s %8.1f MB" % (name, size / 1e6))
    if args.list:
        return 0

    cache = os.path.join(args.out, ".cache")
    if args.no_fetch:
        print("\n==> Skipping download (--no-fetch), using %s" % cache)
    else:
        if not any(listings.values()):
            print("  ! neither host had logs - is the robot on and are you on its"
                  " network? Falling back to whatever is already cached.")
        print("\n==> Fetching into %s" % cache)
        for side, (user, host, path) in sides.items():
            for name, _size in sorted(listings[side]):
                fetch(user, host, path, name, os.path.join(cache, side, name),
                      args.timeout)

    # Pair over everything in the cache, not just this run's downloads, so a
    # re-run works offline and picks up logs pulled by hand.
    local = {"rio": [], "pi": []}
    for side in ("rio", "pi"):
        side_dir = os.path.join(cache, side)
        if not os.path.isdir(side_dir):
            continue
        for name in sorted(os.listdir(side_dir)):
            if name.endswith(".wpilog"):
                local[side].append((name, os.path.join(side_dir, name)))
    if not any(local.values()):
        sys.exit("!! no logs available locally or remotely")

    print("\n==> Reading session ids")
    info = {"rio": {}, "pi": {}}
    readers = {"rio": read_rio, "pi": read_pi}
    for side in ("rio", "pi"):
        for name, path in local[side]:
            try:
                meta = readers[side](path)
            except Exception as e:  # a truncated log should not sink the run
                print("  ! %s/%s unreadable: %s" % (side, name, e), file=sys.stderr)
                continue
            # A RIO log is exactly one session; a Pi log is one entry per session
            # it covers, so it can pair into several folders.
            if side == "rio":
                found = [meta] if meta.get("session") else []
            else:
                found = []
                for sid, fit in meta["sessions"].items():
                    entry = {"session": sid, "git_sha": meta["git_sha"]}
                    entry.update(fit)
                    found.append(entry)

            print("  %-3s %-40s session=%s"
                  % (side, name, ", ".join(e["session"] for e in found) or "<none>"))
            for entry in found:
                entry["file"] = path
                entry["name"] = name
                info[side].setdefault(entry["session"], []).append(entry)

    sessions = sorted(set(info["rio"]) | set(info["pi"]))
    if args.session:
        sessions = [s for s in sessions if s.startswith(args.session)]
        if not sessions:
            sys.exit("!! no session matching %r" % args.session)

    print("\n==> Pairing")
    written = 0
    for sid in sessions:
        rio = info["rio"].get(sid, [])
        pi = info["pi"].get(sid, [])
        if not (rio and pi) and not args.keep_unpaired:
            print("  -- %s: only %s side, skipping (use --keep-unpaired)"
                  % (sid[:8], "RIO" if rio else "Pi"))
            continue

        stamp = stamp_from_name(rio[0]["name"]) if rio else "unknown"
        short = sid[:8]
        folder = os.path.join(args.out, "%s__%s" % (stamp, short))
        os.makedirs(folder, exist_ok=True)

        manifest = {
            "session_id": sid,
            "stamp": stamp,
            "rio": None,
            "pi": None,
            "clock_offset_sec": None,
            "clock_drift_sec_per_sec": None,
            "note": "add clock_offset_sec to a Pi log timestamp to get RIO time. "
                    "A Pi log can span several RIO sessions, so it may be copied "
                    "into more than one folder - pi.pi_span_sec is the slice of it "
                    "that belongs to THIS session, in Pi-log seconds. "
                    "clock_offset_sec is the primary Pi log's; entries in pi_extra "
                    "carry their own.",
        }
        # Two RIO logs cannot share a session id - the id is minted per boot - so
        # that case is corruption and largest-wins is a reasonable guess. Two Pi
        # logs sharing one legitimately means the Pi service restarted mid-session,
        # and both halves are real data, so keep them all in RIO-clock order.
        # clock_offset_sec is (rio - pi), and within one session the RIO's clock is
        # monotonic, so sorting on it orders the Pi processes by when they started.
        if len(rio) > 1:
            print("  !! %s: %d RIO logs share this session id, using the largest"
                  % (short, len(rio)))
            rio.sort(key=lambda m: os.path.getsize(m["file"]), reverse=True)
        pi.sort(key=lambda m: (m.get("clock_offset_sec") is None,
                               m.get("clock_offset_sec") or 0.0))
        if len(pi) > 1:
            print("  ** %s: Pi service restarted mid-session, keeping %d Pi logs"
                  % (short, len(pi)))

        for side, entries in (("rio", rio), ("pi", pi)):
            written_side = []
            for i, src in enumerate(entries):
                suffix = side if i == 0 else "%s%d" % (side, i + 1)
                dest = os.path.join(folder, "%s__%s_%s.wpilog" % (stamp, short, suffix))
                if not os.path.exists(dest):
                    shutil.copy2(src["file"], dest)
                entry = {k: v for k, v in src.items() if k != "file"}
                entry["file"] = os.path.basename(dest)
                entry["size_bytes"] = os.path.getsize(dest)
                written_side.append(entry)
            if not written_side:
                continue
            manifest[side] = written_side[0]
            if len(written_side) > 1:
                manifest[side + "_extra"] = written_side[1:]
            if side == "pi":
                manifest["clock_offset_sec"] = written_side[0].get("clock_offset_sec")
                manifest["clock_drift_sec_per_sec"] = \
                    written_side[0].get("clock_drift_sec_per_sec")

        with open(os.path.join(folder, "manifest.json"), "w") as f:
            json.dump(manifest, f, indent=2)
            f.write("\n")

        both = "RIO+Pi" if (rio and pi) else ("RIO only" if rio else "Pi only")
        off = manifest["clock_offset_sec"]
        print("  ok %s  %-8s  %s%s" % (
            short, both, folder,
            "" if off is None else "  (Pi->RIO offset %+.3f s)" % off))
        written += 1

    print("\n==> %d session(s) written to %s" % (written, args.out))
    if not written:
        print("    Nothing paired. If the Pi never reported a session id, it is")
        print("    running a build from before session ids existed, or it never")
        print("    received a packet from the RIO.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
