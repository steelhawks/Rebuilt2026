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
    tools/pull_logs.py --jobs 8               # more parallel transfers
    tools/pull_logs.py --out ~/frclogs --keep-unpaired

Speed
-----
RIO logs are the big ones (80 MB is normal) and the radio is the bottleneck, so
the pull avoids moving bytes it does not need:

* Pi logs come first - they are ~10x smaller, and their session ids decide which
  RIO logs are worth having at all.
* Each RIO log is then identified from a 32 KB **head probe** in one batched ssh
  call, because AdvantageKit writes ``/RealMetadata`` at ``Logger.start()``. Logs
  whose session has no Pi side, or is already laid out under ``<out>``, are never
  downloaded. Previously they were fetched in full and then discarded.
* Transfers run in parallel (``--jobs``) and compressed (``scp -C``); wpilogs
  measure ~2.8x compressible.
* ``--no-fetch`` talks to the network not at all, rather than waiting out an ssh
  timeout per host.

If pulls are still slow, check how many logs have piled up in ``/home/lvuser/logs``
on the RIO - nothing rotates them.

Downloads are cached under ``<out>/.cache`` and skipped if already present at
the same size, so re-running is cheap.

Re-running never duplicates a session. Sessions already laid out under ``<out>``
are recognised by the id in their ``manifest.json``, not by folder name, so
renaming a folder to something meaningful ("1", "practice-field") does not make
the next run write a second copy of it. ``--refresh`` rewrites them in place,
keeping the folder name you chose. A Pi log that covers several sessions is
hardlinked rather than copied, so it costs its bytes once.
"""

import argparse
import base64
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile
from concurrent import futures

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
    """scp one log into the cache unless a copy is already there.

    ``-C`` is the cheap win: wpilogs are ~2.8x compressible (measured on the
    2026-08-10 82 MB RIO log), and over the robot radio the link, not the CPU on
    either end, is the constraint.
    """
    if os.path.exists(dest):
        return dest
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    tmp = "%s.%d.part" % (dest, os.getpid())
    try:
        run([
            "scp", "-q", "-C", "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=%d" % timeout,
            "-o", "StrictHostKeyChecking=accept-new",
            "%s@%s:%s/%s" % (user, host, path, name), tmp,
        ])
    except subprocess.CalledProcessError as e:
        print("    ! %s failed: %s" % (name, e.stderr.strip()), file=sys.stderr)
        if os.path.exists(tmp):
            os.remove(tmp)
        return None
    os.replace(tmp, dest)
    return dest


def fetch_all(jobs, timeout, jobs_parallel):
    """Fetch in parallel. Transfers are link-bound, so overlap them."""
    todo = [j for j in jobs if not os.path.exists(j[4])]
    if not todo:
        return
    total = sum(j[5] for j in todo)
    print("    %d file(s), %.0f MB to transfer (compressed on the wire)"
          % (len(todo), total / 1e6))
    done = [0]
    with futures.ThreadPoolExecutor(max_workers=jobs_parallel) as ex:
        fs = {ex.submit(fetch, j[0], j[1], j[2], j[3], j[4], timeout): j for j in todo}
        for f in futures.as_completed(fs):
            j = fs[f]
            done[0] += 1
            print("      [%d/%d] %s (%.0f MB)"
                  % (done[0], len(todo), j[3], j[5] / 1e6))


HEAD_PROBE_BYTES = 32768


def probe_sessions(user, host, path, names, timeout):
    """{filename: session_id} for RIO logs, without downloading them.

    A RIO log's identity lives in its ``/RealMetadata`` records, which
    AdvantageKit writes at ``Logger.start()`` - the first 32 KB is enough to read
    session id, git sha, branch and robot (verified against the 2026-08-10 logs,
    where a full log is 83 MB, so this is 0.04% of the bytes).

    That matters because sessions with no Pi side are skipped during pairing.
    Downloading an 83 MB RIO log in full only to discard it is most of why a pull
    took so long. One batched ssh call keeps this to a single round trip.

    Returns {} if the probe fails for any reason; the caller then falls back to
    downloading everything, which is merely slow rather than wrong.
    """
    if not names:
        return {}
    script = (
        "for f in %s; do "
        "  [ -f \"$f\" ] || continue; "
        "  echo \"@@@$(basename \"$f\")\"; "
        "  head -c %d \"$f\" | base64; "
        "done"
    ) % (" ".join("%s/%s" % (path, n) for n in sorted(names)), HEAD_PROBE_BYTES)
    try:
        proc = run([
            "ssh", "-o", "BatchMode=yes", "-C",
            "-o", "ConnectTimeout=%d" % timeout,
            "-o", "StrictHostKeyChecking=accept-new",
            "%s@%s" % (user, host), script,
        ])
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print("  ! head probe unavailable (%s); will download in full"
              % getattr(e, "stderr", e), file=sys.stderr)
        return {}

    out, cur, buf = {}, None, []

    def flush():
        if cur is None:
            return
        try:
            blob = base64.b64decode("".join(buf))
        except Exception:
            return
        tmp = tempfile.NamedTemporaryFile(suffix=".wpilog", delete=False)
        try:
            tmp.write(blob)
            tmp.close()
            out[cur] = read_rio(tmp.name).get("session")
        except Exception:
            pass  # truncated head, or a log with no session id
        finally:
            os.unlink(tmp.name)

    for line in proc.stdout.splitlines():
        if line.startswith("@@@"):
            flush()
            cur, buf = line[3:], []
        elif cur is not None:
            buf.append(line)
    flush()
    return out


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


def existing_sessions(out_dir):
    """{session_id: folder} for everything already laid out under ``out_dir``.

    Keyed on the id inside manifest.json, never on the folder name, because the
    folder gets renamed - people label them "1", "2", "practice". Matching on the
    path is what made re-runs duplicate: the script looked for the name it would
    have chosen, did not find it, and copied a second 80 MB copy of a session
    that was already sitting there under a different name.
    """
    found = {}
    for manifest in sorted(glob.glob(os.path.join(out_dir, "*", "manifest.json"))):
        try:
            with open(manifest) as f:
                sid = json.load(f).get("session_id")
        except (OSError, ValueError):
            continue
        if sid:
            found.setdefault(sid, os.path.dirname(manifest))
    return found


def place(src, dest):
    """Put a cached log at ``dest``: hardlink if we can, copy if we cannot.

    One Pi log can cover several RIO sessions and therefore belong in several
    folders. These run to tens of megabytes and the content is immutable once
    written, so linking keeps a single copy on disk however many sessions
    reference it. Falls back to a copy across filesystems.
    """
    if os.path.exists(dest):
        return
    try:
        os.link(src, dest)
    except OSError:
        shutil.copy2(src, dest)


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
    ap.add_argument("--refresh", action="store_true",
                    help="rewrite sessions already laid out (into their existing "
                         "folder, so renames are kept); default is to skip them")
    ap.add_argument("--timeout", type=int, default=10, help="ssh connect timeout")
    ap.add_argument("--jobs", type=int, default=4,
                    help="parallel transfers (default 4)")
    args = ap.parse_args()

    sides = {
        "rio": (args.rio_user, args.rio_host, args.rio_path),
        "pi": (args.pi_user, args.pi_host, args.pi_path),
    }

    listings = {"rio": [], "pi": []}
    if args.no_fetch:
        # Nothing here is going to be used, and an unreachable robot costs a full
        # ssh connect timeout per host before we even start.
        print("==> Skipping remote listing (--no-fetch)")
    else:
        print("==> Listing remote logs")
        with futures.ThreadPoolExecutor(max_workers=len(sides)) as ex:
            fs = {ex.submit(remote_listing, u, h, p, args.timeout): side
                  for side, (u, h, p) in sides.items()}
            for f in futures.as_completed(fs):
                listings[fs[f]] = f.result()
        for side, (user, host, path) in sides.items():
            print("  %-3s %s@%s:%s  %d log(s)"
                  % (side, user, host, path, len(listings[side])))
            if args.list:
                for name, size in sorted(listings[side]):
                    print("        %-40s %8.1f MB" % (name, size / 1e6))
    if args.list:
        return 0

    cache = os.path.join(args.out, ".cache")
    already = existing_sessions(args.out)
    if args.no_fetch:
        print("\n==> Skipping download (--no-fetch), using %s" % cache)
    else:
        if not any(listings.values()):
            print("  ! neither host had logs - is the robot on and are you on its"
                  " network? Falling back to whatever is already cached.")
        print("\n==> Fetching into %s" % cache)

        # Pi logs first, and all of them: they are ~10x smaller than the RIO's,
        # and their session ids are what decide which RIO logs are worth pulling.
        # A Pi log can also span several sessions, so its id list needs the whole
        # file - there is no useful head probe on this side.
        puser, phost, ppath = sides["pi"]
        fetch_all([(puser, phost, ppath, n, os.path.join(cache, "pi", n), s)
                   for n, s in sorted(listings["pi"])], args.timeout, args.jobs)

        wanted = set(already)
        for name in sorted(os.listdir(os.path.join(cache, "pi"))
                           if os.path.isdir(os.path.join(cache, "pi")) else []):
            if not name.endswith(".wpilog"):
                continue
            try:
                wanted |= set(read_pi(os.path.join(cache, "pi", name))["sessions"])
            except Exception:
                continue

        ruser, rhost, rpath = sides["rio"]
        rio_names = [n for n, _ in listings["rio"]]
        rio_sizes = dict(listings["rio"])
        probe = {} if args.keep_unpaired else probe_sessions(
            ruser, rhost, rpath, rio_names, args.timeout)

        if probe:
            skipped = []
            keep = []
            for n in rio_names:
                sid = probe.get(n)
                if sid is None:            # unreadable head - do not guess, fetch it
                    keep.append(n)
                elif sid in already and not args.refresh:
                    skipped.append((n, "already laid out"))
                elif sid in wanted:
                    keep.append(n)
                else:
                    skipped.append((n, "no Pi log for this session"))
            for n, why in skipped:
                print("    -- skipping %s (%.0f MB): %s"
                      % (n, rio_sizes.get(n, 0) / 1e6, why))
            saved = sum(rio_sizes.get(n, 0) for n, _ in skipped)
            if saved:
                print("    saved %.0f MB of transfer via head probe" % (saved / 1e6))
            rio_names = keep

        fetch_all([(ruser, rhost, rpath, n, os.path.join(cache, "rio", n),
                    rio_sizes.get(n, 0)) for n in sorted(rio_names)],
                  args.timeout, args.jobs)

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

    existing = already  # scanned before fetching, to decide what to skip

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

        short = sid[:8]
        if sid in existing and not args.refresh:
            print("  == %s: already at %s, skipping (use --refresh to rewrite)"
                  % (short, existing[sid]))
            continue

        # Two RIO logs cannot share a session id - the id is minted per boot - so
        # that case is corruption and largest-wins is a reasonable guess. It shows
        # up when AdvantageKit writes under a placeholder name and later renames on
        # DS connect, leaving a stub behind. Two Pi logs sharing one legitimately
        # means the Pi service restarted mid-session, and both halves are real
        # data, so keep them all in RIO-clock order. clock_offset_sec is
        # (rio - pi), and within one session the RIO's clock is monotonic, so
        # sorting on it orders the Pi processes by when they started.
        if len(rio) > 1:
            print("  !! %s: %d RIO logs share this session id, using the largest"
                  % (short, len(rio)))
            rio.sort(key=lambda m: os.path.getsize(m["file"]), reverse=True)
        pi.sort(key=lambda m: (m.get("clock_offset_sec") is None,
                               m.get("clock_offset_sec") or 0.0))
        if len(pi) > 1:
            print("  ** %s: Pi service restarted mid-session, keeping %d Pi logs"
                  % (short, len(pi)))

        # Name the folder from the log actually kept, which is only known after
        # the sort above - taking it from the pre-sort entry named one folder
        # "19ffe10bf1f7cac9__37da3f2a" after the discarded placeholder stub.
        stamp = stamp_from_name(rio[0]["name"]) if rio else "unknown"
        # Reuse the existing folder on --refresh so a rename is not undone.
        folder = existing.get(sid) or os.path.join(args.out, "%s__%s" % (stamp, short))
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
        for side, entries in (("rio", rio), ("pi", pi)):
            written_side = []
            for i, src in enumerate(entries):
                suffix = side if i == 0 else "%s%d" % (side, i + 1)
                dest = os.path.join(folder, "%s__%s_%s.wpilog" % (stamp, short, suffix))
                place(src["file"], dest)
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
