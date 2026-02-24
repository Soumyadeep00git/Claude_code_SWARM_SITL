"""
Process monitor for swarm SITL stack.
Samples /proc filesystem for CPU and memory usage of all swarm processes,
writes CSV logs, and optionally shows a live curses dashboard.

Usage:
    python -m scripts.process_monitor \
        --pids '{"orchestrator":123,"gcs":456,"drone_1":789}' \
        --output-dir output/ --interval 0.5 --duration 300
"""

import os
import sys
import time
import json
import signal
import curses
import argparse
import logging

log = logging.getLogger(__name__)

CLK_TCK = os.sysconf("SC_CLK_TCK")  # Usually 100 on Linux


# ── /proc readers ─────────────────────────────────────

def read_proc_stat(pid: int) -> dict | None:
    """Parse /proc/{pid}/stat for CPU ticks and current core."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            line = f.read()
        # comm field can contain spaces/parens — find last ')' to skip it
        idx = line.rfind(")")
        fields = line[idx + 2:].split()
        # After comm: state(0) ppid(1) ... utime(11) stime(12) ... processor(36)
        return {
            "utime": int(fields[11]),
            "stime": int(fields[12]),
            "core": int(fields[36]),
        }
    except (FileNotFoundError, ProcessLookupError, IndexError, ValueError,
            PermissionError):
        return None


def read_proc_status(pid: int) -> dict | None:
    """Parse /proc/{pid}/status for memory and thread count."""
    try:
        info = {}
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    info["vm_rss_kb"] = int(line.split()[1])
                elif line.startswith("VmPeak:"):
                    info["vm_peak_kb"] = int(line.split()[1])
                elif line.startswith("Threads:"):
                    info["threads"] = int(line.split()[1])
        return info if info else None
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


def read_system_cpu() -> dict[int, dict] | None:
    """Parse /proc/stat for per-core CPU ticks.

    Returns {core_id: {"user": ..., "nice": ..., "system": ..., "idle": ...}}.
    """
    try:
        cores = {}
        with open("/proc/stat") as f:
            for line in f:
                if line.startswith("cpu") and line[3] != " ":
                    parts = line.split()
                    core_id = int(parts[0][3:])
                    cores[core_id] = {
                        "user": int(parts[1]),
                        "nice": int(parts[2]),
                        "system": int(parts[3]),
                        "idle": int(parts[4]),
                        "iowait": int(parts[5]) if len(parts) > 5 else 0,
                    }
        return cores
    except (FileNotFoundError, PermissionError):
        return None


def find_children(pid: int) -> list[int]:
    """Find child PIDs via /proc/{pid}/task/*/children."""
    children = []
    task_dir = f"/proc/{pid}/task"
    try:
        for tid in os.listdir(task_dir):
            children_file = f"{task_dir}/{tid}/children"
            try:
                with open(children_file) as f:
                    for child_pid in f.read().split():
                        children.append(int(child_pid))
            except (FileNotFoundError, PermissionError, ValueError):
                pass
    except FileNotFoundError:
        pass
    return children


def compute_cpu_pct(prev_ticks: int, curr_ticks: int, elapsed: float) -> float:
    """Compute CPU% from tick delta and wall-clock elapsed seconds."""
    if elapsed <= 0:
        return 0.0
    delta = curr_ticks - prev_ticks
    return (delta / (elapsed * CLK_TCK)) * 100.0


def compute_core_pct(prev: dict, curr: dict) -> float:
    """Compute per-core CPU% from two snapshots of /proc/stat fields."""
    prev_busy = prev["user"] + prev["nice"] + prev["system"]
    curr_busy = curr["user"] + curr["nice"] + curr["system"]
    prev_total = prev_busy + prev["idle"] + prev["iowait"]
    curr_total = curr_busy + curr["idle"] + curr["iowait"]
    delta_total = curr_total - prev_total
    if delta_total <= 0:
        return 0.0
    return ((curr_busy - prev_busy) / delta_total) * 100.0


# ── Monitor class ─────────────────────────────────────

class ProcessMonitor:
    def __init__(self, pid_map: dict[str, int], output_dir: str,
                 interval: float, duration: float):
        self.pid_map = pid_map          # role -> pid
        self.output_dir = output_dir
        self.interval = interval
        self.duration = duration
        self.running = True
        self.start_time = 0.0

        # Previous tick snapshots for delta calculation
        self._prev_ticks: dict[int, int] = {}   # pid -> utime+stime
        self._prev_time: float = 0.0
        self._prev_cores: dict[int, dict] = {}  # core_id -> stat fields

        # Discovered children: {parent_pid: [child_pids]}
        self._children: dict[int, list[int]] = {}
        # Reverse map for labeling: child_pid -> "drone_1_sitl"
        self._child_roles: dict[int, str] = {}

        # Latest sample for curses display
        self._latest: list[dict] = []
        self._core_pcts: dict[int, float] = {}

    def _discover_children(self):
        """Scan for child processes of each tracked PID."""
        for role, pid in list(self.pid_map.items()):
            children = find_children(pid)
            if children:
                self._children[pid] = children
                for i, cpid in enumerate(children):
                    child_role = f"{role}_sitl" if i == 0 else f"{role}_child{i}"
                    self._child_roles[cpid] = child_role

    def _all_pids(self) -> list[tuple[str, int, int | None]]:
        """Return list of (role, pid, parent_pid|None) for all tracked processes."""
        result = []
        for role, pid in sorted(self.pid_map.items()):
            result.append((role, pid, None))
            for cpid in self._children.get(pid, []):
                crole = self._child_roles.get(cpid, f"child_{cpid}")
                result.append((crole, cpid, pid))
        return result

    def _sample(self) -> tuple[list[dict], dict[int, float]]:
        """Take one sample of all processes and core utilization."""
        now = time.time()
        elapsed = now - self._prev_time if self._prev_time > 0 else self.interval
        elapsed_since_start = now - self.start_time

        self._discover_children()
        rows = []
        for role, pid, parent in self._all_pids():
            stat = read_proc_stat(pid)
            status = read_proc_status(pid)
            if stat is None:
                continue

            ticks = stat["utime"] + stat["stime"]
            prev = self._prev_ticks.get(pid, ticks)
            cpu_pct = compute_cpu_pct(prev, ticks, elapsed)
            self._prev_ticks[pid] = ticks

            row = {
                "timestamp": now,
                "elapsed_s": round(elapsed_since_start, 2),
                "role": role,
                "pid": pid,
                "cpu_pct": round(cpu_pct, 1),
                "mem_rss_kb": status.get("vm_rss_kb", 0) if status else 0,
                "mem_peak_kb": status.get("vm_peak_kb", 0) if status else 0,
                "threads": status.get("threads", 0) if status else 0,
                "core": stat["core"],
                "children": len(self._children.get(pid, [])),
            }
            rows.append(row)

        # Per-core CPU
        core_stats = read_system_cpu()
        core_pcts = {}
        if core_stats and self._prev_cores:
            for cid, curr in core_stats.items():
                if cid in self._prev_cores:
                    core_pcts[cid] = round(compute_core_pct(self._prev_cores[cid], curr), 1)
        if core_stats:
            self._prev_cores = core_stats

        self._prev_time = now
        return rows, core_pcts

    def run(self):
        """Main loop: sample, write CSV, optionally show curses dashboard."""
        os.makedirs(self.output_dir, exist_ok=True)
        proc_csv = os.path.join(self.output_dir, "process_monitor.csv")
        core_csv = os.path.join(self.output_dir, "core_utilization.csv")

        with open(proc_csv, "w") as pf, open(core_csv, "w") as cf:
            pf.write("timestamp,elapsed_s,role,pid,cpu_pct,mem_rss_kb,"
                      "mem_peak_kb,threads,core,children\n")
            cf.write("timestamp,elapsed_s,core_id,cpu_pct\n")
            pf.flush()
            cf.flush()

            self.start_time = time.time()
            self._prev_time = self.start_time

            # Initial tick snapshot (no CPU% for first sample)
            self._sample()

            use_curses = sys.stdin.isatty() and sys.stdout.isatty()
            if use_curses:
                try:
                    curses.wrapper(lambda stdscr: self._loop_curses(stdscr, pf, cf))
                except curses.error:
                    self._loop_plain(pf, cf)
            else:
                self._loop_plain(pf, cf)

        log.info("Monitor finished. CSV: %s, %s", proc_csv, core_csv)

    def _loop_plain(self, pf, cf):
        """Plain-text fallback loop (no curses)."""
        log.info("Process monitor running (plain mode, %.1fs interval)", self.interval)
        while self.running and (time.time() - self.start_time) < self.duration:
            time.sleep(self.interval)
            rows, core_pcts = self._sample()
            self._latest = rows
            self._core_pcts = core_pcts
            self._write_csv(pf, cf, rows, core_pcts)

    def _loop_curses(self, stdscr, pf, cf):
        """Curses dashboard loop."""
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(int(self.interval * 1000))

        while self.running and (time.time() - self.start_time) < self.duration:
            rows, core_pcts = self._sample()
            self._latest = rows
            self._core_pcts = core_pcts
            self._write_csv(pf, cf, rows, core_pcts)
            self._draw_curses(stdscr, rows, core_pcts)

            # Check for 'q' to quit
            try:
                ch = stdscr.getch()
                if ch == ord("q"):
                    self.running = False
            except curses.error:
                pass

            time.sleep(self.interval)

    def _write_csv(self, pf, cf, rows: list[dict], core_pcts: dict[int, float]):
        """Append one sample to both CSV files."""
        for r in rows:
            pf.write(f"{r['timestamp']:.3f},{r['elapsed_s']},"
                     f"{r['role']},{r['pid']},{r['cpu_pct']},"
                     f"{r['mem_rss_kb']},{r['mem_peak_kb']},"
                     f"{r['threads']},{r['core']},{r['children']}\n")
        pf.flush()

        now = time.time()
        elapsed = round(now - self.start_time, 2)
        for cid, pct in sorted(core_pcts.items()):
            cf.write(f"{now:.3f},{elapsed},{cid},{pct}\n")
        cf.flush()

    def _draw_curses(self, stdscr, rows: list[dict], core_pcts: dict[int, float]):
        """Render the curses dashboard."""
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        elapsed = time.time() - self.start_time

        def safe_addstr(y, x, text, *args):
            if y < h - 1 and x < w:
                try:
                    stdscr.addnstr(y, x, text, w - x - 1, *args)
                except curses.error:
                    pass

        # Header
        border = "=" * min(w - 1, 60)
        safe_addstr(0, 0, border, curses.A_BOLD)
        title = f"  SWARM PROCESS MONITOR  |  Elapsed: {elapsed:.1f}s  "
        safe_addstr(1, 0, title, curses.A_BOLD)
        safe_addstr(2, 0, border, curses.A_BOLD)

        # Column headers
        hdr = f"  {'Role':<18} {'Core':>4}  {'PID':>7}  {'CPU%':>6}  {'MEM(MB)':>8}  {'Thr':>3}"
        safe_addstr(3, 0, hdr, curses.A_UNDERLINE)

        # Process rows
        y = 4
        for r in rows:
            if y >= h - 4:
                break
            indent = "    \u2514\u2500 " if "_sitl" in r["role"] or "_child" in r["role"] else "  "
            name = r["role"]
            mem_mb = r["mem_rss_kb"] / 1024.0
            line = (f"{indent}{name:<16} {r['core']:>4}  {r['pid']:>7}  "
                    f"{r['cpu_pct']:>5.1f}%  {mem_mb:>7.1f}M  {r['threads']:>3}")
            safe_addstr(y, 0, line)
            y += 1

        # Per-core utilization
        y += 1
        if y < h - 2:
            safe_addstr(y, 0, border)
            y += 1
        if core_pcts and y < h - 1:
            parts = [f"[{cid}]={pct:.0f}%" for cid, pct in sorted(core_pcts.items())]
            # Split into rows of ~8 cores each
            chunk_size = 8
            for i in range(0, len(parts), chunk_size):
                if y >= h - 1:
                    break
                chunk = "  Per-Core CPU:  " if i == 0 else "                 "
                chunk += " ".join(parts[i:i + chunk_size])
                safe_addstr(y, 0, chunk)
                y += 1

        if y < h - 1:
            safe_addstr(y, 0, border)
        safe_addstr(h - 1, 0, "  Press 'q' to quit")
        stdscr.refresh()

    def stop(self):
        self.running = False


# ── Main ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Swarm process monitor")
    parser.add_argument("--pids", required=True,
                        help="JSON dict mapping role names to PIDs")
    parser.add_argument("--output-dir", default="output")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Sample interval in seconds")
    parser.add_argument("--duration", type=float, default=300,
                        help="Max monitoring duration in seconds")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="[MONITOR] %(asctime)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    pid_map = json.loads(args.pids)
    # Convert PID values to int
    pid_map = {role: int(pid) for role, pid in pid_map.items()}

    monitor = ProcessMonitor(pid_map, args.output_dir, args.interval, args.duration)

    def shutdown(signum, frame):
        log.info("Received signal %d, stopping...", signum)
        monitor.stop()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    log.info("Monitoring %d processes (interval=%.1fs, max_duration=%.0fs)",
             len(pid_map), args.interval, args.duration)
    log.info("PIDs: %s", pid_map)

    monitor.run()

    # Generate timeline chart on exit
    log.info("Generating process timeline chart...")
    try:
        from scripts.monitor_visualizer import generate_charts
        generate_charts(args.output_dir)
    except Exception as e:
        log.warning("Chart generation failed: %s", e)


if __name__ == "__main__":
    main()
