"""Run a repeatable OpenBus rendering benchmark and summarize Chrome trace scopes."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import warnings
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
BUILD_DIR = ROOT / "build-benchmark"
EXECUTABLE = BUILD_DIR / "OpenBus.exe"
CACHE_FILE = BUILD_DIR / "CMakeCache.txt"
DEFAULT_RESOLUTIONS = ("1280x720", "1920x1080", "2560x1440", "3840x2160", "5120x1440")
PHASES = ("baseline", "cameraCycle", "thirdPersonZoom", "drivingControls", "dashboardInteraction")
TRACE_MAX_EVENTS = 1_000_000


@dataclass
class TraceEvent:
    name: str
    category: str
    thread: str
    sequence: int
    start_us: float
    duration_us: float
    end_us: float
    self_us: float
    child_intervals: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class TraceSummary:
    rows: list[dict[str, Any]]
    measurement_ms: float
    frame_count: int
    measured_event_count: int
    total_event_count: int


def parse_resolution(value: str) -> tuple[int, int]:
    match = re.fullmatch(r"(\d+)x(\d+)", value)
    if match is None:
        raise argparse.ArgumentTypeError(f"invalid resolution {value!r}; use WIDTHxHEIGHT")
    width, height = (int(part) for part in match.groups())
    if not (1 <= width <= 16384 and 1 <= height <= 16384):
        raise argparse.ArgumentTypeError("resolution dimensions must be between 1 and 16384")
    return width, height


def summarize_trace(
    trace_path: Path,
    requested_resolution: str,
    framebuffer_resolution: str,
    run_number: int,
) -> TraceSummary:
    with trace_path.open("r", encoding="utf-8-sig") as trace_file:
        trace = json.load(trace_file)

    events: list[TraceEvent] = []
    for sequence, raw_event in enumerate(trace.get("traceEvents", [])):
        if raw_event.get("ph") != "X" or "ts" not in raw_event or "dur" not in raw_event:
            continue
        start_us = float(raw_event["ts"])
        duration_us = float(raw_event["dur"])
        if not (math.isfinite(start_us) and math.isfinite(duration_us)) or duration_us < 0:
            continue
        events.append(
            TraceEvent(
                name=str(raw_event.get("name", "")),
                category=str(raw_event.get("cat", "")),
                thread=str(raw_event.get("tid", "")),
                sequence=sequence,
                start_us=start_us,
                duration_us=duration_us,
                end_us=start_us + duration_us,
                self_us=duration_us,
            )
        )

    measurement = next(
        (
            event
            for event in events
            if event.category == "benchmark" and event.name == "Benchmark.measure"
        ),
        None,
    )
    if measurement is None:
        raise RuntimeError(
            f"No Benchmark.measure scope found in {trace_path}; the trace may be truncated."
        )

    measured_events = [
        event
        for event in events
        if event.start_us >= measurement.start_us and event.end_us <= measurement.end_us
    ]
    events_by_thread: dict[str, list[TraceEvent]] = defaultdict(list)
    for event in measured_events:
        events_by_thread[event.thread].append(event)

    for thread_events in events_by_thread.values():
        ordered = sorted(
            thread_events,
            key=lambda event: (event.start_us, -event.duration_us, event.sequence),
        )
        stack: list[TraceEvent] = []
        for event in ordered:
            parent: TraceEvent | None = None
            while stack:
                candidate = stack[-1]
                if event.start_us >= candidate.end_us or event.end_us > candidate.end_us:
                    stack.pop()
                    continue
                parent = candidate
                break
            if parent is not None:
                parent.child_intervals.append((event.start_us, event.end_us))
            stack.append(event)

    for event in measured_events:
        covered_until = event.start_us
        child_duration = 0.0
        for child_start, child_end in sorted(event.child_intervals):
            clipped_start = max(event.start_us, child_start)
            clipped_end = min(event.end_us, child_end)
            if clipped_end > covered_until:
                child_duration += clipped_end - max(covered_until, clipped_start)
                covered_until = clipped_end
        event.self_us = max(0.0, event.duration_us - child_duration)

    summary_rows: list[dict[str, Any]] = []

    def add_summary(group_events: list[TraceEvent], phase_name: str) -> None:
        grouped: dict[tuple[str, str], list[TraceEvent]] = defaultdict(list)
        for event in group_events:
            grouped[(event.category, event.name)].append(event)
        for (category, name), items in grouped.items():
            wall_us = sum(event.duration_us for event in items)
            self_us = sum(event.self_us for event in items)
            summary_rows.append(
                {
                    "RequestedResolution": requested_resolution,
                    "FramebufferResolution": framebuffer_resolution,
                    "Run": run_number,
                    "Phase": phase_name,
                    "Category": category,
                    "Scope": name,
                    "Count": len(items),
                    "WallMs": round(wall_us / 1000.0, 3),
                    "SelfMs": round(self_us / 1000.0, 3),
                    "AverageWallMs": round(wall_us / len(items) / 1000.0, 4),
                    "MaxWallMs": round(max(event.duration_us for event in items) / 1000.0, 4),
                }
            )

    add_summary(measured_events, "All")
    phase_markers = [
        event
        for event in measured_events
        if event.category == "benchmark" and event.name.startswith("Benchmark.phase.")
    ]
    actual_phases = {event.name.removeprefix("Benchmark.phase.") for event in phase_markers}
    missing_phases = [phase for phase in PHASES if phase not in actual_phases]
    if missing_phases:
        raise RuntimeError(
            "Trace is missing measured phase marker(s): "
            + ", ".join(missing_phases)
            + "; the trace may be truncated."
        )

    for marker in phase_markers:
        phase_name = marker.name.removeprefix("Benchmark.phase.")
        phase_events = [
            event
            for event in measured_events
            if event.start_us >= marker.start_us and event.end_us <= marker.end_us
        ]
        add_summary(phase_events, phase_name)

    frame_count = sum(
        event.category == "frame" and event.name == "main" for event in measured_events
    )
    return TraceSummary(
        rows=summary_rows,
        measurement_ms=round(measurement.duration_us / 1000.0, 3),
        frame_count=frame_count,
        measured_event_count=len(measured_events),
        total_event_count=len(events),
    )


def parse_log_value(pattern: str, log_text: str, default: str = "") -> str:
    match = re.search(pattern, log_text)
    return match.group(1) if match else default


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the OpenBus render-resolution sweep and export per-scope trace metrics."
    )
    parser.add_argument("--resolutions", nargs="+", default=list(DEFAULT_RESOLUTIONS))
    parser.add_argument("--runs", type=int, default=1, help="runs per resolution (1-20)")
    parser.add_argument("--warmup-frames", type=int, default=60)
    parser.add_argument("--phase-frames", type=int, default=60)
    parser.add_argument("--ready-timeout-seconds", type=int, default=180)
    parser.add_argument(
        "--bus-config",
        default="Vehicles/[SP] Studio Polygon 400MMC/E400MMC_ADL_10.9m_Voith_LowHeight.bus",
    )
    parser.add_argument(
        "--model-config",
        default=(
            "Vehicles/[SP] Studio Polygon 400MMC/Model/Configuration Files/"
            "E400MMC_ADL_10.9m_Voith_LowHeight.cfg"
        ),
    )
    parser.add_argument("--output-directory", default="render-benchmark-results")
    return parser


def run_benchmark(args: argparse.Namespace) -> int:
    if not 1 <= args.runs <= 20:
        raise ValueError("--runs must be between 1 and 20")
    if not 0 <= args.warmup_frames <= 10_000:
        raise ValueError("--warmup-frames must be between 0 and 10000")
    if not 1 <= args.phase_frames <= 100_000:
        raise ValueError("--phase-frames must be between 1 and 100000")
    if not 1 <= args.ready_timeout_seconds <= 3600:
        raise ValueError("--ready-timeout-seconds must be between 1 and 3600")

    parsed_resolutions = [(value, *parse_resolution(value)) for value in args.resolutions]
    if not EXECUTABLE.is_file():
        raise RuntimeError(
            f"Missing executable at {EXECUTABLE}. Run benchmark_rendering.bat to configure and build it."
        )
    if not CACHE_FILE.is_file() or re.search(
        r"^OPENBUS_ENABLE_PERF_TRACE:BOOL=ON$",
        CACHE_FILE.read_text(encoding="utf-8", errors="replace"),
        re.MULTILINE,
    ) is None:
        raise RuntimeError(
            "Performance tracing is not enabled in build-benchmark. "
            "Run benchmark_rendering.bat to configure and build with tracing."
        )

    output_path = Path(args.output_directory)
    if not output_path.is_absolute():
        output_path = ROOT / output_path
    output_path.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, Any]] = []
    run_rows: list[dict[str, Any]] = []
    total_runs = len(parsed_resolutions) * args.runs
    run_index = 0

    for resolution, width, height in parsed_resolutions:
        for run_number in range(1, args.runs + 1):
            run_index += 1
            case_name = f"{resolution}_run{run_number:02d}"
            trace_path = output_path / f"trace_{case_name}.json"
            log_path = output_path / f"log_{case_name}.txt"
            print(f"[{run_index}/{total_runs}] {resolution} run {run_number}/{args.runs}")

            environment = os.environ.copy()
            environment.update(
                {
                    "OPENBUS_BENCHMARK": "1",
                    "OPENBUS_BENCHMARK_WIDTH": str(width),
                    "OPENBUS_BENCHMARK_HEIGHT": str(height),
                    "OPENBUS_BENCHMARK_WARMUP_FRAMES": str(args.warmup_frames),
                    "OPENBUS_BENCHMARK_PHASE_FRAMES": str(args.phase_frames),
                    "OPENBUS_BENCHMARK_READY_TIMEOUT": str(args.ready_timeout_seconds),
                    "OPENBUS_BUS_CONFIG": args.bus_config,
                    "OPENBUS_MODEL_CONFIG": args.model_config,
                    "OPENBUS_TRACE": "1",
                    "OPENBUS_TRACE_FILE": str(trace_path),
                    "OPENBUS_TRACE_MAX_EVENTS": str(TRACE_MAX_EVENTS),
                    "OPENBUS_TRACE_MIN_US": "1",
                    "OPENBUS_VSYNC": "off",
                    "OPENBUS_SCRIPT_HZ": "60",
                    "OPENBUS_REFLECTION_TRANSPARENT": "0",
                    "OPENBUS_REFLECTION_INTERVAL": "1",
                    "OPENBUS_REFLECTION_SIZE": "1024",
                    "OPENBUS_MATERIAL_BATCHING": "1",
                }
            )
            environment.pop("OPENBUS_CAPTURE_VIEWS", None)
            environment.pop("OPENBUS_AI_BUS_CONFIG", None)
            environment.pop("OPENBUS_AI_MODEL_CONFIG", None)

            with log_path.open("w", encoding="utf-8") as log_file:
                process = subprocess.run(
                    [str(EXECUTABLE)],
                    cwd=BUILD_DIR,
                    env=environment,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            if process.returncode != 0:
                raise RuntimeError(
                    f"OpenBus benchmark exited with status {process.returncode}; see {log_path}"
                )

            log_text = log_path.read_text(encoding="utf-8", errors="replace")
            if "BENCHMARK_COMPLETE=1" not in log_text:
                raise RuntimeError(f"Benchmark did not complete successfully; see {log_path}")
            actual_resolution = parse_log_value(
                r"BENCHMARK_FRAMEBUFFER=(\d+x\d+)", log_text
            )
            if not actual_resolution:
                raise RuntimeError(f"OpenBus did not report its framebuffer size; see {log_path}")
            if actual_resolution != resolution:
                warnings.warn(
                    f"Requested {resolution} but received framebuffer {actual_resolution}; "
                    "both sizes will be recorded.",
                    stacklevel=1,
                )

            click_target = (
                "found" if "BENCHMARK_CLICK_TARGET=found" in log_text else "none"
            )
            if click_target == "none":
                warnings.warn(
                    f"No clickable dashboard target was found for {resolution} run {run_number}.",
                    stacklevel=1,
                )
            readiness_frames = int(
                parse_log_value(r"BENCHMARK_READINESS_FRAMES=(\d+)", log_text, "0")
            )

            trace_summary = summarize_trace(
                trace_path, resolution, actual_resolution, run_number
            )
            if trace_summary.total_event_count >= TRACE_MAX_EVENTS:
                warnings.warn(
                    f"Trace reached the {TRACE_MAX_EVENTS:,}-event cap: {trace_path}",
                    stacklevel=1,
                )
            summary_rows.extend(trace_summary.rows)
            run_rows.append(
                {
                    "RequestedResolution": resolution,
                    "FramebufferResolution": actual_resolution,
                    "Run": run_number,
                    "ReadinessFrames": readiness_frames,
                    "WarmupFramesPerPhase": args.warmup_frames,
                    "MeasuredFramesPerPhase": args.phase_frames,
                    "MeasuredFrameCount": trace_summary.frame_count,
                    "MeasurementMs": trace_summary.measurement_ms,
                    "MeasuredTraceEventCount": trace_summary.measured_event_count,
                    "TraceEventCount": trace_summary.total_event_count,
                    "ClickTarget": click_target,
                    "TraceFile": str(trace_path),
                    "LogFile": str(log_path),
                }
            )

    summary_fields = [
        "RequestedResolution",
        "FramebufferResolution",
        "Run",
        "Phase",
        "Category",
        "Scope",
        "Count",
        "WallMs",
        "SelfMs",
        "AverageWallMs",
        "MaxWallMs",
    ]
    run_fields = [
        "RequestedResolution",
        "FramebufferResolution",
        "Run",
        "ReadinessFrames",
        "WarmupFramesPerPhase",
        "MeasuredFramesPerPhase",
        "MeasuredFrameCount",
        "MeasurementMs",
        "MeasuredTraceEventCount",
        "TraceEventCount",
        "ClickTarget",
        "TraceFile",
        "LogFile",
    ]
    summary_rows.sort(
        key=lambda row: (
            row["RequestedResolution"],
            row["Run"],
            row["Phase"],
            row["Category"],
            row["Scope"],
        )
    )
    write_csv(output_path / "render_benchmark_scopes.csv", summary_rows, summary_fields)
    write_csv(output_path / "render_benchmark_runs.csv", run_rows, run_fields)
    print(f"Scope summary: {output_path / 'render_benchmark_scopes.csv'}")
    print(f"Run summary:   {output_path / 'render_benchmark_runs.csv'}")
    for row in run_rows:
        print(
            f"{row['RequestedResolution']:>9} actual {row['FramebufferResolution']:>9} "
            f"run {row['Run']:02d}: {row['MeasurementMs']:>9.3f} ms, "
            f"{row['MeasuredFrameCount']:>4} frames, click target {row['ClickTarget']}"
        )
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run_benchmark(args)
    except (
        argparse.ArgumentTypeError,
        OSError,
        RuntimeError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        print(f"benchmark_rendering: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
