"""Run a repeatable OpenBus rendering benchmark and summarize Chrome trace scopes."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import subprocess
import sys
import warnings
from bisect import bisect_left
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
TRACE_MAX_EVENTS = 2_000_000
FRAME_SCOPES = (
    "RenderLoop::draw",
    "RenderLoop::draw.reflections",
    "ReflectionRenderer::render",
    "RenderLoop::draw.ground",
    "RenderLoop::draw.model",
    "RenderLoop::updateScripts",
    "RenderLoop::endFrame.swapBuffers",
    "Vehicle::pickClickable",
    "Vehicle::drawBatch.prepareAndSubmit",
)
FRAME_SCOPE_MIN_DURATION_US = {"Vehicle::drawBatch.prepareAndSubmit": 100.0}


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
    frame_rows: list[dict[str, Any]]
    frame_times_ms: list[float]
    measurement_ms: float
    frame_count: int
    phase_repeat_count: int
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
    window_resolution: str = "",
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

    for thread, thread_events in events_by_thread.items():
        ordered = sorted(
            thread_events,
            key=lambda event: (event.start_us, -event.duration_us, event.sequence),
        )
        events_by_thread[thread] = ordered
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
    frame_rows: list[dict[str, Any]] = []
    frame_times_ms: list[float] = []
    framebuffer_matches_requested = framebuffer_resolution == requested_resolution
    framebuffer_width, framebuffer_height = parse_resolution(framebuffer_resolution)
    if window_resolution:
        window_width, window_height = parse_resolution(window_resolution)
        framebuffer_scale_x = round(framebuffer_width / window_width, 4)
        framebuffer_scale_y = round(framebuffer_height / window_height, 4)
    else:
        framebuffer_scale_x = ""
        framebuffer_scale_y = ""

    def add_summary(
        group_events: list[TraceEvent], phase_name: str, phase_repeat: int | str = ""
    ) -> None:
        grouped: dict[tuple[str, str], list[TraceEvent]] = defaultdict(list)
        for event in group_events:
            grouped[(event.category, event.name)].append(event)
        for (category, name), items in grouped.items():
            wall_us = sum(event.duration_us for event in items)
            self_us = sum(event.self_us for event in items)
            summary_rows.append(
                {
                    "RequestedResolution": requested_resolution,
                    "WindowResolution": window_resolution,
                    "FramebufferResolution": framebuffer_resolution,
                    "FramebufferScaleX": framebuffer_scale_x,
                    "FramebufferScaleY": framebuffer_scale_y,
                    "FramebufferMatchesRequested": framebuffer_matches_requested,
                    "Run": run_number,
                    "Phase": phase_name,
                    "PhaseRepeat": phase_repeat,
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

    thread_event_starts = {
        thread: [event.start_us for event in thread_events]
        for thread, thread_events in events_by_thread.items()
    }
    phase_repeat_counts: dict[str, int] = defaultdict(int)
    for marker in sorted(phase_markers, key=lambda event: event.start_us):
        phase_name = marker.name.removeprefix("Benchmark.phase.")
        phase_repeat_counts[phase_name] += 1
        phase_repeat = phase_repeat_counts[phase_name]
        phase_events = [
            event
            for event in measured_events
            if event.start_us >= marker.start_us and event.end_us <= marker.end_us
        ]
        add_summary(phase_events, phase_name, phase_repeat)

        phase_frames = sorted(
            (
                event
                for event in phase_events
                if event.category == "frame" and event.name == "main"
            ),
            key=lambda event: event.start_us,
        )
        for frame_index, frame in enumerate(phase_frames, start=1):
            frame_times_ms.append(frame.duration_us / 1000.0)
            frame_metadata = {
                "RequestedResolution": requested_resolution,
                "WindowResolution": window_resolution,
                "FramebufferResolution": framebuffer_resolution,
                "FramebufferScaleX": framebuffer_scale_x,
                "FramebufferScaleY": framebuffer_scale_y,
                "FramebufferMatchesRequested": framebuffer_matches_requested,
                "Run": run_number,
                "Phase": phase_name,
                "PhaseRepeat": phase_repeat,
                "Frame": frame_index,
                "FrameStartUs": round(frame.start_us, 3),
                "EventStartUs": round(frame.start_us, 3),
            }
            frame_rows.append(
                {
                    **frame_metadata,
                    "Scope": "main",
                    "WallMs": round(frame.duration_us / 1000.0, 4),
                    "SelfMs": round(frame.self_us / 1000.0, 4),
                }
            )

            frame_thread_events = events_by_thread.get(frame.thread, [])
            frame_event_starts = thread_event_starts.get(frame.thread, [])
            first_event = bisect_left(frame_event_starts, frame.start_us)
            after_last_event = bisect_left(frame_event_starts, frame.end_us)
            for event in frame_thread_events[first_event:after_last_event]:
                if (
                    event.name in FRAME_SCOPES
                    and event.end_us <= frame.end_us
                    and event.duration_us >= FRAME_SCOPE_MIN_DURATION_US.get(event.name, 0.0)
                ):
                    frame_rows.append(
                        {
                            **frame_metadata,
                            "Scope": event.name,
                            "EventStartUs": round(event.start_us, 3),
                            "WallMs": round(event.duration_us / 1000.0, 4),
                            "SelfMs": round(event.self_us / 1000.0, 4),
                        }
                    )

    frame_count = sum(
        event.category == "frame" and event.name == "main" for event in measured_events
    )
    repeat_counts = {phase_repeat_counts[phase] for phase in PHASES}
    if len(repeat_counts) != 1:
        raise RuntimeError("Trace phase repeat counts are incomplete or unbalanced.")
    return TraceSummary(
        rows=summary_rows,
        frame_rows=frame_rows,
        frame_times_ms=frame_times_ms,
        measurement_ms=round(measurement.duration_us / 1000.0, 3),
        frame_count=frame_count,
        phase_repeat_count=next(iter(repeat_counts)),
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


def percentile(values: list[float], quantile: float) -> float:
    """Return a linearly interpolated percentile using the (n - 1) rank rule."""
    if not values:
        raise ValueError("cannot calculate a percentile without frame timings")
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be between 0 and 1")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * quantile
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def summarize_frame_times(values: list[float]) -> dict[str, float]:
    if not values:
        raise ValueError("cannot summarize an empty frame-timing set")
    return {
        "AverageFrameTimeMs": round(sum(values) / len(values), 4),
        "P90FrameTimeMs": round(percentile(values, 0.90), 4),
        "P95FrameTimeMs": round(percentile(values, 0.95), 4),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the OpenBus render-resolution sweep and export per-scope trace metrics."
    )
    parser.add_argument("--resolutions", nargs="+", default=list(DEFAULT_RESOLUTIONS))
    parser.add_argument("--runs", type=int, default=5, help="runs per resolution (1-20)")
    parser.add_argument(
        "--seed", type=int, default=0, help="seed for deterministic resolution/run order"
    )
    parser.add_argument(
        "--require-exact-resolution",
        action="store_true",
        help="return a nonzero status if any framebuffer differs from the requested size",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=60,
        help="total warm-up frames distributed across all benchmark phases",
    )
    parser.add_argument(
        "--phase-frames",
        type=int,
        default=120,
        help="fixed-step measured frames per phase visit (120 is two seconds at 60 Hz)",
    )
    parser.add_argument(
        "--phase-repeats",
        type=int,
        default=2,
        help="number of times to measure the full set of phases in each resolution run",
    )
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
    if not 1 <= args.phase_repeats <= 100:
        raise ValueError("--phase-repeats must be between 1 and 100")
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
    frame_rows: list[dict[str, Any]] = []
    frametime_rows: list[dict[str, Any]] = []
    run_rows: list[dict[str, Any]] = []
    run_cases = [
        (resolution, width, height, run_number)
        for resolution, width, height in parsed_resolutions
        for run_number in range(1, args.runs + 1)
    ]
    random.Random(args.seed).shuffle(run_cases)
    total_runs = len(run_cases)
    warmup_min, warmup_remainder = divmod(args.warmup_frames, len(PHASES))
    warmup_max = warmup_min + int(warmup_remainder > 0)

    for run_order, (resolution, width, height, run_number) in enumerate(run_cases, start=1):
        case_name = f"{resolution}_run{run_number:02d}"
        trace_path = output_path / f"trace_{case_name}.json"
        log_path = output_path / f"log_{case_name}.txt"
        print(
            f"[{run_order}/{total_runs}] {resolution} run {run_number}/{args.runs} "
            f"(seed {args.seed}; {args.phase_repeats} phase repeats × "
            f"{args.phase_frames} frames)"
        )

        environment = os.environ.copy()
        environment.update(
            {
                "OPENBUS_BENCHMARK": "1",
                "OPENBUS_BENCHMARK_WIDTH": str(width),
                "OPENBUS_BENCHMARK_HEIGHT": str(height),
                "OPENBUS_BENCHMARK_WARMUP_FRAMES": str(args.warmup_frames),
                "OPENBUS_BENCHMARK_PHASE_FRAMES": str(args.phase_frames),
                "OPENBUS_BENCHMARK_PHASE_REPEATS": str(args.phase_repeats),
                "OPENBUS_BENCHMARK_READY_TIMEOUT": str(args.ready_timeout_seconds),
                "OPENBUS_BENCHMARK_REQUIRE_EXACT_RESOLUTION": "1",
                "OPENBUS_SCRIPT_BACKEND": "native",
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
        actual_resolution = parse_log_value(r"BENCHMARK_FRAMEBUFFER=(\d+x\d+)", log_text)
        if not actual_resolution:
            raise RuntimeError(f"OpenBus did not report its framebuffer size; see {log_path}")
        window_resolution = parse_log_value(r"BENCHMARK_WINDOW=(\d+x\d+)", log_text)
        if not window_resolution:
            raise RuntimeError(
                f"OpenBus did not report its window size; rebuild the benchmark target. "
                f"See {log_path}"
            )
        reported_phase_repeats = int(
            parse_log_value(r"BENCHMARK_PHASE_REPEATS=(\d+)", log_text, "0")
        )
        if reported_phase_repeats != args.phase_repeats:
            raise RuntimeError(
                f"Requested {args.phase_repeats} phase repeats but OpenBus reported "
                f"{reported_phase_repeats}; rebuild the benchmark target. See {log_path}"
            )
        framebuffer_width, framebuffer_height = parse_resolution(actual_resolution)
        actual_window_width, actual_window_height = parse_resolution(window_resolution)
        framebuffer_scale_x = round(framebuffer_width / actual_window_width, 4)
        framebuffer_scale_y = round(framebuffer_height / actual_window_height, 4)
        framebuffer_matches_requested = actual_resolution == resolution
        skipped_reason = parse_log_value(r"BENCHMARK_SKIPPED=([^\r\n]+)", log_text)
        if skipped_reason and skipped_reason != "resolution_mismatch":
            raise RuntimeError(
                f"OpenBus reported an unknown benchmark skip reason {skipped_reason!r}; "
                f"see {log_path}"
            )
        if skipped_reason and framebuffer_matches_requested:
            raise RuntimeError(
                f"OpenBus reported a resolution mismatch but the framebuffer matched "
                f"{resolution}: {log_path}"
            )
        if not framebuffer_matches_requested:
            warnings.warn(
                f"Requested framebuffer {resolution}, window is {window_resolution}, "
                f"but received framebuffer {actual_resolution}; this run is marked "
                "as skipped.",
                stacklevel=1,
            )

            readiness_frames = int(
                parse_log_value(r"BENCHMARK_READINESS_FRAMES=(\d+)", log_text, "0")
            )
            skip_note = "framebuffer mismatch"
            run_rows.append(
                {
                    "RequestedResolution": resolution,
                    "WindowResolution": window_resolution,
                    "FramebufferResolution": actual_resolution,
                    "FramebufferScaleX": framebuffer_scale_x,
                    "FramebufferScaleY": framebuffer_scale_y,
                    "FramebufferMatchesRequested": False,
                    "RunOrder": run_order,
                    "Run": run_number,
                    "Status": "Skipped",
                    "SkipReason": skip_note,
                    "ReadinessFrames": readiness_frames,
                    "WarmupFramesTotal": args.warmup_frames,
                    "WarmupFramesPerPhaseMin": warmup_min,
                    "WarmupFramesPerPhaseMax": warmup_max,
                    "MeasuredFramesPerPhase": args.phase_frames,
                    "PhaseRepeats": args.phase_repeats,
                    "MeasuredFrameCount": "",
                    "AverageFrameTimeMs": "",
                    "P90FrameTimeMs": "",
                    "P95FrameTimeMs": "",
                    "MeasurementMs": "",
                    "MeasuredTraceEventCount": "",
                    "TraceEventCount": "",
                    "ClickTarget": "skipped",
                    "TraceFile": str(trace_path),
                    "LogFile": str(log_path),
                }
            )
            frametime_rows.append(
                {
                    "RequestedResolution": resolution,
                    "WindowResolution": window_resolution,
                    "FramebufferResolution": actual_resolution,
                    "FramebufferScaleX": framebuffer_scale_x,
                    "FramebufferScaleY": framebuffer_scale_y,
                    "FramebufferMatchesRequested": False,
                    "RunOrder": run_order,
                    "Run": run_number,
                    "Status": "Skipped",
                    "SkipReason": skip_note,
                    "FrameCount": 0,
                    "AverageFrameTimeMs": "",
                    "P90FrameTimeMs": "",
                    "P95FrameTimeMs": "",
                    "TraceFile": str(trace_path),
                }
            )
            continue
        if skipped_reason:
            raise RuntimeError(f"OpenBus skipped an exact-resolution run: {log_path}")

        click_target = "found" if "BENCHMARK_CLICK_TARGET=found" in log_text else "none"
        if click_target == "none":
            warnings.warn(
                f"No clickable dashboard target was found for {resolution} run {run_number}.",
                stacklevel=1,
            )
        readiness_frames = int(
            parse_log_value(r"BENCHMARK_READINESS_FRAMES=(\d+)", log_text, "0")
        )

        trace_summary = summarize_trace(
            trace_path, resolution, actual_resolution, run_number, window_resolution
        )
        if trace_summary.phase_repeat_count != args.phase_repeats:
            raise RuntimeError(
                f"Trace contains {trace_summary.phase_repeat_count} phase repeat(s), "
                f"expected {args.phase_repeats}: {trace_path}"
            )
        expected_frame_count = len(PHASES) * args.phase_repeats * args.phase_frames
        if trace_summary.frame_count != expected_frame_count:
            raise RuntimeError(
                f"Trace contains {trace_summary.frame_count} measured frames, expected "
                f"{expected_frame_count}: {trace_path}"
            )
        frame_statistics = summarize_frame_times(trace_summary.frame_times_ms)
        if len(trace_summary.frame_times_ms) != trace_summary.frame_count:
            raise RuntimeError(
                f"Trace contains {len(trace_summary.frame_times_ms)} main frame timings but "
                f"reports {trace_summary.frame_count} measured frames: {trace_path}"
            )
        if trace_summary.total_event_count >= TRACE_MAX_EVENTS:
            warnings.warn(
                f"Trace reached the {TRACE_MAX_EVENTS:,}-event cap: {trace_path}",
                stacklevel=1,
            )
        summary_rows.extend(trace_summary.rows)
        frame_rows.extend(trace_summary.frame_rows)
        frametime_rows.append(
            {
                "RequestedResolution": resolution,
                "WindowResolution": window_resolution,
                "FramebufferResolution": actual_resolution,
                "FramebufferScaleX": framebuffer_scale_x,
                "FramebufferScaleY": framebuffer_scale_y,
                "FramebufferMatchesRequested": framebuffer_matches_requested,
                "RunOrder": run_order,
                "Run": run_number,
                "Status": "Completed",
                "SkipReason": "",
                "FrameCount": len(trace_summary.frame_times_ms),
                **frame_statistics,
                "TraceFile": str(trace_path),
            }
        )
        run_rows.append(
            {
                "RequestedResolution": resolution,
                "WindowResolution": window_resolution,
                "FramebufferResolution": actual_resolution,
                "FramebufferScaleX": framebuffer_scale_x,
                "FramebufferScaleY": framebuffer_scale_y,
                "FramebufferMatchesRequested": framebuffer_matches_requested,
                "RunOrder": run_order,
                "Run": run_number,
                "Status": "Completed",
                "SkipReason": "",
                "ReadinessFrames": readiness_frames,
                "WarmupFramesTotal": args.warmup_frames,
                "WarmupFramesPerPhaseMin": warmup_min,
                "WarmupFramesPerPhaseMax": warmup_max,
                "MeasuredFramesPerPhase": args.phase_frames,
                "PhaseRepeats": args.phase_repeats,
                "MeasuredFrameCount": trace_summary.frame_count,
                **frame_statistics,
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
        "WindowResolution",
        "FramebufferResolution",
        "FramebufferScaleX",
        "FramebufferScaleY",
        "FramebufferMatchesRequested",
        "Run",
        "Phase",
        "PhaseRepeat",
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
        "WindowResolution",
        "FramebufferResolution",
        "FramebufferScaleX",
        "FramebufferScaleY",
        "FramebufferMatchesRequested",
        "RunOrder",
        "Run",
        "Status",
        "SkipReason",
        "ReadinessFrames",
        "WarmupFramesTotal",
        "WarmupFramesPerPhaseMin",
        "WarmupFramesPerPhaseMax",
        "MeasuredFramesPerPhase",
        "PhaseRepeats",
        "MeasuredFrameCount",
        "AverageFrameTimeMs",
        "P90FrameTimeMs",
        "P95FrameTimeMs",
        "MeasurementMs",
        "MeasuredTraceEventCount",
        "TraceEventCount",
        "ClickTarget",
        "TraceFile",
        "LogFile",
    ]
    frame_fields = [
        "RequestedResolution",
        "WindowResolution",
        "FramebufferResolution",
        "FramebufferScaleX",
        "FramebufferScaleY",
        "FramebufferMatchesRequested",
        "Run",
        "Phase",
        "PhaseRepeat",
        "Frame",
        "FrameStartUs",
        "EventStartUs",
        "Scope",
        "WallMs",
        "SelfMs",
    ]
    frametime_fields = [
        "RequestedResolution",
        "WindowResolution",
        "FramebufferResolution",
        "FramebufferScaleX",
        "FramebufferScaleY",
        "FramebufferMatchesRequested",
        "RunOrder",
        "Run",
        "Status",
        "SkipReason",
        "FrameCount",
        "AverageFrameTimeMs",
        "P90FrameTimeMs",
        "P95FrameTimeMs",
        "TraceFile",
    ]
    summary_rows.sort(
        key=lambda row: (
            row["RequestedResolution"],
            row["Run"],
            row["Phase"],
            row["PhaseRepeat"],
            row["Category"],
            row["Scope"],
        )
    )
    write_csv(output_path / "render_benchmark_scopes.csv", summary_rows, summary_fields)
    write_csv(output_path / "render_benchmark_runs.csv", run_rows, run_fields)
    frame_rows.sort(
        key=lambda row: (
            row["RequestedResolution"],
            row["Run"],
            row["Phase"],
            row["PhaseRepeat"],
            row["Frame"],
            row["Scope"],
        )
    )
    write_csv(output_path / "render_benchmark_frames.csv", frame_rows, frame_fields)
    frametime_rows.sort(key=lambda row: (row["RequestedResolution"], row["Run"]))
    write_csv(output_path / "render_benchmark_frametimes.csv", frametime_rows, frametime_fields)
    print(f"Scope summary: {output_path / 'render_benchmark_scopes.csv'}")
    print(f"Run summary:   {output_path / 'render_benchmark_runs.csv'}")
    print(f"Frame timings: {output_path / 'render_benchmark_frames.csv'}")
    print(f"Run frame stats: {output_path / 'render_benchmark_frametimes.csv'}")
    for row in run_rows:
        print(
            f"{row['RequestedResolution']:>9} window {row['WindowResolution']:>9} "
            f"framebuffer {row['FramebufferResolution']:>9} "
            f"exact={'yes' if row['FramebufferMatchesRequested'] else 'no ':>3} "
            f"run {row['Run']:02d}: "
            + (
                f"SKIPPED ({row['SkipReason']})"
                if row["Status"] == "Skipped"
                else f"{row['MeasurementMs']:>9.3f} ms, "
            )
            + (
                ""
                if row["Status"] == "Skipped"
                else
                f"{row['MeasuredFrameCount']:>4} frames, "
                f"avg/p90/p95 {row['AverageFrameTimeMs']:>7.3f}/"
                f"{row['P90FrameTimeMs']:>7.3f}/"
                f"{row['P95FrameTimeMs']:>7.3f} ms, "
                f"click target {row['ClickTarget']}"
            )
        )
    if args.require_exact_resolution and any(
        not row["FramebufferMatchesRequested"] for row in run_rows
    ):
        print(
            "benchmark_rendering: at least one framebuffer did not match the requested "
            "resolution; see the CSV files for completed runs.",
            file=sys.stderr,
        )
        return 2
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
