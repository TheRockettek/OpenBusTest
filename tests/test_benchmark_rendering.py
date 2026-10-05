import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import benchmark_rendering


class TraceSummaryTests(unittest.TestCase):
    def make_event(self, name, category, start, duration, thread=1):
        return {
            "name": name,
            "cat": category,
            "ph": "X",
            "ts": start,
            "dur": duration,
            "tid": thread,
        }

    def test_self_time_subtracts_the_union_of_overlapping_children(self):
        events = [
            self.make_event("Benchmark.measure", "benchmark", 0, 600),
            self.make_event("Benchmark.phase.baseline", "benchmark", 10, 100),
            self.make_event("Parent", "test", 20, 60),
            self.make_event("ChildA", "test", 25, 30),
            self.make_event("ChildB", "test", 40, 25),
            self.make_event("main", "frame", 85, 3),
            self.make_event("Benchmark.phase.cameraCycle", "benchmark", 110, 100),
            self.make_event("Benchmark.phase.thirdPersonZoom", "benchmark", 210, 100),
            self.make_event("Benchmark.phase.drivingControls", "benchmark", 310, 100),
            self.make_event("Benchmark.phase.dashboardInteraction", "benchmark", 410, 100),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.json"
            trace_path.write_text(json.dumps({"traceEvents": events}), encoding="utf-8")
            summary = benchmark_rendering.summarize_trace(
                trace_path, "1920x1080", "1920x1080", 1
            )

        parent = next(
            row
            for row in summary.rows
            if row["Phase"] == "baseline" and row["Scope"] == "Parent"
        )
        self.assertEqual(parent["Count"], 1)
        self.assertEqual(parent["WallMs"], 0.06)
        self.assertEqual(parent["SelfMs"], 0.02)
        self.assertEqual(summary.frame_count, 1)
        self.assertEqual(summary.frame_times_ms, [0.003])

    def test_per_run_frame_time_summary_uses_interpolated_percentiles(self):
        summary = benchmark_rendering.summarize_frame_times([10.0, 20.0, 30.0, 40.0])
        self.assertEqual(
            summary,
            {
                "AverageFrameTimeMs": 25.0,
                "P90FrameTimeMs": 37.0,
                "P95FrameTimeMs": 38.5,
            },
        )

    def test_percentile_rejects_empty_samples(self):
        with self.assertRaisesRegex(ValueError, "without frame timings"):
            benchmark_rendering.percentile([], 0.95)

    def test_resolution_mismatch_is_skipped_without_timing_data(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build = root / "build"
            build.mkdir()
            executable = build / "OpenBus.exe"
            executable.touch()
            cache = build / "CMakeCache.txt"
            cache.write_text("OPENBUS_ENABLE_PERF_TRACE:BOOL=ON\n", encoding="utf-8")
            output = root / "results"
            args = benchmark_rendering.build_parser().parse_args(
                [
                    "--resolutions",
                    "1920x1080",
                    "--runs",
                    "1",
                    "--warmup-frames",
                    "0",
                    "--phase-frames",
                    "1",
                    "--phase-repeats",
                    "1",
                    "--output-directory",
                    str(output),
                ]
            )

            def fake_run(command, **kwargs):
                self.assertEqual(
                    kwargs["env"]["OPENBUS_BENCHMARK_REQUIRE_EXACT_RESOLUTION"], "1"
                )
                self.assertNotIn("OPENBUS_REFLECTION_MAX_FPS", kwargs["env"])
                kwargs["stdout"].write(
                    "BENCHMARK_COMPLETE=1\n"
                    "BENCHMARK_WINDOW=1280x720\n"
                    "BENCHMARK_FRAMEBUFFER=1280x720\n"
                    "BENCHMARK_PHASE_REPEATS=1\n"
                    "BENCHMARK_READINESS_FRAMES=0\n"
                    "BENCHMARK_SKIPPED=resolution_mismatch\n"
                )
                return benchmark_rendering.subprocess.CompletedProcess(command, 0)

            with patch.object(benchmark_rendering, "EXECUTABLE", executable), patch.object(
                benchmark_rendering, "CACHE_FILE", cache
            ), patch.object(benchmark_rendering, "BUILD_DIR", build), patch.object(
                benchmark_rendering.subprocess, "run", side_effect=fake_run
            ), patch.dict(
                benchmark_rendering.os.environ, {"OPENBUS_REFLECTION_MAX_FPS": "30"}
            ):
                self.assertEqual(benchmark_rendering.run_benchmark(args), 0)

            with (output / "render_benchmark_frametimes.csv").open(
                newline="", encoding="utf-8-sig"
            ) as result_file:
                timing_rows = list(csv.DictReader(result_file))
            self.assertEqual(len(timing_rows), 1)
            self.assertEqual(timing_rows[0]["Status"], "Skipped")
            self.assertEqual(timing_rows[0]["SkipReason"], "framebuffer mismatch")
            self.assertEqual(timing_rows[0]["FrameCount"], "0")
            self.assertEqual(timing_rows[0]["AverageFrameTimeMs"], "")
            self.assertEqual(timing_rows[0]["P90FrameTimeMs"], "")
            self.assertEqual(timing_rows[0]["P95FrameTimeMs"], "")

    def test_missing_phase_marker_is_reported(self):
        events = [self.make_event("Benchmark.measure", "benchmark", 0, 600)]
        events.extend(
            self.make_event(f"Benchmark.phase.{phase}", "benchmark", index * 100 + 10, 80)
            for index, phase in enumerate(benchmark_rendering.PHASES[:-1])
        )
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.json"
            trace_path.write_text(json.dumps({"traceEvents": events}), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "dashboardInteraction"):
                benchmark_rendering.summarize_trace(
                    trace_path, "1920x1080", "1920x1080", 1
                )

    def test_frame_rows_correlate_scope_timings_and_resolution_match(self):
        events = [
            self.make_event("Benchmark.measure", "benchmark", 0, 500),
            self.make_event("Benchmark.phase.baseline", "benchmark", 10, 100),
            self.make_event("main", "frame", 20, 80),
            self.make_event("RenderLoop::draw", "frame", 25, 50),
            self.make_event("ReflectionRenderer::render", "render", 30, 15),
            self.make_event("RenderLoop::endFrame.swapBuffers", "frame", 85, 5),
        ]
        events.extend(
            self.make_event(f"Benchmark.phase.{phase}", "benchmark", 110 + index * 80, 70)
            for index, phase in enumerate(benchmark_rendering.PHASES[1:])
        )
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.json"
            trace_path.write_text(json.dumps({"traceEvents": events}), encoding="utf-8")
            summary = benchmark_rendering.summarize_trace(
                trace_path, "1280x720", "1280x720", 1, "1280x720"
            )

        draw = next(
            row
            for row in summary.frame_rows
            if row["Phase"] == "baseline" and row["Scope"] == "RenderLoop::draw"
        )
        self.assertEqual(draw["Frame"], 1)
        self.assertEqual(draw["WallMs"], 0.05)
        self.assertTrue(draw["FramebufferMatchesRequested"])

    def test_mismatched_framebuffer_is_flagged(self):
        events = [
            self.make_event("Benchmark.measure", "benchmark", 0, 500),
            self.make_event("Benchmark.phase.baseline", "benchmark", 10, 80),
        ]
        events.extend(
            self.make_event(f"Benchmark.phase.{phase}", "benchmark", 100 + index * 80, 70)
            for index, phase in enumerate(benchmark_rendering.PHASES[1:])
        )
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.json"
            trace_path.write_text(json.dumps({"traceEvents": events}), encoding="utf-8")
            summary = benchmark_rendering.summarize_trace(
                trace_path, "3840x2160", "2564x1421", 1, "2564x1421"
            )

        self.assertFalse(summary.rows[0]["FramebufferMatchesRequested"])

    def test_phase_repeats_are_numbered_independently(self):
        events = [self.make_event("Benchmark.measure", "benchmark", 0, 1200)]
        events.extend(
            self.make_event("main", "frame", start, 20)
            for start in (20, 520)
        )
        events.extend(
            self.make_event(
                f"Benchmark.phase.{phase}",
                "benchmark",
                10 + (repeat * len(benchmark_rendering.PHASES) + index) * 100,
                60,
            )
            for repeat in range(2)
            for index, phase in enumerate(benchmark_rendering.PHASES)
        )
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.json"
            trace_path.write_text(json.dumps({"traceEvents": events}), encoding="utf-8")
            summary = benchmark_rendering.summarize_trace(
                trace_path, "1920x1080", "1920x1080", 1
            )

        baseline_markers = [
            row
            for row in summary.rows
            if row["Phase"] == "baseline" and row["Scope"] == "Benchmark.phase.baseline"
        ]
        self.assertEqual(summary.phase_repeat_count, 2)
        self.assertEqual([row["PhaseRepeat"] for row in baseline_markers], [1, 2])
        baseline_frames = [row for row in summary.frame_rows if row["Phase"] == "baseline"]
        self.assertEqual([row["PhaseRepeat"] for row in baseline_frames], [1, 2])

    def test_repeated_run_default_is_five(self):
        args = benchmark_rendering.build_parser().parse_args([])
        self.assertEqual(args.runs, 5)
        self.assertEqual(args.phase_frames, 120)
        self.assertEqual(args.phase_repeats, 2)


if __name__ == "__main__":
    unittest.main()
