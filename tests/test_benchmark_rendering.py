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

    def test_trace_metadata_does_not_change_scope_summary(self):
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
            trace_path.write_text(
                json.dumps(
                    {
                        "traceEvents": events,
                        "openbusTraceConfig": {
                            "mainThreadFrameStride": 5,
                            "minimumScopeDurationUs": 1,
                            "maxEvents": 10000000,
                        },
                    }
                ),
                encoding="utf-8",
            )
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
                    "--reflection-size",
                    "512",
                    "--reflection-interval",
                    "3",
                    "--texture-scale",
                    "0.5",
                    "--no-material-batching",
                    "--finish-before-swap",
                    "--force-vehicle-spotlights",
                    "--trace-min-duration-us",
                    "10",
                    "--ai-bus-config",
                    "Vehicles/test/ai.bus",
                    "--ai-model-config",
                    "Vehicles/test/model.cfg",
                    "--ai-count",
                    "2",
                    "--output-directory",
                    str(output),
                ]
            )

            def fake_run(command, **kwargs):
                self.assertEqual(
                    kwargs["env"]["OPENBUS_BENCHMARK_REQUIRE_EXACT_RESOLUTION"], "1"
                )
                self.assertEqual(kwargs["env"]["OPENBUS_REFLECTION_SIZE"], "512")
                self.assertEqual(kwargs["env"]["OPENBUS_REFLECTION_INTERVAL"], "3")
                self.assertEqual(kwargs["env"]["OPENBUS_TEXTURE_SCALE"], "0.5")
                self.assertEqual(kwargs["env"]["OPENBUS_MATERIAL_BATCHING"], "0")
                self.assertEqual(kwargs["env"]["OPENBUS_GL_FINISH_BEFORE_SWAP"], "1")
                self.assertEqual(kwargs["env"]["OPENBUS_TRACE_SAMPLE_EVERY_N_FRAMES"], "1")
                self.assertEqual(kwargs["env"]["OPENBUS_TRACE_MIN_US"], "10")
                self.assertEqual(
                    kwargs["env"]["OPENBUS_BENCHMARK_FORCE_VEHICLE_SPOTLIGHTS"], "1"
                )
                self.assertEqual(kwargs["env"]["OPENBUS_AI_BUS_CONFIG"], "Vehicles/test/ai.bus")
                self.assertEqual(
                    kwargs["env"]["OPENBUS_AI_MODEL_CONFIG"], "Vehicles/test/model.cfg"
                )
                self.assertEqual(kwargs["env"]["OPENBUS_AI_COUNT"], "2")
                self.assertEqual(kwargs["env"]["OPENBUS_AI_SPOTLIGHT_NEAR_METERS"], "75")
                self.assertEqual(kwargs["env"]["OPENBUS_AI_SPOTLIGHT_FAR_FRAMES"], "24")
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
                benchmark_rendering.os.environ,
                {
                    "OPENBUS_REFLECTION_MAX_FPS": "30",
                    "OPENBUS_AI_SPOTLIGHT_NEAR_METERS": "75",
                    "OPENBUS_AI_SPOTLIGHT_FAR_FRAMES": "24",
                },
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
            self.assertEqual(timing_rows[0]["FinishBeforeSwap"], "True")
            self.assertEqual(timing_rows[0]["TextureScale"], "0.5")
            self.assertEqual(timing_rows[0]["MaterialBatching"], "False")
            self.assertEqual(timing_rows[0]["ForceVehicleSpotlights"], "True")
            self.assertEqual(timing_rows[0]["TraceMinDurationUs"], "10")
            self.assertEqual(timing_rows[0]["ConfiguredAiVehicleCount"], "2")
            self.assertEqual(timing_rows[0]["AiBusConfig"], "Vehicles/test/ai.bus")
            self.assertEqual(timing_rows[0]["AiSpotlightNearMeters"], "75")
            self.assertEqual(timing_rows[0]["AiSpotlightFarFrames"], "24")
            with (output / "render_benchmark_runs.csv").open(
                newline="", encoding="utf-8-sig"
            ) as result_file:
                run_rows = list(csv.DictReader(result_file))
            self.assertEqual(run_rows[0]["FinishBeforeSwap"], "True")
            self.assertEqual(run_rows[0]["TextureScale"], "0.5")
            self.assertEqual(run_rows[0]["MaterialBatching"], "False")
            self.assertEqual(run_rows[0]["ForceVehicleSpotlights"], "True")
            self.assertEqual(run_rows[0]["TraceMinDurationUs"], "10")
            self.assertEqual(run_rows[0]["ConfiguredAiVehicleCount"], "2")
            self.assertEqual(run_rows[0]["AiModelConfig"], "Vehicles/test/model.cfg")
            self.assertEqual(run_rows[0]["AiSpotlightNearMeters"], "75")

    def test_completed_run_exports_gpu_timing_intervals(self):
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
                    "1280x720",
                    "--runs",
                    "1",
                    "--warmup-frames",
                    "0",
                    "--phase-frames",
                    "1",
                    "--phase-repeats",
                    "1",
                    "--gpu-timing-sample-every-n-frames",
                    "5",
                    "--output-directory",
                    str(output),
                ]
            )
            events = [self.make_event("Benchmark.measure", "benchmark", 0, 500_000)]
            for index, phase in enumerate(benchmark_rendering.PHASES):
                phase_start = 100_000 * index + 10_000
                events.append(
                    self.make_event(
                        f"Benchmark.phase.{phase}", "benchmark", phase_start, 80_000
                    )
                )
                events.append(
                    self.make_event("main", "frame", phase_start + 100, 16_000)
                )

            def fake_run(command, **kwargs):
                self.assertEqual(kwargs["env"]["OPENBUS_GPU_TIMING"], "1")
                self.assertEqual(
                    kwargs["env"]["OPENBUS_GPU_TIMING_SAMPLE_EVERY_N_FRAMES"], "5"
                )
                self.assertEqual(
                    kwargs["env"]["OPENBUS_BENCHMARK_FORCE_VEHICLE_SPOTLIGHTS"], "0"
                )
                self.assertEqual(kwargs["env"]["OPENBUS_TRACE_MIN_US"], "1")
                self.assertNotIn("OPENBUS_AI_BUS_CONFIG", kwargs["env"])
                self.assertNotIn("OPENBUS_AI_MODEL_CONFIG", kwargs["env"])
                self.assertNotIn("OPENBUS_AI_COUNT", kwargs["env"])
                kwargs["stdout"].write(
                    "BENCHMARK_COMPLETE=1\n"
                    "BENCHMARK_WINDOW=1280x720\n"
                    "BENCHMARK_FRAMEBUFFER=1280x720\n"
                    "BENCHMARK_PHASE_REPEATS=1\n"
                    "BENCHMARK_READINESS_FRAMES=0\n"
                    "BENCHMARK_CLICK_TARGET=found\n"
                    "GPU timer queries enabled; sampling every 5 main frame(s)\n"
                    "GPU_TIMING interval_s=2.000 "
                    "main_view_samples=4 main_view_avg_ms=3.250 main_view_max_ms=5.000 "
                    "main_view_dropped=1 reflection_samples=2 reflection_avg_ms=1.500 "
                    "reflection_max_ms=2.000 reflection_dropped=0 "
                    "spotlight_shadow_samples=2 spotlight_shadow_avg_ms=1.250 "
                    "spotlight_shadow_max_ms=1.500 spotlight_shadow_dropped=1 "
                    "spotlight_cull_samples=3 spotlight_cull_avg_ms=0.030 "
                    "spotlight_cull_max_ms=0.040 spotlight_cull_dropped=2 "
                    "map_light_cull_samples=2 map_light_cull_avg_ms=0.050 "
                    "map_light_cull_max_ms=0.060 map_light_cull_dropped=1\n"
                    "LIGHTING_UPLOAD interval_s=2.1 vehicle_selection_views=7 "
                    "player_spotlight_candidates_total=7 ai_spotlight_candidates_total=40 "
                    "spotlight_shadow_frames=6 spotlight_shadow_selected_total=17 "
                    "spotlight_shadow_selected_max=4 spotlight_shadow_updates=11 "
                    "map_registered_max=345 map_gpu_tile_fallback_sets=5 "
                    "map_gpu_tile_fallback_density=5\n"
                    "LIGHTING_UPLOAD interval_s=1.9 vehicle_sets=2 map_registered_max=3\n"
                )
                Path(kwargs["env"]["OPENBUS_TRACE_FILE"]).write_text(
                    json.dumps({"traceEvents": events}), encoding="utf-8"
                )
                return benchmark_rendering.subprocess.CompletedProcess(command, 0)

            with patch.object(benchmark_rendering, "EXECUTABLE", executable), patch.object(
                benchmark_rendering, "CACHE_FILE", cache
            ), patch.object(benchmark_rendering, "BUILD_DIR", build), patch.object(
                benchmark_rendering.subprocess, "run", side_effect=fake_run
            ), patch.dict(
                benchmark_rendering.os.environ,
                {
                    "OPENBUS_GPU_TIMING": "0",
                    "OPENBUS_GPU_TIMING_SAMPLE_EVERY_N_FRAMES": "9",
                },
            ):
                self.assertEqual(benchmark_rendering.run_benchmark(args), 0)

            with (output / "render_benchmark_gpu_timing.csv").open(
                newline="", encoding="utf-8-sig"
            ) as result_file:
                gpu_rows = list(csv.DictReader(result_file))
            self.assertEqual(len(gpu_rows), 1)
            self.assertEqual(gpu_rows[0]["GpuTimingStatus"], "enabled")
            self.assertEqual(gpu_rows[0]["GpuTimingSampleEveryNFrames"], "5")
            self.assertEqual(gpu_rows[0]["MainViewSamples"], "4")
            self.assertEqual(gpu_rows[0]["MainViewAverageMs"], "3.25")
            self.assertEqual(gpu_rows[0]["MainViewDropped"], "1")
            self.assertEqual(gpu_rows[0]["ReflectionSamples"], "2")
            self.assertEqual(gpu_rows[0]["ReflectionMaxMs"], "2.0")
            self.assertEqual(gpu_rows[0]["ReflectionDropped"], "0")
            self.assertEqual(gpu_rows[0]["SpotlightShadowSamples"], "2")
            self.assertEqual(gpu_rows[0]["SpotlightShadowAverageMs"], "1.25")
            self.assertEqual(gpu_rows[0]["SpotlightShadowMaxMs"], "1.5")
            self.assertEqual(gpu_rows[0]["SpotlightShadowDropped"], "1")
            self.assertEqual(gpu_rows[0]["SpotlightCullSamples"], "3")
            self.assertEqual(gpu_rows[0]["SpotlightCullAverageMs"], "0.03")
            self.assertEqual(gpu_rows[0]["SpotlightCullMaxMs"], "0.04")
            self.assertEqual(gpu_rows[0]["SpotlightCullDropped"], "2")
            self.assertEqual(gpu_rows[0]["MapLightCullSamples"], "2")
            self.assertEqual(gpu_rows[0]["MapLightCullAverageMs"], "0.05")
            self.assertEqual(gpu_rows[0]["MapLightCullMaxMs"], "0.06")
            self.assertEqual(gpu_rows[0]["MapLightCullDropped"], "1")
            self.assertEqual(gpu_rows[0]["ForceVehicleSpotlights"], "False")
            self.assertEqual(gpu_rows[0]["TraceMinDurationUs"], "1")
            with (output / "render_benchmark_lighting_upload.csv").open(
                newline="", encoding="utf-8-sig"
            ) as result_file:
                lighting_rows = list(csv.DictReader(result_file))
            self.assertEqual(len(lighting_rows), 2)
            self.assertEqual(lighting_rows[0]["IntervalIndex"], "1")
            self.assertEqual(lighting_rows[0]["IntervalSeconds"], "2.1")
            self.assertEqual(lighting_rows[0]["vehicle_selection_views"], "7")
            self.assertEqual(lighting_rows[0]["ai_spotlight_candidates_total"], "40")
            self.assertEqual(lighting_rows[0]["spotlight_shadow_selected_max"], "4")
            self.assertEqual(lighting_rows[0]["spotlight_shadow_updates"], "11")
            self.assertEqual(lighting_rows[0]["map_gpu_tile_fallback_density"], "5")
            self.assertEqual(lighting_rows[1]["IntervalIndex"], "2")
            self.assertEqual(lighting_rows[1]["map_gpu_tile_fallback_density"], "")
            self.assertEqual(gpu_rows[0]["ConfiguredAiVehicleCount"], "0")
            self.assertEqual(gpu_rows[0]["AiSpotlightNearMeters"], "60")
            self.assertEqual(gpu_rows[0]["AiSpotlightFarFrames"], "20")

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
        self.assertEqual(args.reflection_size, 1024)
        self.assertEqual(args.reflection_interval, 1)
        self.assertFalse(args.finish_before_swap)
        self.assertEqual(args.texture_scale, 1.0)
        self.assertTrue(args.material_batching)
        self.assertFalse(args.gpu_timing)
        self.assertIsNone(args.gpu_timing_sample_every_n_frames)
        self.assertFalse(args.force_vehicle_spotlights)
        self.assertEqual(args.trace_min_duration_us, 1)

    def test_finish_before_swap_flag(self):
        args = benchmark_rendering.build_parser().parse_args(["--finish-before-swap"])
        self.assertTrue(args.finish_before_swap)

    def test_gpu_timing_cli_options(self):
        args = benchmark_rendering.build_parser().parse_args(
            ["--gpu-timing", "--gpu-timing-sample-every-n-frames", "5"]
        )
        self.assertTrue(args.gpu_timing)
        self.assertEqual(args.gpu_timing_sample_every_n_frames, 5)
        with self.assertRaises(SystemExit):
            benchmark_rendering.build_parser().parse_args(
                ["--gpu-timing-sample-every-n-frames", "0"]
            )

    def test_vehicle_spotlight_and_trace_size_options(self):
        args = benchmark_rendering.build_parser().parse_args(
            ["--force-vehicle-spotlights", "--trace-min-duration-us", "10"]
        )
        self.assertTrue(args.force_vehicle_spotlights)
        self.assertEqual(args.trace_min_duration_us, 10)
        with self.assertRaises(SystemExit):
            benchmark_rendering.build_parser().parse_args(["--trace-min-duration-us", "0"])

    def test_explicit_ai_vehicle_configuration_requires_both_paths_and_valid_count(self):
        args = benchmark_rendering.build_parser().parse_args(["--ai-bus-config", "Vehicles/test/ai.bus"])
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build = root / "build"
            build.mkdir()
            (build / "OpenBus.exe").touch()
            (build / "CMakeCache.txt").write_text(
                "OPENBUS_ENABLE_PERF_TRACE:BOOL=ON\n", encoding="utf-8"
            )
            with patch.object(benchmark_rendering, "EXECUTABLE", build / "OpenBus.exe"), patch.object(
                benchmark_rendering, "CACHE_FILE", build / "CMakeCache.txt"
            ), patch.object(benchmark_rendering, "BUILD_DIR", build):
                with self.assertRaisesRegex(ValueError, "must be provided together"):
                    benchmark_rendering.run_benchmark(args)
                invalid_count_args = benchmark_rendering.build_parser().parse_args(
                    [
                        "--ai-bus-config",
                        "Vehicles/test/ai.bus",
                        "--ai-model-config",
                        "Vehicles/test/model.cfg",
                        "--ai-count",
                        "201",
                    ]
                )
                with self.assertRaisesRegex(ValueError, "between 1 and 200"):
                    benchmark_rendering.run_benchmark(invalid_count_args)

    def test_gpu_timing_configuration_uses_the_initialized_sample_stride(self):
        configuration = benchmark_rendering.gpu_timing_configuration(
            "[INFO] Game GPU timer queries enabled; sampling every 5 main frame(s)",
            {"OPENBUS_GPU_TIMING": "1"},
        )
        self.assertEqual(
            configuration,
            {
                "GpuTimingStatus": "enabled",
                "GpuTimingSampleEveryNFrames": 5,
            },
        )
        self.assertEqual(
            benchmark_rendering.gpu_timing_configuration("", {}),
            {"GpuTimingStatus": "disabled", "GpuTimingSampleEveryNFrames": ""},
        )

    def test_lighting_upload_parser_handles_old_and_invalid_logs(self):
        samples = benchmark_rendering.parse_lighting_upload_summaries(
            "LIGHTING_UPLOAD interval_s=1.5 vehicle_sets=2 map_lights_max=9\n"
            "LIGHTING_UPLOAD interval_s=2 map_gpu_tile_fallback_density=4\n"
        )
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0]["vehicle_sets"], 2)
        self.assertEqual(samples[0]["map_gpu_tile_fallback_density"], "")
        self.assertEqual(samples[1]["map_gpu_tile_fallback_density"], 4)
        for line in (
            "LIGHTING_UPLOAD vehicle_sets=1",
            "LIGHTING_UPLOAD interval_s=nan vehicle_sets=1",
            "LIGHTING_UPLOAD interval_s=-1 vehicle_sets=1",
            "LIGHTING_UPLOAD interval_s=1 vehicle_sets=-1",
            "LIGHTING_UPLOAD interval_s=1 vehicle_sets=broken",
            "LIGHTING_UPLOAD interval_s=1 vehicle_sets=1 vehicle_sets=2",
        ):
            with self.subTest(line=line), self.assertRaises(ValueError):
                benchmark_rendering.parse_lighting_upload_summaries(line)

    def test_gpu_timing_summary_parser_reads_samples_and_dropped_counts(self):
        log_text = (
            "[INFO] Game GPU_TIMING interval_s=2.000 "
            "main_view_samples=4 main_view_avg_ms=3.250 main_view_max_ms=5.000 "
            "main_view_dropped=1 reflection_samples=2 reflection_avg_ms=1.500 "
            "reflection_max_ms=2.000 reflection_dropped=0\n"
            "[INFO] Game GPU_TIMING interval_s=2.100 "
            "main_view_samples=0 main_view_dropped=0 "
            "reflection_samples=0 reflection_dropped=2\n"
        )
        summaries = benchmark_rendering.parse_gpu_timing_summaries(log_text)
        self.assertEqual(
            summaries,
            [
                {
                    "IntervalSeconds": 2.0,
                    "MainViewSamples": 4,
                    "MainViewAverageMs": 3.25,
                    "MainViewMaxMs": 5.0,
                    "MainViewDropped": 1,
                    "ReflectionSamples": 2,
                    "ReflectionAverageMs": 1.5,
                    "ReflectionMaxMs": 2.0,
                    "ReflectionDropped": 0,
                    "SpotlightShadowSamples": "",
                    "SpotlightShadowAverageMs": "",
                    "SpotlightShadowMaxMs": "",
                    "SpotlightShadowDropped": "",
                    "SpotlightCullSamples": "",
                    "SpotlightCullAverageMs": "",
                    "SpotlightCullMaxMs": "",
                    "SpotlightCullDropped": "",
                    "MapLightCullSamples": "",
                    "MapLightCullAverageMs": "",
                    "MapLightCullMaxMs": "",
                    "MapLightCullDropped": "",
                },
                {
                    "IntervalSeconds": 2.1,
                    "MainViewSamples": 0,
                    "MainViewAverageMs": "",
                    "MainViewMaxMs": "",
                    "MainViewDropped": 0,
                    "ReflectionSamples": 0,
                    "ReflectionAverageMs": "",
                    "ReflectionMaxMs": "",
                    "ReflectionDropped": 2,
                    "SpotlightShadowSamples": "",
                    "SpotlightShadowAverageMs": "",
                    "SpotlightShadowMaxMs": "",
                    "SpotlightShadowDropped": "",
                    "SpotlightCullSamples": "",
                    "SpotlightCullAverageMs": "",
                    "SpotlightCullMaxMs": "",
                    "SpotlightCullDropped": "",
                    "MapLightCullSamples": "",
                    "MapLightCullAverageMs": "",
                    "MapLightCullMaxMs": "",
                    "MapLightCullDropped": "",
                },
            ],
        )

    def test_gpu_timing_summary_parser_rejects_malformed_values(self):
        with self.assertRaisesRegex(ValueError, "Malformed GPU timing summary"):
            benchmark_rendering.parse_gpu_timing_summaries(
                "GPU_TIMING interval_s=2.000 main_view_samples=broken"
            )
        base = (
            "GPU_TIMING interval_s=2 main_view_samples=0 main_view_dropped=0 "
            "reflection_samples=0 reflection_dropped=0 "
        )
        for suffix in (
            "spotlight_shadow_samples=1",
            "spotlight_shadow_samples=1 spotlight_shadow_dropped=0",
            "spotlight_shadow_samples=bad spotlight_shadow_dropped=0",
        ):
            with self.subTest(suffix=suffix), self.assertRaisesRegex(
                ValueError, "Malformed GPU timing summary"
            ):
                benchmark_rendering.parse_gpu_timing_summaries(base + suffix)
        with self.assertRaisesRegex(ValueError, "Invalid GPU timing values"):
            benchmark_rendering.parse_gpu_timing_summaries(
                base + "spotlight_shadow_samples=0 spotlight_shadow_dropped=-1"
            )
        with self.assertRaisesRegex(ValueError, "Invalid GPU timing values"):
            benchmark_rendering.parse_gpu_timing_summaries(
                base + "spotlight_shadow_samples=1 spotlight_shadow_dropped=0 "
                "spotlight_shadow_avg_ms=nan spotlight_shadow_max_ms=1"
            )
        for suffix in (
            "spotlight_cull_samples=1",
            "spotlight_cull_samples=1 spotlight_cull_dropped=0",
            "spotlight_cull_samples=bad spotlight_cull_dropped=0",
        ):
            with self.subTest(suffix=suffix), self.assertRaisesRegex(
                ValueError, "Malformed GPU timing summary"
            ):
                benchmark_rendering.parse_gpu_timing_summaries(base + suffix)
        with self.assertRaisesRegex(ValueError, "Invalid GPU timing values"):
            benchmark_rendering.parse_gpu_timing_summaries(
                base + "spotlight_cull_samples=0 spotlight_cull_dropped=-1"
            )
        with self.assertRaisesRegex(ValueError, "Invalid GPU timing values"):
            benchmark_rendering.parse_gpu_timing_summaries(
                base
                + "spotlight_cull_samples=1 spotlight_cull_dropped=0 "
                "spotlight_cull_avg_ms=nan spotlight_cull_max_ms=1"
            )
        for suffix in (
            "map_light_cull_samples=1",
            "map_light_cull_samples=1 map_light_cull_dropped=0",
            "map_light_cull_samples=bad map_light_cull_dropped=0",
        ):
            with self.subTest(suffix=suffix), self.assertRaisesRegex(
                ValueError, "Malformed GPU timing summary"
            ):
                benchmark_rendering.parse_gpu_timing_summaries(base + suffix)
        with self.assertRaisesRegex(ValueError, "Invalid GPU timing values"):
            benchmark_rendering.parse_gpu_timing_summaries(
                base + "map_light_cull_samples=0 map_light_cull_dropped=-1"
            )
        with self.assertRaisesRegex(ValueError, "Invalid GPU timing values"):
            benchmark_rendering.parse_gpu_timing_summaries(
                base + "map_light_cull_samples=1 map_light_cull_dropped=0 "
                "map_light_cull_avg_ms=nan map_light_cull_max_ms=1"
            )

    def test_gpu_spotlight_timing_with_no_samples(self):
        shadow_summary = benchmark_rendering.parse_gpu_timing_summaries(
            "GPU_TIMING interval_s=2 main_view_samples=0 main_view_dropped=0 "
            "reflection_samples=0 reflection_dropped=0 "
            "spotlight_shadow_samples=0 spotlight_shadow_dropped=4"
        )[0]
        self.assertEqual(shadow_summary["SpotlightShadowSamples"], 0)
        self.assertEqual(shadow_summary["SpotlightShadowAverageMs"], "")
        self.assertEqual(shadow_summary["SpotlightShadowMaxMs"], "")
        self.assertEqual(shadow_summary["SpotlightShadowDropped"], 4)
        summary = benchmark_rendering.parse_gpu_timing_summaries(
            "GPU_TIMING interval_s=2 main_view_samples=0 main_view_dropped=0 "
            "reflection_samples=0 reflection_dropped=0 "
            "spotlight_cull_samples=0 spotlight_cull_dropped=7"
        )[0]
        self.assertEqual(summary["SpotlightCullSamples"], 0)
        self.assertEqual(summary["SpotlightCullAverageMs"], "")
        self.assertEqual(summary["SpotlightCullMaxMs"], "")
        self.assertEqual(summary["SpotlightCullDropped"], 7)
        summary = benchmark_rendering.parse_gpu_timing_summaries(
            "GPU_TIMING interval_s=2 main_view_samples=0 main_view_dropped=0 "
            "reflection_samples=0 reflection_dropped=0 "
            "map_light_cull_samples=0 map_light_cull_dropped=3"
        )[0]
        self.assertEqual(summary["MapLightCullSamples"], 0)
        self.assertEqual(summary["MapLightCullAverageMs"], "")
        self.assertEqual(summary["MapLightCullMaxMs"], "")
        self.assertEqual(summary["MapLightCullDropped"], 3)


if __name__ == "__main__":
    unittest.main()
