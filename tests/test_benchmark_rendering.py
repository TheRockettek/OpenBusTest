import json
import sys
import tempfile
import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
