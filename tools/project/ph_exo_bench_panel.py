"""Compatibility entry; desktop implementation lives in tools/bench_panel."""
from pathlib import Path
__file__ = str(Path(__file__).resolve().parents[1]/"bench_panel/ph_exo_bench_panel.py")
exec(compile(Path(__file__).read_bytes(), __file__, "exec"), globals())
