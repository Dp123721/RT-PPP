"""Direct entry point for the RT-PPP research application.

Run this file without arguments from PyCharm to open the GUI. The same file
also accepts the existing CLI commands, for example:

    python main.py ppp --config configs/ppp_analysis.example.json
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable


PROJECT_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = PROJECT_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

if sys.version_info < (3, 11):
    version = ".".join(str(value) for value in sys.version_info[:3])
    raise SystemExit(
        f"RT-PPP Research requires Python 3.11 or newer; current interpreter is {version}. "
        "Create a new Python 3.11/3.12 virtual environment and select it in PyCharm."
    )

from rtppp_analysis.cli import main as cli_main  # noqa: E402
from rtppp_analysis.gui import main as gui_main  # noqa: E402


def run(
    argv: list[str] | None = None,
    *,
    cli: Callable[[list[str] | None], int] = cli_main,
    gui: Callable[[], int] = gui_main,
) -> int:
    """Start the GUI by default or dispatch explicit arguments to the CLI."""
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments or arguments == ["gui"] or arguments == ["--gui"]:
        return gui()
    return cli(arguments)


if __name__ == "__main__":
    raise SystemExit(run())
