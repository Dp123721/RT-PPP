"""Implements the research application's desktop interface and task routing."""

from __future__ import annotations

import json
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any

from .config_editor import ConfigEditor


COMMAND_LABELS = {
    "orbit": "Orbit and clock",
    "observations": "RINEX observations",
    "ppp": "PPP results",
}
ANALYSIS_TO_COMMAND = {
    "orbit_clock": "orbit",
    "observations": "observations",
    "ppp": "ppp",
}


def command_for_config(path: str | Path) -> str | None:
    """Return the CLI command implied by a JSON configuration, if recognized."""
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    analysis = value.get("analysis") if isinstance(value, dict) else None
    return ANALYSIS_TO_COMMAND.get(analysis)


def build_command(
    command: str, config_path: str | Path, output_directory: str | Path | None = None
) -> list[str]:
    """Build the source-compatible subprocess command used by the GUI."""
    if command not in COMMAND_LABELS:
        raise ValueError(f"Unsupported analysis command: {command}")
    resolved_config = Path(config_path).expanduser().resolve()
    if not resolved_config.is_file():
        raise FileNotFoundError(resolved_config)
    args = [sys.executable, "-m", "rtppp_analysis", command, "--config", str(resolved_config)]
    if output_directory is not None and str(output_directory).strip():
        args.extend(["--output", str(Path(output_directory).expanduser().resolve())])
    return args


class AnalysisGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("RT-PPP Research Analysis")
        self.geometry("820x560")
        self.minsize(680, 440)

        self.command_var = tk.StringVar(value=COMMAND_LABELS["orbit"])
        self.config_var = tk.StringVar()
        self.output_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Ready")
        self._events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._process: subprocess.Popen[str] | None = None

        self._build_widgets()
        self.after(100, self._poll_process)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build_widgets(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        header = ttk.Frame(self, padding=(18, 16, 18, 8))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="RT-PPP Research Analysis", font=("Segoe UI", 15, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(header, textvariable=self.status_var).grid(row=0, column=1, sticky="e")

        form = ttk.LabelFrame(self, text="Analysis", padding=12)
        form.grid(row=1, column=0, padx=18, pady=(0, 10), sticky="ew")
        form.columnconfigure(1, weight=1)
        ttk.Label(form, text="Type").grid(row=0, column=0, padx=(0, 8), pady=4, sticky="w")
        self.command_box = ttk.Combobox(
            form,
            textvariable=self.command_var,
            values=list(COMMAND_LABELS.values()),
            state="readonly",
            width=28,
        )
        self.command_box.grid(row=0, column=1, columnspan=2, pady=4, sticky="ew")

        ttk.Label(form, text="Config JSON").grid(row=1, column=0, padx=(0, 8), pady=4, sticky="w")
        ttk.Entry(form, textvariable=self.config_var).grid(row=1, column=1, pady=4, sticky="ew")
        config_actions = ttk.Frame(form)
        config_actions.grid(row=1, column=2, padx=(8, 0), pady=4)
        ttk.Button(config_actions, text="Browse...", command=self._choose_config).grid(
            row=0, column=0, padx=(0, 6)
        )
        ttk.Button(config_actions, text="New...", command=self._new_config).grid(
            row=0, column=1, padx=(0, 6)
        )
        ttk.Button(config_actions, text="Edit...", command=self._edit_config).grid(row=0, column=2)

        ttk.Label(form, text="Output override").grid(
            row=2, column=0, padx=(0, 8), pady=4, sticky="w"
        )
        ttk.Entry(form, textvariable=self.output_var).grid(row=2, column=1, pady=4, sticky="ew")
        ttk.Button(form, text="Browse...", command=self._choose_output).grid(
            row=2, column=2, padx=(8, 0), pady=4
        )

        actions = ttk.Frame(form)
        actions.grid(row=3, column=0, columnspan=3, pady=(10, 0), sticky="e")
        self.run_button = ttk.Button(actions, text="Run analysis", command=self._run)
        self.run_button.grid(row=0, column=0, padx=(0, 8))
        self.stop_button = ttk.Button(actions, text="Stop", command=self._stop, state="disabled")
        self.stop_button.grid(row=0, column=1)

        log_frame = ttk.LabelFrame(self, text="Run log", padding=8)
        log_frame.grid(row=2, column=0, padx=18, pady=(0, 18), sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, wrap="word", state="disabled", height=16)
        self.log.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scrollbar.set)

    def _choose_config(self) -> None:
        path = filedialog.askopenfilename(
            title="Select analysis configuration",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        self.config_var.set(path)
        try:
            command = command_for_config(path)
        except (OSError, json.JSONDecodeError, AttributeError) as exc:
            self._append_log(f"Could not inspect configuration: {exc}")
            return
        if command:
            self.command_var.set(COMMAND_LABELS[command])

    def _choose_output(self) -> None:
        path = filedialog.askdirectory(title="Select output directory")
        if path:
            self.output_var.set(path)

    def _new_config(self) -> None:
        try:
            ConfigEditor(self, self._selected_command(), self._config_saved)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            messagebox.showerror("Cannot create configuration", str(exc), parent=self)

    def _edit_config(self) -> None:
        config_text = self.config_var.get().strip()
        if not config_text:
            messagebox.showerror(
                "Missing configuration", "Choose a JSON configuration file first.", parent=self
            )
            return
        path = Path(config_text).expanduser().resolve()
        try:
            command = command_for_config(path)
            if command is None:
                raise ValueError("The configuration has an unsupported analysis value")
            ConfigEditor(self, command, self._config_saved, path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            messagebox.showerror("Cannot edit configuration", str(exc), parent=self)

    def _config_saved(self, path: Path, command: str) -> None:
        self.config_var.set(str(path))
        self.command_var.set(COMMAND_LABELS[command])
        self.output_var.set("")
        self._append_log(f"Validated configuration saved: {path}")

    def _selected_command(self) -> str:
        for command, label in COMMAND_LABELS.items():
            if self.command_var.get() == label:
                return command
        raise ValueError("Select an analysis type")

    def _run(self) -> None:
        if self._process is not None:
            return
        config_text = self.config_var.get().strip()
        if not config_text:
            messagebox.showerror("Missing configuration", "Choose a JSON configuration file first.")
            return
        try:
            command = self._selected_command()
            config_path = Path(config_text).expanduser().resolve()
            args = build_command(command, config_path, self.output_var.get())
        except (OSError, ValueError) as exc:
            messagebox.showerror("Cannot start analysis", str(exc))
            return

        environment = os.environ.copy()
        source_root = str(Path(__file__).resolve().parents[1])
        existing_python_path = environment.get("PYTHONPATH")
        environment["PYTHONPATH"] = os.pathsep.join(
            item for item in (source_root, existing_python_path) if item
        )
        self._append_log("$ " + subprocess.list2cmdline(args))
        self.status_var.set("Running")
        self.run_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        try:
            creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
            self._process = subprocess.Popen(
                args,
                cwd=str(config_path.parent),
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                creationflags=creation_flags,
            )
        except OSError as exc:
            self._process = None
            self.run_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
            self.status_var.set("Failed to start")
            messagebox.showerror("Cannot start analysis", str(exc))
            return
        threading.Thread(target=self._read_process, daemon=True).start()

    def _read_process(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self._events.put(("line", line.rstrip()))
        self._events.put(("done", process.wait()))

    def _poll_process(self) -> None:
        try:
            while True:
                event, value = self._events.get_nowait()
                if event == "line":
                    self._append_log(str(value))
                elif event == "done":
                    self._process = None
                    self.run_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    if value == 0:
                        self.status_var.set("Completed")
                    else:
                        self.status_var.set(f"Failed (exit code {value})")
        except queue.Empty:
            pass
        self.after(100, self._poll_process)

    def _stop(self) -> None:
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
            self._append_log("Analysis stopped by user.")
            self.status_var.set("Stopping")

    def _append_log(self, message: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", message + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _close(self) -> None:
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
        self.destroy()


def main() -> int:
    app = AnalysisGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
