"""Builds and validates the research configuration editor UI."""

from __future__ import annotations

import json
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable

from .config import ConfigError
from .contracts import validate_config


@dataclass(frozen=True)
class FieldSpec:
    key: str
    label: str
    kind: str = "text"
    choices: tuple[str, ...] = ()
    required: bool = True


RECORD_FIELDS = {
    "orbit": (
        FieldSpec("id", "Product ID"),
        FieldSpec("sp3", "SP3 file", "file"),
        FieldSpec("clk", "CLK file", "file", required=False),
        FieldSpec("time_scale", "Time scale", "choice", ("GPS", "UTC")),
        FieldSpec("orbit_reference", "Orbit reference", "choice", ("APC", "COM", "UNKNOWN")),
        FieldSpec("reference_frame", "Reference frame"),
    ),
    "observations": (
        FieldSpec("id", "Dataset ID"),
        FieldSpec("station", "Station"),
        FieldSpec("rinex", "RINEX file", "file"),
        FieldSpec("time_scale", "Time scale", "choice", ("GPS", "UTC")),
        FieldSpec("telemetry_directory", "Telemetry directory", "directory", required=False),
        FieldSpec("telemetry_source_id", "Telemetry source ID", required=False),
    ),
    "ppp": (
        FieldSpec("id", "Dataset ID"),
        FieldSpec("algorithm_label", "Algorithm label"),
        FieldSpec("path", "Compact PPP JSON", "file"),
        FieldSpec("reference_x", "Reference ECEF X (m)"),
        FieldSpec("reference_y", "Reference ECEF Y (m)"),
        FieldSpec("reference_z", "Reference ECEF Z (m)"),
        FieldSpec("velocity_x", "Reference velocity X (m/s)", required=False),
        FieldSpec("velocity_y", "Reference velocity Y (m/s)", required=False),
        FieldSpec("velocity_z", "Reference velocity Z (m/s)", required=False),
        FieldSpec("reference_epoch_utc", "Reference epoch UTC", required=False),
        FieldSpec("truth_trajectory", "Dynamic truth trajectory CSV", "file", required=False),
        FieldSpec("residual_directory", "PPP residual telemetry directory", "directory", required=False),
        FieldSpec("residual_source_id", "Residual source ID", required=False),
    ),
}

OPTION_FIELDS = {
    "orbit": (
        ("orbit_interpolation_points", "Orbit interpolation points", "int", 9, ()),
        ("max_orbit_gap_seconds", "Maximum orbit gap (s)", "float", 900.0, ()),
        ("max_clock_gap_seconds", "Maximum clock gap (s)", "float", 120.0, ()),
        (
            "clock_datum",
            "Clock datum",
            "choice",
            "constellation_epoch_median",
            ("constellation_epoch_median",),
        ),
        ("pairwise_centers", "Compare candidate pairs", "bool", True, ()),
    ),
    "observations": (
        ("expected_interval_seconds", "Expected interval (s), 0 = infer", "float", 1.0, ()),
        ("gap_factor", "Gap factor", "float", 1.5, ()),
        ("write_parquet", "Write ZSTD Parquet partitions", "bool", False, ()),
    ),
    "ppp": (
        ("horizontal_threshold_m", "Horizontal threshold (m)", "float", 0.1, ()),
        ("vertical_threshold_m", "Vertical threshold (m)", "float", 0.2, ()),
        ("sustain_seconds", "Sustain duration (s)", "float", 600.0, ()),
        ("convergence_gap_factor", "Convergence gap factor", "float", 1.5, ()),
        (
            "convergence_max_gap_seconds",
            "Maximum convergence gap (s), 0 = infer",
            "float",
            0.0,
            (),
        ),
        ("jump_mad_factor", "Jump MAD factor", "float", 8.0, ()),
        ("jump_minimum_m", "Minimum jump (m)", "float", 0.05, ()),
        ("bootstrap_samples", "Bootstrap samples", "int", 2000, ()),
        ("bootstrap_confidence", "Bootstrap confidence", "float", 0.95, ()),
        ("residual_outlier_sigma", "Residual outlier threshold (sigma)", "float", 4.0, ()),
    ),
}

ANALYSIS_NAMES = {
    "orbit": "orbit_clock",
    "observations": "observations",
    "ppp": "ppp",
}


def compose_config(
    command: str,
    output_directory: str,
    records: list[dict[str, Any]],
    options: dict[str, Any],
) -> dict[str, Any]:
    if command not in ANALYSIS_NAMES:
        raise ValueError(f"Unsupported analysis command: {command}")
    if command == "orbit":
        references = [item for item in records if item.get("_role") == "reference"]
        if len(references) != 1:
            raise ValueError("Orbit configuration requires exactly one reference product")
        reference = {key: value for key, value in references[0].items() if key != "_role"}
        candidates = [
            {key: value for key, value in item.items() if key != "_role"}
            for item in records
            if item.get("_role") == "candidate"
        ]
        return {
            "analysis": "orbit_clock",
            "output_directory": output_directory,
            "reference": reference,
            "candidates": candidates,
            "options": options,
        }
    return {
        "analysis": ANALYSIS_NAMES[command],
        "output_directory": output_directory,
        "datasets": records,
        "options": options,
    }


class RecordDialog(tk.Toplevel):
    def __init__(
        self,
        parent: tk.Misc,
        title: str,
        fields: tuple[FieldSpec, ...],
        values: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(parent)
        self.title(title)
        self.resizable(True, False)
        self.transient(parent)
        self.result: dict[str, str] | None = None
        self._fields = fields
        self._vars: dict[str, tk.StringVar] = {}
        body = ttk.Frame(self, padding=14)
        body.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)

        source = values or {}
        for row, spec in enumerate(fields):
            ttk.Label(body, text=spec.label).grid(
                row=row, column=0, padx=(0, 8), pady=4, sticky="w"
            )
            variable = tk.StringVar(value=str(source.get(spec.key, "")))
            self._vars[spec.key] = variable
            if spec.kind == "choice":
                widget = ttk.Combobox(
                    body,
                    textvariable=variable,
                    values=spec.choices,
                    state="readonly",
                )
                if not variable.get() and spec.choices:
                    variable.set(spec.choices[0])
            else:
                widget = ttk.Entry(body, textvariable=variable)
            widget.grid(row=row, column=1, pady=4, sticky="ew")
            if spec.kind in {"file", "directory"}:
                ttk.Button(
                    body,
                    text="Browse...",
                    command=(
                        (lambda key=spec.key: self._browse_directory(key))
                        if spec.kind == "directory"
                        else (lambda key=spec.key: self._browse_file(key))
                    ),
                ).grid(row=row, column=2, padx=(8, 0), pady=4)

        actions = ttk.Frame(body)
        actions.grid(row=len(fields), column=0, columnspan=3, pady=(12, 0), sticky="e")
        ttk.Button(actions, text="Cancel", command=self.destroy).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(actions, text="Apply", command=self._apply).grid(row=0, column=1)
        self.bind("<Escape>", lambda _event: self.destroy())
        self.grab_set()
        self.wait_visibility()
        self.focus_set()

    def _browse_file(self, key: str) -> None:
        filetypes = {
            "sp3": (("SP3 files", "*.sp3"), ("All files", "*.*")),
            "clk": (("CLK files", "*.clk"), ("All files", "*.*")),
            "rinex": (
                ("RINEX observation files", "*.rnx *.obs *.o"),
                ("All files", "*.*"),
            ),
            "path": (("JSON files", "*.json"), ("All files", "*.*")),
        }.get(key, (("All files", "*.*"),))
        path = filedialog.askopenfilename(parent=self, title="Select input file", filetypes=filetypes)
        if path:
            self._vars[key].set(path)

    def _browse_directory(self, key: str) -> None:
        path = filedialog.askdirectory(parent=self, title="Select telemetry directory")
        if path:
            self._vars[key].set(path)

    def _apply(self) -> None:
        result = {key: variable.get().strip() for key, variable in self._vars.items()}
        missing = [spec.label for spec in self._fields if spec.required and not result[spec.key]]
        if missing:
            messagebox.showerror(
                "Missing values",
                "Complete the following fields: " + ", ".join(missing),
                parent=self,
            )
            return
        self.result = result
        self.destroy()


class ConfigEditor(tk.Toplevel):
    def __init__(
        self,
        parent: tk.Misc,
        command: str,
        on_saved: Callable[[Path, str], None],
        config_path: Path | None = None,
    ) -> None:
        super().__init__(parent)
        if command not in ANALYSIS_NAMES:
            raise ValueError(f"Unsupported analysis command: {command}")
        self.command = command
        self.on_saved = on_saved
        self.config_path = config_path
        self.title(f"Configure {ANALYSIS_NAMES[command]}")
        self.geometry("980x620")
        self.minsize(760, 500)
        self.transient(parent)
        self._data = self._load_data(config_path)
        self._records = self._load_records()
        self.output_var = tk.StringVar(value=str(self._data.get("output_directory", "")))
        self._option_vars: dict[str, tk.Variable] = {}

        self._build_widgets()
        self._refresh_records()

    def _load_data(self, path: Path | None) -> dict[str, Any]:
        if path is None:
            return {}
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("Configuration must be a JSON object")
        return value

    def _load_records(self) -> list[dict[str, Any]]:
        if self.command == "orbit":
            reference = dict(self._data.get("reference", {}))
            reference["_role"] = "reference"
            records = [reference]
            for candidate in self._data.get("candidates", []):
                item = dict(candidate)
                item["_role"] = "candidate"
                records.append(item)
            return records
        return [dict(item) for item in self._data.get("datasets", [])]

    def _build_widgets(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        output = ttk.Frame(self, padding=(16, 14, 16, 8))
        output.grid(row=0, column=0, sticky="ew")
        output.columnconfigure(1, weight=1)
        ttk.Label(output, text="Output directory").grid(row=0, column=0, padx=(0, 8), sticky="w")
        ttk.Entry(output, textvariable=self.output_var).grid(row=0, column=1, sticky="ew")
        ttk.Button(output, text="Browse...", command=self._browse_output).grid(
            row=0, column=2, padx=(8, 0)
        )

        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, padx=16, pady=8, sticky="nsew")
        records_tab = ttk.Frame(notebook, padding=10)
        options_tab = ttk.Frame(notebook, padding=10)
        notebook.add(
            records_tab,
            text="Products" if self.command == "orbit" else "Datasets",
        )
        notebook.add(options_tab, text="Options")
        records_tab.columnconfigure(0, weight=1)
        records_tab.rowconfigure(0, weight=1)

        columns = self._tree_columns()
        self.record_tree = ttk.Treeview(
            records_tab,
            columns=[key for key, _label, _width in columns],
            show="headings",
            selectmode="browse",
        )
        for key, label, width in columns:
            self.record_tree.heading(key, text=label)
            self.record_tree.column(key, width=width, minwidth=70, stretch=True)
        self.record_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(records_tab, orient="vertical", command=self.record_tree.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.record_tree.configure(yscrollcommand=scrollbar.set)
        self.record_tree.bind("<Double-1>", lambda _event: self._edit_record())

        record_actions = ttk.Frame(records_tab)
        record_actions.grid(row=1, column=0, columnspan=2, pady=(10, 0), sticky="e")
        add_label = "Add candidate" if self.command == "orbit" else "Add dataset"
        ttk.Button(record_actions, text=add_label, command=self._add_record).grid(
            row=0, column=0, padx=(0, 8)
        )
        ttk.Button(record_actions, text="Edit", command=self._edit_record).grid(
            row=0, column=1, padx=(0, 8)
        )
        ttk.Button(record_actions, text="Remove", command=self._remove_record).grid(
            row=0, column=2
        )

        options_tab.columnconfigure(1, weight=1)
        configured_options = self._data.get("options", {})
        for row, (key, label, kind, default, choices) in enumerate(OPTION_FIELDS[self.command]):
            ttk.Label(options_tab, text=label).grid(
                row=row, column=0, padx=(0, 8), pady=5, sticky="w"
            )
            initial = configured_options.get(key, default)
            if kind == "bool":
                variable: tk.Variable = tk.BooleanVar(value=bool(initial))
                widget: tk.Widget = ttk.Checkbutton(options_tab, variable=variable)
            else:
                variable = tk.StringVar(value=str(initial))
                if kind == "choice":
                    widget = ttk.Combobox(
                        options_tab,
                        textvariable=variable,
                        values=choices,
                        state="readonly",
                    )
                else:
                    widget = ttk.Entry(options_tab, textvariable=variable)
            self._option_vars[key] = variable
            widget.grid(row=row, column=1, pady=5, sticky="ew")

        footer = ttk.Frame(self, padding=(16, 8, 16, 16))
        footer.grid(row=2, column=0, sticky="ew")
        ttk.Button(footer, text="Cancel", command=self.destroy).pack(side="right")
        ttk.Button(footer, text="Validate and save", command=self._save).pack(
            side="right", padx=(0, 8)
        )

    def _tree_columns(self) -> tuple[tuple[str, str, int], ...]:
        if self.command == "orbit":
            return (
                ("role", "Role", 90),
                ("id", "ID", 120),
                ("sp3", "SP3", 220),
                ("clk", "CLK", 180),
                ("time_scale", "Time", 70),
                ("orbit_reference", "Reference", 90),
                ("reference_frame", "Frame", 90),
            )
        if self.command == "observations":
            return (
                ("id", "ID", 140),
                ("station", "Station", 100),
                ("rinex", "RINEX", 280),
                ("time_scale", "Time", 80),
                ("telemetry_directory", "Telemetry", 220),
            )
        return (
            ("id", "ID", 130),
            ("algorithm_label", "Algorithm", 120),
            ("path", "PPP JSON", 300),
            ("reference_ecef_m", "Reference ECEF (m)", 260),
        )

    def _record_values(self, record: dict[str, Any]) -> tuple[Any, ...]:
        if self.command == "orbit":
            return (
                record.get("_role", "candidate"),
                record.get("id", ""),
                record.get("sp3", ""),
                record.get("clk", ""),
                record.get("time_scale", ""),
                record.get("orbit_reference", ""),
                record.get("reference_frame", ""),
            )
        if self.command == "observations":
            return tuple(
                record.get(key, "")
                for key in ("id", "station", "rinex", "time_scale", "telemetry_directory")
            )
        reference = record.get("reference_ecef_m", [])
        return (
            record.get("id", ""),
            record.get("algorithm_label", ""),
            record.get("path", ""),
            ", ".join(str(value) for value in reference),
        )

    def _dialog_values(self, record: dict[str, Any]) -> dict[str, Any]:
        values = dict(record)
        if self.command == "ppp":
            reference = record.get("reference_ecef_m", [])
            for index, key in enumerate(("reference_x", "reference_y", "reference_z")):
                values[key] = reference[index] if index < len(reference) else ""
            velocity = record.get("reference_velocity_ecef_mps", [])
            for index, key in enumerate(("velocity_x", "velocity_y", "velocity_z")):
                values[key] = velocity[index] if index < len(velocity) else ""
        return values

    def _record_from_dialog(self, values: dict[str, str], role: str | None = None) -> dict[str, Any]:
        if self.command == "ppp":
            try:
                reference = [
                    float(values.pop("reference_x")),
                    float(values.pop("reference_y")),
                    float(values.pop("reference_z")),
                ]
            except ValueError as exc:
                raise ValueError("Reference ECEF coordinates must be finite numbers") from exc
            if not all(math.isfinite(value) for value in reference):
                raise ValueError("Reference ECEF coordinates must be finite numbers")
            values["reference_ecef_m"] = reference
            velocity_text = [values.pop(key).strip() for key in ("velocity_x", "velocity_y", "velocity_z")]
            if any(velocity_text):
                if not all(velocity_text):
                    raise ValueError("Reference velocity requires all three ECEF components")
                velocity = [float(value) for value in velocity_text]
                if not all(math.isfinite(value) for value in velocity):
                    raise ValueError("Reference velocity must contain finite numbers")
                values["reference_velocity_ecef_mps"] = velocity
        if self.command == "orbit":
            if not values.get("clk"):
                values.pop("clk", None)
            values["_role"] = role or "candidate"
        for spec in RECORD_FIELDS[self.command]:
            if not spec.required and not values.get(spec.key):
                values.pop(spec.key, None)
        return values

    def _refresh_records(self) -> None:
        self.record_tree.delete(*self.record_tree.get_children())
        for index, record in enumerate(self._records):
            self.record_tree.insert("", "end", iid=str(index), values=self._record_values(record))

    def _selected_index(self) -> int | None:
        selected = self.record_tree.selection()
        return int(selected[0]) if selected else None

    def _add_record(self) -> None:
        dialog = RecordDialog(
            self,
            "Add candidate" if self.command == "orbit" else "Add dataset",
            RECORD_FIELDS[self.command],
        )
        self.wait_window(dialog)
        if dialog.result is None:
            return
        try:
            record = self._record_from_dialog(dialog.result)
        except ValueError as exc:
            messagebox.showerror("Invalid values", str(exc), parent=self)
            return
        self._records.append(record)
        self._refresh_records()

    def _edit_record(self) -> None:
        index = self._selected_index()
        if index is None:
            messagebox.showinfo("No selection", "Select a row to edit.", parent=self)
            return
        current = self._records[index]
        role = current.get("_role") if self.command == "orbit" else None
        dialog = RecordDialog(
            self,
            "Edit product" if self.command == "orbit" else "Edit dataset",
            RECORD_FIELDS[self.command],
            self._dialog_values(current),
        )
        self.wait_window(dialog)
        if dialog.result is None:
            return
        try:
            self._records[index] = self._record_from_dialog(dialog.result, role)
        except ValueError as exc:
            messagebox.showerror("Invalid values", str(exc), parent=self)
            return
        self._refresh_records()
        self.record_tree.selection_set(str(index))

    def _remove_record(self) -> None:
        index = self._selected_index()
        if index is None:
            messagebox.showinfo("No selection", "Select a row to remove.", parent=self)
            return
        if self.command == "orbit" and self._records[index].get("_role") == "reference":
            messagebox.showerror(
                "Reference required",
                "The reference product cannot be removed; edit it instead.",
                parent=self,
            )
            return
        del self._records[index]
        self._refresh_records()

    def _browse_output(self) -> None:
        path = filedialog.askdirectory(parent=self, title="Select output directory")
        if path:
            self.output_var.set(path)

    def _option_values(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, _label, kind, _default, _choices in OPTION_FIELDS[self.command]:
            raw = self._option_vars[key].get()
            try:
                if kind == "int":
                    result[key] = int(str(raw))
                elif kind == "float":
                    result[key] = float(str(raw))
                elif kind == "bool":
                    result[key] = bool(raw)
                else:
                    result[key] = str(raw)
            except ValueError as exc:
                raise ValueError(f"Option {key} has an invalid numeric value") from exc
        return result

    def _save(self) -> None:
        try:
            config = compose_config(
                self.command,
                self.output_var.get().strip(),
                self._records,
                self._option_values(),
            )
            validate_config(self.command, config)
        except (ConfigError, ValueError) as exc:
            messagebox.showerror("Configuration is invalid", str(exc), parent=self)
            return
        initial_directory = self.config_path.parent if self.config_path else Path.cwd()
        initial_file = self.config_path.name if self.config_path else f"{ANALYSIS_NAMES[self.command]}.json"
        selected = filedialog.asksaveasfilename(
            parent=self,
            title="Save validated configuration",
            initialdir=str(initial_directory),
            initialfile=initial_file,
            defaultextension=".json",
            filetypes=(("JSON files", "*.json"),),
        )
        if not selected:
            return
        path = Path(selected).resolve()
        try:
            path.write_text(
                json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
        except OSError as exc:
            messagebox.showerror("Cannot save configuration", str(exc), parent=self)
            return
        self.on_saved(path, self.command)
        self.destroy()
