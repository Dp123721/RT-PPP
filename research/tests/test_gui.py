"""Tests research GUI configuration editing and command dispatch."""

from __future__ import annotations

import importlib.util
import json
import tomllib
import unittest
from pathlib import Path
from unittest.mock import Mock

from _support import RESEARCH_ROOT
from rtppp_analysis.config_editor import compose_config
from rtppp_analysis.contracts import validate_config
from rtppp_analysis.gui import build_command, command_for_config


class GuiEntryTests(unittest.TestCase):
    def test_requirement_files_match_project_dependencies(self) -> None:
        project = tomllib.loads((RESEARCH_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
        expected = set(project["project"]["dependencies"])
        requirements = {
            line.strip()
            for line in (RESEARCH_ROOT / "requirements.txt").read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        self.assertEqual(requirements, expected)

        parquet_lines = {
            line.strip()
            for line in (RESEARCH_ROOT / "requirements-parquet.txt")
            .read_text(encoding="utf-8")
            .splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        self.assertIn("-r requirements.txt", parquet_lines)
        parquet_lines.remove("-r requirements.txt")
        self.assertEqual(
            parquet_lines,
            set(project["project"]["optional-dependencies"]["full"]),
        )

    def test_direct_entry_defaults_to_gui_and_dispatches_cli(self) -> None:
        entry_path = RESEARCH_ROOT / "main.py"
        spec = importlib.util.spec_from_file_location("rtppp_research_entry", entry_path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)

        gui = Mock(return_value=7)
        cli = Mock(return_value=8)
        self.assertEqual(entry.run([], cli=cli, gui=gui), 7)
        gui.assert_called_once_with()
        cli.assert_not_called()

        gui.reset_mock()
        self.assertEqual(entry.run(["ppp", "--config", "config.json"], cli=cli, gui=gui), 8)
        cli.assert_called_once_with(["ppp", "--config", "config.json"])
        gui.assert_not_called()

    def test_configuration_analysis_is_mapped_to_cli_command(self) -> None:
        self.assertEqual(
            command_for_config(RESEARCH_ROOT / "configs" / "orbit_compare.example.json"),
            "orbit",
        )
        self.assertEqual(
            command_for_config(RESEARCH_ROOT / "configs" / "observation_analysis.example.json"),
            "observations",
        )
        self.assertEqual(
            command_for_config(RESEARCH_ROOT / "configs" / "ppp_analysis.example.json"),
            "ppp",
        )

    def test_build_command_is_source_compatible(self) -> None:
        config = RESEARCH_ROOT / "configs" / "ppp_analysis.example.json"
        command = build_command("ppp", config, RESEARCH_ROOT / "workspace" / "gui-output")
        self.assertEqual(command[1:5], ["-m", "rtppp_analysis", "ppp", "--config"])
        self.assertEqual(Path(command[5]), config.resolve())
        self.assertEqual(command[6], "--output")
        self.assertEqual(Path(command[7]), (RESEARCH_ROOT / "workspace" / "gui-output").resolve())

    def test_interactive_editor_composes_all_configuration_types(self) -> None:
        examples = (
            ("orbit", "orbit_compare.example.json"),
            ("observations", "observation_analysis.example.json"),
            ("ppp", "ppp_analysis.example.json"),
        )
        for command, name in examples:
            with self.subTest(command=command):
                expected = json.loads(
                    (RESEARCH_ROOT / "configs" / name).read_text(encoding="utf-8")
                )
                if command == "orbit":
                    reference = dict(expected["reference"])
                    reference["_role"] = "reference"
                    records = [reference]
                    for candidate in expected["candidates"]:
                        item = dict(candidate)
                        item["_role"] = "candidate"
                        records.append(item)
                else:
                    records = expected["datasets"]
                actual = compose_config(
                    command,
                    expected["output_directory"],
                    records,
                    expected["options"],
                )
                self.assertEqual(actual, expected)
                validate_config(command, actual)


if __name__ == "__main__":
    unittest.main()
