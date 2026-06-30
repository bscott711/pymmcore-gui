# CLAUDE.md — pymmcore-gui

Pure-Python Micro-Manager GUI built on pymmcore-plus + pymmcore-widgets + ndv,
using PyQt6 and PyQt6Ads dockable panels.

## Orchestration rule
- Before implementing anything non-trivial, write the execution plan to
  `.claude/current_plan.md` and keep it updated as work proceeds.

## Commands (run from this repo root via `uv`)
- `just run`    — launch the app (`uv run python -m pymmcore_gui`)
- `just test`   — full pytest suite
- `just lint`   — pre-commit (ruff, mypy, etc.) on all files
- `just fix`    — auto-fix ruff lint + format
- `just bundle` — PyInstaller app bundle (`app/mmgui.spec`)
- Single test: `uv run pytest tests/test_widgets.py -q`

## Directory map (`src/pymmcore_gui/`)
- `_app.py` / `__main__.py` — `MMQApplication` + entrypoint
- `_main_window.py`         — `MMMainWindow`; builds menus/toolbars/docks from actions
- `actions/`               — action registry (the central wiring layer)
  - `_action_info.py` — pydantic `ActionInfo` base + `QCoreAction`
  - `core_actions.py` — `CoreAction` (commands)
  - `widget_actions.py` — `WidgetAction` (lazy-singleton dock widgets)
- `widgets/`              — Qt panels (MDA, console, stage, CRISP, image_preview, …)
- `_ndv_viewers.py`       — ndv viewer lifecycle (`NDVViewersManager`)
- `_multi_camera_handler.py` — per-camera MDA file saving
- `_settings.py`         — pydantic-settings persistence
- `_notification_manager.py`, `_sentry.py` — UX + telemetry
- `asi_z_stack/`         — separate `microscope-control` entrypoint (ASI z-stack engine)
- `tests/`               — pytest + pytest-qt suite

## Architectural boundaries (do not cross)
- **One core, one singleton.** All hardware access goes through
  `CMMCorePlus.instance()`. Never talk to device adapters or serial ports directly.
- **Event-driven, not polling.** React to psygnal signals (`propertyChanged`,
  MDA `sequenceStarted/Finished`, etc.). Poll only read-only sensor values, and
  only while the relevant panel is visible.
- **Add features through the action registry.** New menu items / dock widgets are
  new `ActionInfo` entries (`CoreAction` or `WidgetAction`) — not ad-hoc wiring in
  `_main_window.py`.
- **Reuse the ecosystem.** Prefer existing `pymmcore-widgets` and `superqt`
  components over new custom widgets.
- **Editable local deps.** `pymmcore-widgets`, `superqt`, `crispy` resolve via
  `[tool.uv.sources]` to sibling clones; `just run` re-syncs from the lockfile, so
  edits to those clones are live. Don't hardcode paths around this.
- Keep `src`/`tests` layout; package is importable as `pymmcore_gui`.
