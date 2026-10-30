from __future__ import annotations

import shutil
from pathlib import Path
from typing import TYPE_CHECKING

import pytest
from pymmcore_plus import CMMCorePlus

from pymmcore_gui.asi_z_stack import config_save
from pymmcore_gui.asi_z_stack.camera_handoff import (
    CameraHandoffSnapshot,
    _CameraSnapshot,
)
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    WorkerConfigGroup,
    _detach_unloadable_config_groups,
)
from pymmcore_gui.asi_z_stack.config_save import save_system_configuration

if TYPE_CHECKING:
    from collections.abc import Iterator

RIG_CFG = Path(__file__).parents[1] / "hardware_profiles" / "20260630-OPM.cfg"


@pytest.fixture
def core(mmcore: CMMCorePlus, tmp_path: Path) -> CMMCorePlus:
    """The global core, loaded from a copy of the demo config in tmp_path."""
    mmcore.loadSystemConfiguration()
    src = tmp_path / "source.cfg"
    shutil.copy(_loaded(mmcore), src)
    mmcore.loadSystemConfiguration(src)
    return mmcore


@pytest.fixture(autouse=True)
def _reset_active() -> Iterator[None]:
    yield
    CameraWorkerService._active = None


def _loaded(core: CMMCorePlus) -> Path:
    path = core.systemConfigurationFile()
    assert path
    return Path(path)


def _activate(core: CMMCorePlus) -> CameraWorkerService:
    """Release the demo ``Camera`` the way ``begin_release`` would, and register."""
    groups = _detach_unloadable_config_groups(core, {"Camera"})
    core.unloadDevice("Camera")
    svc = CameraWorkerService()
    svc._mmc = core
    svc._snapshot = CameraHandoffSnapshot(
        n_cameras=1,
        camera_labels=("Camera",),
        core_camera_role="Camera",
        image_width=16,
        image_height=16,
        bytes_per_pixel=2,
        n_components=1,
        per_camera={"Camera": _CameraSnapshot(None, None, {})},
    )
    svc._worker_config_groups = groups
    CameraWorkerService._active = svc
    return svc


def _lines(path: Path) -> list[str]:
    return path.read_text().splitlines()


def test_no_service_is_plain_save(core: CMMCorePlus, tmp_path: Path) -> None:
    assert CameraWorkerService.get_active() is None
    ours, stock = tmp_path / "ours.cfg", tmp_path / "stock.cfg"
    save_system_configuration(core, ours)
    core.saveSystemConfiguration(str(stock))
    assert ours.read_bytes() == stock.read_bytes()


def test_active_service_restores_cameras_groups_and_role(
    core: CMMCorePlus, tmp_path: Path
) -> None:
    svc = _activate(core)
    out = tmp_path / "out.cfg"

    # The stock writer alone loses the camera, its groups and the role.
    stock = tmp_path / "stock.cfg"
    core.saveSystemConfiguration(str(stock))
    assert "Device,Camera,DemoCamera,DCam" not in _lines(stock)
    assert not any(ln.startswith("Property,Core,Camera,") for ln in _lines(stock))

    save_system_configuration(core, out)

    lines = _lines(out)
    assert "Device,Camera,DemoCamera,DCam" in lines
    assert "Property,Core,Camera,Camera" in lines
    assert lines.index("Device,Camera,DemoCamera,DCam") < lines.index(
        "Property,Core,Initialize,1"
    )
    for group in svc.worker_config_groups:
        assert any(ln.startswith(f"ConfigGroup,{group},") for ln in lines)
    assert not list(tmp_path.glob("*.tmp"))

    # The result loads in a fresh core with the camera and its group back.
    fresh = CMMCorePlus()
    fresh.loadSystemConfiguration(out)
    assert "Camera" in fresh.getLoadedDevices()
    assert fresh.getCameraDevice() == "Camera"
    assert "Camera" in fresh.getAvailableConfigGroups()
    assert set(fresh.getAvailableConfigs("Camera")) == set(
        svc.worker_config_groups["Camera"].presets
    )


def test_edited_preset_is_saved(core: CMMCorePlus, tmp_path: Path) -> None:
    svc = _activate(core)
    group = svc.worker_config_groups["Camera"]
    group.presets["MyEdit"] = (("Camera", "Binning", "2"),)
    out = tmp_path / "out.cfg"

    save_system_configuration(core, out)

    assert "ConfigGroup,Camera,MyEdit,Camera,Binning,2" in _lines(out)
    fresh = CMMCorePlus()
    fresh.loadSystemConfiguration(out)
    assert "MyEdit" in fresh.getAvailableConfigs("Camera")


def test_save_over_the_loaded_file(core: CMMCorePlus, tmp_path: Path) -> None:
    _activate(core)
    src = _loaded(core)
    assert src.parent == tmp_path

    save_system_configuration(core, src)

    assert "Device,Camera,DemoCamera,DCam" in _lines(src)
    fresh = CMMCorePlus()
    fresh.loadSystemConfiguration(src)
    assert "Camera" in fresh.getLoadedDevices()
    # Saving again (now from the spliced file) stays complete.
    save_system_configuration(core, src)
    assert _lines(src).count("Device,Camera,DemoCamera,DCam") == 1


def test_missing_source_refuses_and_leaves_destination(
    core: CMMCorePlus, tmp_path: Path
) -> None:
    _activate(core)
    _loaded(core).unlink()
    dest = tmp_path / "dest.cfg"
    dest.write_text("untouched")

    with pytest.raises(RuntimeError, match="unable to read"):
        save_system_configuration(core, dest)

    assert dest.read_text() == "untouched"
    assert not list(tmp_path.glob("*.tmp"))


def test_never_loaded_source_refuses(core: CMMCorePlus, tmp_path: Path) -> None:
    _activate(core)
    core._last_sys_config = None
    dest = tmp_path / "dest.cfg"
    with pytest.raises(RuntimeError, match="no configuration file"):
        save_system_configuration(core, dest)
    assert not dest.exists()


def test_rig_cfg_splice_restores_cameras_and_groups() -> None:
    source = RIG_CFG.read_text()
    released = {"Camera-1", "Camera-2", "Multi Camera"}
    lines = config_save._collect_released_lines(source, released)
    assert lines.devices == [
        ln for ln in source.splitlines() if ln.split(",")[:2] in _dev(released)
    ]
    assert len(lines.devices) == 3

    # Fake the stock save: no released devices, no Cameras/System groups, no role.
    stripped = [
        ln
        for ln in source.splitlines()
        if ln.split(",")[1:2] not in ([x] for x in released)
        and not ln.startswith(("ConfigGroup,Cameras", "ConfigGroup,System"))
        and not ln.startswith("Property,Core,Camera,")
    ]
    saved = "\r\n".join(stripped) + "\r\n"
    groups = {
        "Cameras": WorkerConfigGroup(
            presets={"Speed": (("Camera-1", "Port", "Speed"),)}, current="Speed"
        )
    }

    merged = config_save.splice_worker_state(saved, lines, groups, "Multi Camera")

    assert "\n" not in merged.replace("\r\n", "")  # newline style preserved
    out = merged.splitlines()
    init = out.index("Property,Core,Initialize,1")
    for dev in lines.devices:
        assert out.index(dev) < init
    assert out[-1] == "Property,Core,Camera,Multi Camera"
    assert "ConfigGroup,Cameras,Speed,Camera-1,Port,Speed" in out
    # Existing same-named groups are replaced, not duplicated.
    assert not any(ln.startswith("ConfigGroup,Cameras,Sens") for ln in out)


def _dev(labels: set[str]) -> list[list[str]]:
    return [["Device", x] for x in labels]


def test_pre_and_post_init_lines_are_collected() -> None:
    source = "\n".join(
        [
            "Property,Core,Initialize,0",
            "Device,Hub,Lib,H",
            "Device,Cam,Lib,C",
            "Property,Cam,Port,COM1",
            "Property,Hub,Other,1",
            "Parent,Cam,Hub",
            "Property,Core,Initialize,1",
            "Delay,Cam,5",
            "FocusDirection,Hub,1",
            "Label,Cam,0,zero",
            "",
        ]
    )
    lines = config_save._collect_released_lines(source, {"Cam"})
    assert lines.devices == ["Device,Cam,Lib,C"]
    assert lines.pre_init == ["Property,Cam,Port,COM1", "Parent,Cam,Hub"]
    assert lines.post_init == ["Delay,Cam,5", "Label,Cam,0,zero"]

    saved = "\n".join(
        [
            "Property,Core,Initialize,0",
            "Device,Hub,Lib,H",
            "Property,Hub,Other,1",
            "Property,Core,Initialize,1",
            "",
        ]
    )
    out = config_save.splice_worker_state(saved, lines, {}, "Cam").splitlines()
    assert out == [
        "Property,Core,Initialize,0",
        "Device,Hub,Lib,H",
        "Device,Cam,Lib,C",
        "Property,Hub,Other,1",
        "Property,Cam,Port,COM1",
        "Parent,Cam,Hub",
        "Property,Core,Initialize,1",
        "Delay,Cam,5",
        "Label,Cam,0,zero",
        "# Camera role",
        "Property,Core,Camera,Cam",
    ]


def test_splice_without_initialize_line_raises() -> None:
    with pytest.raises(RuntimeError, match="Initialize"):
        config_save.splice_worker_state(
            "Device,A,B,C\n", config_save._ReleasedLines(), {}, "Cam"
        )
