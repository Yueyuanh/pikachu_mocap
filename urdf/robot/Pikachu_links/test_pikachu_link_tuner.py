import importlib.util
import math
from pathlib import Path

import pytest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "pikachu_link_tuner_server", HERE / "pikachu_link_tuner_server.py"
)
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)


@pytest.mark.parametrize("name", ["robot.urdf", "run-01_export", "Pikachu links.json"])
def test_safe_name_accepts_single_path_segment(name):
    assert SERVER.LinkTunerHandler._safe_name(name) == name


@pytest.mark.parametrize("name", ["../robot.urdf", "nested/robot.urdf", r"nested\\robot.urdf", ".."])
def test_safe_name_rejects_path_traversal(name):
    with pytest.raises(ValueError):
        SERVER.LinkTunerHandler._safe_name(name)


def test_atomic_write_replaces_complete_file(tmp_path):
    dest = tmp_path / "robot.urdf"
    SERVER.LinkTunerHandler._atomic_write(dest, b"first")
    SERVER.LinkTunerHandler._atomic_write(dest, b"second")
    assert dest.read_bytes() == b"second"
    assert not list(tmp_path.glob(".tuner-*.tmp"))


def test_export_directory_is_versioned_instead_of_overwritten(tmp_path):
    handler = object.__new__(SERVER.LinkTunerHandler)
    handler.directory = str(tmp_path)
    (tmp_path / "robot_export").mkdir()
    name, path = handler._unique_output_dir("robot_export")
    assert name.startswith("robot_export-")
    assert Path(path).parent == tmp_path
    assert not Path(path).exists()


def _handler_for_payload(tmp_path, payload):
    handler = object.__new__(SERVER.LinkTunerHandler)
    handler.directory = str(tmp_path)
    handler._read_json = lambda: (payload, None)
    sent = []
    handler._send_json = lambda code, body: sent.append((code, body))
    return handler, sent


def test_batch_export_is_complete_and_non_overwriting(tmp_path):
    payload = {
        "dir": "robot_export",
        "files": [
            {"filename": "robot.urdf", "content": "<robot/>"},
            {"filename": "validation.json", "content": "{}"},
        ],
    }
    handler, sent = _handler_for_payload(tmp_path, payload)
    handler._api_save_dir()
    code, first = sent[-1]
    assert code == 200
    assert (tmp_path / first["dir"] / "robot.urdf").read_text() == "<robot/>"
    assert sorted(p.name for p in (tmp_path / first["dir"]).iterdir()) == [
        "robot.urdf",
        "validation.json",
    ]

    handler, sent = _handler_for_payload(tmp_path, payload)
    handler._api_save_dir()
    code, second = sent[-1]
    assert code == 200
    assert second["dir"] != first["dir"]
    assert (tmp_path / second["dir"] / "validation.json").read_text() == "{}"


def test_batch_export_rejects_duplicate_names_without_partial_directory(tmp_path):
    payload = {
        "dir": "broken_export",
        "files": [
            {"filename": "same.json", "content": "one"},
            {"filename": "same.json", "content": "two"},
        ],
    }
    handler, sent = _handler_for_payload(tmp_path, payload)
    handler._api_save_dir()
    code, body = sent[-1]
    assert code == 400
    assert "重名" in body["error"]
    assert not (tmp_path / "broken_export").exists()


def test_npz_arm_bias_mapping_preserves_other_joints():
    row = list(range(14))
    mapped = SERVER._map_npz_to_27(row, "v-90")
    assert mapped["left_hip_pitch_joint"] == 0
    assert mapped["left_arm_roll_joint"] == pytest.approx(11 - math.pi / 2)
    assert mapped["right_arm_roll_joint"] == pytest.approx(13 - math.pi / 2)


def test_quaternion_identity_matrix():
    assert SERVER._quat_wxyz_to_mat([1, 0, 0, 0]) == [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ]


def test_html_keeps_fixed_joint_tree_and_has_validation_gate():
    html = (HERE / "pikachu_link_tuner.html").read_text(encoding="utf-8")
    assert "const ux=a/n,uy=b/n,uz=c/n" in html
    assert "joint.setAttribute('type','fixed')" in html
    assert 'id="validationList"' in html
    assert "_validation.json" in html
