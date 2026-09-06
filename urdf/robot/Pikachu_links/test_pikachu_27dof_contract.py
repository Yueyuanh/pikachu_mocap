import re
from pathlib import Path


HERE = Path(__file__).resolve().parent
XACRO = HERE / "default" / "27dof" / "pikachu_sample_links_27dof.xacro"
TUNER = HERE / "pikachu_link_tuner.html"

EXPECTED_MEASUREMENTS = {
    "measure_ear_root_height": 0.449,
    "measure_ear_root_half_width": 0.116,
    "measure_head_pitch_height": 0.219,
    "measure_shoulder_half_width": 0.116,
    "measure_arm_plane_offset_x": 0.083,
    "measure_upper_arm_length": 0.100,
    "measure_forearm_length": 0.066,
    "measure_hip_roll_half_width": 0.100,
    "measure_torso_length": 0.166,
    "measure_thigh_length": 0.083,
    "measure_shank_length": 0.086,
    "measure_foot_length_x": 0.093,
    "measure_tail_axis_height_from_ankle": 0.166,
    "measure_tail_center_offset_x": 0.143,
}


def _properties(text):
    return dict(re.findall(r'<xacro:property\s+name="([\w]+)"\s+value="([^"]+)"\s*/>', text))


def test_27dof_has_one_complete_measurement_profile():
    text = XACRO.read_text(encoding="utf-8")
    props = _properties(text)
    assert props["tune_profile"] == "pikachu_27dof_v1"
    assert float(props["tune_measurement_tolerance"]) == 0.0005
    assert {name: float(props[name]) for name in EXPECTED_MEASUREMENTS} == EXPECTED_MEASUREMENTS
    assert {name for name in props if name.startswith("measure_")} == set(EXPECTED_MEASUREMENTS)


def test_measured_dimensions_drive_derived_geometry():
    text = XACRO.read_text(encoding="utf-8")
    props = _properties(text)
    expected_aliases = {
        "shoulder_width": "${measure_shoulder_half_width}",
        "arm_forward_offset": "${measure_arm_plane_offset_x}",
        "upper_arm_len": "${measure_upper_arm_length}",
        "lower_arm_len": "${measure_forearm_length}",
        "torso_len": "${measure_torso_length}",
        "thigh_len": "${measure_thigh_length}",
        "shank_len": "${measure_shank_length}",
        "foot_len": "${measure_foot_length_x}",
        "hip_width": "${measure_hip_roll_half_width}",
        "head_height": "${measure_head_pitch_height}",
        "ear_height": "${measure_ear_root_height}",
    }
    assert {name: props[name] for name in expected_aliases} == expected_aliases
    assert props["ear_off_y"] == "${measure_ear_root_half_width}"
    assert "-thigh_len - shank_len + measure_tail_axis_height_from_ankle" in props["tail_root_z"]
    assert "-measure_tail_center_offset_x + tail_len/2.0" in props["tail_root_x"]


def test_shoulder_axes_are_coincident_and_tail_visual_matches_collision():
    text = XACRO.read_text(encoding="utf-8")
    assert '<origin xyz="0 0 0" rpy="${sn*1.5707963267948966} 0 0"/>' in text
    assert '<origin xyz="${tail_root_x} 0 ${tail_root_z}" rpy="0 0 0"/>' in text
    tail = text.split('<link name="tail_yaw_link">', 1)[1].split('</link>', 1)[0]
    assert "<mesh " not in tail
    assert tail.count('<box size="${tail_len} ${tail_wid} ${tail_thick}"/>') == 2
    assert tail.count('<origin xyz="${-tail_len/2.0} 0 0" rpy="0 0 0"/>') == 2


def test_tuner_implements_all_measurements_and_delivery_gate():
    html = TUNER.read_text(encoding="utf-8")
    assert "pikachu_27dof_v1" in html
    for index, prop in enumerate(EXPECTED_MEASUREMENTS, start=1):
        assert f"M{index:02d}" in html
        assert f"targetProp:'{prop}'" in html
    assert "modelMeasurements(computeTFs(Model,{}))" in html
    assert "MEASUREMENT_TOLERANCE" in html
    assert "MEASUREMENT_INCOMPLETE" in html
