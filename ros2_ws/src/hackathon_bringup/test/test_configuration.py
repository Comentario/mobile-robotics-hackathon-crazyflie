"""Configuration: required files and overlay precedence."""
import pytest
from hackathon_bringup.stack import config_file, param_files, shared_frames


def test_missing_config_has_explicit_error(tmp_path):
    with pytest.raises(ValueError, match='Configuration file does not exist'):
        param_files(str(tmp_path), 'missing.yaml', '')


def test_absolute_paths_and_overlay_precedence(tmp_path):
    base, overlay = tmp_path/'base.yaml', tmp_path/'overlay.yaml'
    base.write_text('/**:\n  ros__parameters:\n    base_frame: base_link\n')
    overlay.write_text('/**:\n  ros__parameters:\n    base_frame: drone_body\n')
    paths = param_files(str(tmp_path), str(base), 'overlay.yaml')
    assert paths == [str(base), str(overlay)]
    assert shared_frames(paths) == 'drone_body'
    assert config_file(str(tmp_path), 'base.yaml') == str(base)
