"""Training selection and opt-in output contract, without device access."""
from pathlib import Path
import pytest
import yaml
from mech_assist_control.robot.policies import make_policy, ImpedanceTrainingPolicy


def test_canonical_training_preview_selects_impedance_and_keeps_motor_output_off():
    root = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((root/'config/dm4310_impedance_training_preview.yaml').read_text())['exoskeleton_control']['ros__parameters']
    assert params['policy'] == 'impedance_training'
    assert params['output_mode'] == 'preview'
    assert not params['allow_motor_output'] and not params['joint_mapping_verified']
    assert all(topic.endswith('/training_command') for topic in params['controller_topics'])
    assert params['max_command'] == [7., 7.]
    assert params['run_limit_s'] == 5.
    assert 'dm4310_impedance_training_preview.yaml' in (root/'launch/motor_training_preview.launch.py').read_text()
    assert isinstance(make_policy('impedance_training'), ImpedanceTrainingPolicy)
    assert make_policy('resistance_training').name == 'resistance_training'
