"""Real ROS pub/sub with synthetic fixtures in a unique namespace; no drivers."""
import math
import pytest
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Imu
from mech_exo_state.msg import RobotState
from state_node import StateNode


@pytest.mark.parametrize('confirmed', [False, True])
def test_ros_graph_and_dropout(monkeypatch, confirmed):
    # Scalar environment values must be replaced, not PATH-style appended.
    monkeypatch.setenv('ROS_DOMAIN_ID', '176')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    ns = '/state_test_' + uuid.uuid4().hex
    rclpy.init(args=['--ros-args', '-r', '__ns:=' + ns,
                    '-p', 'mapping_confirmed:=' + str(confirmed).lower(),
                    '-p', 'joint_topic:=' + ns + '/joints',
                    '-p', 'imu1_topic:=' + ns + '/imu1',
                    '-p', 'imu2_topic:=' + ns + '/imu2'])
    state = fixture = executor = None
    try:
        state, fixture = StateNode(), Node('fixture')
        assert not state.set_parameters([Parameter('mapping_confirmed', value=True)])[0].successful
        executor = SingleThreadedExecutor()
        executor.add_node(state)
        executor.add_node(fixture)
        seen = []
        fixture.create_subscription(RobotState, 'ph_exo/state', seen.append, qos_profile_sensor_data)
        joints = fixture.create_publisher(JointState, ns + '/joints', qos_profile_sensor_data)
        imus = [fixture.create_publisher(Imu, ns + '/imu' + str(i), qos_profile_sensor_data) for i in (1, 2)]

        def publish(with_imu):
            stamp = fixture.get_clock().now().to_msg()
            js = JointState()
            js.header.stamp = stamp
            js.name, js.position = ['motor105_joint', 'motor104_joint'], [1.0, 1.2]
            joints.publish(js)
            if with_imu:
                for i, publisher in enumerate(imus):
                    msg = Imu()
                    msg.header.stamp, msg.header.frame_id = stamp, 'imu_' + str(i+1) + '_link'
                    msg.orientation.w = 1.
                    publisher.publish(msg)

        # First obtain FK using motor fixtures alone; no IMU publishers send yet.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            publish(False)
            executor.spin_once(timeout_sec=.01)
            if seen and seen[-1].sources[0].valid:
                break
        assert seen and seen[-1].sources[0].valid
        assert not seen[-1].messages_aligned
        assert seen[-1].q_valid == confirmed
        if confirmed:
            expected = (.1, .3193405910490444)  # frozen reference for these motor fixtures
            assert max(abs(x-y) for x,y in zip(seen[-1].q_position,expected)) < 1e-8
        else:
            assert all(math.isnan(x) for x in seen[-1].q_position)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            publish(True)
            executor.spin_once(timeout_sec=.01)
            if seen and seen[-1].messages_aligned:
                break
        assert seen and seen[-1].messages_aligned
        valid = seen[-1]
        assert list(valid.joints.name) == ['motor104_joint', 'motor105_joint']
        assert valid.alpha_valid == confirmed and not valid.alpha_velocity_valid
        assert valid.q_valid == confirmed
        assert not valid.hardware_freshness_verified
        assert abs(valid.alpha_position[0] - (1.0 - 3.141592653589793/4)) < 1e-12
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            publish(False)
            executor.spin_once(timeout_sec=.01)
        assert seen[-1].sources[0].valid
        assert not seen[-1].sources[1].valid and not seen[-1].sources[2].valid
        assert not seen[-1].messages_aligned
        assert not any(seen[-1].imu_orientation_valid)
        assert seen[-1].q_valid == confirmed  # IMU dropout never gates FK.
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)  # stop motor source as well
        assert not seen[-1].q_valid
        assert all(math.isnan(x) for x in seen[-1].q_position)
        assert seen[-1].q_reason == 'joint_source_invalid'
    finally:
        if executor:
            executor.shutdown()
        if state:
            state.destroy_node()
        if fixture:
            fixture.destroy_node()
        rclpy.shutdown()


def test_mit_ros_feedback_and_independent_timeout(monkeypatch):
    from control_msgs.msg import DynamicJointState, InterfaceValue
    from mit_feedback import FIELDS
    monkeypatch.setenv('ROS_DOMAIN_ID', '176')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    ns = '/mit_state_test_' + uuid.uuid4().hex
    rclpy.init(args=['--ros-args', '-r', '__ns:=' + ns,
                     '-p', 'motor_source:=mit', '-p', 'mit_topic:=' + ns + '/feedback'])
    state, fixture = StateNode(), Node('feedback_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(state)
    executor.add_node(fixture)
    seen = []
    fixture.create_subscription(RobotState, 'ph_exo/state', seen.append, qos_profile_sensor_data)
    pub = fixture.create_publisher(DynamicJointState, ns + '/feedback', qos_profile_sensor_data)
    seq = 0
    def sample(ids, fault=0):
        nonlocal seq
        seq += 1
        msg = DynamicJointState()
        for mid in ids:
            msg.joint_names.append(f'mit_comm_motor{mid}')
            code = fault if mid == 105 else 0
            row = InterfaceValue()
            row.interface_names = list(FIELDS)
            row.values = [float(v) for v in (mid, seq, time.monotonic_ns()//1000, int(code==0),
                          72.0 if mid==104 else 81.5, -120, -.15, 41., code)]
            msg.interface_values.append(row)
        pub.publish(msg)

    def until(ids, fault, condition):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            sample(ids, fault)
            executor.spin_once(timeout_sec=.005)
            if seen and condition(seen[-1]):
                return seen[-1]
        pytest.fail('MIT feedback condition timed out')
    try:
        good = until([105,104], 0, lambda msg: msg.sources[0].valid)
        assert [a.drive_id for a in good.mit_motors] == [104,105]
        assert [a.position_deg for a in good.mit_motors] == [72.,81.5]
        assert list(good.joints.position) == pytest.approx([math.radians(72.),math.radians(81.5)])
        assert not good.joints.velocity and not good.joints.effort and not good.q_valid
        assert good.mit_motors[1].iq_a == -.15 and good.mit_motors[0].erpm == -120.
        faulted = until([104,105], 3, lambda msg: msg.mit_motors[1].fault_code == 3)
        assert faulted.mit_motors[0].valid and not faulted.mit_motors[1].valid
        assert not faulted.sources[0].valid and faulted.mit_motors[1].temperature_c == 41.
        expired = until([105], 0, lambda msg: msg.mit_motors[0].reason == 'feedback_timeout'
                        and msg.mit_motors[1].valid)
        assert not expired.sources[0].valid and expired.mit_motors[0].position_deg == 72.
    finally:
        executor.shutdown()
        state.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()
