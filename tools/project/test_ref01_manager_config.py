"""Pure Windows-safe tests for generated software manager fixtures."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from test_ref01_manager import build_configuration, JOINTS, COMMANDS, STATES, PLUGIN, TYPE


class ConfigurationTests(unittest.TestCase):
    def fixture(self, **changes):
        values=dict(model=Path.cwd()/"model & one.xml", output=Path.cwd()/"output",
                    case="psi", duration=30., namespace="/ph_exo_manager_test")
        values.update(changes)
        return build_configuration(**values)

    def test_only_simulation_hardware_and_exact_interfaces(self):
        xml, params = self.fixture()
        robot=ET.fromstring(xml)
        self.assertEqual([n.text for n in robot.findall(".//hardware/plugin")],[PLUGIN])
        self.assertEqual(robot.find(".//hardware/param[@name='simulation_only']").text,"true")
        self.assertEqual(robot.find(".//hardware/param[@name='max_samples']").text,"6001")
        for joint in robot.findall(".//ros2_control/joint"):
            self.assertIn(joint.get("name"),JOINTS)
            self.assertEqual({i.get("name") for i in joint.findall("command_interface")},set(COMMANDS))
            self.assertEqual({i.get("name") for i in joint.findall("state_interface")},set(STATES))
        self.assertNotIn("tty",xml)
        self.assertNotIn("socketcan",xml.lower())
        self.assertEqual(params["/ph_exo_manager_test/ref01_impedance"]["ros__parameters"]["simulation_only"],True)

    def test_manager_uses_wall_clock_200hz(self):
        _, params=self.fixture()
        actual=json.loads(json.dumps(params))
        manager=actual["/ph_exo_manager_test/controller_manager"]["ros__parameters"]
        self.assertEqual(manager["update_rate"],200)
        self.assertIs(manager["use_sim_time"],False)
        self.assertIs(manager["lock_memory"],False)
        self.assertEqual(manager["ref01_impedance"]["type"],TYPE)
        self.assertNotIn("cpu_affinity",manager)

    def test_escapes_model_path(self):
        xml,_=self.fixture()
        self.assertEqual(ET.fromstring(xml).find(".//hardware/param[@name='model_path']").text,
                         str(Path.cwd()/"model & one.xml"))

    def test_rejects_bad_cases_or_durations(self):
        for update in (dict(case="motor"),dict(duration=float("nan")),dict(duration=0),dict(duration=121),dict(duration=2.5025)):
            with self.subTest(update=update), self.assertRaises(ValueError): self.fixture(**update)

    def test_rejects_unscoped_namespaces(self):
        for namespace in ("controller_manager","/robot","/ph_exo_manager_bad/child"):
            with self.subTest(namespace=namespace),self.assertRaises(ValueError): self.fixture(namespace=namespace)

    def test_rejects_relative_paths(self):
        with self.assertRaises(ValueError): self.fixture(model="model.xml")
        with self.assertRaises(ValueError): self.fixture(output="output")


if __name__=="__main__": unittest.main()
