"""DDS startup staging tests; no host /etc or ROS processes are touched."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "docker/scripts/configure-dds.sh"
PROFILE = ROOT / "docker/middleware_profiles/fastdds.xml"
NS = {"dds": "http://www.eprosima.com"}


class FastDdsProfileTests(unittest.TestCase):
    def test_transport_and_all_locators_are_loopback_unicast(self):
        root = ET.parse(PROFILE).getroot()
        transports = root.findall("dds:profiles/dds:transport_descriptors/dds:transport_descriptor", NS)
        self.assertEqual(len(transports), 1)
        transport = transports[0]
        self.assertEqual(transport.findtext("dds:type", namespaces=NS), "UDPv4")
        self.assertEqual(
            [node.text for node in transport.findall("dds:interfaceWhiteList/dds:address", NS)],
            ["127.0.0.1"],
        )
        participant = root.find("dds:profiles/dds:participant", NS)
        self.assertEqual(participant.attrib["is_default_profile"], "true")
        rtps = participant.find("dds:rtps", NS)
        self.assertEqual(rtps.findtext("dds:useBuiltinTransports", namespaces=NS), "false")
        self.assertEqual(
            rtps.findtext("dds:userTransports/dds:transport_id", namespaces=NS),
            transport.findtext("dds:transport_id", namespaces=NS),
        )
        for path in (
            "dds:defaultUnicastLocatorList",
            "dds:builtin/dds:metatrafficUnicastLocatorList",
            "dds:builtin/dds:initialPeersList",
        ):
            locators = rtps.findall(path + "/dds:locator", NS)
            self.assertEqual(len(locators), 1, path)
            self.assertEqual(locators[0].findtext("dds:udpv4/dds:address", namespaces=NS), "127.0.0.1")
            self.assertEqual([node.tag for node in locators[0]], ["{http://www.eprosima.com}udpv4"])
        for path in ("dds:defaultMulticastLocatorList", "dds:builtin/dds:metatrafficMulticastLocatorList"):
            # XSD requires a locator inside these elements. Omit them and use
            # explicit metatraffic unicast, as in the official multicast guide.
            self.assertIsNone(rtps.find(path, NS))
        self.assertEqual(
            rtps.findtext("dds:builtin/dds:discovery_config/dds:discoveryProtocol", namespaces=NS),
            "SIMPLE",
        )

    @unittest.skipUnless(os.environ.get("ATD_FASTDDS_XSD"), "Set ATD_FASTDDS_XSD to an official Fast DDS 3.x schema")
    def test_official_fastdds_schema(self):
        validator = shutil.which("xmllint")
        if not validator:
            self.skipTest("xmllint is not installed")
        result = subprocess.run(
            [validator, "--noout", "--nonet", "--schema", os.environ["ATD_FASTDDS_XSD"], str(PROFILE)],
            text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


class ConfigureDdsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.config = self.base / "config with spaces"
        self.bin = self.base / "bin"
        self.bin.mkdir()
        self.fake_id = self.bin / "id"
        self.fake_id.write_text("#!/bin/sh\nprintf '%s\\n' \"${ATD_TEST_UID:-0}\"\n")
        self.fake_id.chmod(0o755)
        self.env = os.environ.copy()
        self.env.update({
            "PATH": str(self.bin) + os.pathsep + self.env["PATH"],
            "ATD_DDS_CONFIG_DIR": str(self.config),
        })

    def invoke(self, **extra):
        return subprocess.run(["bash", str(SCRIPT)], env={**self.env, **extra}, text=True, capture_output=True)

    def test_stages_configuration_and_clears_conflicting_environment(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        profile = self.config / "fastdds.xml"
        self.assertEqual(profile.read_bytes(), PROFILE.read_bytes())
        env_file = self.config / "dds.env.sh"
        script = r'''source "$1"
python3 - <<'CHECK'
import json, os
keys = ['RMW_IMPLEMENTATION', 'FASTDDS_DEFAULT_PROFILES_FILE',
        'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
        'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_DISCOVERY_SERVER',
        'FASTDDS_ENVIRONMENT_FILE', 'ROS_LOCALHOST_ONLY', 'SKIP_DEFAULT_XML']
print(json.dumps({key: os.environ.get(key) for key in keys}))
CHECK
'''
        polluted = {key: "external" for key in (
            "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "ROS_DISCOVERY_SERVER",
            "FASTDDS_ENVIRONMENT_FILE", "ROS_LOCALHOST_ONLY", "SKIP_DEFAULT_XML",
        )}
        process = subprocess.run(
            ["bash", "-c", script, "dds-test", str(env_file)],
            env={**self.env, **polluted, "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                 "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET", "ROS_STATIC_PEERS": "10.0.0.2"},
            text=True, capture_output=True,
        )
        self.assertEqual(process.returncode, 0, process.stderr)
        values = json.loads(process.stdout)
        self.assertEqual(values["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
        self.assertEqual(Path(values["FASTDDS_DEFAULT_PROFILES_FILE"]), profile.resolve())
        self.assertEqual(values["ROS_AUTOMATIC_DISCOVERY_RANGE"], "SYSTEM_DEFAULT")
        self.assertEqual(values["ROS_STATIC_PEERS"], "")
        self.assertEqual(values["RMW_FASTRTPS_USE_QOS_FROM_XML"], "0")
        for key in polluted:
            self.assertIsNone(values[key], key)
        self.assertEqual(profile.stat().st_mode & 0o777, 0o644)
        self.assertEqual(env_file.stat().st_mode & 0o777, 0o644)

    def test_unchanged_start_keeps_existing_files_and_source_update_is_applied(self):
        self.assertEqual(self.invoke().returncode, 0)
        paths = [self.config / "fastdds.xml", self.config / "dds.env.sh"]
        before = [path.stat().st_mtime_ns for path in paths]
        self.assertEqual(self.invoke().returncode, 0)
        self.assertEqual([path.stat().st_mtime_ns for path in paths], before)
        updated = self.base / "updated.xml"
        updated.write_text(PROFILE.read_text().replace("<!-- DDS", "<!-- Updated DDS", 1))
        result = self.invoke(ATD_DDS_PROFILE_SOURCE=str(updated))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(paths[0].read_bytes(), updated.read_bytes())
        self.assertEqual(list(self.config.glob(".*dds*")), [])

    def test_non_root_and_missing_profile_fail_before_configuration(self):
        result = self.invoke(ATD_TEST_UID="1000")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("root", result.stderr)
        self.assertFalse(self.config.exists())
        result = self.invoke(ATD_DDS_PROFILE_SOURCE=str(self.base / "missing.xml"))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.config.exists())


if __name__ == "__main__":
    unittest.main()
