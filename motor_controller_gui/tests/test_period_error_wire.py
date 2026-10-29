import unittest

from protocol import parse_error_byte, ProtocolVersion


class PeriodErrorWireTest(unittest.TestCase):
    def test_v2_protection_subcode(self):
        info = parse_error_byte(ProtocolVersion.V2_0_0, 0x10)
        self.assertFalse(info["error_is_iap"])
        self.assertEqual(info["error_subcode"], 0x10)
        self.assertIn("超速", info["error_name"])

    def test_v2_gate_nfault(self):
        info = parse_error_byte(ProtocolVersion.V2_0_0, 0x1D)
        self.assertIn("nFAULT", info["error_name"])

    def test_iap_flag(self):
        info = parse_error_byte(ProtocolVersion.V2_0_0, 0x81)
        self.assertTrue(info["error_is_iap"])
        self.assertEqual(info["error_subcode"], 0x01)


if __name__ == "__main__":
    unittest.main()
