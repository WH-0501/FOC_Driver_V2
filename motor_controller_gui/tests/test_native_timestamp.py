import unittest
import numpy as np
from canbus.zlg_native import ZlgCanSession
from core.waveform_buffer import WaveformBuffer

class TimestampTests(unittest.TestCase):
    def test_large_uint64_offset_keeps_millisecond_resolution(self):
        origin = (1 << 64) - 100000000
        raw = [origin + i*1000 for i in range(6001)]
        old = np.diff([x/1000000.0 for x in raw])
        self.assertGreater(np.count_nonzero(old == 0), 4000)
        session = ZlgCanSession()
        b = WaveformBuffer(7000)
        for stamp in raw:
            b.append_sample({}, session._timestamp_seconds(stamp))
        np.testing.assert_allclose(np.diff(b.times), .001, atol=1e-12)
        self.assertEqual(b.timestamp_fallbacks, 0)
        self.assertAlmostEqual(b.latest_time, 6)
        self.assertEqual(session.timestamp_origin_us, origin)
        self.assertEqual(session.last_timestamp_us, raw[-1])

    def test_zero_and_backward_stamps_are_not_masked(self):
        s = ZlgCanSession()
        self.assertEqual(s._timestamp_seconds(0), 0)
        self.assertEqual(s._timestamp_seconds(1000), .001)
        self.assertEqual(s._timestamp_seconds(0), 0)
