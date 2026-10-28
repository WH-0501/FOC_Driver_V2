"""Versioned electrical verification payloads and per-run integrity checks."""
import struct
import math


def parse_verify_sample(payload):
    if len(payload) < 32 or payload[0] != 1 or payload[1] > 2:
        return None
    version, phase, flags, reserved, session, seq, ms, target, measured, error, mech, expected = struct.unpack_from(
        '>BBBBHHIiiiii', payload)
    return dict(session=session, sequence=seq, time_s=ms / 1000,
                phase=phase, settled=bool(flags & 1),
                target=target / 1e6, measured=measured / 1e6,
                error=error / 1e6, mechanical=mech / 1e6,
                expected_mechanical=expected / 1e6)


def parse_verify_result(payload):
    if len(payload) < 12 or payload[0] != 1:
        return None
    version, error, session, count, reserved, maximum = struct.unpack_from('>BBHHHi', payload)
    return dict(session=session, error=error, count=count, max_error=maximum / 1e6)


class VerificationRun:
    def __init__(self, session):
        self.session = session
        self.samples = []
        self.gaps = 0
        self.invalid = False
        self.result = None

    def accept(self, sample):
        if self.result is not None or sample['session'] != self.session:
            return False
        expected = self.samples[-1]['sequence'] + 1 if self.samples else 0
        if sample['sequence'] < expected:
            return False  # stale or duplicate; do not draw an old point
        if len(self.samples) >= 1000:
            self.invalid = True
            return False
        self.gaps += sample['sequence'] - expected
        if self.samples and sample['time_s'] < self.samples[-1]['time_s']:
            self.invalid = True
        self.samples.append(sample)
        return True

    def finish(self, result):
        if result['session'] != self.session or self.result is not None:
            return False
        self.result = result
        return True

    @property
    def complete(self):
        return (self.result is not None and not self.invalid and self.gaps == 0
                and len(self.samples) == self.result['count']
                and sum(s['settled'] for s in self.samples) == 9
                and bool(self.samples) and self.samples[-1]['time_s'] >= 7.0)

    @property
    def passed(self):
        return self.complete and self.result['error'] == 0

    def degrees(self, key):
        return [math.degrees(s[key]) for s in self.samples]
