"""
    Tests for the C implementation of the DCC protocol
    https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1_electrical_standards_for_digital_command_control.pdf
    https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-92-2004-07.pdf
"""

import ctypes
import random
from enum import IntEnum
from pathlib import Path
from random import random as rand
from random import randrange, seed
from unittest import TestCase, main

from matplotlib import pyplot as plt

__all__ = []

LIBNAME = Path(__file__).resolve().parents[1] / "dcc.so"  # TODO: dynamic?

# =============================================================================
# REPLICATE dcc.h TYPES
# =============================================================================

DCC_BUF_LEN = 64  # TODO: coordinate with dcc.h


class dcc_packet_t(ctypes.Structure):
    """ctypes implementation of dcc_packet_t in dcc.h"""
    _fields_ = [
        ("address", ctypes.c_uint8),
        ("instruction", ctypes.c_uint8),
        ("ERROR_detection", ctypes.c_uint8),
    ]


class dcc_state_t:
    """
        ctypes implementation of dcc_state_t in dcc.h
        https://v4.chriskrycho.com/2015/ctypes-structures-and-dll-exports.html
    """
    AWAITING_START_BIT = 0
    VALIDATING_PREAMBLE = 1
    AWAITING_DATA_BYTES = 2
    DECODING_PACKET = 3
    PACKET_RECEIVED = 4
    ERROR = 5

    @classmethod
    def from_param(cls, obj):
        return int(obj)


class dcc_receiver_t(ctypes.Structure):
    """ctypes implementation of dcc_receiver_t in dcc.h"""
    _fields_ = [
        ("packet", dcc_packet_t),
        ("state", ctypes.c_int),
        ("buffer", ctypes.c_uint32 * DCC_BUF_LEN),
        ("w_idx", ctypes.c_uint8),
        ("r_idx", ctypes.c_uint8),
    ]

# =============================================================================
# HELPERS
# =============================================================================


def plot_buffer(dcc_receiver_t):
    "Plot dcc rx buffer contents. returns fig"
    f, ax = plt.subplots()
    edges = sorted(v for v in dcc_receiver_t.buffer)
    return plot_edges(edges)


def plot_edges(edges, ax=None):
    if ax is None:
        f, ax = plt.subplots()
    values = [i % 2 for i in range(len(edges) - 1)]
    ax.stairs(values, edges)
    return ax


# =============================================================================
# TESTING
# TODO: debug TR1D false detections
# =============================================================================


class MySuper(TestCase):
    "Convenience superclass for DCC setup & testing"

    def setUp(self):
        self.lib = ctypes.CDLL(LIBNAME)
        for attr in ("TR1_MIN", "TR1_MAX", "TR1D", "TR0_MIN", "TR0_MAX", "DCC_BUF_LEN"):
            setattr(self, attr, getattr(self.lib, f"get_{attr}")())
        seed(0)

    def freshdev(self):
        "create a zero-init'd dcc receiver object and ptr"
        dev = dcc_receiver_t()
        devptr = ctypes.pointer(dev)
        self.lib.init_decoder(ctypes.pointer(dev))
        return dev, devptr

    def pushbit(self, dev, bit, kind="ideal"):
        "push bit onto buffer. kind options: ideal, noise, corrupt"
        assert bit == 1 or bit == 0
        assert kind in ("ideal", "noise", "corrupt")

        if bit == 0:
            lo = self.TR0_MIN
            hi = self.TR0_MAX
        else:
            lo = self.TR1_MIN
            hi = self.TR1_MAX

        if kind == "ideal":
            dt1 = (lo + hi) // 2
            dt2 = dt1
        else:
            dt1 = randrange(lo, hi)
            if bit == 0:
                dt2 = randrange(lo, hi)
            else:
                dt2 = randrange(
                    *sorted(
                        [max(lo, dt1 - self.TR1D), min(hi, dt1 + self.TR1D)]
                    )
                )

            if kind == "corrupt":
                r = rand()
                # if r < 0.6:  # corrupt single half bit
                if rand() < 0.5:  # corrupt low/high
                    dt = randrange(1, lo - 1)
                else:
                    if bit == 0:
                        dt = randrange(hi + 1, hi * 10)
                    else:
                        dt = randrange(self.TR1_MAX + 1, self.TR0_MIN - 1)
                if rand() < 0.5:  # select bit to corrupt
                    dt1 = dt
                else:
                    dt2 = dt
                # TODO: why does tr1d corruption not work?
                # else:  # corrupt both
                #     if bit == 0:
                #         if rand() < 0.5:  # first bit corrupt low/high
                #             dt1 = randrange(1, lo - 1)
                #         else:
                #             dt1 = randrange(hi + 1, 10 * hi)
                #         if rand() < 0.5:  # second bit corrupt low/high
                #             dt2 = randrange(1, lo - 1)
                #         else:
                #             dt2 = randrange(hi + 1, 10 * hi)
                #     else:
                #         # TODO: corrupt both widths
                #         # if rand() < 0.5:
                #         # corrupt width diff
                #         # if rand() < 0.5:
                #         dt2 = randrange(1, lo // 2)
                #         # else:
                #         # dt2 = randrange(dt1 + self.TR1D + 1, 10 * hi)
                #         # else: # corrupt
                #         #     pass

        tbase = dev.buffer[(dev.w_idx - 1) % self.DCC_BUF_LEN]
        if dev.w_idx == 0 and tbase == 0:  # nothing pushed yet
            tbase = 0
            self.lib.push_timestamp(ctypes.pointer(dev), tbase)
        self.lib.push_timestamp(ctypes.pointer(dev), tbase + dt1)
        self.lib.push_timestamp(ctypes.pointer(dev), tbase + dt1 + dt2)

    def assertState(self, device, state, nb=5, na=1, idx=None):
        """Test if device.state == state. If not, print nb:na elements in buffer
            around idx.

            INPUTS:
                device -- dcc_receiver_t
                state -- dcc_state_t.state
                nb -- number elements prior to device.w_idx to print, default:5
                nb -- number elements following device.w_idx to print, default:1
                idx -- index to print buffer around
        """
        assert nb >= 0, f"nb must be >= 0, not {nb}"
        assert na >= 0, f"na must be >= 0, not {na}"
        if idx is None:
            idx = device.w_idx
        msg = [
            device.buffer[(idx + i) % self.DCC_BUF_LEN]
            for i in range(-nb, na + 1)
        ]
        msg = str(msg) + f" idx={idx}"
        return self.assertEqual(device.state, state, msg=msg)


class TestParseBit(MySuper):
    "Explicitly test timing correctness of C-code parsebit"

    def _3timestamps(self, t1, t2, t3):
        """push & parse a single bit represented by t1<t2<t3"""
        dev, devptr = self.freshdev()
        for t in [t1, t2, t3]:
            self.lib.push_timestamp(devptr, t)
        idx = dev.w_idx - 3
        return self.lib.parse_bit(devptr, idx)

    def test_1_typ(self):
        dt = (self.TR1_MAX + self.TR1_MIN) // 2
        self.assertEqual(self._3timestamps(0, dt, dt * 2), 1)

    def test_1_shortest(self):
        dt = self.TR1_MIN
        self.assertEqual(self._3timestamps(0, dt, dt * 2), 1)

    def test_1_longest(self):
        dt = self.TR1_MAX
        self.assertEqual(self._3timestamps(0, dt, dt * 2), 1)

    def test_1_max_diff(self):
        dt = (self.TR1_MAX + self.TR1_MIN) // 2
        self.assertEqual(self._3timestamps(0, dt, dt * 2 + self.TR1D), 1)

    def test_0_typ(self):
        dt = (self.TR0_MAX + self.TR0_MIN) // 2
        self.assertEqual(self._3timestamps(0, dt, dt * 2), 0)

    def test_0_stretched(self):
        t1 = (self.TR0_MAX + self.TR0_MIN) // 2
        t2 = 11000  # spec says max 12000
        self.assertEqual(self._3timestamps(0, t1, t2), 0)

    def test_1_first_half_too_short(self):
        self.assertEqual(self._3timestamps(
            0, self.TR1_MIN - 1, self.TR1_MIN * 2 - 1), -1)

    def test_1_second_half_too_short(self):
        self.assertEqual(self._3timestamps(
            0, self.TR1_MIN, self.TR1_MIN * 2 - 1), -1)

    def test_1_too_diff(self):
        self.assertEqual(self._3timestamps(
            0, self.TR1_MIN, self.TR1_MIN * 2 + self.TR1D + 1), -1)

    def test_1_first_half_too_long(self):
        self.assertEqual(self._3timestamps(
            0, self.TR1_MAX + 1, self.TR1_MAX * 2 + 1), -1)

    def test_1_second_half_too_long(self):
        self.assertEqual(self._3timestamps(
            0, self.TR1_MIN, self.TR1_MIN + self.TR1_MAX + 1), -1)

    def test_0_first_half_too_short(self):
        self.assertEqual(self._3timestamps(
            0, self.TR0_MIN - 1, self.TR0_MIN * 2 - 1), -1)

    def test_0_second_half_too_short(self):
        self.assertEqual(self._3timestamps(
            0, self.TR0_MIN, self.TR0_MIN * 2 - 1), -1)

    def test_0_first_half_too_long(self):
        self.assertEqual(self._3timestamps(
            0, self.TR0_MAX + 1, self.TR0_MAX * 2 + 1), -1)

    def test_0_second_half_too_long(self):
        self.assertEqual(self._3timestamps(
            0, self.TR0_MIN, self.TR0_MIN + self.TR0_MAX + 1), -1)


class TestPushBit(MySuper):
    """
        Test `MySuper.pushbit` without making sublcasses repeat the tests.

    In conjunction with `TestParseBit`, this ensures that the timing 
    characteristics are aligned in our C and Python implementations.
    """

    def test_pushbit_ideal(self):
        dev, devptr = self.freshdev()

        self.pushbit(dev, 1)
        idx = (dev.w_idx - 3) % self.DCC_BUF_LEN
        self.assertEqual(1, self.lib.parse_bit(devptr, idx), msg=dev.buffer)
        self.assertState(dev, dcc_state_t.AWAITING_START_BIT)

        self.pushbit(dev, 0)
        idx = (dev.w_idx - 3) % self.DCC_BUF_LEN
        self.assertEqual(0, self.lib.parse_bit(devptr, idx), msg=dev.buffer)
        self.assertState(dev, dcc_state_t.VALIDATING_PREAMBLE)

    def test_pushbit_noise(self):
        for bit in (0, 1):
            for _ in range(1000):

                dev, devptr = self.freshdev()
                self.pushbit(dev, bit, kind="noise")

                idx = (dev.w_idx - 3) % self.DCC_BUF_LEN
                buf = [dev.buffer[idx + i] for i in range(3)]
                self.assertEqual(bit, self.lib.parse_bit(devptr, idx), msg=buf)

                if bit == 1:
                    self.assertState(dev, dcc_state_t.AWAITING_START_BIT)
                else:
                    self.assertState(dev, dcc_state_t.VALIDATING_PREAMBLE)

    def test_pushbit_corrupt(self):
        for bit in (0, 1):
            for _ in range(1000):
                dev, devptr = self.freshdev()
                self.pushbit(dev, bit, kind="corrupt")
                idx = (dev.w_idx - 3) % self.DCC_BUF_LEN
                buf = [dev.buffer[idx + i] for i in range(3)]
                self.assertEqual(-1, self.lib.parse_bit(devptr, idx), msg=buf)
                self.assertState(dev, dcc_state_t.AWAITING_START_BIT)

    def test_monte_carlo(self):
        "test decoding of many randomly-generated, possibly-corrupted bits"
        dev, devptr = self.freshdev()
        for i in range(1000):
            # generate possibly-corrupted bit
            bit = 1 if rand() < 0.5 else 0
            expected_value = bit
            kind = "noise"
            if rand() < 0.5:
                expected_value = -1
                kind = "corrupt"
            self.pushbit(dev, bit, kind)
            # check decoded value
            idx = (dev.w_idx - 3) % self.DCC_BUF_LEN
            self.assertEqual(
                self.lib.parse_bit(devptr, idx),
                expected_value,
                msg=[
                    dev.buffer[
                        (dev.w_idx - i) % self.DCC_BUF_LEN
                    ]
                    for i in range(3, 1, -1)
                ]
            )


class TestPreamble(MySuper):
    def test_basic_start_bit_detection(self):
        dev, devptr = self.freshdev()
        self.assertEqual(dev.state, dcc_state_t.AWAITING_START_BIT)  # defense
        self.pushbit(dev, 0)
        self.assertEqual(dev.state, dcc_state_t.VALIDATING_PREAMBLE)

    def test_minimum_preamble(self):

        # push preamble
        dev, devptr = self.freshdev()
        for _ in range(10):  # min. 10 '1' bits in preamble
            self.pushbit(dev, 1)
        self.assertState(dev, dcc_state_t.AWAITING_START_BIT)  # sanity

        # push start bit
        self.pushbit(dev, 0)
        self.assertState(dev, dcc_state_t.VALIDATING_PREAMBLE)  # sanity

        # check bit decoding for sanity
        i = (dev.w_idx - 3) % self.DCC_BUF_LEN
        self.assertEqual(0, self.lib.parse_bit(devptr, dev.r_idx))
        self.assertEqual(0, self.lib.parse_bit(devptr, i))

        i = (dev.w_idx - 5) % self.DCC_BUF_LEN
        self.assertEqual(1, self.lib.parse_bit(devptr, i))

        i = (dev.r_idx - 2) % self.DCC_BUF_LEN
        self.assertEqual(1, self.lib.parse_bit(devptr, i))

        # check state machine advanced properly
        self.assertEqual(self.lib.validate_preamble(devptr), dev.state)
        self.assertState(dev, dcc_state_t.AWAITING_DATA_BYTES)

    def test_too_short_preamble(self):
        dev, devptr = self.freshdev()
        for _ in range(9):  # min. 10 '1' bits in preamble
            self.pushbit(dev, 1)
        self.pushbit(dev, 0)
        self.assertEqual(self.lib.validate_preamble(devptr), dev.state)
        self.assertEqual(dev.state, dcc_state_t.AWAITING_START_BIT)

    def _buffer_wrap_workhorse(self, minus_amount):
        "workhore method for buffer wrap testing"
        dev, devptr = self.freshdev()
        for _ in range(self.DCC_BUF_LEN - minus_amount):
            self.pushbit(dev, 1)
            self.assertState(dev, dcc_state_t.AWAITING_START_BIT)
        self.pushbit(dev, 0)
        self.assertState(dev, dcc_state_t.VALIDATING_PREAMBLE)

    def test_buffer_wrap_plus2(self):
        return self._buffer_wrap_workhorse(-2)

    def test_buffer_wrap_plus1(self):
        return self._buffer_wrap_workhorse(-1)

    def test_buffer_wrap_minus0(self):
        return self._buffer_wrap_workhorse(0)

    def test_buffer_wrap_minus1(self):
        return self._buffer_wrap_workhorse(1)

    def test_buffer_wrap_minus2(self):
        return self._buffer_wrap_workhorse(2)

    def test_buffer_wrap_multiple(self):
        return self._buffer_wrap_workhorse(int(3.25 * self.DCC_BUF_LEN))

    def test_monte_carlo(self):
        "TODO: update error probabilities for 50% chance of preamble corruption"
        dev, devptr = self.freshdev()
        for _ in range(1000):
            # push preamble
            n1bits = randrange(1, 3 * self.DCC_BUF_LEN)
            corrupt_idxs = []  # track corrupted bits
            for i in range(n1bits):
                if rand() < 0.93:  # each bit has 7% chance of corruption, ~50% chance for 10-bit preamble
                    self.pushbit(dev, 1, kind="noise")
                else:
                    self.pushbit(dev, 1, kind="corrupt")
                    corrupt_idxs.append(i)
                self.assertState(dev, dcc_state_t.AWAITING_START_BIT)

            # push start bit
            if rand() < 0.5:
                self.pushbit(dev, 0, kind="corrupt")
                self.assertEqual(dev.state, dcc_state_t.AWAITING_START_BIT)
            else:
                self.pushbit(dev, 0, kind="noise")
                self.assertState(dev, dcc_state_t.VALIDATING_PREAMBLE)

                # validate preamble
                state = self.lib.validate_preamble(devptr)
                if n1bits < 10 or any([v > (n1bits - 11) for v in corrupt_idxs]):
                    self.assertEqual(dev.state, dcc_state_t.AWAITING_START_BIT)
                else:
                    self.assertState(
                        dev, dcc_state_t.AWAITING_DATA_BYTES, 20, 2, dev.r_idx)

                # cleanup
                dev.state = dcc_state_t.AWAITING_START_BIT
                self.lib.init_decoder(devptr)


if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser("DCC library testing suite.")
    parser.add_argument(
        "arr", nargs="*", help="will plot an array of timestamps")
    args = parser.parse_args()
    arr = args.arr

    if len(arr) == 0:
        main()
    else:
        arr2 = [int(v.replace('[', '').replace(']', ''))
                for v in "".join(arr).split(',')]
        plot_edges(arr2)
        plt.show()
