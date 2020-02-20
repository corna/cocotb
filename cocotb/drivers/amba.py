# Copyright (c) 2014 Potential Ventures Ltd
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Potential Ventures Ltd,
#       SolarFlare Communications Inc nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL POTENTIAL VENTURES LTD BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Drivers for Advanced Microcontroller Bus Architecture."""

import array

import cocotb
from cocotb.triggers import RisingEdge, ReadOnly, Lock
from cocotb.drivers import BusDriver
from cocotb.binary import BinaryValue
from cocotb.drivers import BusDriver
from cocotb.result import ReturnValue, TestError
from cocotb.triggers import Lock, ReadOnly, RisingEdge


class AXIProtocolError(Exception):
    pass


class AXI4Master(BusDriver):
    """AXI4 Master

    TODO: Kill all pending transactions if reset is asserted.
    """

    _signals = \
        ["AWVALID", "AWADDR", "AWREADY", "AWID", "AWLEN", "AWSIZE", "AWBURST",
         "WVALID", "WREADY", "WDATA", "WSTRB",
         "BVALID", "BREADY", "BRESP", "BID",
         "ARVALID", "ARADDR", "ARREADY", "ARID", "ARLEN", "ARSIZE", "ARBURST",
         "RVALID", "RREADY", "RRESP", "RDATA", "RID", "RLAST"]

    _optional_signals = ["AWREGION", "AWLOCK", "AWCACHE", "AWPROT", "AWQOS",
                         "WLAST",
                         "ARREGION", "ARLOCK", "ARCACHE", "ARPROT", "ARQOS"]

    _xRESP_msg = ("OKAY", "EXOKAY", "SLVERR", "DECERR")
    _BURST_values = {"FIXED": 0b00, "INCR": 0b01, "WRAP": 0b10}

    def __init__(self, entity, name, clock, **kwargs):
        BusDriver.__init__(self, entity, name, clock, **kwargs)

        # Drive some sensible defaults (setimmediatevalue to avoid x asserts)
        self.bus.AWVALID.setimmediatevalue(0)
        self.bus.WVALID.setimmediatevalue(0)
        self.bus.ARVALID.setimmediatevalue(0)
        self.bus.BREADY.setimmediatevalue(1)
        self.bus.RREADY.setimmediatevalue(1)

        # Set the default value (0) for the unsupported signals
        unsupported_signals = \
            ["AWID", "AWREGION", "AWLOCK", "AWCACHE", "AWPROT", "AWQOS",
             "ARID", "ARREGION", "ARLOCK", "ARCACHE", "ARPROT", "ARQOS"]
        for signal in unsupported_signals:
            try:
                getattr(self.bus, signal).setimmediatevalue(0)
            except AttributeError:
                pass

        # Mutex for each channel that we master to prevent contention
        self.write_address_busy = Lock("%s_wabusy" % name)
        self.read_address_busy = Lock("%s_rabusy" % name)
        self.write_data_busy = Lock("%s_wbusy" % name)

    @staticmethod
    def _check_burst(burst):
        """Check that the provided transfer burst type is valid."""
        if burst not in AXI4Master._BURST_values:
            raise TestError(
                "{} is not a valid burst type; valid burst types are: {}"
                .format(burst, ", ".join(AXI4Master._BURST_values.keys())))

    @staticmethod
    def _check_length(length, burst):
        """Check that the provided transfer length is valid."""
        if length <= 0:
            raise TestError("Transfer length must be a positive integer")

        if burst == "INCR":
            if length > 256:
                raise TestError("Maximum transfer length for INCR bursts is "
                                "256")
        elif burst == "WRAP":
            if length not in (1, 2, 4, 8, 16):
                raise TestError("Transfer length for WRAP bursts must be 1, "
                                "2, 4, 8 or 16")
        else:
            if length > 16:
                raise TestError("Maximum transfer length for FIXED bursts is "
                                "16")

    @staticmethod
    def _check_size(size, data_bus_width):
        """Check that the provided transfer size is valid."""
        if size > data_bus_width:
            raise TestError("Transfer size ({} B) is larger than the bus "
                            "width ({} B)".format(size, data_bus_width))
        elif size <= 0 or size & (size - 1) != 0:
            raise TestError("Transfer size must be a positive power of 2")

    @cocotb.coroutine
    def _send_write_address(self, address, length, burst, size, delay=0):
        """
        Send the write address, with optional delay (in clocks)
        """
        yield self.write_address_busy.acquire()
        for cycle in range(delay):
            yield RisingEdge(self.clock)

        # Set the address and, if present on the bus, burst, length and size
        self.bus.AWADDR <= address
        self.bus.AWVALID <= 1

        if hasattr(self.bus, "AWBURST"):
            self.bus.AWBURST <= AXI4Master._BURST_values[burst]

        if hasattr(self.bus, "AWLEN"):
            self.bus.AWLEN <= length - 1

        if hasattr(self.bus, "AWSIZE"):
            self.bus.AWSIZE <= size.bit_length() - 1

        # Wait until acknowledged
        while True:
            yield ReadOnly()
            if self.bus.AWREADY.value:
                break
            yield RisingEdge(self.clock)
        yield RisingEdge(self.clock)
        self.bus.AWVALID <= 0
        self.write_address_busy.release()

    @cocotb.coroutine
    def _send_write_data(self, data, delay=0, byte_enable=0xF):
        """Send the write data, with optional delay (in clocks)."""
        yield self.write_data_busy.acquire()

        try:
            byte_enable_iterator = iter(byte_enable)
        except TypeError:
            byte_enable_iterator = iter((byte_enable,))

        for i, word in enumerate(data):
            for cycle in range(delay):
                yield RisingEdge(self.clock)

            self.bus.WDATA <= word
            self.bus.WVALID <= 1

            try:
                self.bus.WSTRB <= next(byte_enable_iterator)
            except StopIteration:
                # Do not update WSTRB if we have reached the end of the iter
                pass

            if hasattr(self.bus, "WLAST"):
                if i == len(data) - 1:
                    self.bus.WLAST <= 1
                else:
                    self.bus.WLAST <= 0

            while True:
                yield ReadOnly()
                if self.bus.WREADY.value:
                    break
                yield RisingEdge(self.clock)
            yield RisingEdge(self.clock)

            self.bus.WVALID <= 0

        self.write_data_busy.release()

    @cocotb.coroutine
    def write(self, address, value, size=None, burst="INCR", byte_enable=0xf,
              address_latency=0, data_latency=0, sync=True):
        """Write a value to an address.

        Args:
            address (int): The address to write to.
            value (Union[int, Sequence[int]]): The data value(s) to write.
            size (int, optional): The size (in bytes) of each beat.
                Defaults to the width of the data bus.
            burst (str, optional): The burst type, either FIXED, INCR or WRAP.
                Defaults to INCR.
            byte_enable (int or iterable of int, optional): Which bytes in
                value to actually write.
                Default is to write all bytes.
            address_latency (int, optional): Delay before setting the address
                (in clock cycles).
                Default is no delay.
            data_latency (int, optional): Delay before setting the data value
                (in clock cycles).
                Default is no delay.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            BinaryValue: The write response values.

        Raises:
            TestError: If any of the input parameters is invalid.
            AXIProtocolError: If write response from AXI is not ``OKAY``.
        """

        try:
            iter(value)
        except TypeError:
            value = (value,)    # If value is not an iterable, make it

        AXI4Master._check_burst(burst)
        AXI4Master._check_length(len(value), burst)

        if not size:
            size = len(self.bus.WDATA) // 8
        else:
            AXI4Master._check_size(size, len(self.bus.WDATA) // 8)

        if sync:
            yield RisingEdge(self.clock)

        c_addr = cocotb.fork(self._send_write_address(address,
                                                      len(value),
                                                      burst,
                                                      size,
                                                      delay=address_latency))
        c_data = cocotb.fork(self._send_write_data(value,
                                                   byte_enable=byte_enable,
                                                   delay=data_latency))

        if c_addr:
            yield c_addr.join()
        if c_data:
            yield c_data.join()

        # Wait for the response
        while True:
            yield ReadOnly()
            if self.bus.BVALID.value and self.bus.BREADY.value:
                result = self.bus.BRESP.value
                break
            yield RisingEdge(self.clock)

        yield RisingEdge(self.clock)

        if int(result):
            if len(value) == 1:
                err_msg = "Write to address {0:#x}"
            else:
                err_msg = "Write (start address {0:#x}"
            err_msg += " failed with BRESP: {1} ({2})"

            raise AXIProtocolError(
                err_msg.format(address, int(result),
                               AXI4Master._xRESP_msg[int(result)]))

        return result

    @cocotb.coroutine
    def read(self, address, length=1, size=None, burst="INCR", sync=True):
        """Read from an address.

        Args:
            address (int): The address to read from.
            length (int, optional): Number of words to transfer
                Defaults to 1.
            size (int, optional): The size (in bytes) of each beat.
                Defaults to the width of the data bus.
            burst (str, optional): The burst type, either FIXED, INCR or WRAP.
                Defaults to INCR.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            List[BinaryValue]: The read data values.

        Raises:
            TestError: If any of the input parameters is invalid.
            AXIProtocolError: If read response from AXI is not ``OKAY``.
        """

        AXI4Master._check_burst(burst)
        AXI4Master._check_length(length, burst)

        if not size:
            size = len(self.bus.RDATA) // 8
        else:
            AXI4Master._check_size(size, len(self.bus.RDATA) // 8)

        if sync:
            yield RisingEdge(self.clock)

        self.bus.ARADDR <= address
        self.bus.ARVALID <= 1

        if hasattr(self.bus, "AWLEN"):
            self.bus.ARLEN <= length - 1

        if hasattr(self.bus, "ARSIZE"):
            self.bus.ARSIZE <= size.bit_length() - 1

        if hasattr(self.bus, "AWBURST"):
            self.bus.ARBURST <= AXI4Master._BURST_values[burst]

        while True:
            yield ReadOnly()
            if self.bus.ARREADY.value:
                break
            yield RisingEdge(self.clock)

        yield RisingEdge(self.clock)
        self.bus.ARVALID <= 0

        data = []
        rresp = []

        while True:
            while True:
                yield ReadOnly()
                if self.bus.RVALID.value and self.bus.RREADY.value:
                    data.append(self.bus.RDATA.value)
                    rresp.append(self.bus.RRESP.value)
                    break
                yield RisingEdge(self.clock)

            if not hasattr(self.bus, "RLAST") or self.bus.RLAST.value:
                break

            yield RisingEdge(self.clock)

        for beat_number, beat_result in enumerate(rresp):
            if beat_result:
                if length == 1:
                    err_msg = "Read on address {1:#x}"
                else:
                    err_msg = "Read on beat {0} (start address {1:#x})"
                err_msg += " failed with RRESP: {2} ({3})"

                err_msg = err_msg.format(
                    beat_number, address, beat_result.integer,
                    AXI4Master._xRESP_msg[beat_result.integer])

                raise AXIProtocolError(err_msg)

        return data

    def __len__(self):
        return 2**len(self.bus.ARADDR)


class AXI4LiteMaster(AXI4Master):
    """AXI4-Lite Master"""

    _signals = ["AWVALID", "AWADDR", "AWREADY",        # Write address channel
                "WVALID", "WREADY", "WDATA", "WSTRB",  # Write data channel
                "BVALID", "BREADY", "BRESP",           # Write response channel
                "ARVALID", "ARADDR", "ARREADY",        # Read address channel
                "RVALID", "RREADY", "RRESP", "RDATA"]  # Read data channel

    _optional_signals = []

    @cocotb.coroutine
    def write(self, address, value, byte_enable=0xf, address_latency=0,
              data_latency=0, sync=True):
        """Write a value to an address.
        Args:
            address (int): The address to write to.
            value (int): The data value to write.
            byte_enable (int, optional): Which bytes in value to actually
                write.
                Default is to write all bytes.
            address_latency (int, optional): Delay before setting the address
                (in clock cycles).
                Default is no delay.
            data_latency (int, optional): Delay before setting the data value
                (in clock cycles).
                Default is no delay.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

            Returns:
                BinaryValue: The write response value.

            Raises:
                TestError: If any of the input parameters is invalid.
                AXIProtocolError: If write response from AXI is not ``OKAY``.
        """

        try:
            iter(value)
            raise TestError("AXI4Lite does not support multi-beat transfers")
        except TypeError:
            yield super(AXI4LiteMaster, self).write(
                address=address, value=value, size=None, burst="INCR",
                byte_enable=byte_enable, address_latency=address_latency,
                data_latency=data_latency, sync=sync)

    @cocotb.coroutine
    def read(self, address, sync=True):
        """Read from an address.

        Args:
            address (int): The address to read from.
            length (int): Number of words to transfer
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            BinaryValue: The read data value.

        Raises:
            AXIProtocolError: If read response from AXI is not ``OKAY``.
        """

        ret = yield super(AXI4LiteMaster, self).read(
            address=address, length=1, size=None, burst="INCR", sync=sync)
        raise ReturnValue(ret[0])


class AXI4Slave(BusDriver):
    '''
    AXI4 Slave

    Monitors an internal memory and handles read and write requests.
    '''
    _signals = [
        "ARREADY", "ARVALID", "ARADDR",             # Read address channel
        "ARLEN",   "ARSIZE",  "ARBURST", "ARPROT",

        "RREADY",  "RVALID",  "RDATA",   "RLAST",   # Read response channel

        "AWREADY", "AWADDR",  "AWVALID",            # Write address channel
        "AWPROT",  "AWSIZE",  "AWBURST", "AWLEN",

        "WREADY",  "WVALID",  "WDATA",

    ]

    # Not currently supported by this driver
    _optional_signals = [
        "WLAST",   "WSTRB",
        "BVALID",  "BREADY",  "BRESP",   "RRESP",
        "RCOUNT",  "WCOUNT",  "RACOUNT", "WACOUNT",
        "ARLOCK",  "AWLOCK",  "ARCACHE", "AWCACHE",
        "ARQOS",   "AWQOS",   "ARID",    "AWID",
        "BID",     "RID",     "WID"
    ]

    def __init__(self, entity, name, clock, memory, callback=None, event=None,
                 big_endian=False, **kwargs):

        BusDriver.__init__(self, entity, name, clock, **kwargs)
        self.clock = clock

        self.big_endian = big_endian
        self.bus.ARREADY.setimmediatevalue(1)
        self.bus.RVALID.setimmediatevalue(0)
        self.bus.RLAST.setimmediatevalue(0)
        self.bus.AWREADY.setimmediatevalue(1)
        self._memory = memory

        self.write_address_busy = Lock("%s_wabusy" % name)
        self.read_address_busy = Lock("%s_rabusy" % name)
        self.write_data_busy = Lock("%s_wbusy" % name)

        cocotb.fork(self._read_data())
        cocotb.fork(self._write_data())

    def _size_to_bytes_in_beat(self, AxSIZE):
        if AxSIZE < 7:
            return 2 ** AxSIZE
        return None

    @cocotb.coroutine
    def _write_data(self):
        clock_re = RisingEdge(self.clock)

        while True:
            while True:
                self.bus.WREADY <= 0
                yield ReadOnly()
                if self.bus.AWVALID.value:
                    self.bus.WREADY <= 1
                    break
                yield clock_re

            yield ReadOnly()
            _awaddr = int(self.bus.AWADDR)
            _awlen = int(self.bus.AWLEN)
            _awsize = int(self.bus.AWSIZE)
            _awburst = int(self.bus.AWBURST)
            _awprot = int(self.bus.AWPROT)

            burst_length = _awlen + 1
            bytes_in_beat = self._size_to_bytes_in_beat(_awsize)

            if __debug__:
                self.log.debug(
                    "AWADDR  %d\n" % _awaddr +
                    "AWLEN   %d\n" % _awlen +
                    "AWSIZE  %d\n" % _awsize +
                    "AWBURST %d\n" % _awburst +
                    "BURST_LENGTH %d\n" % burst_length +
                    "Bytes in beat %d\n" % bytes_in_beat)

            burst_count = burst_length

            yield clock_re

            while True:
                if self.bus.WVALID.value:
                    word = self.bus.WDATA.value
                    word.big_endian = self.big_endian
                    _burst_diff = burst_length - burst_count
                    _st = _awaddr + (_burst_diff * bytes_in_beat)  # start
                    _end = _awaddr + ((_burst_diff + 1) * bytes_in_beat)  # end
                    self._memory[_st:_end] = array.array('B', word.get_buff())
                    burst_count -= 1
                    if burst_count == 0:
                        break
                yield clock_re

    @cocotb.coroutine
    def _read_data(self):
        clock_re = RisingEdge(self.clock)

        while True:
            while True:
                yield ReadOnly()
                if self.bus.ARVALID.value:
                    break
                yield clock_re

            yield ReadOnly()
            _araddr = int(self.bus.ARADDR)
            _arlen = int(self.bus.ARLEN)
            _arsize = int(self.bus.ARSIZE)
            _arburst = int(self.bus.ARBURST)
            _arprot = int(self.bus.ARPROT)

            burst_length = _arlen + 1
            bytes_in_beat = self._size_to_bytes_in_beat(_arsize)

            word = BinaryValue(n_bits=bytes_in_beat*8, bigEndian=self.big_endian)

            if __debug__:
                self.log.debug(
                    "ARADDR  %d\n" % _araddr +
                    "ARLEN   %d\n" % _arlen +
                    "ARSIZE  %d\n" % _arsize +
                    "ARBURST %d\n" % _arburst +
                    "BURST_LENGTH %d\n" % burst_length +
                    "Bytes in beat %d\n" % bytes_in_beat)

            burst_count = burst_length

            yield clock_re

            while True:
                self.bus.RVALID <= 1
                yield ReadOnly()
                if self.bus.RREADY.value:
                    _burst_diff = burst_length - burst_count
                    _st = _araddr + (_burst_diff * bytes_in_beat)
                    _end = _araddr + ((_burst_diff + 1) * bytes_in_beat)
                    word.buff = self._memory[_st:_end].tostring()
                    self.bus.RDATA <= word
                    if burst_count == 1:
                        self.bus.RLAST <= 1
                yield clock_re
                burst_count -= 1
                self.bus.RLAST <= 0
                if burst_count == 0:
                    break
