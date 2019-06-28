''' Copyright (c) 2014 Potential Ventures Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Potential Ventures Ltd,
      SolarFlare Communications Inc nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL POTENTIAL VENTURES LTD BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. '''

"""Drivers for Advanced Microcontroller Bus Architecture."""

import array
import binascii

import cocotb
from cocotb.binary import BinaryValue
from cocotb.drivers import BusDriver
from cocotb.result import ReturnValue
from cocotb.triggers import (ClockCycles, FallingEdge, Lock, ReadOnly,
                             RisingEdge)


class AXIProtocolError(Exception):
    pass


class AXI4LiteMaster(BusDriver):
    """AXI4-Lite Master.

    TODO: Kill all pending transactions if reset is asserted.
    """

    _signals = ["AWVALID", "AWADDR", "AWREADY",        # Write address channel
                "WVALID", "WREADY", "WDATA", "WSTRB",  # Write data channel
                "BVALID", "BREADY", "BRESP",           # Write response channel
                "ARVALID", "ARADDR", "ARREADY",        # Read address channel
                "RVALID", "RREADY", "RRESP", "RDATA"]  # Read data channel

    def __init__(self, entity, name, clock):
        BusDriver.__init__(self, entity, name, clock)

        # Drive some sensible defaults (setimmediatevalue to avoid x asserts)
        self.bus.AWVALID.setimmediatevalue(0)
        self.bus.WVALID.setimmediatevalue(0)
        self.bus.ARVALID.setimmediatevalue(0)
        self.bus.BREADY.setimmediatevalue(1)
        self.bus.RREADY.setimmediatevalue(1)

        # Mutex for each channel that we master to prevent contention
        self.write_address_busy = Lock("%s_wabusy" % name)
        self.read_address_busy = Lock("%s_rabusy" % name)
        self.write_data_busy = Lock("%s_wbusy" % name)

    @cocotb.coroutine
    def _send_write_address(self, address, delay=0):
        """
        Send the write address, with optional delay (in clocks)
        """
        yield self.write_address_busy.acquire()
        for cycle in range(delay):
            yield RisingEdge(self.clock)

        self.bus.AWADDR <= address
        self.bus.AWVALID <= 1

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
        """Send the write address, with optional delay (in clocks)."""
        yield self.write_data_busy.acquire()
        for cycle in range(delay):
            yield RisingEdge(self.clock)

        self.bus.WDATA <= data
        self.bus.WVALID <= 1
        self.bus.WSTRB <= byte_enable

        while True:
            yield ReadOnly()
            if self.bus.WREADY.value:
                break
            yield RisingEdge(self.clock)
        yield RisingEdge(self.clock)
        self.bus.WVALID <= 0
        self.write_data_busy.release()

    @cocotb.coroutine
    def write(self, address, value, byte_enable=0xf, address_latency=0,
              data_latency=0, sync=True):
        """Write a value to an address.

        Args:
            address (int): The address to write to.
            value (int): The data value to write.
            byte_enable (int, optional): Which bytes in value to actually write.
                Default is to write all bytes.
            address_latency (int, optional): Delay before setting the address (in clock cycles).
                Default is no delay.
            data_latency (int, optional): Delay before setting the data value (in clock cycles).
                Default is no delay.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            BinaryValue: The write response value.

        Raises:
            AXIProtocolError: If write response from AXI is not ``OKAY``.
        """
        if sync:
            yield RisingEdge(self.clock)

        c_addr = cocotb.fork(self._send_write_address(address,
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
            raise AXIProtocolError("Write to address 0x%08x failed with BRESP: %d"
                               % (address, int(result)))

        raise ReturnValue(result)

    @cocotb.coroutine
    def read(self, address, sync=True):
        """Read from an address.

        Args:
            address (int): The address to read from.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            BinaryValue: The read data value.

        Raises:
            AXIProtocolError: If read response from AXI is not ``OKAY``.
        """
        if sync:
            yield RisingEdge(self.clock)

        self.bus.ARADDR <= address
        self.bus.ARVALID <= 1

        while True:
            yield ReadOnly()
            if self.bus.ARREADY.value:
                break
            yield RisingEdge(self.clock)

        yield RisingEdge(self.clock)
        self.bus.ARVALID <= 0

        while True:
            yield ReadOnly()
            if self.bus.RVALID.value and self.bus.RREADY.value:
                data = self.bus.RDATA.value
                result = self.bus.RRESP.value
                break
            yield RisingEdge(self.clock)

        if int(result):
            raise AXIProtocolError("Read address 0x%08x failed with RRESP: %d" %
                               (address, int(result)))

        raise ReturnValue(data)

    def __len__(self):
        return 2**len(self.bus.ARADDR)

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
                 big_endian=False):

        BusDriver.__init__(self, entity, name, clock)
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


class Axi4StreamMaster(BusDriver):
    """
    AXI4-Stream Master
    """

    _signals = ["TVALID"]

    _optional_signals = [
        "TREADY", "TDATA", "TLAST",

        # Not currently supported by this driver
        "TSTRB", "TKEEP", "TID", "TDEST", "TUSER"
    ]

    def __init__(self, entity, name, clock):
        """
            Initialization of AxiStreamMaster

            Args:
                entity: handle to the simulator entity
                name: Name of the bus
                clock: handle to the clk associated with this bus
        """

        BusDriver.__init__(self, entity, name, clock)
        self.clock = clock

        # Drive some sensible defaults (setimmediatevalue to avoid x asserts)
        self.bus.TVALID.setimmediatevalue(0)
        if hasattr(self.bus, "TDATA"):
            self.bus.TDATA.setimmediatevalue(0)
        if hasattr(self.bus, "TLAST"):
            self.bus.TLAST.setimmediatevalue(0)

    @cocotb.coroutine
    def write(self, data, sync=True, tlast_on_last=True):
        """
        Write one or more values on the bus.

        Args:
            data (int or iterable of int): the data value(s) to write. If TDATA
                is not present on the bus, this argument is used to know the
                number of transfers to perform.
            sync (bool, optional): wait for rising edge on clock initially.
                Defaults to True.
            tlast_on_last(bool, optional): assert TLAST on the last word
                written on the bus.
                Defaults to True.
        """

        try:
            iter(data)
        except TypeError:
            data = (data,)    # If data is not iterable, make it

        if sync:
            yield RisingEdge(self.clock)

        self.bus.TVALID <= 1

        for index, word in enumerate(data):

            # If TDATA is not present, use data just to keep track of the
            # number of transfer cycles
            if hasattr(self.bus, "TDATA"):
                self.bus.TDATA <= word

            if hasattr(self.bus, "TLAST") and tlast_on_last and \
               index == len(data) - 1:
                self.bus.TLAST <= 1

            while True:
                yield ReadOnly()
                if not hasattr(self.bus, "TREADY") or self.bus.TREADY.value:
                    break
                yield RisingEdge(self.clock)

            yield RisingEdge(self.clock)

        if hasattr(self.bus, "TLAST") and tlast_on_last:
            self.bus.TLAST <= 0

        self.bus.TVALID <= 0


class Axi4StreamSlave(BusDriver):
    """
    AXI4-Stream Slave

    Generates configurable TREADY patterns.
    """

    _signals = ["TVALID"]

    # Only TREADY is used in this design, all other signals are unused
    _optional_signals = [
        "TREADY", "TDATA", "TLAST", "TSTRB", "TKEEP", "TID", "TDEST", "TUSER"
    ]

    def __init__(self, entity, name, clock, tready_delay=-1,
                 consecutive_transfers=0):
        """
            Initialization of AxiStreamSlave

            Args:
                entity: handle to the simulator entity
                name: name of the bus
                clock: handle to the clk associated with this bus
                tready_delay (int or callable, optional): number of delay clock
                    cycles between a high TVALID and the assertion of TREADY.
                    Can also be a function, which is called at the TVALID
                    rising edge should return the number of delay cycles to
                    assert TREADY.
                    Defaults to -1 , which sets TREADY always to 1, regardless
                    of the value of TVALID.
                consecutive_transfers (int or callable, optional): the maximum
                    number of uninterrupted transfers (high TVALID and TREADY
                    on consecutive clock cycles) after which TREADY will be
                    de-asserted and the tready_delay will be restarted.
                    Can also be a function, which is called at the beginning of
                    each transfer and should return the maximum allowed number
                    of consecutive transfers.
                    Default to 0, which allows an unlimited number of
                    consecutive transfers.
        """

        BusDriver.__init__(self, entity, name, clock)

        # If TREADY is not present, this Driver does nothing
        if hasattr(self.bus, "TREADY"):
            if not callable(tready_delay) and tready_delay == -1:
                # If TREADY has to be always high, just set it to 1
                self.bus.TREADY.setimmediatevalue(1)
            else:
                # If not, set it to zero and fork _receive_data
                self.bus.TREADY.setimmediatevalue(0)
                self.clock = clock
                self.tready_delay = tready_delay
                self.consecutive_transfers = consecutive_transfers

                cocotb.fork(self._receive_data())

    @cocotb.coroutine
    def _receive_data(self):

        while True:
            # Wait for a high TVALID, if not already high
            if not self.bus.TVALID.value:
                yield RisingEdge(self.bus.TVALID)

            # Wait either for the required number of clock cycles or for a low
            # TVALID. By AXI4-Stream standard, the master should not de-assert
            # TVALID until at least one transfer has been performed but, if it
            # does it anyways, just re-start the wait.
            if callable(self.tready_delay):
                tready_high_delay = ClockCycles(self.clock,
                                                self.tready_delay(self.bus))
            else:
                tready_high_delay = ClockCycles(self.clock, self.tready_delay)

            trigger = yield [tready_high_delay, FallingEdge(self.bus.TVALID)]

            if trigger is tready_high_delay:
                self.bus.TREADY <= 1

                if callable(self.consecutive_transfers):
                    num_cycles = self.consecutive_transfers(self.bus)
                else:
                    num_cycles = self.consecutive_transfers

                if num_cycles != 0:
                    yield [ClockCycles(self.clock, num_cycles),
                           FallingEdge(self.bus.TVALID)]
                else:
                    yield FallingEdge(self.bus.TVALID)

                self.bus.TREADY <= 0
