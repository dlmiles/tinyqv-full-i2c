# SPDX-FileCopyrightText: © 2025 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from tqv import TinyQV

from cocotb_stuff import *
from cocotb_stuff.cocotbutil import debug
import cocotb_stuff.cocotbutil as cocotbutil


# When submitting your design, change this to the peripheral number
# in peripherals.v.  e.g. if your design is i_user_peri05, set this to 5.
# The peripheral number is not used by the test harness.
PERIPHERAL_NUM = 0

ADR00_DATA = 0x00
ADR01_STAT = 0x04
ADR02_CTRL = 0x08
ADR03_CONF = 0x0c
ADR04_CMUX = 0x10

TICKS = 16
SLOW = 100
STANDARD = 40
FAST = 10
FASTPLUS = 4

CLOCKS_PER_SCL = STANDARD * TICKS

# REG_CTRL
CTRL_FSM_START      = 0x001
CTRL_FSM_RUN        = CTRL_FSM_START # same bit0
CTRL_FSM_STOP       = 0x002
CTRL_FSM_RESET      = 0x003

STAT_ERR_TIMEOUT    = 0x800
STAT_ERR_IO         = 0x400
STAT_ERR_GENERIC    = 0x200
STAT_INTR_RAW       = 0x100
STAT_INTR_EN        = 0x080
STAT_INTR_EDGE      = 0x040
STAT_TXE_OVERRUN    = 0x020
STAT_TX_FULL        = 0x010
STAT_TX_EMPTY       = 0x008
STAT_RXE_OVERRUN    = 0x004
STAT_RX_FULL        = 0x002
STAT_RX_EMPTY       = 0x001

class Accessor():
    def __init__(self, dut, scl_id: int, sda_id: int, scl_oe_id: int = None, sda_oe_id: int = None, scl_i_id: int = None, sda_i_id: int = None):
        assert dut is not None
        assert type(scl_id)    is int and scl_id >= 0
        assert type(sda_id)    is int and sda_id >= 0

        self._dut = dut
        self._scl_id = scl_id
        self._sda_id = sda_id

        if scl_oe_id is not None:
            assert type(scl_oe_id) is int and scl_oe_id >= 0
        if sda_oe_id is not None:
            assert type(sda_oe_id) is int and sda_oe_id >= 0
        self._scl_oe_id = scl_oe_id
        self._sda_oe_id = sda_oe_id

        if scl_i_id is not None:
            assert type(scl_i_id) is int and scl_i_id >= 0
        if sda_i_id is not None:
            assert type(sda_i_id) is int and sda_i_id >= 0
        self._scl_i_id = scl_i_id
        self._sda_i_id = sda_i_id

        self.X_VALUE = True

        return None

    def _signal_name(self, bitid: int) -> str:
        if bitid == self._scl_id:
            return "SCL"
        if bitid == self._sda_id:
            return "SDA"
        if bitid == self._scl_oe_id:
            return "SCL_OE"
        if bitid == self._sda_oe_id:
            return "SDA_OE"
        return f"bitid#{bitid}"

    def _signal_accessor(self, signal, bitid: int, x_value: bool = None) -> bool:
        sh = signal[bitid].value
        if sh.is_resolvable:
            return sh.integer != 0

        if type(x_value) is bool:
            return x_value
        s = self._signal_name(bitid)
        raise Exception(f"{s}(bitid={bitid}, x_value={x_value}) = {str(sh)}")

    def validate(self) -> None:
        pass	# FIXME placeholder to check CMUX mapping

    def get_scl(self, x_value: bool = None) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._scl_id, x_value)

    def get_scl_oe(self, x_value: bool = None) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._scl_oe_id, x_value)

    def get_sda(self, x_value: bool = None) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._sda_id, x_value)

    def get_sda_oe(self, x_value: bool = None) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._sda_oe_id, x_value)

    @property
    def scl(self) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._scl_id, self.X_VALUE)

    @property
    def scl_oe(self) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._scl_oe_id, self.X_VALUE)

    @property
    def sda(self) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._sda_id, self.X_VALUE)

    @property
    def sda_oe(self) -> bool:
        return self._signal_accessor(self._dut.uo_out, self._sda_oe_id, self.X_VALUE)

    @property
    def scl_i(self) -> int:
        return self._scl_i_id # returns bit-id not mask

    @property
    def sda_i(self) -> int:
        return self._sda_i_id # returns bit-id not mask

    @staticmethod
    def build_mask(m: int) -> int:
        is_odd  = m & 0x55 != 0 # odd bits
        is_even = m & 0xaa != 0 # even bits
        if is_odd and not is_even:
            return 0x55
        elif is_even and not is_odd:
            return 0xaa
        else:
            return m	# no change to mask provided


# cocotb.start_soon(LoopbackTask(args...).build_task())
class LoopbackTask():
    def __init__(self, dut, accessor: Accessor, scl_mask: int = 0, sda_mask: int = 0):
        assert dut is not None
        assert type(accessor) is Accessor

        self._dut = dut
        self._accessor = accessor
        self._running = True

        self.SCL_MASK = scl_mask
        self.SDA_MASK = sda_mask
        dut._log.info(f"LoopbackTask(scl_mask=0x{scl_mask:x} sda_mask=0x{sda_mask:x}) - mimic external wire loopback and pull-ups scl_o->scl_i sda_o->sda_i")
        self._i = 0
        return None


    # This provides loopback of output to input
    @staticmethod
    def loopback_update(dut, accessor: Accessor, scl_mask: int, sda_mask: int) -> int:
        m = 0
        # SIGNAL=0 && OE=1 then set LOW (we apply demorgans so only one expression needs to be evaluated)
        # otherwise mimic external pull-up HIGH
        m |= scl_mask if accessor.scl or not accessor.scl_oe else 0
        m |= sda_mask if accessor.sda or not accessor.sda_oe else 0
        #dut._log.info(f"loopback_update(scl={accessor.scl} sda={accessor.sda} scl_oe={accessor.scl_oe} sda_oe={accessor.sda_oe} scl_mask={scl_mask:x} sda_mask={sda_mask:x} = mask={m:x})")
        ##dut.ui_in.value = m
        return m

    @cocotb.coroutine
    def loopback_coroutine(self) -> None:
        while self._running:
            m = LoopbackTask.loopback_update(self._dut, self._accessor, self.SCL_MASK, self.SDA_MASK)
            self._dut.ui_in.value = m
            yield ClockCycles(self._dut.clk, 1)

    def stop(self) -> None:
        self._running = False
        return None

    def build_task(self) -> cocotb.Task:
        return cocotb.create_task(self.loopback_coroutine())


def reg_to_string(addr: int) -> str:
    if addr == ADR00_DATA:
        return "ADR00_DATA"
    if addr == ADR01_STAT:
        return "ADR01_STAT"
    if addr == ADR02_CTRL:
        return "ADR02_CTRL"
    if addr == ADR03_CONF:
        return "ADR03_CONF"
    if addr == ADR04_CMUX:
        return "ADR04_CMUX"
    return None


async def reg_read32(tqv, addr: int) -> int:
    assert (addr >= ADR00_DATA and addr <= ADR04_CMUX), f"address=0x{address:03x}"
    assert (addr & 0x003) == 0, f"address=0x{address:03x} is not 32bit aligned address"
    assert type(tqv) is TinyQV, f"reg_read32(tqv={type(tqv)}, ...) wrong type for argument, MAYBE YOU USED dut AND NOT tqv"
    value = await tqv.read_word_reg(addr)
    return value


def compute_xmask(v):
    if v is None:
        return value
    if type(v) is int:
        return value
    if type(v) is BinaryValue:
        biti = 0
        res = 0
        s = v.binstr
        for i in range(0, v.n_bits+1):
            if s[biti] == 'x':
                m = 1 << biti
                res |= m
            biti += 1
        nv = BinaryValue(s, n_bits=v.n_bits)
        println(f"v={v} nv={nv} s={s}")
        return nv
    raise Exception(f"compute_xmask(v={v}) [{type(v)}] ERROR unexpected type")


def replace_xmask(v, xmask: int, with_val: int):
    if v is None and (xmask is None or xmask == 0):
        return None
    # FIXME make this work with BinaryValue
    assert type(v) is int, f"unexpected type {type(v)} need int"
    v = v & ~xmask
    return v


async def reg_read32_and_check(tqv, addr: int, expect: int, andmask: int = ~0, can_raise: bool = True) -> bool:
    # all registers only use bottom 12bit, top bits must always be zero
    assert (expect & ~0xfff) == 0, f"expect=0x{expect:x} out-of-range"
    andmask |= ~0xfff # high bit should be zero
    value = await reg_read32(tqv, addr)
    #xmask = compute_xmask(value)
    #if (xmask & andmask) != 0:
    #    # resolve X (if in mask, need to raise)
    #    raise Exception(f"")
    #value = replace_xmask(value, xmask, '0') # otherwise replace with 0
    mvalue = value & andmask
    bf = mvalue == expect
    if not bf and can_raise:
        raise Exception(f"reg_read32_and_check(addr={reg_to_string(addr)}, expect=0x{expect:03x}, andmask=0x{andmask & 0xffffffff:03x}) = 0x{value:03x} 0b{value:012b} [actual] ([masked] 0x{mvalue:03x} != 0x{expect:03x} [expect])")
    return bf


async def reg_data_check(tqv, expect: int, andmask: int = ~0, can_raise: bool = True) -> bool:
    # REG_DATA bottom 12bit are in use, top bits must always be zero
    return await reg_read32_and_check(tqv, ADR00_DATA, expect, andmask, can_raise)


async def reg_stat_check(tqv, expect: int, andmask: int = ~0, can_raise: bool = True) -> bool:
    # REG_STAT bottom 12bit are in use, top bits must always be zero
    return await reg_read32_and_check(tqv, ADR01_STAT, expect, andmask, can_raise)


async def reg_ctrl_check(tqv, expect: int, andmask: int = ~0, can_raise: bool = True) -> bool:
    # REG_CTRL bottom 12bit are in use, top bits must always be zero
    return await reg_read32_and_check(tqv, ADR02_CTRL, expect, andmask, can_raise)


async def user_interrupt_check(dut, expect: bool, can_raise: bool = True) -> bool:
    assert type(dut) is not TinyQV, f"reg_read32(dut={type(dut)}, ...) wrong type for argument, MAYBE YOU USED tqv AND NOT dut"
    signal_path = "test_harness.user_interrupt"
    s = cocotbutil.design_element(dut, signal_path)
    if s is None:
        return None
    bf = s.value == expect
    if not bf and can_raise:
        raise Exception(f"user_interrupt_check(expect=0x{expect:x}) = 0x{bf:x} [actual] (failed)")
    return bf


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Set the clock period to 100 ns (10 MHz)
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())


    debug(dut, '001_RESET')

    # Interact with your design's registers through this TinyQV class.
    # This will allow the same test to be run when your design is integrated
    # with TinyQV - the implementation of this class will be replaces with a
    # different version that uses Risc-V instructions instead of the SPI test
    # harness interface to read and write the registers.
    tqv = TinyQV(dut, PERIPHERAL_NUM)

    # Reset
    await tqv.reset()

    dut._log.info("Test project behavior")
    debug(dut, '002')

    dut.ui_in.value = 0xff	# simulate external pull-up (before LoopbackTask takes over below)

    # Test register write and read back
    debug(dut, '003 CONF')
    await tqv.write_word_reg(ADR03_CONF, 0x00000000)
    await ClockCycles(dut.clk, 2)

    await tqv.write_word_reg(ADR03_CONF, 400 - 1)   # CHECKME clkdiv_limit==SLOW
    await ClockCycles(dut.clk, 2)

    await tqv.write_word_reg(ADR03_CONF, 10 - 1)    # CHECKME clkdiv_limit==FAST
    await ClockCycles(dut.clk, 2)

    await tqv.write_word_reg(ADR03_CONF, 4 - 1)     # CHECKME clkdiv_limit==FASTPLUS
    await ClockCycles(dut.clk, 2)

    await tqv.write_word_reg(ADR03_CONF, 40 - 1)    # CHECKME clkdiv_limit==STANDARD
    await ClockCycles(dut.clk, 2)

    debug(dut, '004 CTRL')
    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_RESET)    # FSM_RESET command
    await ClockCycles(dut.clk, 2)

    debug(dut, '005 CMUX')
    await tqv.write_word_reg(ADR04_CMUX, 0x00000000)    # DEFAULT
    await ClockCycles(dut.clk, 2)

    debug(dut, '005 CMUX 0x02a')
    await tqv.write_word_reg(ADR04_CMUX, 0x0000002a)    # DEFAULT as below 12'b000000101010
    #await tqv.write_word_reg(ADR04_CMUX, 0x00000c2a)    # DEFAULT as below 12'b110000101010 inverted OE
    await tqv.write_word_reg(ADR04_CMUX, 0x000000ea)    # DEFAULT as below 12'b000011101010 direct
    await ClockCycles(dut.clk, 2)

    # CMUX setup
    accessor = Accessor(
        dut=dut,
        # uo_out[]
        scl_id=0,	# reg_cmux[5:4] = 2'b10 ui_out[0] GPIO05 I2C0 SCL    i2c0_with_oe
        sda_id=3,	# reg_cmux[5:4] = 2'b10 ui_out[3] GPIO08 I2C0 SDA    i2c0_with_oe
        scl_oe_id=2,	# reg_cmux[5:4] = 2'b10 ui_out[2] GPIO07      SCL OE i2c0_with_oe
        sda_oe_id=1,	# reg_cmux[5:4] = 2'b10 ui_out[1] GPIO06      SDA OE i2c0_with_oe
        # ui_in[]
        scl_i_id=4,	# reg_cmux[1:0] = 2'b10 ui_in[4]  GPIO17 I2C0 SCL
        sda_i_id=3,	# reg_cmux[3:2] = 2'b10 ui_in[3]  GPIO12 I2C0 SDA
        # FIXME testing inversions
        #		# reg_cmux[6]  SCL mode (pull-down by default)
        #		# reg_cmux[7]  SDA mode (pull-down by default)
        #		# reg_cmux[8]  SCL invert (non-inverted by default)
        #		# reg_cmux[9]  SDA invert (non-inverted by default)
        #		# reg_cmux[10] SCL OE invert (non-inverted by default)
        #		# reg_cmux[11] SDA OE invert (non-inverted by default)
    )
    cocotb.start_soon(LoopbackTask(dut, accessor,
        Accessor.build_mask(1 << accessor.scl_i),
        Accessor.build_mask(1 << accessor.sda_i)).build_task())

    debug(dut, '006 STAT')
    await tqv.write_word_reg(ADR01_STAT, 0x00000080)	# INTR_EN
    await ClockCycles(dut.clk, 2)

    # The following assertion is just an example of how to check the output values.
    # Change it to match the actual expected output of your module:
    #assert dut.uo_out.value == 0x00	# pull-up

    #assert await tqv.read_byte_reg(ADDR_DATA) == 0x00

    # Zero should be read back from register 2
    ##assert await tqv.read_word_reg(8) == 0

    dut._log.info(f"SEND")

    if True:
        return

    # A second write should work
    debug(dut, '007 DATA 0xa5')
    await tqv.write_word_reg(ADR00_DATA, 0xa5)
    await ClockCycles(dut.clk, 2)

    debug(dut, '008 FSM_RUN 0xa5')
    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_RUN)	# FSM_RUN

    for i in range(0, 12):
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    debug(dut, '009 DATA 0x5a')
    await tqv.write_word_reg(ADR00_DATA, 0x5a)
    await ClockCycles(dut.clk, 2)

    for i in range(0, 8):
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    debug(dut, '010 DATA 0x55')
    await tqv.write_word_reg(ADR00_DATA, 0x55)
    await ClockCycles(dut.clk, 2)

    debug(dut, '011 CTRL NOP')
    await tqv.write_word_reg(ADR02_CTRL, 0x00)	# NOP
    await ClockCycles(dut.clk, 2)

    for i in range(0, 4):	# short to overlap SEND_STOP before end
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    debug(dut, '012 STOP')
    await tqv.write_word_reg(ADR02_CTRL, 0x08)	# SEND_STOP
    await ClockCycles(dut.clk, 2)

    for i in range(0, 9):
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    debug(dut, '013 DONE')

    ### FIXME check expected states, interrupts, etc...

    debug(dut, '014 FSM_STOP')
    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_STOP)	# FSM_STOP

    ## check FSM_RUN status
    await reg_ctrl_check(tqv, expect=0x000, andmask=CTRL_FSM_RUN) # FSM_RUNNING==0

    debug(dut, '015 IDLE')
    ### FIXME check expected states, interrupts, etc...

    ## check interrupt status (FIXME this should work!)
    await reg_stat_check(tqv, expect=STAT_INTR_EN, andmask=STAT_INTR_RAW|STAT_INTR_EN|STAT_INTR_EDGE) # FSM_RUNNING==0
    ## check interrupt signal
    await user_interrupt_check(dut, expect=0)

    for i in range(0, 4):
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)


    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_RESET) # FSM_RESET command
    await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    # FIXME restore now RX is in TX buf
    await reg_stat_check(tqv, expect=STAT_TX_EMPTY, andmask=STAT_TXE_OVERRUN|STAT_TX_FULL|STAT_TX_EMPTY)


    # 10bit address example (can push both bytes into FIFO)
    #  or it could be 7bit address, then command byte, before byte receive
    debug(dut, '100 SEND 0x11') # 8'b1111010R  MAGIC=11110 A9=1 A8=0 RW=0 (read)
    await tqv.write_word_reg(ADR00_DATA, 0x011)

    #
    await reg_stat_check(tqv, expect=0, andmask=STAT_TX_EMPTY)

    debug(dut, '101 SEND 0x08') # 8'b00001000  A7..A0=8
    await tqv.write_word_reg(ADR00_DATA, 0x008)

    dut._log.info(f"RECV")

    debug(dut, '102 RECV')
    # FIXME add NEED_STOP bit here
    await tqv.write_word_reg(ADR00_DATA, 0x100) # RECV+NEED_STOP

    # check no TXE_OVERRUN
    await reg_stat_check(tqv, expect=0, andmask=STAT_TXE_OVERRUN)

    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_RUN) # FSM_RUN command
    for i in range(0, 9*3):	# data+acknack
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)
    for i in range(0, 4):	# stop
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    await reg_ctrl_check(tqv, expect=CTRL_FSM_RUN, andmask=CTRL_FSM_RUN) # FSM_RUNNING

    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_STOP) # FSM_STOP command

    await reg_ctrl_check(tqv, expect=0x000, andmask=CTRL_FSM_RUN) # !FSM_RUNNING


    ## wait for SDA_OE to appear
    ## check SDA==1
    ## wait/count SDA==0

    ## wait for SCL_OE to appear
    ## check SCL==1
    ## wait/count SCL==0

    ## FIXME CPU push ADR00_DATA for txe_overrun and user_interrupt
    ## FIXME recover after user_interrupt

    ##assert dut.uo_out.value == 70
    debug(dut, '199 IDLE')
    for i in range(0, 8):
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    ###############################################################################
    # Test TXE_OVERRUN (and recovery)
    debug(dut, '200 TXE_OVERRUN')
    signal = cocotbutil.design_element(dut, 'test_harness.user_peripheral_dlmiles_i2c.tqvp_dlmiles_i2c_fifo.RX_FIFO_DEPTH')
    RX_FIFO_DEPTH = signal.value if signal is not None else None
    dut._log.info(f"RX_FIFO_DEPTH={RX_FIFO_DEPTH} [{type(RX_FIFO_DEPTH)}]")
    assert type(RX_FIFO_DEPTH) is not None
    await tqv.write_word_reg(ADR02_CTRL, CTRL_FSM_RESET) # FSM_RESET command (does not empty)
    await tqv.write_word_reg(ADR01_STAT, STAT_TX_EMPTY) # TX_EMPTY command
    await reg_stat_check(tqv, expect=STAT_TX_EMPTY, andmask=STAT_TX_EMPTY)
    for i in range(1, RX_FIFO_DEPTH+1):
        e = STAT_TX_EMPTY if i == 1 else 0
        #e |= STAT_TX_FULL if i == RX_FIFO_DEPTH else 0
        #dut._log.info(f"i={i} e=0x{e:03x}")
        await reg_stat_check(tqv, expect=e, andmask=STAT_TXE_OVERRUN|STAT_TX_FULL|STAT_TX_EMPTY)
        await tqv.write_word_reg(ADR00_DATA, i)
    debug(dut, '201 TXE_OVERRUN')
    await reg_stat_check(tqv, expect=STAT_TX_FULL, andmask=STAT_TXE_OVERRUN|STAT_TX_FULL|STAT_TX_EMPTY)

    debug(dut, '202 IDLE')
    await tqv.write_word_reg(ADR01_STAT, STAT_TX_EMPTY) # TX_EMPTY command
    await reg_stat_check(tqv, expect=STAT_TX_EMPTY, andmask=STAT_TX_EMPTY)

    debug(dut, '299 IDLE')
    for i in range(0, 8):
        await ClockCycles(dut.clk, CLOCKS_PER_SCL)

    ## FSM_RESET
    ## TXE_OVERRUN==0
    ## TXE_OVERRUN (cause with FIFO overrun)
    ## TXE_OVERRUN==1
    ## FSM_RESET
    ## TXE_OVERRUN==0

    debug(dut, '800 INTR')
    # Test the interrupt, generated when ui_in[6] goes high
    dut.ui_in[6].value = 1
    await ClockCycles(dut.clk, 1)
    dut.ui_in[6].value = 0

    # Interrupt asserted
    await ClockCycles(dut.clk, 3)
    ##assert await tqv.is_interrupt_asserted()

    # Interrupt doesn't clear
    await ClockCycles(dut.clk, 10)
    ##assert await tqv.is_interrupt_asserted()

    # Write bottom bit of address 8 high to clear
    await tqv.write_byte_reg(8, 1)
    ##assert not await tqv.is_interrupt_asserted()

    debug(dut, '999')
    await ClockCycles(dut.clk, 100)

