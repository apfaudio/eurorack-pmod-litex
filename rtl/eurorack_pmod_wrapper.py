import os

from migen import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.gpio import GPIOOut

from litex.soc.cores.dma import *
from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr_eventmanager import *

SOURCES_ROOT = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../deps/eurorack-pmod/gateware"
        )

class EurorackPmod(Module, AutoCSR):
    def add_fifos(self, depth=256):

        layout_rfifo = [("in0", 16),
                        ("in1", 16),
                        ("in2", 16),
                        ("in3", 16)]

        cdc_in0 = ClockDomainCrossing(
                layout=layout_rfifo,
                cd_from="clk_fs",
                cd_to="sys"
        )
        self.submodules += cdc_in0

        rfifo = stream.SyncFIFO(layout_rfifo, depth)
        self.submodules += rfifo
        rfifo_almost_full = rfifo.level > (depth - 4)

        self.rlevel = CSRStatus(16)
        self.comb += [
            self.rlevel.status.eq(rfifo.level)
        ]

        # IRQ logic
        self.ev = EventManager()
        self.ev.almost_full = EventSourceProcess(edge="rising")
        self.submodules += self.ev
        self.submodules += self.ev.almost_full
        self.comb += [
            self.ev.almost_full.trigger.eq(rfifo_almost_full),
        ]
        self.ev.finalize()

        self.comb += [
            # CDC <-> I2S (clk_fs domain)
            # ADC -> CDC
            cdc_in0.sink.valid.eq(1),
            cdc_in0.sink.in0.eq(self.cal_in0),
            cdc_in0.sink.in1.eq(self.cal_in1),
            cdc_in0.sink.in2.eq(self.cal_in2),
            cdc_in0.sink.in3.eq(self.cal_in3),

            # ADC -> CDC -> Router -> DRAM
            cdc_in0.source.connect(rfifo.sink),
        ]

        self.bus = bus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        rd_ack = Signal()
        wr_ack = Signal()
        self.comb += [
            If(bus.we,
                bus.ack.eq(wr_ack),
            ).Else(
                bus.ack.eq(rd_ack),
            )
        ]

        bus_read    = Signal()
        bus_read_d  = Signal()
        rd_ack_pipe = Signal()
        self.comb += bus_read.eq(bus.cyc & bus.stb & ~bus.we & (bus.cti == 0))
        self.sync += [  # This is the bus responder -- only works for uncached memory regions
            bus_read_d.eq(bus_read),
            If(bus_read & ~bus_read_d, # One response, one cycle
                rd_ack_pipe.eq(1),
                If(rfifo.level != 0,
                    bus.dat_r.eq(rfifo.source.payload.in0 | rfifo.source.payload.in1 << 16),
                    rfifo.source.ready.eq(1),
                ).Else(
                    # Don't stall the bus indefinitely if we try to read from an empty fifo...just
                    # return garbage
                    bus.dat_r.eq(0xdeadbeef),
                    rfifo.source.ready.eq(0),
                )
            ).Else(
                rfifo.source.ready.eq(0),
                rd_ack_pipe.eq(0),
            ),
            rd_ack.eq(rd_ack_pipe),
        ]

        # BUS responder // WRITE side
        """
        self.sync += [
            # This is the bus responder -- need to check how this interacts with uncached memory
            # region
            If(bus.cyc & bus.stb & bus.we & ~bus.ack,
                If(~fifo.full,
                    fifo.wr_d.eq(bus.dat_w),
                    fifo.wren.eq(~tx_reset),
                    wr_ack.eq(1),
                ).Else(
                    fifo.wren.eq(0),
                    wr_ack.eq(0),
                )
            ).Else(
                fifo.wren.eq(0),
                wr_ack.eq(0),
            )
            ]
        """

    def __init__(self, platform, pads, w=16, output_csr_read_only=True, with_fifos=True, sim=False):
        self.w = w
        self.cal_mem_file = os.path.join(SOURCES_ROOT, "cal/cal_mem_default_r33.hex")
        self.codec_cfg_file = os.path.join(SOURCES_ROOT, "drivers/ak4619-cfg.hex")
        self.led_cfg_file = os.path.join(SOURCES_ROOT, "drivers/pca9635-cfg.hex")

        # Exposed signals

        self.clk_256fs = ClockSignal("clk_256fs")
        self.clk_fs = ClockSignal("clk_fs")

        self.rst = ResetSignal("sys")

        self.cal_in0 = Signal((w, True))
        self.cal_in1 = Signal((w, True))
        self.cal_in2 = Signal((w, True))
        self.cal_in3 = Signal((w, True))
        self.cal_out0 = Signal((w, True))
        self.cal_out1 = Signal((w, True))
        self.cal_out2 = Signal((w, True))
        self.cal_out3 = Signal((w, True))

        self.eeprom_mfg = Signal(8)
        self.eeprom_dev = Signal(8)
        self.eeprom_serial = Signal(32)
        self.jack = Signal(8)

        # Exposed (for debugging)

        self.sample_adc0 = Signal((w, True))
        self.sample_adc1 = Signal((w, True))
        self.sample_adc2 = Signal((w, True))
        self.sample_adc3 = Signal((w, True))

        self.force_dac_output = Signal((w, True))

        # Internal signals

        self.i2c_scl_oe = Signal()
        self.i2c_scl_i = Signal()
        self.i2c_sda_oe = Signal()
        self.i2c_sda_i = Signal()

        # Verilog sources

        platform.add_verilog_include_path(SOURCES_ROOT)
        platform.add_sources(SOURCES_ROOT, "eurorack_pmod.sv")
        platform.add_sources(SOURCES_ROOT, "drivers/pmod_i2c_master.sv")
        platform.add_sources(SOURCES_ROOT, "drivers/ak4619.sv")
        platform.add_sources(SOURCES_ROOT, "cal/cal.sv")
        platform.add_sources(SOURCES_ROOT, "external/no2misc/rtl/i2c_master.v")

        self.specials += Instance("eurorack_pmod",
            # Parameters
            p_W = self.w,
            p_CODEC_CFG_FILE = self.codec_cfg_file,
            p_LED_CFG_FILE = self.led_cfg_file,

            # Ports (clk + reset)
            i_clk_256fs = self.clk_256fs,
            i_clk_fs = self.clk_fs,
            i_rst = self.rst,

            # Pads (tristate, require different logic to hook these
            # up to pads depending on the target platform).
            o_i2c_scl_oe = self.i2c_scl_oe,
            i_i2c_scl_i = self.i2c_scl_i,
            o_i2c_sda_oe = self.i2c_sda_oe,
            i_i2c_sda_i = self.i2c_sda_i,

            # Pads (directly hooked up to pads without extra logic required)
            o_pdn = pads.pdn,
            o_mclk = pads.mclk,
            o_sdin1 = pads.sdin1,
            i_sdout1 = pads.sdout1,
            o_lrck = pads.lrck,
            o_bick = pads.bick,

            # Ports (clock at clk_fs)
            o_cal_in0 = self.cal_in0,
            o_cal_in1 = self.cal_in1,
            o_cal_in2 = self.cal_in2,
            o_cal_in3 = self.cal_in3,
            i_cal_out0 = self.cal_out0,
            i_cal_out1 = self.cal_out1,
            i_cal_out2 = self.cal_out2,
            i_cal_out3 = self.cal_out3,

            # Ports (serialized data fetched over I2C)
            o_eeprom_mfg = self.eeprom_mfg,
            o_eeprom_dev = self.eeprom_dev,
            o_eeprom_serial = self.eeprom_serial,
            o_jack = self.jack,

            # Debug ports
            o_sample_adc0 = self.sample_adc0,
            o_sample_adc1 = self.sample_adc1,
            o_sample_adc2 = self.sample_adc2,
            o_sample_adc3 = self.sample_adc3,
            i_force_dac_output = self.force_dac_output,
        )


        if not sim:
            # FIXME: For now these tristate implementations are ECP5 specific.

            self.specials += Instance("TRELLIS_IO",
                p_DIR = "BIDIR",
                i_B   = pads.i2c_scl,
                i_I   = 0,
                o_O   = self.i2c_scl_i,
                i_T   = ~self.i2c_scl_oe
            )

            self.specials += Instance("TRELLIS_IO",
                p_DIR = "BIDIR",
                i_B   = pads.i2c_sda,
                i_I   = 0,
                o_O   = self.i2c_sda_i,
                i_T   = ~self.i2c_sda_oe
            )
        else:
            # No need for special IO buffers if in simulation.
            self.comb += [
                pads.i2c_sda.eq(~self.i2c_sda_oe),
                pads.i2c_scl.eq(~self.i2c_scl_oe),
            ]

        # Exposed CSRs

        self.csr_cal_in0 = CSRStatus(16)
        self.csr_cal_in1 = CSRStatus(16)
        self.csr_cal_in2 = CSRStatus(16)
        self.csr_cal_in3 = CSRStatus(16)

        if output_csr_read_only:
            self.csr_cal_out0 = CSRStatus(16)
            self.csr_cal_out1 = CSRStatus(16)
            self.csr_cal_out2 = CSRStatus(16)
            self.csr_cal_out3 = CSRStatus(16)
        else:
            self.csr_cal_out0 = CSRStorage(16)
            self.csr_cal_out1 = CSRStorage(16)
            self.csr_cal_out2 = CSRStorage(16)
            self.csr_cal_out3 = CSRStorage(16)

        self.csr_eeprom_mfg = CSRStatus(8)
        self.csr_eeprom_dev = CSRStatus(8)
        self.csr_eeprom_serial = CSRStatus(32)
        self.csr_jack = CSRStatus(8)

        # Connect CSRs directly to inputs and outputs

        self.comb += [
                self.csr_cal_in0.status.eq(self.cal_in0),
                self.csr_cal_in1.status.eq(self.cal_in1),
                self.csr_cal_in2.status.eq(self.cal_in2),
                self.csr_cal_in3.status.eq(self.cal_in3),
                self.csr_eeprom_mfg.status.eq(self.eeprom_mfg),
                self.csr_eeprom_dev.status.eq(self.eeprom_dev),
                self.csr_eeprom_serial.status.eq(self.eeprom_serial),
                self.csr_jack.status.eq(self.jack)
        ]

        if output_csr_read_only:
            self.comb += [
                    self.csr_cal_out0.status.eq(self.cal_out0),
                    self.csr_cal_out1.status.eq(self.cal_out1),
                    self.csr_cal_out2.status.eq(self.cal_out2),
                    self.csr_cal_out3.status.eq(self.cal_out3),
            ]
        else:
            self.comb += [
                    self.cal_out0.eq(self.csr_cal_out0.storage),
                    self.cal_out1.eq(self.csr_cal_out1.storage),
                    self.cal_out2.eq(self.csr_cal_out2.storage),
                    self.cal_out3.eq(self.csr_cal_out3.storage),
            ]

        if with_fifos:
            self.add_fifos()
