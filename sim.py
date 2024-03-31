#!/usr/bin/env python3

import lxbuildenv

from litex.tools.litex_sim import main

from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.cores.dma import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.integration.soc import SoCRegion

from rtl.eurorack_pmod_wrapper import *

CLK_FREQ_SYS = 5e6
CLK_FREQ_256FS = 1e6
CLK_FREQ_FS = CLK_FREQ_256FS / 256

_io_extra_clockers = [
    ("clocker_256fs", 0, Pins(1)),
]

_io_eurorack_pmod = [
    ("eurorack_pmod_p0", 0,
        Subsignal("mclk",    Pins(1)),
        Subsignal("pdn",     Pins(1)),
        Subsignal("i2c_sda", Pins(1)),
        Subsignal("i2c_scl", Pins(1)),
        Subsignal("sdin1",   Pins(1)),
        Subsignal("sdout1",  Pins(1)),
        Subsignal("lrck",    Pins(1)),
        Subsignal("bick",    Pins(1)),
    ),
]

def add_eurorack_pmod(soc):
    soc.platform.add_extension(_io_eurorack_pmod)

    # Create 1*Fs clock domain using a division register
    soc.cd_clk_fs = ClockDomain()
    clkdiv_fs = Signal(8)
    soc.sync.clk_256fs += clkdiv_fs.eq(clkdiv_fs+1)
    soc.comb += soc.cd_clk_fs.clk.eq(clkdiv_fs[-1])

    # Now instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request("eurorack_pmod_p0")
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads, sim=True)
    soc.add_module("eurorack_pmod0", eurorack_pmod)
    soc.bus.add_slave("i2s", eurorack_pmod.bus,
                      SoCRegion(origin=0xb1000000, size=512*16, cached=False))
    soc.irq.add("eurorack_pmod0", use_loc_if_exists=True)

def sim_soc_extension(sim_config, soc):
    soc.platform.add_extension(_io_extra_clockers)
    sim_config.add_clocker("clocker_256fs", freq_hz=int(CLK_FREQ_256FS))
    soc.cd_clk_256fs = ClockDomain()
    soc.comb += [
        soc.cd_clk_256fs.clk.eq(soc.platform.request("clocker_256fs")),
    ]
    add_eurorack_pmod(soc)
    sim_config.add_module("i2s", "eurorack_pmod_p0")

if __name__ == "__main__":
    main(sys_clk_freq=CLK_FREQ_SYS, soc_extension_hook=sim_soc_extension)
