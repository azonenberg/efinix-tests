# Clock inputs
create_clock -period 20 -waveform {0.0 10.0} -name clk_50mhz [get_ports clk_50mhz]

# can't constrain the pll input clock because it's not actually part of the core netlist!
#create_clock -period 13.333 -waveform {0.0 6.666} -name fmc_clk [get_ports fmc_clk]

# Constrain pll output
create_clock -period 13.333 -waveform {0.0 6.666} -name pclk [get_ports \bridge/pll~CLKOUT0]

# for now make them async
#set_clock_groups -asynchronous -group [get_clocks clk_50mhz] -group [get_clocks pclk]

# 10ns CDC constraint
set_max_delay -from [get_clocks pclk] -to [get_clocks clk_50mhz] 10.000
set_max_delay -from [get_clocks clk_50mhz] -to [get_clocks pclk] 10.000
