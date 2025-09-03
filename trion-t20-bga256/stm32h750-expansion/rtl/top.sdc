# Clock inputs
create_clock -period 20 -waveform {0.0 10.0} -name clk_50mhz [get_ports clk_50mhz]
create_clock -period 13.333 -waveform {0.0 6.666} -name fmc_clk [get_ports fmc_clk]

# rename the PLL clock
#create_generated_clock -name pclk -source [get_pins pll/CLKOUT0] [get_clocks pll/CLKOUT0]
create_generated_clock -name pclk -source [get_clocks fmc_clk] -multiply_by 1 [get_pins pll/CLKOUT0]

# for now make them async
