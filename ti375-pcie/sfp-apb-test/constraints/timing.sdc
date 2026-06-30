# Constrain pll outputs
create_clock -period 10.000 -waveform {0.0 5.000} -name clk_100mhz [get_ports pll~CLKOUT0]

# Constrain SERDES clocks (322.265625 MHz)
create_clock -period 3.103 -waveform {0.0 1.552} -name sfp_a_tx_clk [get_ports sfp_a_tx_clk]
create_clock -period 3.103 -waveform {0.0 1.552} -name sfp_a_rx_clk [get_ports sfp_a_rx_clk]
