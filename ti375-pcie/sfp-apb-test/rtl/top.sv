`timescale 1ns / 1ps
`default_nettype none

module top(

	//PLL input clock
	input wire	clk_25m_2,

	//Debug LEDs
	output wire[5:0] led,

	//SDIO pins used for GPIO expansion
	output wire		sdio_dat0,
	input wire		sdio_dat1,

	//serdes apparently needs this enabled?
	//we *cannot* have the enable pin hooked up, even tied to a constant, or we get osc_rule_ena_pin_transceiver errors
	(* syn_peri_port = 0 *) input wire osc_CLKOUT,

	//SFP+ interface: SERDES
	(* syn_peri_port = 0 *) input wire	sfp_quad_PMA_CMN_READY,
	(* syn_peri_port = 0 *) input wire	sfp_a_tx_clk,
	(* syn_peri_port = 0 *) input wire	sfp_a_rx_clk,
	(* syn_peri_port = 0 *) output wire sfp_a_PCS_RST_N_RX,
	(* syn_peri_port = 0 *) output wire sfp_a_PCS_RST_N_TX,
	(* syn_peri_port = 0 *) output wire sfp_a_PHY_RESET_N,
	//(* syn_peri_port = 0 *) output wire sfp_a_PMA_TX_ELEC_IDLE,
	(* syn_peri_port = 0 *) input wire[63:0] sfp_a_RXD,		//only 31:0 used
	(* syn_peri_port = 0 *) output wire[63:0] sfp_a_TXD,		//only 31:0 used
	(* syn_peri_port = 0 *) input wire sfp_a_PMA_XCVR_PLLCLK_EN_ACK,
	(* syn_peri_port = 0 *) output wire sfp_a_PMA_XCVR_PLLCLK_EN,
	(* syn_peri_port = 0 *) input wire[3:0] sfp_a_PMA_XCVR_POWER_STATE_ACK,
	(* syn_peri_port = 0 *) output wire[3:0] sfp_a_PMA_XCVR_POWER_STATE_REQ,
	(* syn_peri_port = 0 *) input wire sfp_a_PMA_RX_SIGNAL_DETECT,

	//SFP+ interface: low speed GPIOs
	input wire			sfp_a_tx_fault,
	output wire			sfp_a_tx_disable,
	input wire			sfp_a_rx_los,
	output wire[1:0]	sfp_a_rs,
	input wire			sfp_a_mod_abs
	//ignore i2c bus for now
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off reset etc for now

	assign sfp_a_PCS_RST_N_RX = 1;
	assign sfp_a_PCS_RST_N_TX = 1;
	assign sfp_a_PHY_RESET_N = 1;
	//assign sfp_a_PMA_TX_ELEC_IDLE = 0;
	assign sfp_a_PMA_XCVR_PLLCLK_EN = 1;
	assign sfp_a_PMA_XCVR_POWER_STATE_REQ = 4'b0001;	//TX/RX active

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug: send PRBS31

	wire[19:0] prbs_out;

	PRBS31 #(
		.WIDTH(20),
		.INITIAL_SEED(31'h1)
	) prbs (
		.clk(sfp_a_tx_clk),
		.update(1),
		.init(0),
		.seed(31'h1),
		.dout(prbs_out)
	);

	//Weird lane mapping... 51:32 and 19:0 are valid in 40 bit mode??
	//assign sfp_a_TXD = { 12'h0, prbs_out[39:20], 12'h0, prbs_out[19:0] };
	assign sfp_a_TXD = { 44'h0, prbs_out[19:0] };
	//assign sfp_a_TXD = { 24'h0, prbs_out };

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Extract incoming raw data

	// Seems like there's no 8b10b decoding available in PMA direct mode? do we have to do that ourselves?

	wire[19:0]	sfp_a_rx_data = sfp_a_RXD[19:0];
	wire		rx_bitslip;
	(* mark_debug *) wire[19:0]	sfp_rx_data_slipped;

	BitslipAligner #(
		.WIDTH(20),
		.REVERSE(1)
	) rx_gearbox (
		.clk(sfp_a_rx_clk),
		.data_in(sfp_a_rx_data),
		.bitslip(rx_bitslip),
		.data_out(sfp_rx_data_slipped)
	);

	wire		rx_symbol_locked;
	wire		rx_no_commas;
	SymbolAligner8b10b rx_comma_aligner(
		.clk(sfp_a_rx_clk),
		.codeword_valid(1),
		.comma_window(sfp_rx_data_slipped),
		.locked(rx_symbol_locked),
		.no_commas(rx_no_commas),
		.bitslip(rx_bitslip)
	);

	(* mark_debug *) wire[1:0]	rx_data_is_ctl;
	(* mark_debug *) wire[15:0]	rx_data;
	(* mark_debug *) wire[1:0]	rx_symbol_err;

	for(genvar g=0; g<2; g++) begin : rxlanes
		Decode8b10b_Lane rx_decode(
			.clk(sfp_a_rx_clk),
			.codeword_valid(1),
			.codeword_in(sfp_rx_data_slipped[g*10 +: 10]),
			.data_valid(),
			.data(rx_data[g*8 +: 8]),
			.data_is_ctl(rx_data_is_ctl[g]),
			.symbol_err(rx_symbol_err[g])
		);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Seems we have no direct clock inputs, everything has to go through a PLL

	wire	pll_lock;
	wire	clk_100mhz;

	EFX_FPLL_V1 #(
		.N(1),				//25 MHz / 1 = 25 MHz reference at the PFD
		.M(4),				//25 MHz at the PFD * 4 = 100 MHz Fpll
		.O(16),				//100 MHz * 16 * 2 = 3200 MHz VCO (CLKOUT0_DIV is in feedback path(
		.CLKOUT0_DIV(2),	//100 MHz system clock
		.CLKOUT1_DIV(1),
		.CLKOUT2_DIV(1),
		.CLKOUT3_DIV(1),
		.CLKOUT4_DIV(1),
		.CLKOUT0_PHASE_STEP(0),
		.CLKOUT1_PHASE_STEP(0),
		.CLKOUT2_PHASE_STEP(0),
		.CLKOUT3_PHASE_STEP(0),
		.CLKOUT4_PHASE_STEP(0),
		.FEEDBACK_CLK("CLK0"),
		.FEEDBACK_MODE("LOCAL"),
		.REFCLK_FREQ(25),
		.IS_CLKOUT0_INVERTED(0),
		.IS_CLKOUT1_INVERTED(0),
		.IS_CLKOUT2_INVERTED(0),
		.IS_CLKOUT3_INVERTED(0),
		.IS_CLKOUT4_INVERTED(0),
		.CLKOUT3_CONN_TYPE("GCLK"),
		.CLKOUT4_CONN_TYPE("GCLK"),
		.CLKOUT0_DYNPHASE_EN(0),
		.CLKOUT1_DYNPHASE_EN(0),
		.CLKOUT2_DYNPHASE_EN(0),
		.CLKOUT3_DYNPHASE_EN(0),
		.CLKOUT4_DYNPHASE_EN(0),
		.DYNAMIC_CFG_EN(0),
		.SSC_MODE("DISABLE"),
		.CLKOUT1_FRAC_EN(0),
		.CLKOUT1_DC_ODD(0),
		.CLKOUT1_PROG_DUTY_CYCLE_EN(0)
	) pll (
		.CLKIN({3'b0, clk_25m_2}),
		.CLKSEL(2'b0),
		.RSTN(1'b1),
		.FBK(clk_100mhz),
		.CLKOUT0(clk_100mhz),
		.CLKOUT1(),
		.CLKOUT2(),
		.CLKOUT3(),
		.CLKOUT4(),
		.LOCKED(pll_lock),
		.SHIFT(3'b0),
		.SHIFT_SEL(5'b0),
		.SHIFT_ENA(1'b0),
		.CFG_CLK(1'b0),
		.CFG_DATA_IN(1'b0),
		.CFG_DATA_OUT(),
		.CFG_SEL(1'b0),
		.USER_SSC_EN(1'b0)
	);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug APB bridge from PC to FPGA

	APB #(.ADDR_WIDTH(32), .DATA_WIDTH(32), .USER_WIDTH(0)) apbRoot();

	UART_APBBridge #(
		.DEBUG_ROM_ADDR(32'h4000_0000)
	) uartBridge (
		.clk(clk_100mhz),
		.rst_n(1'b1),
		.baud_div(/*16'd100*/16'd33),	//3 Mbaud (highest supported by FT232R)

		.uart_rx(sdio_dat1),
		.uart_tx(sdio_dat0),

		.apb(apbRoot));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Top level APB interconnect

	localparam NUM_L1 = 3;
	APB #(.ADDR_WIDTH(10), .DATA_WIDTH(32), .USER_WIDTH(0)) apbLevel1[NUM_L1-1:0]();

	APBBridge #(
		.BASE_ADDR(32'h4000_0000),
		.BLOCK_SIZE(32'h400),
		.NUM_PORTS(NUM_L1)
	) rootBridge (
		.upstream(apbRoot),
		.downstream(apbLevel1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ROM table at start of debug APB (0x4000_0000)

	DebugROM #(
		.DEVICE_0_TYPE("GPIO"),
		.DEVICE_0_ADDR(32'h4000_0400),

		.DEVICE_1_TYPE("VIO_"),
		.DEVICE_1_ADDR(32'h4000_0800)
	) rom (
		.apb(apbLevel1[0])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// GPIO controller (0x4000_0400)

	wire[31:0] gpio_out;
	wire[31:0] gpio_in;

	APB_GPIO #(
		.TRIS_INIT(0),
		.OUT_INIT(0)
	) gpio (
		.apb(apbLevel1[1]),
		.gpio_out(gpio_out),
		.gpio_in(gpio_in),
		.gpio_tris());

	assign led 				= gpio_out[5:0];
	assign sfp_a_tx_disable	= gpio_out[6];
	assign sfp_a_rs			= gpio_out[8:7];

	assign gpio_in[0]		= sfp_a_tx_fault;
	assign gpio_in[1]		= sfp_a_rx_los;
	assign gpio_in[2]		= sfp_a_mod_abs;
	assign gpio_in[31:3]	= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// VIO (0x4000_0800)

	APB_VIO #(
		.OUT0_NAME("foo"),
		.OUT0_WIDTH(16),
		.OUT1_NAME("bar"),
		.OUT1_WIDTH(32),
		.OUT2_NAME("baz"),
		.OUT2_WIDTH(64),
		.OUT3_NAME("foobar"),
		.OUT3_WIDTH(64),

		.IN0_NAME("input_a"),
		.IN0_WIDTH(32),
		.IN1_NAME("kibby"),
		.IN1_WIDTH(32)
	) vio (
		.apb(apbLevel1[2]),

		.probe_out0(),
		.probe_out1(),
		.probe_out2(),
		.probe_out3(),
		.probe_out4(),
		.probe_out5(),
		.probe_out6(),
		.probe_out7(),

		.probe_in0(32'hcafebabe),
		.probe_in1(32'hc0def00d),
		.probe_in2(),
		.probe_in3(),
		.probe_in4(),
		.probe_in5(),
		.probe_in6(),
		.probe_in7()
	);

endmodule
