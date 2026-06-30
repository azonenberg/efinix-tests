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

	//SFP+ interface
	(* syn_peri_port = 0 *) input wire	sfp_quad_PMA_CMN_READY,
	(* syn_peri_port = 0 *) input wire	prbs_tx_clk,
	(* syn_peri_port = 0 *) input wire	prbs_rx_clk,
	(* syn_peri_port = 0 *) output wire prbs_gen_PCS_RST_N_RX,
	(* syn_peri_port = 0 *) output wire prbs_gen_PCS_RST_N_TX,
	(* syn_peri_port = 0 *) output wire prbs_gen_PHY_RESET_N,
	(* syn_peri_port = 0 *) output wire prbs_gen_PMA_TX_ELEC_IDLE,
	(* syn_peri_port = 0 *) input wire[63:0] prbs_gen_RXD,		//only 31:0 used
	(* syn_peri_port = 0 *) output wire[63:0] prbs_gen_TXD,		//only 31:0 used
	(* syn_peri_port = 0 *) input wire prbs_gen_PMA_XCVR_PLLCLK_EN_ACK,
	(* syn_peri_port = 0 *) output wire prbs_gen_PMA_XCVR_PLLCLK_EN,
	(* syn_peri_port = 0 *) input wire[3:0] prbs_gen_PMA_XCVR_POWER_STATE_ACK,
	(* syn_peri_port = 0 *) output wire[3:0] prbs_gen_PMA_XCVR_POWER_STATE_REQ,
	(* syn_peri_port = 0 *) input wire prbs_gen_PMA_RX_SIGNAL_DETECT
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off reset etc for now

	assign prbs_gen_PCS_RST_N_RX = 1;
	assign prbs_gen_PCS_RST_N_TX = 1;
	assign prbs_gen_PHY_RESET_N = 1;
	assign prbs_gen_PMA_TX_ELEC_IDLE = 0;
	assign prbs_gen_PMA_XCVR_PLLCLK_EN = 1;
	assign prbs_gen_PMA_XCVR_POWER_STATE_REQ = 4'b0001;	//TX/RX active

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug: send PRBS31

	wire[31:0] prbs_out;

	PRBS31 #(
		.WIDTH(32),
		.INITIAL_SEED(31'h1)
	) prbs (
		.clk(prbs_tx_clk),
		.update(1),
		.init(0),
		.seed(31'h1),
		.dout(prbs_out)
	);

	assign prbs_gen_TXD = { 32'h0000_0000, prbs_out };

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

	APB_GPIO gpio(
		.apb(apbLevel1[1]),
		.gpio_out(gpio_out),
		.gpio_in(32'hff0055aa),
		.gpio_tris());

	assign led = gpio_out[5:0];

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
