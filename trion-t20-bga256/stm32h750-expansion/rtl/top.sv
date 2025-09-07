`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* efinix-tests                                                                                                         *
*                                                                                                                      *
* Copyright (c) 2023-2025 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

module top(

	//50 MHz board clock
	input wire			clk_50mhz,

	//FMC interface
	input wire			fmc_clk,
	output wire			fmc_nwait,
	input wire			fmc_noe,
	inout wire[15:0]	fmc_ad,
	input wire			fmc_nwe,
	input wire[1:0]		fmc_nbl,
	input wire			fmc_nl_nadv,
	input wire[6:0]		fmc_a_hi,
	input wire			fmc_ne1,

	//SPI flash for FPGA boot image
	output wire			flash_cs_n,
	output wire			flash_sck,
	output wire			flash_mosi,
	input wire			flash_miso,

	//GPIO LEDs (inverted)
	output wire[7:0]	led_n,

	//Ethernet
	output wire			eth_rst_n,
	inout wire			eth_mdio,
	output wire			eth_mdc
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output port glue

	//Invert active-low LED signals
	wire[7:0]		led_int;
	assign led_n = ~led_int;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset logic clocked by the external 50 MHz clock (real designs will need external reset input)

	logic		pll_rst_n	= 0;
	logic[7:0]	rst_count	= 1;

	always_ff @(posedge clk_50mhz) begin
		if(!pll_rst_n) begin
			rst_count	<= rst_count + 8'h1;
			if(rst_count == 0)
				pll_rst_n	<= 1;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PLL

	wire	pll_lock;
	wire	clk_fb;
	wire	pclk;
	EFX_PLL_V2 #(
		.N(1),				//pre divide = 1, so xx MHz at the PFD (range 10 - 100)
		.M(1),				//multiplier 16, so xx MHz at the VCO (range 500 - 1600 for internal FB, 500 - 3600 for other)
		.O(8),				//post divider between VCO and all outputs
							//(must be 2 or higher if multiple outputs active)
		.CLKOUT0_DIV(4),	//60 MHz output for feedback alignment
		.CLKOUT1_DIV(4),	//60 MHz output but with adjustable phase
		.CLKOUT2_DIV(128),	//not used, slow to save power
		.CLKOUT0_PHASE(0),
		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(0),
		.FEEDBACK_CLK("CLK0"),
		.FEEDBACK_MODE("LOCAL"),
		.REFCLK_FREQ(62.5)	//refclk frequency in MHz
	) pll (
		.CLKIN({3'b0, fmc_clk}),
		.CLKSEL(2'b00),
		.RSTN(pll_rst_n),
		.FBK(pclk),
		.CLKOUT0(pclk),
		.CLKOUT1(),
		.CLKOUT2(),
		.LOCKED(pll_lock)
	);

	//Synchronize PLL lock signal into the APB clock domain
	wire	pll_lock_sync;
	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_pll_lock(
		.clk_in(pclk),
		.din(pll_lock),
		.clk_out(pclk),
		.dout(pll_lock_sync));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB bridge

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(27), .USER_WIDTH(0)) fmc_apb();
	APB #(.DATA_WIDTH(64), .ADDR_WIDTH(27), .USER_WIDTH(0)) fmc_apb64();

	FMC_APBBridge_EFX bridge(
		.apb_x32(fmc_apb),
		.apb_x64(fmc_apb64),

		.pll_lock(pll_lock),

		.fmc_clk(pclk),
		.fmc_nwait(fmc_nwait),
		.fmc_noe(fmc_noe),
		.fmc_ad(fmc_ad),
		.fmc_nwe(fmc_nwe),
		.fmc_nbl(fmc_nbl),
		.fmc_nl_nadv(fmc_nl_nadv),
		.fmc_a_hi({3'b0, fmc_a_hi}),
		.fmc_cs_n(fmc_ne1)
	);

	//Root APB interconnect
	//Two 16-bit bus segments at 0xc000_0000 (APB1) and c001_0000 (APB2) for core peripherals
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(16), .USER_WIDTH(0)) rootAPB[1:0]();
	APBBridge #(
		.BASE_ADDR(32'h0000_0000),
		.BLOCK_SIZE(32'h1_0000),
		.NUM_PORTS(2)
	) root_bridge (
		.upstream(fmc_apb),
		.downstream(rootAPB)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB1 (0xc000_0000 - c000_ffff)

	Peripherals_APB1 apb1(
		.apb(rootAPB[0]),

		.flash_cs_n(flash_cs_n),
		.flash_sck(flash_sck),
		.flash_mosi(flash_mosi),
		.flash_miso(flash_miso),

		.led(led_int),

		.eth_rst_n(eth_rst_n),
		.eth_mdio(eth_mdio),
		.eth_mdc(eth_mdc)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB2 (0xc001_0000 - c001_ffff) eventually

	//tieoffs
	assign rootAPB[1].prdata = 0;
	assign rootAPB[1].pslverr = 0;
	assign rootAPB[1].pready = rootAPB[1].penable;
	assign rootAPB[1].pruser = 0;
	assign rootAPB[1].pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TODO: whatever lives on apb64

	//tieoffs
	assign fmc_apb64.prdata = 0;
	assign fmc_apb64.pslverr = 0;
	assign fmc_apb64.pready = fmc_apb64.penable;
	assign fmc_apb64.pruser = 0;
	assign fmc_apb64.pbuser = 0;

endmodule
