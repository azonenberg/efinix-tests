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

	output wire[7:0]	led_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output assignments so we have no top level logic ports b/c efinix tools are weird

	//(also invert active-low LED signals)
	logic[7:0]		led_int = 0;
	assign led_n = ~led_int;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset logic clocked by the external 50 MHz clock (real designs will need external reset input)

	logic		pll_rst_n	= 0;
	logic[7:0]	rst_count	= 1;

	always_ff @(posedge clk_50mhz) begin
		if(!pll_rst_n) begin
			rst_count	<= rst_count + 1;
			if(rst_count == 0)
				pll_rst_n	<= 1;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB bridge

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(25), .USER_WIDTH(0)) fmc_apb();
	APB #(.DATA_WIDTH(64), .ADDR_WIDTH(25), .USER_WIDTH(0)) fmc_apb64();

	FMC_APBBridge_EFX bridge(
		.apb_x32(fmc_apb),
		.apb_x64(fmc_apb64),

		.pll_rst_n(pll_rst_n),

		.fmc_clk(fmc_clk),
		.fmc_nwait(fmc_nwait),
		.fmc_noe(fmc_noe),
		.fmc_ad(fmc_ad),
		.fmc_nwe(fmc_nwe),
		.fmc_nbl(fmc_nbl),
		.fmc_nl_nadv(fmc_nl_nadv),
		.fmc_a_hi({3'b0, fmc_a_hi}),
		.fmc_cs_n(fmc_ne1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Root APB interconnect
	// Two 16-bit bus segments at 0xc000_0000 (APB1) and c001_0000 (APB2) for core peripherals

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
	// APB1 segment (1 kB segments starting at c000_0000)

	localparam NUM_PERIPHERALS	= 2;
	localparam BLOCK_SIZE		= 32'h400;
	localparam ADDR_WIDTH		= $clog2(BLOCK_SIZE);
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) apb1[NUM_PERIPHERALS-1:0]();
	APBBridge #(
		.BASE_ADDR(32'h0000_0000),
		.BLOCK_SIZE(BLOCK_SIZE),
		.NUM_PORTS(NUM_PERIPHERALS)
	) apb1_bridge (
		.upstream(rootAPB[0]),
		.downstream(apb1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// GPIO for LEDs (0xc000_0000)

	wire[31:0]	gpio_out;
	APB_GPIO gpio(
		.apb(apb1[0]),
		.gpio_out(gpio_out),
		.gpio_in(gpio_out),	//loop back for readback
		.gpio_tris()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs

	always_ff @(posedge apb1[0].pclk) begin
		led_int	<= gpio_out[7:0];
	end

endmodule
