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
	input wire			fmc_clk,

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
	// PLL

	//not synchronized to anything, may glitch, probably useless lol
	wire	pll_lock;
	always_ff @(posedge clk_50mhz) begin
		led_int[7]	<= pll_lock;
		led_int[6]	<= pll_rst_n;
	end

	wire	pclk;
	EFX_PLL_V2 #(
		.N(1),				//pre divide = 1, so 75 MHz at the PFD (range 10 - 100)
		.M(10),				//multiplier 10, so 750 MHz at the VCO (range 500 - 1600 for internal FB, 500 - 3600 for other)
		.O(2),				//post divider between VCO and all outputs (must be 2 or higher if multiple outputs active)
		.CLKOUT0_DIV(5),	//75 MHz output but with adjustable phase
		.CLKOUT1_DIV(128),	//not used, slow to save power
		.CLKOUT2_DIV(128),	//not used, slow to save power
		.CLKOUT0_PHASE(0),
		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(0),
		.FEEDBACK_CLK("INTERNAL"),
		.FEEDBACK_MODE("INTERNAL"),
		.REFCLK_FREQ(75)	//refclk frequency in MHz
	) pll (
		.CLKIN({3'b0, fmc_clk}),
		.CLKSEL(2'b00),
		.RSTN(pll_rst_n),
		.FBK(1'b0),
		.CLKOUT0(pclk),
		.CLKOUT1(),
		.CLKOUT2(),
		.LOCKED(pll_lock)
	);

	logic[20:0] count = 0;
	always_ff @(posedge pclk) begin
		count <= count + 1;
		if(count == 0)
			led_int[5:0]	<= led_int[5:0] + 1;
	end

endmodule
