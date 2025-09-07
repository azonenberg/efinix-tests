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

module Peripherals_APB1(

	//Upstream bus interface
	APB.completer		apb,

	//GPIO LEDs
	output wire[7:0]	led,

	//Boot flash
	output wire			flash_cs_n,
	output wire			flash_sck,
	output wire			flash_mosi,
	input wire			flash_miso
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Root interconnect bridge (c000_0000, 1 kB / 0x400 segments per peripheral)

	//APB1 segment
	localparam NUM_PERIPHERALS	= 3;
	localparam BLOCK_SIZE		= 32'h400;
	localparam ADDR_WIDTH		= $clog2(BLOCK_SIZE);
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) apb1[NUM_PERIPHERALS-1:0]();
	APBBridge #(
		.BASE_ADDR(32'h0000_0000),
		.BLOCK_SIZE(BLOCK_SIZE),
		.NUM_PORTS(NUM_PERIPHERALS)
	) bridge (
		.upstream(apb),
		.downstream(apb1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Device information (c000_0000)

	//Trion has no way to determine runtime identity of the part, so we have to hard code here for compatibility
	APB_DeviceInfo_Trion #(
		.IDCODE(32'h00210a79)	//T20F256
	) devinfo (
		.apb(apb1[0])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LED GPIO (c000_0400)

	wire[31:0]	gpio_out;
	APB_GPIO gpioa (
		.apb(apb1[1]),
		.gpio_out(gpio_out),
		.gpio_in(gpio_out),	//loop back for readback
		.gpio_tris()
	);

	assign led = gpio_out[7:0];

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SPI controller for boot flash (c000_0800)

	APB_SPIHostInterface spi1(
		.apb(apb1[2]),

		.spi_sck(flash_sck),
		.spi_mosi(flash_mosi),
		.spi_miso(flash_miso),
		.spi_cs_n(flash_cs_n)
	);

endmodule
