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

module Peripherals_APB2(

	//125 MHz RGMII TX clock
	input wire					clk_125mhz,

	//Upstream bus interface
	APB.completer				apb,

	//Ethernet status flags
	input wire					link_up_phyclk,

	//Ethernet AXI interfaces
	AXIStream.receiver			eth_axi_rx,
	AXIStream.transmitter		eth_axi_tx,

	//IRQ pin to GPIO block
	output wire					irq
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Root interconnect bridge (c001_0000, 4 kB / 0x1000 segments per peripheral)

	//APB1 segment
	localparam NUM_PERIPHERALS	= 2;
	localparam BLOCK_SIZE		= 32'h1000;
	localparam ADDR_WIDTH		= $clog2(BLOCK_SIZE);
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) apb2[NUM_PERIPHERALS-1:0]();
	APBBridge #(
		.BASE_ADDR(32'h0000_0000),
		.BLOCK_SIZE(BLOCK_SIZE),
		.NUM_PORTS(NUM_PERIPHERALS)
	) bridge (
		.upstream(apb),
		.downstream(apb2)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cross AXI RX buses for Ethernet into management clock domain

	AXIStream #(.DATA_WIDTH(32), .ID_WIDTH(0), .DEST_WIDTH(0), .USER_WIDTH(1)) eth_axi_rx_pclk();

	AXIS_CDC #(
		.FIFO_DEPTH(1024)
	) eth_rx_cdc (
		.axi_rx(eth_axi_rx),

		.tx_clk(apb.pclk),
		.axi_tx(eth_axi_rx_pclk)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Shift link state flags into management clock domain

	wire	link_up;

	ThreeStageSynchronizer #(
		.IN_REG(1)
	) sync_link_up(
		.clk_in(eth_axi_rx.aclk),
		.din(link_up_phyclk),
		.clk_out(apb.pclk),
		.dout(link_up));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Interrupt pin generation

	wire	rx_frame_ready;

	assign irq = rx_frame_ready;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFOs for storing inbound/outbound Ethernet frames

	//RGMII RX FIFO (0xc001_0000)
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) eth_apb_rx();
	APBRegisterSlice #(.UP_REG(0), .DOWN_REG(1))
		apb_regslice_rx( .upstream(apb2[0]), .downstream(eth_apb_rx) );

	APB_AXIS_EthernetRxBuffer eth_rx_fifo(
		.apb(eth_apb_rx),
		.axi_rx(eth_axi_rx_pclk),
		.eth_link_up(link_up),
		.rx_frame_ready(rx_frame_ready)
	);

	//RGMII TX FIFO (0xc001_1000)
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) eth_apb_tx();
	APBRegisterSlice #(.UP_REG(0), .DOWN_REG(1))
		apb_regslice_tx( .upstream(apb2[1]), .downstream(eth_apb_tx) );

	APB_AXIS_EthernetTxBuffer eth_tx_fifo(
		.apb(eth_apb_tx),

		.tx_clk(clk_125mhz),
		.link_up_pclk(link_up),
		.axi_tx(eth_axi_tx)
	);

endmodule
