module top(
	input wire 			clk_50mhz,

	output logic[7:0]	led_n
);

	//efinix tools don't let you initialize outputs at declaration
	initial led_n = 8'hff;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual blinky

	logic[7:0] led = 0;
	logic[20:0] count = 0;

	always_ff @(posedge clk_50mhz) begin
		count	<= count + 1;
		if(count == 0)
			led <= led + 1;

		led_n	<= ~led;
	end

endmodule
