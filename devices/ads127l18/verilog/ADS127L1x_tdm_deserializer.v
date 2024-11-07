//////////////////////////////////////////////////////////////////////////////////
// Company: TEXAS INSTRUMENTS
// Target Devices: ADS127L18, ADS127L14
// Description: Module for latching ADC data from DATA Port of ADS127L1x
// Version: 3.3
//////////////////////////////////////////////////////////////////////////////////

module ADS127L18_tdm_deserializer #(     
    parameter LANE_COUNT = 4,           // Select [1|2|4|8]
    parameter BITS_PER_PACKET = 24      // Select [16|24|32|40] (packets may contain: status[8|0] + data[24|16] + crc[8|0])
)(
    input ADC_FSYNC,    // FSYNC pin from ADC
    input ADC_DCLK,     // DCLK pin from ADC
    input ADC_DOUT0,    // DOUT0 pin from ADC
    input ADC_DOUT1,    // DOUT1 pin from ADC
    input ADC_DOUT2,    // DOUT2 pin from ADC
    input ADC_DOUT3,    // DOUT3 pin from ADC
    input ADC_DOUT4,    // DOUT4 pin from ADC
    input ADC_DOUT5,    // DOUT5 pin from ADC
    input ADC_DOUT6,    // DOUT6 pin from ADC
    input ADC_DOUT7,    // DOUT7 pin from ADC
    
    output reg [BITS_PER_PACKET-1:0] ch0_packet, // CH0 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch1_packet, // CH1 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch2_packet, // CH2 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch3_packet, // CH3 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch4_packet, // CH4 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch5_packet, // CH5 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch6_packet, // CH6 data packet (latched)
    output reg [BITS_PER_PACKET-1:0] ch7_packet, // CH7 data packet (latched)
    
    output reg data_ready   // Goes high for at least 1 DCLK period after data is latched
);

    localparam CHANNEL_COUNT = 8;                                       // ADS127L18 = 8 channels, ADS127L14 = 4 channels
    localparam CHANNELS_PER_LANE = (CHANNEL_COUNT / LANE_COUNT);        // integer value between '1' and '8'
    localparam BITS_PER_LANE = (BITS_PER_PACKET * CHANNELS_PER_LANE);
    localparam DCLK_DATA_COUNT = (CHANNELS_PER_LANE * BITS_PER_PACKET) - 1;   // Max value = 320 - 1 (to account for counter = 0)
    
    //// Shift registers ////
    // Generate a shift-register for each DOUT lane. Data will be MSB first and shifted on each risging edge of DCLK. 
    reg [BITS_PER_LANE-1:0] lane_shifters [LANE_COUNT-1:0]; // 2D array
    generate 
        always @(posedge ADC_DCLK) begin
            if (LANE_COUNT >= 1) begin
                lane_shifters[0][BITS_PER_LANE-1:1] <= lane_shifters[0][BITS_PER_LANE-2:0];
                lane_shifters[0][0] <= ADC_DOUT0;
            end
            if (LANE_COUNT >= 2) begin
                lane_shifters[1][BITS_PER_LANE-1:1] <= lane_shifters[1][BITS_PER_LANE-2:0];
                lane_shifters[1][0] <= ADC_DOUT1;
            end
            if (LANE_COUNT >= 4) begin
                lane_shifters[2][BITS_PER_LANE-1:1] <= lane_shifters[2][BITS_PER_LANE-2:0];
                lane_shifters[2][0] <= ADC_DOUT2;
                
                lane_shifters[3][BITS_PER_LANE-1:1] <= lane_shifters[3][BITS_PER_LANE-2:0];
                lane_shifters[3][0] <= ADC_DOUT3;
            end
            if (LANE_COUNT >= 8) begin
                lane_shifters[4][BITS_PER_LANE-1:1] <= lane_shifters[4][BITS_PER_LANE-2:0];
                lane_shifters[4][0] <= ADC_DOUT4;
                
                lane_shifters[5][BITS_PER_LANE-1:1] <= lane_shifters[5][BITS_PER_LANE-2:0];
                lane_shifters[5][0] <= ADC_DOUT5;
                
                lane_shifters[6][BITS_PER_LANE-1:1] <= lane_shifters[6][BITS_PER_LANE-2:0];
                lane_shifters[6][0] <= ADC_DOUT6;
                
                lane_shifters[7][BITS_PER_LANE-1:1] <= lane_shifters[7][BITS_PER_LANE-2:0];
                lane_shifters[7][0] <= ADC_DOUT7;
            end
        end
    endgenerate
    
    
    //// Lane decoder ////
    // Decoding to locate which lane and which shift-register bits contain the desired packet...
    wire [BITS_PER_PACKET-1:0] packet_wires [CHANNEL_COUNT-1:0];
    genvar channel;
    generate
        for (channel = 0; channel < CHANNEL_COUNT; channel = channel + 1) begin : lane_decoder
            localparam LANE_NUMBER = (channel / CHANNELS_PER_LANE);
            localparam PACKET_INDEX = CHANNELS_PER_LANE - (channel % CHANNELS_PER_LANE);    // 1-indexed, reverse-order
            localparam STOP_BIT = (PACKET_INDEX * BITS_PER_PACKET) - 1;
            localparam START_BIT = (PACKET_INDEX - 1) * BITS_PER_PACKET;
            assign packet_wires[channel] = lane_shifters[LANE_NUMBER][STOP_BIT:START_BIT];
        end
    endgenerate
    
    
    //// Data latch ////
    // Latch the decoded data after the required # of DCLK pulses have occurred...
    reg fsync_went_low = 1'b0;  // Flag to check if we've pasted the mid-point of a frame (FSYNC goes LOW)
    reg [8:0] counter = 9'd0;   // DCLK decrementing counter. When counter value reaches zero, all data bits have been clocked into the shift register(s).
    always @(posedge ADC_DCLK) begin
        if (counter > 0) begin
            counter <= counter - 1'b1;  // Decrement counter
            data_ready <= 1'b0;         // 'data_ready' is zero while counter is decrementing
        end
        if ((counter == 0) & (data_ready == 1'b0)) begin    
            ch0_packet <= packet_wires[0];  // Latch data packets as soon as the DCLK counter counter reaches zero
            ch1_packet <= packet_wires[1];  // 'data_ready' is used to determine if the data has already been latched
            ch2_packet <= packet_wires[2];
            ch3_packet <= packet_wires[3];
            ch4_packet <= packet_wires[4];
            ch5_packet <= packet_wires[5];
            ch6_packet <= packet_wires[6];
            ch7_packet <= packet_wires[7];
        end
        if (ADC_FSYNC == 1'b0) begin    // Set the 'fsync_went_low' flag as soon FSYNC goes low. This is used to prevent the    
            fsync_went_low <= 1'b1;     // counter from resetting in cases where all data bits are latched before FSYNC goes low.
        end
        if ((counter == 0) & ADC_FSYNC & fsync_went_low) begin
            counter <= DCLK_DATA_COUNT[8:0];    // Reset the counter
            fsync_went_low <= 1'b0;             // Reset the 'fsync_went_low' flag
        end
        if ((counter == 0)) begin
            data_ready <= 1'b1;     // assert 'data_ready'
        end
    end

endmodule