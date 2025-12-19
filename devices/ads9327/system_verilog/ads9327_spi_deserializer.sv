/*
 * Copyright (C) 2024-2025 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

`timescale 1ns / 1ps

module ads9327_spi_deserializer #(
    parameter LANES = 0,                 //Can be [0|1|2]
    parameter PACKET_SIZE_PER_LANE = 8   //Can be [8|10|12|16|20|24|48]
)(
    input logic smpl_clk,                //free running smpling clock (SP)
    input logic spi_clk,                 //free running SCLK which eventually goes to ADC
    input logic SPI_EN,                  //When 1, in SPI Read/write mode, when 0, in ADC data capture mode (1->0 toggle required to begin)
    input logic tx_trn,                  //SPI write signal, 0->1 toggle required to begin SP write
    input logic rx_trn,                  //SPI read signal, 0->1 toggle required to begin SP read
    input logic [7:0] addr,              //Reg address to write data to (SPI)
    input logic [15:0] wr_data,          //Data to be written to Reg address (SPI)
    input logic rst,                     //FPGA reset
    input logic [2:0] Dec_mode,          //Decimation mode to divide the free running sampling clock
    output logic CS_Z,                   //CSZ
    output logic SPI_MOSI,               //SDI
    output logic SCLK,                   //SCLK
    output logic [23:0] read_data,       //SPI readout data
    input logic DIN_A,                   //ADC D3
    input logic DIN_B,                   //ADC D2
    input logic DIN_C,                   //ADC D1
    input logic DIN_D,                   //ADC D0
    input logic sel_tx_clk,              //Selects the edge of SCLK on which the FPGA transmits data to the ADC
    input logic prog_en,                 //For customization of latching clk, reset clock etc
    input logic sel_cs_z,                //Programmability to invert CSz
    input logic [6:0] csz_cnt_low,       //Counter Limit to decide CSZ low time
    input logic [6:0] csz_cnt_high,      //Counter Limit to decide CSZ low time
    input logic [6:0] nsclk_cnt,         //Used as initial value of deser_count
    input logic [6:0] sclk_low_cnt,      //Counter Limit to decide the number of SCLKs going to DUT
    input logic [6:0] sclk_high_cnt,     //Counter Limit to decide the number of SCLKs going to DUT
    input logic [3:0] rst_clk_cnt,       //Decides when to reset the shift registers used for deserilization
    input logic [6:0] lat_clk_cnt,       //Decides when to latch the deserialized data
    input logic [6:0] drdy_cnt,          //Decides when DRDY (Fifo write clock) should go high
    output logic [47:0] DOUT,            //Deserialized output
    output logic DRDY,                   //DRDY generated once data capture is completed
    input logic sclk_del_clk,            //Delayed/Advanced version of spi_clk, used for delay calibration b/w SCLK and SDOs
    input logic sel_lat_sclk_sdo3,       //Select pos/neg edge latched D3 data
    input logic sel_lat_sclk_sdo2,       //Select pos/neg edge latched D2 data
    input logic sel_lat_sclk_sdo1,       //Select pos/neg edge latched D1 data
    input logic sel_lat_sclk_sdo0,       //Select pos/neg edge latched D0 data
    input logic [1:0] sel_sdout3_shift,  //Selects the appropriate phase of D3 latched data
    input logic [1:0] sel_sdout2_shift,  //Selects the appropriate phase of D2 latched data
    input logic [1:0] sel_sdout1_shift,  //Selects the appropriate phase of D1 latched data
    input logic [1:0] sel_sdout0_shift   //Selects the appropriate phase of D0 latched data
); 
    
    logic sclk_out;
    logic [23:0] tx_buff;
    logic tx_out;
    logic trn;
    logic [23:0] temp_tx;
    logic busy;
    logic tx_trn_pulse;
    logic rx_trn_pulse;
    logic [6:0] tx_trn_delay;
    logic [6:0] rx_trn_delay;
    logic spi_counter_trig;
    logic [4:0] SPI_EN_DLY;
    
    //DSER logic initialization
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_shift_reg_a;
    logic [26:0] pos_shift_reg_spi;
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_shift_reg_b;
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_shift_reg_c;
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_shift_reg_d;
    
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_shift_reg_a;
    logic [26:0] neg_shift_reg_spi;
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_shift_reg_b;
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_shift_reg_c;
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_shift_reg_d;
    
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_sdout3_lat;
    logic [26:0] pos_spi_lat;
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_sdout2_lat;
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_sdout1_lat;
    logic [PACKET_SIZE_PER_LANE + 2:0] pos_sdout0_lat;
    
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_sdout3_lat;
    logic [26:0] neg_spi_lat;
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_sdout2_lat;
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_sdout1_lat;
    logic [PACKET_SIZE_PER_LANE + 2:0] neg_sdout0_lat;
    
    logic [PACKET_SIZE_PER_LANE + 2:0] sdout3_edge;
    logic [26:0] spi_edge;
    logic [PACKET_SIZE_PER_LANE + 2:0] sdout2_edge;
    logic [PACKET_SIZE_PER_LANE + 2:0] sdout1_edge;
    logic [PACKET_SIZE_PER_LANE + 2:0] sdout0_edge;
    
    logic [PACKET_SIZE_PER_LANE - 1:0] sdout3_shift;
    logic [23:0] spi_shift;
    logic [PACKET_SIZE_PER_LANE - 1:0] sdout2_shift;
    logic [PACKET_SIZE_PER_LANE - 1:0] sdout1_shift;
    logic [PACKET_SIZE_PER_LANE - 1:0] sdout0_shift;
    
    logic [47:0] dout;
    logic drdy_out;
    logic [6:0] nsclk;
    logic [6:0] deser_count;
    logic st_dc;
    logic st_data_cap_pulse;    //Goes 1 with SAMP_CLK fall and SCLK rise if st_data_cap is high
    logic st_data_cap_pulse_lat;
    logic en_out;               //SCLK enable signal, selected by SPI_EN
    logic csz_detect;           //Ensures deser SCLK is not high when CSZ is high
    
    logic [6:0] csz_cnt_low_mx;
    logic [6:0] csz_cnt_high_mx;
    logic [6:0] sclk_cnt_low_mx;
    logic [6:0] sclk_cnt_high_mx;
    
    logic [6:0] rst_clk_cnt_mx;
    logic [6:0] lat_clk_cnt_mx;
    logic [6:0] drdy_cnt_mx;
    logic [6:0] load_deser_cnt;
    
    logic tx_clk;
    logic sclk_out_z;
    
    logic smpl_clkf;
    
    logic d128;
    logic d64;
    logic d32;
    logic d16;
    logic d8;
    logic d4;
    logic d2;
    
    logic dn128;
    logic dn64;
    logic dn32;
    logic dn6;
    logic dn8;
    logic dn4;
    logic dn2;
    
    logic [6:0] st_dc_dly;
    logic [1:0] smpl_clkf_dly;
    logic cs_z_cntr;
    logic cs_z_cntr_gen;
    logic cs_z_f;
    logic cs_z_cntr_gen_z;
    
    logic lat_clk_en;
    logic lat_clk;
    logic rst_clk_en;
    logic rst_clk;
    
    //Latch SPI_EN signal;
    //if SPI_EN = 1->0 (toggle) we stay in data capture mode as long as SPI_EN = 0
    //if SPI_EN = 1 and tx_trn = (0->1) we perform SPI write
    //if SPI_EN = 1 and rx_trn = (0->1) we perform SPI read
    always @(posedge sclk_del_clk) begin
        if (rst) begin
            SPI_EN_DLY <= 4'b0;
        end
        else begin
            SPI_EN_DLY[0] <= SPI_EN;
            SPI_EN_DLY[1] <= SPI_EN_DLY[0];
            SPI_EN_DLY[2] <= SPI_EN_DLY[1];
            SPI_EN_DLY[3] <= SPI_EN_DLY[2];
            SPI_EN_DLY[4] <= SPI_EN_DLY[3];
        end
    end
    
    //use double floped SPI_EN signal as st_dc
    assign st_dc = ~SPI_EN_DLY[1];
    
    //data_capture pulse generator; changed edge from pos to neg
    always @(posedge smpl_clk) begin
        if (rst) begin
            st_dc_dly <= 7'b0;
        end
        else begin
            st_dc_dly[0] <= st_dc;
            st_dc_dly[1] <= st_dc_dly[0];
            st_dc_dly[2] <= st_dc_dly[1];
            st_dc_dly[3] <= st_dc_dly[2];
            st_dc_dly[4] <= st_dc_dly[3];
            st_dc_dly[5] <= st_dc_dly[4];
            st_dc_dly[6] <= st_dc_dly[5];
        end
    end
    
    assign st_data_cap_pulse = st_dc_dly[1] & (~st_dc_dly[2]);      //Using SPI_EN to generate a pulse that indicates the start of ADC data capture
    
    always @(posedge sclk_del_clk) begin
        st_data_cap_pulse_lat <= st_data_cap_pulse;
    end
    
    //tx_trn 0->1 (toggle) calls for SPI write
    always @(posedge sclk_del_clk or posedge rst) begin
        if (rst) begin
            tx_trn_delay <= 7'b0;
        end
        else begin
            tx_trn_delay[0] <= tx_trn;
            tx_trn_delay[1] <= tx_trn_delay[0];
            tx_trn_delay[2] <= tx_trn_delay[1];
            tx_trn_delay[3] <= tx_trn_delay[2];
            tx_trn_delay[4] <= tx_trn_delay[3];
            tx_trn_delay[5] <= tx_trn_delay[4];
            tx_trn_delay[6] <= tx_trn_delay[5];
        end
    end
    
    //rx_trn 0->1 (toggle) calls for SPI read
    always @(posedge sclk_del_clk or posedge rst) begin
        if (rst) begin
            rx_trn_delay <= 7'b0;
        end
        else begin
            rx_trn_delay[0] <= rx_trn;
            rx_trn_delay[1] <= rx_trn_delay[0];
            rx_trn_delay[2] <= rx_trn_delay[1];
            rx_trn_delay[3] <= rx_trn_delay[2];
            rx_trn_delay[4] <= rx_trn_delay[3];
            rx_trn_delay[5] <= rx_trn_delay[4];
            rx_trn_delay[6] <= rx_trn_delay[5];
        end
    end
    
    assign tx_trn_pulse     = tx_trn_delay[1] & (~tx_trn_delay[6]);    //Pulse to begin SPI write
    assign rx_trn_pulse     = rx_trn_delay[1] & (~rx_trn_delay[6]);    //Pulse to begin SPI read
    assign spi_counter_trig = (tx_trn_pulse || rx_trn_pulse);          //Counter triggers for either read/write transaction
    assign trn              = (tx_trn_pulse | rx_trn_pulse) & (!busy);
    assign tx_buff          = {addr,wr_data};
    
    
    //Clock divider, packaged as an IP, divided clocks used for decimation
    ads9327_clock_divider sample_clk_divider (
        .CLK(~smpl_clk),                            //input wire CLK
        //.CE(En_dec_mode),                         //input wire CE
        .Q({dn128,dn64,dn32,dn16,dn8,dn4,dn2})      //Sampling clock divided by 2x to 128x
    );
    
    assign d128 = ~dn128;
    assign d64  = ~dn64;
    assign d32  = ~dn32;
    assign d16  = ~dn16;
    assign d8   = ~dn8;
    assign d4   = ~dn4;
    assign d2   = ~dn2;
    
    //if SPI_EN == 0, smpl_clkf = free running sampling clock; else spi_counter_trig
    assign smpl_clkf = ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd7)) ? (d128) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd6)) ? (d64) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd5)) ? (d32) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd4)) ? (d16) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd3)) ? (d8) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd2)) ? (d4) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd1)) ? (d2) :
        ((SPI_EN_DLY[1] == 1'b0) && (Dec_mode == 3'd0)) ? smpl_clk :
        ((SPI_EN_DLY[1] == 1'b1))                       ? spi_counter_trig :
        smpl_clk;
    
    always @ (posedge sclk_del_clk or posedge rst) begin
        if (rst) begin
            smpl_clkf_dly <= 2'd0;
        end
        else begin
            smpl_clkf_dly[0] <= smpl_clkf;
            smpl_clkf_dly[1] <= smpl_clkf_dly[0];
        end
    end
    
    assign nsclk          = (prog_en) ? nsclk_cnt : (PACKET_SIZE_PER_LANE+4);
    assign load_deser_cnt = nsclk + 1'd1;
    
    //Deserializer counter
    always @(posedge sclk_del_clk or posedge rst) begin
        if (rst) begin
            deser_count <= 7'd0;
        end
        //Loading deser_count when we see Data-capture or Read/Write pulse
        else if (st_data_cap_pulse_lat || (trn)) begin
            deser_count <= load_deser_cnt;
        end
        //For continuous data capture operation replenishing deser_count
        else if ((smpl_clkf_dly[0] == 1'b0) && (smpl_clkf_dly[1] == 1'b1) && (deser_count == 1'd0)) begin
            deser_count <= nsclk;
        end
        else if (deser_count>7'd0) begin
            deser_count <= deser_count - 1'd1;
        end
        else begin
            deser_count <= deser_count ;
        end
    end
    
    //Logic for making CSZ low as long as deser_count is between low_count and high_count
    assign csz_cnt_low_mx  = (prog_en) ? csz_cnt_low : (7'd2);
    assign csz_cnt_high_mx = (prog_en) ? csz_cnt_high : (PACKET_SIZE_PER_LANE+4);
    assign cs_z_cntr_gen   = ((deser_count <= csz_cnt_high_mx) && (deser_count >= csz_cnt_low_mx)) ? 1'b0 : 1'b1;   //CSZ low for a total of 11 SCLKs if PACKET_SIZE_PER_LANE = 8 (12+2-1)
    assign cs_z_cntr_gen_z = ((deser_count <= csz_cnt_high_mx) && (deser_count >= csz_cnt_low_mx)) ? 1'b1 : 1'b0;
    assign cs_z_f          = (sel_cs_z) ? cs_z_cntr_gen_z : cs_z_cntr_gen;   //Programmability to invert CSz
    
    always @(posedge sclk_del_clk) begin
        cs_z_cntr <= cs_z_f;
    end
    assign CS_Z = cs_z_cntr;
    
    //SCLK (going to DUT) generation logic
    always @(negedge sclk_del_clk) begin
        if (cs_z_cntr == 1) begin
            csz_detect <= 0;
        end
        else begin
            csz_detect <= 1;
        end
    end
    
    assign sclk_cnt_low_mx  = (prog_en) ? sclk_low_cnt : (7'd3);
    assign sclk_cnt_high_mx = (prog_en) ? sclk_high_cnt : (PACKET_SIZE_PER_LANE+2);
    assign en_out           = (csz_detect && ((deser_count <= sclk_cnt_high_mx) && (deser_count >= sclk_cnt_low_mx))); //csz_detect width = 11 sclks; en_out width = 8 sclks if PACKET_SIZE_PER_LANE = 8
    assign busy             = en_out;
    
    //Using en_out to gate free running SCLK using BUFGCE IP
    BUFGCE BUFGCE_inst (
        .O(sclk_out),     //Clock output
        .CE(en_out),      //Clock enable signal
        .I(spi_clk)       //CLock input
    );
    
    assign sclk_out_z = ~sclk_out;
    assign tx_clk     = (sel_tx_clk) ? (sclk_out) : sclk_out_z;     //Programmability to invert tx_clk
    
    //SPI write shift register
    always @(posedge tx_clk or posedge tx_trn_pulse) begin
        if (tx_trn_pulse) begin
            temp_tx <= tx_buff;
        end
        else if (SPI_EN_DLY[0]) begin
            temp_tx <= {temp_tx[22:0],1'b0};
        end
        else begin
            temp_tx <= 24'h000000;
        end
    end
    
    assign tx_out   = temp_tx[23];
    assign SPI_MOSI = ((!cs_z_cntr) & SPI_EN_DLY[1] == 1) ? (tx_out) : 1'b0; //Sending out SPI_MOSI (SDI) only if CSZ is low and SPI_EN = 1
    assign SCLK     = sclk_out;
    
    //Latching data on the pos and neg edge of sclk_del_clk through IP iDDR available in Artix 7 set of devices
    iDDR iDDR_DIN_A (
        .DIN (DIN_A),
        .CK (sclk_del_clk),
        .Q1 (pos_DIN_A),
        .Q2 (neg_DIN_A),
        .CE (1)
    );
    
    iDDR iDDR_DIN_B (
        .DIN (DIN_B),
        .CK (sclk_del_clk),
        .Q1 (pos_DIN_B),
        .Q2 (neg_DIN_B),
        .CE (1)
    );
    
    iDDR iDDR_DIN_C (
        .DIN (DIN_C),
        .CK (sclk_del_clk),
        .Q1 (pos_DIN_C),
        .Q2 (neg_DIN_C),
        .CE (1)
    );
    
    iDDR iDDR_DIN_D (
        .DIN (DIN_D),
        .CK (sclk_del_clk),
        .Q1 (pos_DIN_D),
        .Q2 (neg_DIN_D),
        .CE (1)
    );
    
    //Creating a reset clock/pulse to clear the contents of the shift register in one sampling cycle
    assign rst_clk_cnt_mx = (prog_en) ? rst_clk_cnt : (PACKET_SIZE_PER_LANE+4);
    assign rst_clk_en     = (deser_count == rst_clk_cnt_mx) ? 1'b1 : 1'b0;
    always @(negedge sclk_del_clk) begin
        rst_clk <= rst_clk_en;
    end
    
    //Shift register operation
    always @(posedge sclk_del_clk) begin
        if (rst_clk) begin
            pos_shift_reg_a   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            pos_shift_reg_spi = 27'b0;
            pos_shift_reg_b   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            pos_shift_reg_c   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            pos_shift_reg_d   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            
            neg_shift_reg_a   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            neg_shift_reg_spi = 27'b0;
            neg_shift_reg_b   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            neg_shift_reg_c   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
            neg_shift_reg_d   = {(PACKET_SIZE_PER_LANE+3){1'b0}};
        end
        else begin
            pos_shift_reg_a[PACKET_SIZE_PER_LANE+2:1] <= pos_shift_reg_a[PACKET_SIZE_PER_LANE+1:0];
            pos_shift_reg_a[0]                        <= pos_DIN_A;
            neg_shift_reg_a[PACKET_SIZE_PER_LANE+2:1] <= neg_shift_reg_a[PACKET_SIZE_PER_LANE+1:0];
            neg_shift_reg_a[0]                        <= neg_DIN_A;
            pos_shift_reg_spi[26:1]                   <= pos_shift_reg_spi[25:0];
            pos_shift_reg_spi[0]                      <= pos_DIN_A;
            neg_shift_reg_spi[26:1]                   <= neg_shift_reg_spi[25:0];
            neg_shift_reg_spi[0]                      <= neg_DIN_A;
            
            pos_shift_reg_b[PACKET_SIZE_PER_LANE+2:1] <= pos_shift_reg_b[PACKET_SIZE_PER_LANE+1:0];
            pos_shift_reg_b[0]                        <= pos_DIN_B;
            neg_shift_reg_b[PACKET_SIZE_PER_LANE+2:1] <= neg_shift_reg_b[PACKET_SIZE_PER_LANE+1:0];
            neg_shift_reg_b[0]                        <= neg_DIN_B;
            
            pos_shift_reg_c[PACKET_SIZE_PER_LANE+2:1] <= pos_shift_reg_c[PACKET_SIZE_PER_LANE+1:0];
            pos_shift_reg_c[0]                        <= pos_DIN_C;
            neg_shift_reg_c[PACKET_SIZE_PER_LANE+2:1] <= neg_shift_reg_c[PACKET_SIZE_PER_LANE+1:0];
            neg_shift_reg_c[0]                        <= neg_DIN_C;
            
            pos_shift_reg_d[PACKET_SIZE_PER_LANE+2:1] <= pos_shift_reg_d[PACKET_SIZE_PER_LANE+1:0];
            pos_shift_reg_d[0]                        <= pos_DIN_D;
            neg_shift_reg_d[PACKET_SIZE_PER_LANE+2:1] <= neg_shift_reg_d[PACKET_SIZE_PER_LANE+1:0];
            neg_shift_reg_d[0]                        <= neg_DIN_D;
        end
    end
    
    //Creating a latching clock/pulse to latch the deserialized content
    assign lat_clk_cnt_mx = (prog_en) ? lat_clk_cnt : 7'd0;
    assign lat_clk_en     = (deser_count == lat_clk_cnt_mx) ? 1'b1 : 1'b0;
    always @(negedge sclk_del_clk) begin
        lat_clk <= lat_clk_en;
    end
    
    always @(posedge lat_clk) begin
        pos_sdout3_lat <= pos_shift_reg_a;
        pos_spi_lat    <= pos_shift_reg_spi;
        pos_sdout2_lat <= pos_shift_reg_b;
        pos_sdout1_lat <= pos_shift_reg_c;
        pos_sdout0_lat <= pos_shift_reg_d;
        
        neg_sdout3_lat <= neg_shift_reg_a;
        neg_spi_lat    <= neg_shift_reg_spi;
        neg_sdout2_lat <= neg_shift_reg_b;
        neg_sdout1_lat <= neg_shift_reg_c;
        neg_sdout0_lat <= neg_shift_reg_d;
    end
    
    //Programmability: Select pos/neg edge latched data
    assign sdout3_edge = sel_lat_sclk_sdo3 ? pos_sdout3_lat : neg_sdout3_lat;
    assign spi_edge    = sel_lat_sclk_sdo3 ? pos_spi_lat : neg_spi_lat;
    assign sdout2_edge = sel_lat_sclk_sdo2 ? pos_sdout2_lat : neg_sdout2_lat;
    assign sdout1_edge = sel_lat_sclk_sdo1 ? pos_sdout1_lat : neg_sdout1_lat;
    assign sdout0_edge = sel_lat_sclk_sdo0 ? pos_sdout0_lat : neg_sdout0_lat;
    
    //Programmability: Select the relevant bits of the latched data (SCLK is free runnig need to find the correct phase)
    assign sdout3_shift = (sel_sdout3_shift == 2'b00) ? {sdout3_edge[PACKET_SIZE_PER_LANE+1:2]} :
        (sel_sdout3_shift == 2'b01) ? {sdout3_edge[PACKET_SIZE_PER_LANE:1]} :
        (sel_sdout3_shift == 2'b10) ? {sdout3_edge[PACKET_SIZE_PER_LANE-1:0]} :
        (sel_sdout3_shift == 2'b11) ? {sdout3_edge[PACKET_SIZE_PER_LANE+2:3]} :
        {sdout3_edge[PACKET_SIZE_PER_LANE+1:2]};
    
    assign spi_shift = (sel_sdout3_shift == 2'b00) ? {spi_edge[25:2]} :
        (sel_sdout3_shift == 2'b01) ? {spi_edge[24:1]} :
        (sel_sdout3_shift == 2'b10) ? {spi_edge[23:0]} :
        (sel_sdout3_shift == 2'b11) ? {spi_edge[26:3]} :
        {spi_edge[25:2]};
    
    assign sdout2_shift = (sel_sdout2_shift == 2'b00) ? {sdout2_edge[PACKET_SIZE_PER_LANE+1:2]} :
        (sel_sdout2_shift == 2'b01) ? {sdout2_edge[PACKET_SIZE_PER_LANE:1]} :
        (sel_sdout2_shift == 2'b10) ? {sdout2_edge[PACKET_SIZE_PER_LANE-1:0]} :
        (sel_sdout2_shift == 2'b11) ? {sdout2_edge[PACKET_SIZE_PER_LANE+2:3]} :
        {sdout2_edge[PACKET_SIZE_PER_LANE+1:2]};
    
    assign sdout1_shift = (sel_sdout1_shift == 2'b00) ? {sdout1_edge[PACKET_SIZE_PER_LANE+1:2]} :
        (sel_sdout1_shift == 2'b01) ? {sdout1_edge[PACKET_SIZE_PER_LANE:1]} :
        (sel_sdout1_shift == 2'b10) ? {sdout1_edge[PACKET_SIZE_PER_LANE-1:0]} :
        (sel_sdout1_shift == 2'b11) ? {sdout1_edge[PACKET_SIZE_PER_LANE+2:3]} :
        {sdout1_edge[PACKET_SIZE_PER_LANE+1:2]};
    
    assign sdout0_shift = (sel_sdout0_shift == 2'b00) ? {sdout0_edge[PACKET_SIZE_PER_LANE+1:2]} :
        (sel_sdout0_shift == 2'b01) ? {sdout0_edge[PACKET_SIZE_PER_LANE:1]} :
        (sel_sdout0_shift == 2'b10) ? {sdout0_edge[PACKET_SIZE_PER_LANE-1:0]} :
        (sel_sdout0_shift == 2'b11) ? {sdout0_edge[PACKET_SIZE_PER_LANE+2:3]} :
        {sdout0_edge[PACKET_SIZE_PER_LANE+1:2]};
    
    //Concatenating the data depending on the lane and packet size
    assign dout = (LANES == 0 && PACKET_SIZE_PER_LANE == 8) ? ({8'd0, sdout3_shift[PACKET_SIZE_PER_LANE-1:0], sdout2_shift[PACKET_SIZE_PER_LANE-1:0], 8'd0, sdout1_shift[PACKET_SIZE_PER_LANE-1:0], sdout0_shift[PACKET_SIZE_PER_LANE-1:0]}) :  //16 Bit ADC A, 16 Bit ADC B
        (LANES == 1 && PACKET_SIZE_PER_LANE == 16) ? ({8'd0, sdout3_shift[PACKET_SIZE_PER_LANE-1:0], 8'd0, sdout1_shift[PACKET_SIZE_PER_LANE-1:0]}) :
        (LANES == 0 && PACKET_SIZE_PER_LANE == 10) ? ({4'd0, sdout3_shift[PACKET_SIZE_PER_LANE-1:0], sdout2_shift[PACKET_SIZE_PER_LANE-1:0], 4'd0, sdout1_shift[PACKET_SIZE_PER_LANE-1:0], sdout0_shift[PACKET_SIZE_PER_LANE-1:0]}) :
        (LANES == 1 && PACKET_SIZE_PER_LANE == 20) ? ({4'd0, sdout3_shift[PACKET_SIZE_PER_LANE-1:0], 4'd0, sdout1_shift[PACKET_SIZE_PER_LANE-1:0]}) :
        (LANES == 0 && PACKET_SIZE_PER_LANE == 12) ? ({sdout3_shift[PACKET_SIZE_PER_LANE-1:0], sdout2_shift[PACKET_SIZE_PER_LANE-1:0], sdout1_shift[PACKET_SIZE_PER_LANE-1:0], sdout0_shift[PACKET_SIZE_PER_LANE-1:0]}) :
        (LANES == 1 && PACKET_SIZE_PER_LANE == 24) ? ({sdout3_shift[PACKET_SIZE_PER_LANE-1:0], sdout1_shift[PACKET_SIZE_PER_LANE-1:0]}) :
        (48'h000000000000);
    
    assign read_data = spi_shift;  //SPI readout data is always 24 bits
    
    //Creating DRDY signal to latch dout
    assign drdy_cnt_mx = (prog_en) ? drdy_cnt : 7'd9;
    always @(negedge sclk_del_clk) begin
        if (deser_count == drdy_cnt_mx) begin
            drdy_out <= 1;
        end
        else begin
            drdy_out <= 0;
        end
    end
    
    assign DRDY = drdy_out;
    
    always @(negedge DRDY) begin
        DOUT <= dout;
    end
    
endmodule
