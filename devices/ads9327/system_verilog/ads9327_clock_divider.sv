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

module ads9327_clock_divider (
    input  logic clk,           // Input clock
    output logic [6:0] Q
);

    // Counter register - 7 bits to count up to 128
    logic [6:0] counter;
    
    // Counter logic
    always_ff @(posedge clk or negedge rst_n) begin
        counter <= counter + 1'b1;
    end
    
    // Generate divided clocks using counter bits
    // Each bit of the counter represents a clock division
    assign Q[0] = counter[0];   // Toggles every 1 clock cycle (div by 2)
    assign Q[1] = counter[1];   // Toggles every 2 clock cycles (div by 4)
    assign Q[2] = counter[2];   // Toggles every 4 clock cycles (div by 8)
    assign Q[3] = counter[3];   // Toggles every 8 clock cycles (div by 16)
    assign Q[4] = counter[4];   // Toggles every 16 clock cycles (div by 32)
    assign Q[5] = counter[5];   // Toggles every 32 clock cycles (div by 64)
    assign Q[6] = counter[6];   // Toggles every 64 clock cycles (div by 128)

endmodule