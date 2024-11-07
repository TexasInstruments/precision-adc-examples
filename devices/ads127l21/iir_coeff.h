/**
 * \copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
 *
 */

#ifndef IIR_COEFF_H_
#define IIR_COEFF_H_

#define IIR_COEFF_LENGTH    (25)
#define IIR_COEFF_CRC       (0xA4)

int32_t default_IIR[IIR_COEFF_LENGTH] = {
    0x40000000, // g5
    0x00000000, // a42
    0x00000000, // a41
    0x00000000, // b42
    0x00000000, // b41
    0x40000000, // b40
    0x40000000, // g4
    0x00000000, // a32
    0x00000000, // a31
    0x00000000, // b32
    0x00000000, // b31
    0x40000000, // b30
    0x40000000, // g3
    0x00000000, // a22
    0x00000000, // a21
    0x00000000, // b22
    0x00000000, // b21
    0x40000000, // b20
    0x40000000, // g2
    0x00000000, // a12
    0x00000000, // a11
    0x00000000, // b12
    0x00000000, // b11
    0x40000000, // b10
    0x40000000  // g1
};

#endif /* IIR_COEFF_H_ */
