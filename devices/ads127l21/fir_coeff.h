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

#ifndef FIR_COEFF_H_
#define FIR_COEFF_H_

#define FIR_COEFF_LENGTH    (128)
#define FIR_COEFF_CRC       (0x8A51)        // TODO: Verify that this doesn't raise a STATUS error

int32_t default_FIR[FIR_COEFF_LENGTH] = {
      0x00000000,   // h127 (MSB, MSB-1, MSB-2, LSB)
      0x00000000,   // h126
      0x00000000,   // ...
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0xffffab25,
      0xffffb68d,
      0x0000a9b4,
      0x0001654e,
      0x00000800,
      0xfffddde9,
      0xffff039e,
      0x000326b9,
      0x00033a9a,
      0xfffca90a,
      0xfff98363,
      0x00021533,
      0x000a8c85,
      0x0001a121,
      0xfff192f1,
      0xfff7850a,
      0x00109840,
      0x0012a9fe,
      0xfff10485,
      0xffe094cf,
      0x00075968,
      0x002cc2bf,
      0x0008320d,
      0xffc8a2ef,
      0xffdf8205,
      0x003ac2dc,
      0x00409b6a,
      0xffce1838,
      0xff9ac5a2,
      0x0018170a,
      0x00885e98,
      0x0015dbb8,
      0xff5e7615,
      0xffa7b6ae,
      0x00a674a3,
      0x00ab8c92,
      0xff73ae9f,
      0xfef8caa9,
      0x00497b4c,
      0x015dc82b,
      0x0028ae22,
      0xfe62e52f,
      0xff33f038,
      0x01af279e,
      0x019c967f,
      0xfe8521f5,
      0xfd703d5b,
      0x00e3d743,
      0x03952ab5,
      0x0039b0ec,
      0xfb68050f,
      0xfde79fae,
      0x05814b07,
      0x0543f39b,
      0xf9c508ae,
      0xf43e587b,
      0x06b28e26,
      0x281e7711,
      0x3924281e,
      0x281e7711,
      0x06b28e26,
      0xf43e587b,
      0xf9c508ae,
      0x0543f39b,
      0x05814b07,
      0xfde79fae,
      0xfb68050f,
      0x0039b0ec,
      0x03952ab5,
      0x00e3d743,
      0xfd703d5b,
      0xfe8521f5,
      0x019c967f,
      0x01af279e,
      0xff33f038,
      0xfe62e52f,
      0x0028ae22,
      0x015dc82b,
      0x00497b4c,
      0xfef8caa9,
      0xff73ae9f,
      0x00ab8c92,
      0x00a674a3,
      0xffa7b6ae,
      0xff5e7615,
      0x0015dbb8,
      0x00885e98,
      0x0018170a,
      0xff9ac5a2,
      0xffce1838,
      0x00409b6a,
      0x003ac2dc,
      0xffdf8205,
      0xffc8a2ef,
      0x0008320d,
      0x002cc2bf,
      0x00075968,
      0xffe094cf,
      0xfff10485,
      0x0012a9fe,
      0x00109840,
      0xfff7850a,
      0xfff192f1,
      0x0001a121,
      0x000a8c85,
      0x00021533,
      0xfff98363,
      0xfffca90a,
      0x00033a9a,
      0x000326b9,
      0xffff039e,
      0xfffddde9,
      0x00000800,
      0x0001654e,
      0x0000a9b4,
      0xffffb68d,
      0xffffab25    // h0
};

#endif /* FIR_COEFF_H_ */
