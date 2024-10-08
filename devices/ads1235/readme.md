# ADS1235 Example C Code

Reference this code as an example to help to you get started writing your own ADS1235 code.

## Included modules

**`ADS1235`** - Low-level functions and register map definitions for communicating with the ADS1235 and related devices.
\
**`Hardware Abstraction Layer (HAL)`** - Processor specific functions required by the `ADS1235` module.

**How to use this code...**
\
The `ADS1235` and `HAL` modules can be copied into your project; however, you will have to edit all of the function contents inside the *hal.c* file and add all required library references to *hal.h* to ensure compatibility with your processor. Once that is complete, include *ads1235.h* in your application and you can begin calling `ADS1235` functions in your code.

---

**IMPORTANT**: This code was tested on a TM4C1294NCPDT 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 16.9.1.LTS ARM compiler. This code is provided only as example to aid in the creation of your own software implementation and should not be considered to be fully verified and ready for use in your application.

---

>**LICENSE**
\
Copyright (C) 2021 Texas Instruments Incorporated - [http://www.ti.com/](http://www.ti.com/)
\
\
Redistribution and use in source and binary forms, with or without
\
modification, are permitted provided that the following conditions
\
are met:
\
Redistributions of source code must retain the above copyright
\
notice, this list of conditions and the following disclaimer.
\
\
Redistributions in binary form must reproduce the above copyright
\
notice, this list of conditions and the following disclaimer in the
\
documentation and/or other materials provided with the
\
distribution.
\
\
Neither the name of Texas Instruments Incorporated nor the names of
\
its contributors may be used to endorse or promote products derived
\
from this software without specific prior written permission.
\
\
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
\
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
\
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
\
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
\
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
\
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
\
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
\
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
\
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
\
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
\
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.