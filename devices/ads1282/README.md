ADS1282 Example C Code
======================

Table of Contents
-----------------
This file contains the following sections:
* Hardware used
* Software used
* Links to Additional Information

--------------------------------------------
**Hardware used:**     
--------------------------------------------

1. [PAMB Controller](https://confluence.itg.ti.com/display/ASCSYSAPP/PAMB+Controller)

2. [EVM2LP](https://confluence.itg.ti.com/pages/viewpage.action?pageId=271629380)
    - JP1/JP2: "External"
    - JP3: "Internal"
    - JP4: If "External", then connect power supply to J6
    - JP5: Don't care<br><br>

3. [ADS1282EVM](https://www.ti.com/tool/ADS1282EVM-PDK)
    - J1: (1-2, default) "REF" - REF5050 connected to U3 buffer
    - S1: "ON BRD" - U3/AVSS selected as referece source 
    - J2: (1-2, default) "OBCLK" - Selects on board clock source
    - J3: (1-2) "DVDD" - Enables on board oscillator
    - J4/J9: (default) Not populated
    - J10: (1-2, default) "DIN" - SPI mode<br><br>

4. XDS110 Debugger ([TMDSEMU110-U](https://www.ti.com/tool/TMDSEMU110-U) or [MSP432E LaunchPad](https://www.ti.com/tool/MSP-EXP432E401Y))

5. JTAG Cable (See "[Mini JTAG Connector Info...](https://confluence.itg.ti.com/display/ASCSYSAPP/PAMB+Controller)")

6. External +/-10V power supply

--------------------------------------------
**Software used:**     
--------------------------------------------

1. [CCSTUDIO](https://www.ti.com/tool/CCSTUDIO) - Code Composer Studio™ integrated development environment (IDE)
    - CCS v11.0.0.00012
    - MSP432 tool-chain v8.2.4
    - Compiler version: TI v20.2.5.LTS<br><br>

2. [SIMPLELINK-MSP432-SDK](https://www.ti.com/tool/SIMPLELINK-MSP432-SDK) - SimpleLink MSP432 Software Development Kit (SDK) - v4.20.00.12

--------------------------------------------
Digital header (J5A) pin mapping:
--------------------------------------------

                /--------\
               -|PQ1  PN2|- M0
          SCLK -|PQ0  GND|- GND
               -|PM2  PM6|- M1
               -|PM0  PD4|- MCLK
         nDRDY -|PM1  GND|- GND
           DIN -|PQ2  PH3|- AVDD polarity select
          DOUT -|PQ3  PH1|- nPWDN
         nDRDY -|PN3  PN5|- SCLK
          SYNC -|PM7  GND|- GND
        CLKSEL -|PK6  PN4|- SDA
                \--------/

-------------------------------
Links to Additional Information
-------------------------------

- [ADS1282 Apps Knowledge Base](https://confluence.itg.ti.com/display/ASCSYSAPP/ADS128x+Family)
- [PAMB Pin Mapping tool](https://sps16.itg.ti.com/sites/DataConverters/PADC/SysApps/Team%20Documents/EVMs/PAMB%20Pinmap%20Tool.xlsx?web=1)
- [Other Example Code Projects](https://confluence.itg.ti.com/display/ASCSYSAPP/Device+Example+C+Code)
