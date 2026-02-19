# CRC module Documentation
## What is this? 

This module facilitates the calculation of solutions for CRC8 and CRC16 bit error corrections.  The computaion polynomial and initial seed value are configurable using ``` #define ``` statements in the ``` crc.h ``` file. 
CRC computation can be computed for a particular value, or series of values.  The solutions can be computed on during runtime or generated as a lookup table. 

## How to use

1. In the ``` crc.h ``` file, look for the **Constants** section.
   
2. Enable or disable the relveant ``` #define ``` statements.  Be sure to define:
- CRC8 or CRC16
- CRCWORD variable type
- The initial seed value
- The CRC polynomial

3. Initalize the CRC functions using ``` void initCRC(void); ```  in your application.

4. Call ``` getCRC(dataBytes, numBytes, initialValue); ``` to compute the CRC value, where **dataBytes** is an array of values to compute, and **initialValue** is either the seed value, or the result of a previous computation.
 
