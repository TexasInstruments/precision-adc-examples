// Driver/Inc/ads_device.h
#ifndef ADS_DEVICE_H_
#define ADS_DEVICE_H_

#define ADS_DEVICE_C02          // choose just one of these 4 statements. 
//#define ADS_DEVICE_C04        //
//#define ADS_DEVICE_S02        // 
//#define ADS_DEVICE_S04        // 
//////////////////////////////////////////////////////////////////////



#if   defined(ADS_DEVICE_C02)
    #include "Driver/Inc/ads1x1c02.h"
#elif defined(ADS_DEVICE_C04)
    #include "Driver/Inc/ads1x1c04.h"
#elif defined(ADS_DEVICE_S02)
    #include "Driver/Inc/ads1x1s02.h"
#elif defined(ADS_DEVICE_S04)
    #include "Driver/Inc/ads1x1s04.h"
#endif

#endif