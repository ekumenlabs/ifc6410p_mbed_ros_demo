#ifndef MPL3115A2_H
#define MPL3115A2_H

#include "mbed.h"

// Oversampling value and minimum time between sample
#define OVERSAMPLE_RATIO_1      0       // 6 ms
#define OVERSAMPLE_RATIO_2      1       // 10 ms
#define OVERSAMPLE_RATIO_4      2       // 18 ms
#define OVERSAMPLE_RATIO_8      3       // 34 ms
#define OVERSAMPLE_RATIO_16     4       // 66 ms
#define OVERSAMPLE_RATIO_32     5       // 130 ms
#define OVERSAMPLE_RATIO_64     6       // 258 ms
#define OVERSAMPLE_RATIO_128    7       // 512 ms

/* Mode */
#define ALTIMETER_MODE      1
#define BAROMETRIC_MODE     2

/**
* MPL3115A2 Altimeter example
*
* @code
* #include "mbed.h"
* #include "MPL3115A2.h"
* 
* #define MPL3115A2_I2C_ADDRESS (0x60<<1)
* MPL3115A2 wigo_sensor1(PTE0, PTE1, MPL3115A2_I2C_ADDRESS);
* Serial pc(USBTX, USBRX);
*
* // pos [0] = altitude or pressure value 
* // pos [1] = temperature value 
* float sensor_data[2];
*
* int main(void) {
*
*    pc.baud( 230400);
*    pc.printf("MPL3115A2 Altimeter mode. [%d]\r\n", wigo_sensor1.getDeviceID()); 
*
*    wigo_sensor1.Oversample_Ratio( OVERSAMPLE_RATIO_32);
*    
*    while(1) {
*        //
*        if ( wigo_sensor1.isDataAvailable()) {
*            wigo_sensor1.getAllData( &sensor_data[0]);
*            pc.printf("\tAltitude: %f\tTemperature: %f\r\n", sensor_data[0], sensor_data[1]);
*        }
*        //
*        wait( 0.001);
*    }
*
* }
* @endcode
*/
class MPL3115A2
{
public:
    /**
    * MPL3115A2 constructor
    *
    * @param sda SDA pin
    * @param sdl SCL pin
    * @param addr addr of the I2C peripheral
    */
    MPL3115A2(PinName sda, PinName scl, int addr);
    
    /**
    * Get the value of the WHO_AM_I register
    *
    * @returns DEVICE_ID value == 0xC4
    */
    uint8_t getDeviceID();
    
    /**
    * Return the STATUS register value
    *
    * @returns STATUS register value
    */
    unsigned char getStatus( void);
    
    /**
    * Get the altimeter value
    *
    * @returns altimeter value as float
    */
    float getAltimeter( void);

    /**
    * Get the altimeter value in raw mode
    *
    * @param    dt      pointer to unsigned char array
    * @returns 1 if data are available, 0 if not.
    */
    unsigned int getAltimeterRaw( unsigned char *dt);
    
    /**
    * Get the pressure value
    *
    * @returns pressure value as float
    */
    float getPressure( void);

    /**
    * Get the pressure value in raw mode
    *
    * @param    dt      pointer to unsigned char array
    * @returns 1 if data are available, 0 if not.
    */
    unsigned int  getPressureRaw( unsigned char *dt);
    
    /**
    * Get the temperature value
    *
    * @returns temperature value as float
    */
    float getTemperature( void);

    /**
    * Get the temperature value in raw mode
    *
    * @param    dt      pointer to unsigned char array
    * @returns 1 if data are available, 0 if not.
    */
    unsigned int getTemperatureRaw( unsigned char *dt);
    
    /**
    * Set the Altimeter Mode
    *
    * @returns none
    */
    void Altimeter_Mode( void);
    
    /**
    * Set the Barometric Mode
    *
    * @returns none
    */
    void Barometric_Mode( void);
    
    /**
    * Get the altimeter or pressure and temperature values
    *
    * @param array of float f[2]
    * @returns 0 no data available, 1 for data available
    */
    unsigned int  getAllData( float *f);

    /**
    * Get the altimeter or pressure and temperature values and the delta values
    *
    * @param array of float f[2], array of float d[2]
    * @returns 0 no data available, 1 for data available
    */
    unsigned int getAllData( float *f, float *d);
    
    /**
    * Get the altimeter or pressure and temperature captured maximum value
    *
    * @param array of float f[2]
    * @returns 0 no data available, 1 for data available
    */
    void getAllMaximumData( float *f);

    /**
    * Get the altimeter or pressure and temperature captured minimum value
    *
    * @param array of float f[2]
    * @returns 0 no data available, 1 for data available
    */
    void getAllMinimumData( float *f);

    /**
    * Get the altimeter or pressure, and temperature values in raw mode
    *
    * @param array of unsigned char[5]
    * @returns 1 if data are available, 0 if not.
    */    
    unsigned int getAllDataRaw( unsigned char *dt);
    
    /** 
    * Return if there are date available
    * 
    * @return 0 for no data available, bit0 set for Temp data available, bit1 set for Press/Alti data available
    *         bit2 set for both Temp and Press/Alti data available
    */
    unsigned int isDataAvailable( void);
    
    /** 
    * Set the oversampling rate value
    *
    * @param oversampling values. See MPL3115A2.h
    * @return none
    */
    void Oversample_Ratio( unsigned int ratio);

    /**
    * Configure the sensor to streaming data using Interrupt
    *
    * @param user functin callback, oversampling values. See MPL3115A2.h
    * @return none
    */
    void DataReady( void(*fptr)(void), unsigned char OS);

    /**
    * Configure the sensor to generate an Interrupt crossing the center threshold
    *
    * @param user functin callback, level in meter
    * @return none
    */
    void AltitudeTrigger( void(*fptr)(void), unsigned short level);
    
    /**
    * Soft Reset
    *
    * @param none
    * @return none
    */    
    void Reset( void);

    /**
    * Configure the Pressure offset.
    * Pressure user accessible offset trim value expressed as an 8-bit 2's complement number. 
    * The user offset registers may be adjusted to enhance accuracy and optimize the system performance. 
    * Range is from -512 to +508 Pa, 4 Pa per LSB.
    * In RAW output mode no scaling or offsets will be applied in the digital domain
    *
    * @param    offset
    * @return   none
    */    
    void SetPressureOffset( char offset);
    
    /**
    * Configure the Temperature offset.
    * Temperature user accessible offset trim value expressed as an 8-bit 2's complement number. 
    * The user offset registers may be adjusted to enhance accuracy and optimize the system performance. 
    * Range is from -8 to +7.9375°C 0.0625°C per LSB.
    * In RAW output mode no scaling or offsets will be applied in the digital domain
    *
    * @param    offset
    * @return   none
    */    
    void SetTemperatureOffset( char offset);
    
    /**
    * Configure the Altitude offset.
    * Altitude Data User Offset Register (OFF_H) is expressed as a 2’s complement number in meters. 
    * The user offset register provides user adjustment to the vertical height of the Altitude output. 
    * The range of values are from -128 to +127 meters.
    * In RAW output mode no scaling or offsets will be applied in the digital domain
    *
    * @param    offset
    * @return   none
    */    
    void SetAltitudeOffset( char offset);        

private:
    I2C m_i2c;
    int m_addr;
    unsigned char MPL3115A2_mode;
    unsigned char MPL3115A2_oversampling;
    void DataReady_IRQ( void);
    void AltitudeTrg_IRQ( void);
    
    /** Set the device in active mode
    */
    void Active( void);
    
    /** Set the device in standby mode
    */
    void Standby( void);
    
    /** Get the altimiter value from the sensor.
    *
    * @param    reg the register from which read the data. 
    *           Can be: REG_ALTIMETER_MSB for altimeter value
    *                   REG_ALTI_MIN_MSB for the minimum value captured
    *                   REG_ALTI_MAX_MSB for the maximum value captured
    */
    float getAltimeter( unsigned char reg);    

    /** Get the pressure value from the sensor.
    *
    * @param    reg the register from which read the data. 
    *           Can be: REG_PRESSURE_MSB for altimeter value
    *                   REG_PRES_MIN_MSB for the minimum value captured
    *                   REG_PRES_MAX_MSB for the maximum value captured
    */    
    float getPressure( unsigned char reg);

    /** Get the altimiter value from the sensor.
    *
    * @param    reg the register from which read the data. 
    *           Can be: REG_TEMP_MSB for altimeter value
    *                   REG_TEMP_MIN_MSB for the minimum value captured
    *                   REG_TEMP_MAX_MSB for the maximum value captured
    */
    float getTemperature( unsigned char reg);
    
    void readRegs(int addr, uint8_t * data, int len);
    void writeRegs(uint8_t * data, int len);

};

#endif
