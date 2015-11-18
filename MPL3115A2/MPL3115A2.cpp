#include "MPL3115A2.h"

#define REG_WHO_AM_I        0x0C        // return 0xC4 by default
#define REG_STATUS          0x00
#define REG_CTRL_REG_1      0x26
#define REG_CTRL_REG_3      0x28
#define REG_CTRL_REG_4      0x29
#define REG_CTRL_REG_5      0x2A
#define REG_PRESSURE_MSB    0x01        // 3 byte pressure data
#define REG_ALTIMETER_MSB   0x01        // 3 byte altimeter data
#define REG_TEMP_MSB        0x04        // 2 byte temperature data
#define REG_PT_DATA_CFG     0x13
#define REG_P_TGT_MSB       0x16
#define REG_P_WND_MSB       0x19
#define REG_OFF_P           0x2b
#define REG_OFF_T           0x2c
#define REG_OFF_H           0x2d
#define REG_PRES_MIN_MSB    0x1c
#define REG_ALTI_MIN_MSB    0x1c
#define REG_TEMP_MIN_MSB    0x1f
#define REG_PRES_MAX_MSB    0x21
#define REG_ALTI_MAX_MSB    0x21
#define REG_TEMP_MAX_MSB    0x24
#define REG_PRES_DELTA_MSB  0x07
#define REG_ALTI_DELTA_MSB  0x07
#define REG_TEMP_DELTA_MSB  0x0a


#define UINT14_MAX        16383

// Status flag for data ready.
#define PTDR_STATUS       0x03        // Pressure Altitude and Temperature ready
#define PDR_STATUS        0x02        // Pressure and Altitude data ready
#define TDR_STATUS        0x01        // Temperature data ready

/** Interrupt schema
*
* :: The Altitude Trigger use the IRQ1.
* 
*   Altitude Trigger -- MPL3115A2_Int1.fall --- AltitudeTrg_IRQ --- MPL3115A2_usr1_fptr
*
*
* :: The Data ready use the IRQ2. 
*
*   Data Ready -- MPL3115A2_Int2.fall --- DataReady_IRQ --- MPL3115A2_usr2_fptr
*
*/
void (*MPL3115A2_usr2_fptr)(void);               // Pointers to user function called after
void (*MPL3115A2_usr1_fptr)(void);               // IRQ assertion.

//
InterruptIn MPL3115A2_Int1( D17);       // INT1
InterruptIn MPL3115A2_Int2( D2);      // INT2

MPL3115A2::MPL3115A2(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    unsigned char data[6];
    
    MPL3115A2_mode = BAROMETRIC_MODE;
    MPL3115A2_oversampling = OVERSAMPLE_RATIO_1;
    //
    MPL3115A2_usr1_fptr = NULL;
    MPL3115A2_usr2_fptr = NULL;
    MPL3115A2_Int1.fall( NULL);
    MPL3115A2_Int2.fall( NULL);
    
    Reset();
    
    data[0]=REG_PRES_MIN_MSB;
    data[1]=0;data[2]=0;data[3]=0;data[4]=0;data[5]=0;
    writeRegs( &data[0], 6);
}

void MPL3115A2::Reset( void)
{
    unsigned char t;
    
    // soft reset...
    readRegs( REG_CTRL_REG_1, &t, 1);
    unsigned char data[2] = { REG_CTRL_REG_1, t|0x04};
    writeRegs(data, 2);    
    wait( 0.1);

}

void MPL3115A2::DataReady( void(*fptr)(void), unsigned char OS)
{
    unsigned char dt[5];
    unsigned char data[2];
    
    // Soft Reset
    Reset();
    
    Standby();
    
    // Clear all interrupts by reading the output registers.
    readRegs( REG_ALTIMETER_MSB, &dt[0], 5);
    getStatus();
    // Configure INT active low and pullup
    data[0] = REG_CTRL_REG_3;
    data[1] = 0x00;
    writeRegs(data, 2);    
    // Enable Interrupt fot data ready
    data[0] = REG_CTRL_REG_4;
    data[1] = 0x80;
    writeRegs(data, 2);    
    // Configure Interrupt to route to INT2
    data[0] = REG_CTRL_REG_5;
    data[1] = 0x00;
    writeRegs(data, 2);    
    data[0] = REG_PT_DATA_CFG;
    data[1] = 0x07;
    writeRegs(data, 2);    

    // Configure the OverSampling rate, Altimeter/Barometer mode and set the sensor Active
    data[0] = REG_CTRL_REG_1;
    data[1] = (OS<<3);
    //
    if (MPL3115A2_mode == BAROMETRIC_MODE)
        data[1] &= 0x7F;
    else
        data[1] |= 0x80;
    //
    data[1] |= 0x01; 
    writeRegs(data, 2);    

    MPL3115A2_usr2_fptr = fptr;
    MPL3115A2_Int2.fall( this, &MPL3115A2::DataReady_IRQ);

}

void MPL3115A2::DataReady_IRQ( void)
{
    // Clear the IRQ flag
    getStatus();
    // Run the user supplied function
    MPL3115A2_usr2_fptr();   
}

void MPL3115A2::AltitudeTrigger( void(*fptr)(void), unsigned short level)
{
    unsigned char dt[5];
    unsigned char data[2];

    // Soft Reset
    Reset();
    
    // The device is on standby
    Standby();
    
    // Clear all interrupts by reading the output registers.
    readRegs( REG_ALTIMETER_MSB, &dt[0], 5);
    getStatus();

    // Write Target and Window Values
    dt[0] = REG_P_TGT_MSB;
    dt[1] = (level>>8);
    dt[2] = (level&0xFF);
    writeRegs( dt, 3);
    
    // Window values are zero
    dt[0] = REG_P_WND_MSB;
    dt[1] = 0;
    dt[2] = 0;
    writeRegs( dt, 3);

    // Enable Pressure Threshold interrupt
    data[0] = REG_CTRL_REG_4;
    data[1] = 0x08;
    writeRegs( data, 2);
    // Interrupt is routed to INT1
    data[0] = REG_CTRL_REG_5;
    data[1] = 0x08;
    writeRegs( data, 2);
    data[0] = REG_PT_DATA_CFG;
    data[1] = 0x07;
    writeRegs(data, 2);    
    // Configure the OverSampling rate, Altimeter mode and set the sensor Active
    data[0] = REG_CTRL_REG_1;
    data[1] = 0x81 | (MPL3115A2_oversampling<<3);    
    writeRegs(data, 2);    

    MPL3115A2_usr1_fptr = fptr;
    MPL3115A2_Int1.fall( this, &MPL3115A2::AltitudeTrg_IRQ);

}

void MPL3115A2::AltitudeTrg_IRQ( void)
{
    // Clear the IRQ flag
    getStatus();
    // Run the user supplied function
    MPL3115A2_usr1_fptr();   

}

void MPL3115A2::Barometric_Mode( void)
{
    unsigned char t;
    unsigned char data[2];
    
    Standby();

    // soft reset...
    Reset();
        
    Standby();
    readRegs( REG_CTRL_REG_1, &t, 1);
    
    // Set the Barometric mode
    data[0] = REG_CTRL_REG_1;
    data[1] = t&0x7F;
    writeRegs(data, 2);    

    data[0] = REG_PT_DATA_CFG;
    data[1] = 0x07;
    writeRegs(data, 2);    

    Oversample_Ratio( MPL3115A2_oversampling);
    
    Active();
    
    MPL3115A2_mode = BAROMETRIC_MODE;
}

void MPL3115A2::Altimeter_Mode( void)
{
    unsigned char t;
    unsigned char data[2];
    
    Standby();

    // soft reset...
    Reset();    
    
    Standby();
    readRegs( REG_CTRL_REG_1, &t, 1);

    data[0] = REG_CTRL_REG_1;
    data[1] = t|0x80;
    writeRegs(data, 2);    

    data[0] = REG_PT_DATA_CFG;
    data[1] = 0x07;
    writeRegs(data, 2);    

    Oversample_Ratio( MPL3115A2_oversampling);
    
    Active();
    
    MPL3115A2_mode = ALTIMETER_MODE;
}

void MPL3115A2::Oversample_Ratio( unsigned int ratio)
{
    unsigned char t;
    
    Standby();
    readRegs( REG_CTRL_REG_1, &t, 1);

    t = t & 0xE7;
    t = t | ( ratio<<3);

    unsigned char data[2] = { REG_CTRL_REG_1, t};
    writeRegs(data, 2);    

    Active();
    
    MPL3115A2_oversampling = ratio;
}


void MPL3115A2::Active( void)
{
    unsigned char t;
    
    // Activate the peripheral
    readRegs(REG_CTRL_REG_1, &t, 1);
    unsigned char data[2] = {REG_CTRL_REG_1, t|0x01};
    writeRegs(data, 2);
}

void MPL3115A2::Standby( void)
{
    unsigned char t;
    
    // Standby
    readRegs(REG_CTRL_REG_1, &t, 1);
    unsigned char data[2] = {REG_CTRL_REG_1, t&0xFE};
    writeRegs(data, 2);
}

unsigned char MPL3115A2::getDeviceID() {
    unsigned char device_id = 0;
    readRegs(REG_WHO_AM_I, &device_id, 1);
    return device_id;
}

unsigned int MPL3115A2::isDataAvailable( void)
{
    unsigned char status;
    
    readRegs( REG_STATUS, &status, 1);

    return ((status>>1));
    
}

unsigned char MPL3115A2::getStatus( void)
{
    unsigned char status;
    
    readRegs( REG_STATUS, &status, 1);
    return status;
}

unsigned int MPL3115A2::getAllData( float *f)
{
    if ( isDataAvailable() & PTDR_STATUS) {
        if ( MPL3115A2_mode == ALTIMETER_MODE) {
            f[0] = getAltimeter( REG_ALTIMETER_MSB);
        } else {
            f[0] = getPressure( REG_PRESSURE_MSB);
        }
        
        f[1] = getTemperature( REG_TEMP_MSB);
        //
        return 1;
    } else
        return 0;
}

unsigned int MPL3115A2::getAllData( float *f, float *d)
{
    if ( isDataAvailable() & PTDR_STATUS) {
        if ( MPL3115A2_mode == ALTIMETER_MODE) {
            f[0] = getAltimeter();
            d[0] = getAltimeter( REG_ALTI_DELTA_MSB);
        } else {
            f[0] = getPressure();
            d[0] = getPressure( REG_PRES_DELTA_MSB);
        }
        
        f[1] = getTemperature();
        d[1] = getTemperature( REG_TEMP_DELTA_MSB);
        //
        return 1;
    } else
        return 0;
}

void MPL3115A2::getAllMaximumData( float *f)
{
    if ( MPL3115A2_mode == ALTIMETER_MODE) {
        f[0] = getAltimeter( REG_ALTI_MAX_MSB);
    } else {
        f[0] = getPressure( REG_PRES_MAX_MSB);
    }
    
    f[1] = getTemperature( REG_TEMP_MAX_MSB);
}

void MPL3115A2::getAllMinimumData( float *f)
{
    if ( MPL3115A2_mode == ALTIMETER_MODE) {
        f[0] = getAltimeter( REG_ALTI_MIN_MSB);
    } else {
        f[0] = getPressure( REG_PRES_MIN_MSB);
    }
    
    f[1] = getTemperature( REG_TEMP_MIN_MSB);
}

float MPL3115A2::getAltimeter( void)
{
    float a;
    
    a = getAltimeter( REG_ALTIMETER_MSB);
    return a;
}

float MPL3115A2::getAltimeter( unsigned char reg)
{
    unsigned char dt[3];
    unsigned short altm;
    short tmp;
    float faltm;
    
    /*
    * dt[0] = Bits 12-19 of 20-bit real-time Altitude sample. (b7-b0)
    * dt[1] = Bits 4-11 of 20-bit real-time Altitude sample. (b7-b0)
    * dt[2] = Bits 0-3 of 20-bit real-time Altitude sample (b7-b4)
    */
    readRegs( reg, &dt[0], 3);
    altm = (dt[0]<<8) | dt[1];
    //
    if ( dt[0] > 0x7F) {
        // negative number
        tmp = ~altm + 1;
        faltm = (float)tmp * -1.0f;
    } else {
        faltm = (float)altm * 1.0f;
    }
    //
    faltm = faltm+((float)(dt[2]>>4) * 0.0625f);
    return faltm;
}

float MPL3115A2::getPressure( void)
{
    float a;
    
    a = getPressure( REG_PRESSURE_MSB);
    return a;
}

float MPL3115A2::getPressure( unsigned char reg)
{
    unsigned char dt[3];
    unsigned int prs;
    int tmp;
    float fprs;
    
    /*
    * dt[0] = Bits 12-19 of 20-bit real-time Pressure sample. (b7-b0)
    * dt[1] = Bits 4-11 of 20-bit real-time Pressure sample. (b7-b0)
    * dt[2] = Bits 0-3 of 20-bit real-time Pressure sample (b7-b4)
    */
    readRegs( reg, &dt[0], 3);
    prs = ((dt[0]<<10) | (dt[1]<<2) | (dt[2]>>6));
    //
    if ( dt[0] > 0x7f) {
        // negative number
        if ( dt[0] & 0x80)
            prs |= 0xFFFC0000;      // set at 1 the bits to complete the word len
        else
            prs |= 0xFFFE0000;
        tmp = ~prs + 1;             // make the complemets. At this point all the bits are inverted.
        fprs = (float)tmp * -1.0f;  // set the signe..
    } else {
        fprs = (float)prs * 1.0f;
    }

    if ( dt[2] & 0x10)              // I did some experiment to set the fractional parte.
        fprs += 0.25f;              // ** Warning: the DS is wrong! **
    if ( dt[2] & 0x20)
        fprs += 0.5f;
        
    return fprs;
}

float MPL3115A2::getTemperature( void)
{
    float a;
    
    a = getTemperature( REG_TEMP_MSB);
    return a;
}

float MPL3115A2::getTemperature( unsigned char reg)
{
    unsigned char dt[2];
    unsigned short temp;
    float ftemp;
    
    /*
    * dt[0] = Bits 4-11 of 16-bit real-time temperature sample. (b7-b0)
    * dt[1] = Bits 0-3 of 16-bit real-time temperature sample. (b7-b4)
    */
    readRegs( reg, &dt[0], 2);
    temp = dt[0];
    //
    if ( dt[0] > 0x7F) {
        temp = ~temp + 1;
        ftemp = (float)temp * -1.0f;
    } else {
        ftemp = (float)temp * 1.0f;
    }
    //
    ftemp = ftemp+((float)(dt[1]>>4) * 0.0625f);
    return ftemp;

}


unsigned int MPL3115A2::getAllDataRaw( unsigned char *dt)
{
    // Check for Press/Alti and Temp value ready
    if ( isDataAvailable() & PTDR_STATUS) {
        if ( MPL3115A2_mode == ALTIMETER_MODE) {
            getAltimeterRaw( &dt[0]);               // 3 bytes
        } else {
            getPressureRaw( &dt[0]);                   // 3 bytes
        }
        
        getTemperatureRaw( &dt[3]);                    // 2 bytes
        
        return 1;
    } else {
        return 0;
    }
}

unsigned int MPL3115A2::getAltimeterRaw( unsigned char *dt)
{
    
    /*
    * dt[0] = Bits 12-19 of 20-bit real-time Pressure sample. (b7-b0)
    * dt[1] = Bits 4-11 of 20-bit real-time Pressure sample. (b7-b0)
    * dt[2] = Bits 0-3 of 20-bit real-time Pressure sample (b7-b4)
    */
    
    // Check for Press/Alti value ready
    if ( isDataAvailable() & PDR_STATUS) {
        readRegs( REG_ALTIMETER_MSB, &dt[0], 3);    
        return 1;
    } else
        return 0;
}

unsigned int  MPL3115A2::getPressureRaw( unsigned char *dt)
{
    
    /*
    * dt[0] = Bits 12-19 of 20-bit real-time Pressure sample. (b7-b0)
    * dt[1] = Bits 4-11 of 20-bit real-time Pressure sample. (b7-b0)
    * dt[2] = Bits 0-3 of 20-bit real-time Pressure sample (b7-b4)
    */
    
    // Check for Press/Alti value ready
    if ( isDataAvailable() & PDR_STATUS) {
        readRegs( REG_PRESSURE_MSB, &dt[0], 3);
        return 1;
    } else 
        return 0;
        
}

unsigned int MPL3115A2::getTemperatureRaw( unsigned char *dt)
{
    
    /*
    * dt[0] = Bits 4-11 of 16-bit real-time temperature sample. (b7-b0)
    * dt[1] = Bits 0-3 of 16-bit real-time temperature sample. (b7-b4)
    */
    
    // Check for Temp value ready
    if ( isDataAvailable() & TDR_STATUS) {
        readRegs( REG_TEMP_MSB, &dt[0], 2);
        return 1;
    } else
        return 0;        
}

void MPL3115A2::SetPressureOffset( char offset)
{
    unsigned char data [2] = {REG_OFF_P, offset};

    Standby();
    writeRegs(data,2);

    Active(); 
}

void MPL3115A2::SetTemperatureOffset( char offset)
{
    unsigned char data [2] = {REG_OFF_T, offset};

    Standby();
    writeRegs(data,2);

    Active(); 
}

void MPL3115A2::SetAltitudeOffset( char offset)
{
    unsigned char data [2] = {REG_OFF_H, offset};

    Standby();
    writeRegs(data,2);

    Active(); 
}

void MPL3115A2::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}

void MPL3115A2::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}
