#include "SparkFunLSM303C.h"
#include "i2c-dev.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

// Public methods
status_t LSM303C::begin()
{
  printf("\n");
  return
  begin(// Default to I2C bus
        MODE_I2C,
        // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
        MAG_DO_40_Hz,
        // Initialize magnetic field full scale to +/-16 gauss
        MAG_FS_16_Ga,
        // Enabling block data updating
        MAG_BDU_ENABLE,
        // Initialize magnetometer X/Y axes ouput data rate to high-perf mode
        MAG_OMXY_HIGH_PERFORMANCE,
        // Initialize magnetometer Z axis performance mode
        MAG_OMZ_HIGH_PERFORMANCE,
        // Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
        MAG_MD_CONTINUOUS,
        // Initialize acceleration full scale to +/-2g
        ACC_FS_2g,
        // Enable block data updating
        ACC_BDU_ENABLE,
        // Enable X, Y, and Z accelerometer axes
        ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
        // Initialize accelerometer output data rate to 100 Hz (turn on device)
        ACC_ODR_100_Hz
        );
}

void LSM303C::enableIMU()
{


        int res, bus,  size;

        char filename[20];
        sprintf(filename, "/dev/i2c-%d", 1);
        file = open(filename, O_RDWR);
        if (file<0) {
		       printf("Unable to open I2C bus!");
           _exit(1);
        }

 // Enable accelerometer.
	//writeAccReg(LSM303_CTRL_REG1_A, 0b01010111); //  z,y,x axis enabled , 100Hz data rate
	//writeAccReg(LSM303_CTRL_REG4_A, 0b00101000); // +/- 8G full scale: FS = 10 on DLHC, high resolution output mode

 // Enable magnetometer
  //      writeMagReg(LSM303_MR_REG_M, 0x00);  // enable magnometer

}

status_t LSM303C::begin(InterfaceMode_t im, MAG_DO_t modr, MAG_FS_t mfs,
    MAG_BDU_t mbu, MAG_OMXY_t mxyodr, MAG_OMZ_t mzodr, MAG_MD_t mm,
    ACC_FS_t afs, ACC_BDU_t abu, uint8_t aea, ACC_ODR_t aodr)
{
  uint8_t successes = 0;
  // Select I2C or SPI
  interfaceMode = im;
  enableIMU();
  
  ////////// Initialize Magnetometer //////////
  // Initialize magnetometer output data rate
  successes += MAG_SetODR(modr);
  // Initialize magnetic field full scale
  successes += MAG_SetFullScale(mfs);
  // Enabling block data updating
  successes += MAG_BlockDataUpdate(mbu);
  // Initialize magnetometer X/Y axes ouput data rate
  successes += MAG_XY_AxOperativeMode(mxyodr);
  // Initialize magnetometer Z axis performance mode
  successes += MAG_Z_AxOperativeMode(mzodr);
  // Initialize magnetometer run mode.
  successes += MAG_SetMode(mm);

  ////////// Initialize Accelerometer //////////
  // Initialize acceleration full scale
  successes += ACC_SetFullScale(afs);
  // Enable block data updating
  successes += ACC_BlockDataUpdate(abu);
  // Enable X, Y, and Z accelerometer axes
  successes += ACC_EnableAxis(aea);
  // Initialize accelerometer output data rate
  successes += ACC_SetODR(aodr);

  return (successes == IMU_SUCCESS) ? IMU_SUCCESS : IMU_HW_ERROR;
}

float LSM303C::readMagX()
{
  return readMag(xAxis);
}

float LSM303C::readMagY()
{
  return readMag(yAxis);
}

float LSM303C::readMagZ()
{
  return readMag(zAxis);
}

float LSM303C::readAccelX()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    printf("AERROR\n");
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_X_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( ACC_ReadReg(ACC_OUT_X_H, valueH) )
    {
	    return IMU_HW_ERROR;
    }
  
    if ( ACC_ReadReg(ACC_OUT_X_L, valueL) )
    {
	    return IMU_HW_ERROR;
    }
  
    // printf("Fresh raw data\n");

    //convert from LSB to mg
    return int16_t(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
  }

  // Should never get here
  printf("Returning NAN\n");
  return NAN;
}

float LSM303C::readAccelY()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    printf("AERROR\n");
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_Y_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( ACC_ReadReg(ACC_OUT_Y_H, valueH) )
    {
	    return IMU_HW_ERROR;
    }
  
    if ( ACC_ReadReg(ACC_OUT_Y_L, valueL) )
    {
	    return IMU_HW_ERROR;
    }
  
    // printf("Fresh raw data\n");

    //convert from LSB to mg
    return int16_t(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
  }

  // Should never get here
  printf("Returning NAN\n");
  return NAN;
}

float LSM303C::readAccelZ()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    printf("AERROR\n");
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_Z_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( ACC_ReadReg(ACC_OUT_Z_H, valueH) )
    {
	    return IMU_HW_ERROR;
    }
  
    if ( ACC_ReadReg(ACC_OUT_Z_L, valueL) )
    {
	    return IMU_HW_ERROR;
    }
  
    // printf("Fresh raw data\n");

    //convert from LSB to mg
    return(int16_t(( (valueH << 8) | valueL )) * SENSITIVITY_ACC);
  }

  // Should never get here
  printf("Returning NAN\n");
  return NAN;
}


float LSM303C::readTempC()
{
  uint8_t valueL;
  uint8_t valueH;
  float temperature;

  // Make sure temperature sensor is enabled
  if( MAG_TemperatureEN(MAG_TEMP_EN_ENABLE))
  {
    return NAN;
  }

	if( MAG_ReadReg(MAG_TEMP_OUT_L, valueL) )
  {
    return NAN;
  }

  if( MAG_ReadReg(MAG_TEMP_OUT_H, valueH) )
  {
    return NAN;
  }

  temperature = (float)( (valueH << 8) | valueL );
  temperature /= 8; // 8 digits/˚C
  temperature += 25;// Reads 0 @ 25˚C

  return temperature;  
}

float LSM303C::readTempF()
{
  return( (readTempC() * 9.0 / 5.0) + 32.0);
}



////////////////////////////////////////////////////////////////////////////////
////// Protected methods

float LSM303C::readAccel(AXIS_t dir)
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
  
  if (response != IMU_SUCCESS)
  {
    printf("AERROR\n");
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_ZYX_NEW_DATA_AVAILABLE)
  {
    response = ACC_GetAccRaw(accelData);
    // printf("Fresh raw data\n");
  }
  //convert from LSB to mg
  switch (dir)
  {
  case xAxis:
    return accelData.xAxis * SENSITIVITY_ACC;
    break;
  case yAxis:
    return accelData.yAxis * SENSITIVITY_ACC;
    break;
  case zAxis:
    return accelData.zAxis * SENSITIVITY_ACC;
    break;
  default:
    return NAN;
  }

  // Should never get here
  printf("Returning NAN\n");
  return NAN;
}

float LSM303C::readMag(AXIS_t dir)
{
  MAG_XYZDA_t flag_MAG_XYZDA;
  status_t response = MAG_XYZ_AxDataAvailable(flag_MAG_XYZDA);
  
  if (response != IMU_SUCCESS)
  {
    printf("MERROR\n");
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  if (flag_MAG_XYZDA & MAG_XYZDA_YES)
  {
    response = MAG_GetMagRaw(magData);
    // printf("Fresh raw data\n");
  }
  //convert from LSB to Gauss
  switch (dir)
  {
  case xAxis:
    return magData.xAxis * SENSITIVITY_MAG;
    break;
  case yAxis:
    return magData.yAxis * SENSITIVITY_MAG;
    break;
  case zAxis:
    return magData.zAxis * SENSITIVITY_MAG;
    break;
  default:
    return NAN;
  }

  // Should never get here
  printf("Returning NAN\n");
  return NAN;
}

status_t LSM303C::MAG_GetMagRaw(AxesRaw_t& buff)
{
  // printf("EMPTY\n");
  uint8_t valueL;
  uint8_t valueH;
  
  // printf("& was false\n");
  if( MAG_ReadReg(MAG_OUTX_L, valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTX_H, valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff.xAxis = (int16_t)( (valueH << 8) | valueL );
  
  if( MAG_ReadReg(MAG_OUTY_L, valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTY_H, valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff.yAxis = (int16_t)( (valueH << 8) | valueL );
  
  if( MAG_ReadReg(MAG_OUTZ_L, valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTZ_H, valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff.zAxis = (int16_t)( (valueH << 8) | valueL );

  return IMU_SUCCESS;
}

// Methods required to get device up and running
status_t LSM303C::MAG_SetODR(MAG_DO_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if(MAG_ReadReg(MAG_CTRL_REG1, value))
  {
    printf("Failed Read from MAG_CTRL_REG1\n");
    return IMU_HW_ERROR;
  }

  // Mask and only change DO0 bits (4:2) of MAG_CTRL_REG1
  value &= ~MAG_DO_80_Hz;
  value |= val;

  if(MAG_WriteReg(MAG_CTRL_REG1, value))
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_SetFullScale(MAG_FS_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG2, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_FS_16_Ga; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG2, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_BlockDataUpdate(MAG_BDU_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG5, value) )
  {
    return IMU_HW_ERROR;
  }


  value &= ~MAG_BDU_ENABLE; //mask
  value |= val;		

  if ( MAG_WriteReg(MAG_CTRL_REG5, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_XYZ_AxDataAvailable(MAG_XYZDA_t& value)
{
  if ( MAG_ReadReg(MAG_STATUS_REG, (uint8_t&)value) )
  {
    return IMU_HW_ERROR;
  }

  value = (MAG_XYZDA_t)((int8_t)value & (int8_t)MAG_XYZDA_YES);

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_XY_AxOperativeMode(MAG_OMXY_t val)
{
  // printf("EMPTY\n");

  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }
	
  value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_Z_AxOperativeMode(MAG_OMZ_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG4, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_SetMode(MAG_MD_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG3, value) )
  {
    printf("Failed to read MAG_CTRL_REG3. 'Read': 0x");
    printf("0x%d\n",value);
    return IMU_HW_ERROR;
  }

  value &= ~MAG_MD_POWER_DOWN_2;
  value |= val;		

  if ( MAG_WriteReg(MAG_CTRL_REG3, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::ACC_SetFullScale(ACC_FS_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL4, value) )
  {
    printf("Failed ACC read\n");
    return IMU_HW_ERROR;
  }

  value &= ~ACC_FS_8g;
  value |= val;	


  if ( ACC_WriteReg(ACC_CTRL4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::ACC_BlockDataUpdate(ACC_BDU_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~ACC_BDU_ENABLE;
  value |= val;	

  if ( ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::ACC_EnableAxis(uint8_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL1, value) )
  {
    printf("AERROR\n");
    return IMU_HW_ERROR;
  }

  value &= ~0x07;
  value |= val;	

  if ( ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::ACC_SetODR(ACC_ODR_t val)
{
  // printf("EMPTY\n");
  uint8_t value;

  if ( ACC_ReadReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~ACC_ODR_MASK;
  value |= val;	
	
  if ( ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_TemperatureEN(MAG_TEMP_EN_t val){
  uint8_t value;

  if( MAG_ReadReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_TEMP_EN_ENABLE; //mask
  value |= val;	

  if( MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::selectDevice(int file, int addr)
{
        char device[3];
        if (addr == 1)
                device == "L3G";
        else
                device == "LSM";


        if (ioctl(file, I2C_SLAVE, addr) < 0) {
          return IMU_HW_ERROR;
        }
        
        return IMU_SUCCESS;
}

status_t LSM303C::MAG_ReadReg(MAG_REG_t reg, uint8_t& data)
{
  // printf("Reading register 0x");
  // printf("0x%d\n", reg);
  status_t ret = IMU_GENERIC_ERROR;
    
  if (interfaceMode == MODE_I2C)
  {
    ret = I2C_ByteRead(MAG_I2C_ADDR, reg, data);
  }
  else
  {
    ret = IMU_GENERIC_ERROR; // Shouldn't get here
  }

  return ret;
}

uint8_t  LSM303C::MAG_WriteReg(MAG_REG_t reg, uint8_t data)
{
  // printf("EMPTY\n");
  uint8_t ret;
    
  if (interfaceMode == MODE_I2C)
  {
    ret = I2C_ByteWrite(MAG_I2C_ADDR, reg, data);
  }
  else
  {
    ret = IMU_GENERIC_ERROR;
  }

  return ret;
}

status_t LSM303C::ACC_ReadReg(ACC_REG_t reg, uint8_t& data)
{
  // printf("Reading address 0x");
  // printf("0x%d\n", reg);
  status_t ret;
    
  if (interfaceMode == MODE_I2C)
  {
    ret = I2C_ByteRead(ACC_I2C_ADDR, reg, data);
  }
  else
  {
    ret = IMU_HW_ERROR;
  }

  return ret;
}





uint8_t  LSM303C::ACC_WriteReg(ACC_REG_t reg, uint8_t data)
{
  // printf("EMPTY\n");
  uint8_t ret;
    
  if (interfaceMode == MODE_I2C)
  {
    ret = I2C_ByteWrite(ACC_I2C_ADDR, reg, data);
  }
  else
  {
    ret = IMU_GENERIC_ERROR;
  }

  return ret;
}

uint8_t  LSM303C::I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg,
    uint8_t data)
{
  uint8_t ret = IMU_GENERIC_ERROR;
  
  ret = selectDevice(file, slaveAddress);
  if (ret != IMU_SUCCESS) {
    return ret;
  }
  int result = i2c_smbus_write_byte_data(file, reg, data);
  if (result == -1)
  {
      printf ("Failed to write byte to I2C Acc.");
      ret = IMU_HW_ERROR;
  } else {
    ret = IMU_SUCCESS;
  }
  
  return ret;
}

status_t LSM303C::I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg,
    uint8_t& data)
{
  status_t ret = IMU_GENERIC_ERROR;
  // printf("Reading from I2C address: 0x");
  // printf("%x\n", slaveAddress);
  // printf(", register 0x");
  // printf("%x\n", reg);
  
  ret = selectDevice(file,slaveAddress);
  
  if (ret != IMU_SUCCESS) {
    return ret;
  }
  
  int result = i2c_smbus_read_i2c_block_data(file, reg, 1, &data);
  if (result != 1)
  {
      printf("Failed to read block from I2C.");
      ret = IMU_HW_ERROR;
  } else {
    ret = IMU_SUCCESS;
  }
  return ret;
}

// uint8_t  LSM303C::I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg,
//     uint8_t data)
// {
//   uint8_t ret = IMU_GENERIC_ERROR;
//   Wire.beginTransmission(slaveAddress);  // Initialize the Tx buffer
//   // returns num bytes written
//   if (Wire.write(reg))
//   {
//     ret = Wire.write(data);
//     if (ret)
//     {
//       printf("Wrote: 0x");
//       printf("%x\n", data);
//       switch (Wire.endTransmission())
//       {
//       case 0:
//         ret = IMU_SUCCESS;
//         break;
//       case 1: // Data too long to fit in transmit buffer
//       case 2: // Received NACK on transmit of address
//       case 3: // Received NACK on transmit of data
//       case 4: // Other Error
//       default:
//         ret = IMU_HW_ERROR;
//       }
//     }
//     else
//     {
//       ret = IMU_HW_ERROR;
//     }
//   }
//   else
//   {
//     ret = IMU_HW_ERROR;
//   }
//   return ret;
// }

// status_t LSM303C::I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg,
//     uint8_t& data)
// {
//   status_t ret = IMU_GENERIC_ERROR;
//   printf("Reading from I2C address: 0x");
//   printf("%x\n", slaveAddress);
//   printf(", register 0x");
//   printf("%x\n", reg);
//   Wire.beginTransmission(slaveAddress); // Initialize the Tx buffer
//   if (Wire.write(reg))  // Put slave register address in Tx buff
//   {
//     if (Wire.endTransmission(false))  // Send Tx, send restart to keep alive
//     {
//       printf("Error: I2C buffer didn't get sent!\n");
//       printf("Slave address: 0x");
//       printf("%x\n", slaveAddress);
//       printf("Register: 0x");
//       printf("%x\n", reg);

//       ret = IMU_HW_ERROR;
//     }
//     else if (Wire.requestFrom(slaveAddress, 1))
//     {
//       data = Wire.read();
//       printf("Read: 0x");
//       printf("%x\n", data);
//       ret = IMU_SUCCESS;
//     }
//     else
//     {
//       printf("IMU_HW_ERROR\n");
//       ret = IMU_HW_ERROR;
//     }
//   }
//   else
//   {
//     printf("Error: couldn't send slave register address\n");
//   }
//   return ret;
// }

status_t LSM303C::ACC_Status_Flags(uint8_t& val)
{
  // printf("Getting accel status\n");
  if( ACC_ReadReg(ACC_STATUS, val) )
  {
    printf("AERROR\n");
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::ACC_GetAccRaw(AxesRaw_t& buff)
{
  uint8_t valueL;
  uint8_t valueH;

  if ( ACC_ReadReg(ACC_OUT_X_H, valueH) )
  {
	  return IMU_HW_ERROR;
  }
  
  if ( ACC_ReadReg(ACC_OUT_X_L, valueL) )
  {
	  return IMU_HW_ERROR;
  }
  
  buff.xAxis = (int16_t)( (valueH << 8) | valueL );
  
  if ( ACC_ReadReg(ACC_OUT_Y_H, valueH) )
  {
	  return IMU_HW_ERROR;
  }
  
  if ( ACC_ReadReg(ACC_OUT_Y_L, valueL) )
  {
	  return IMU_HW_ERROR;
  }
  
  buff.yAxis = (int16_t)( (valueH << 8) | valueL );
  
  if ( ACC_ReadReg(ACC_OUT_Z_H, valueH) )
  {
	  return IMU_HW_ERROR;
  }
  
  if ( ACC_ReadReg(ACC_OUT_Z_L, valueL) )
  {
	  return IMU_HW_ERROR;
  }

  buff.zAxis = (int16_t)( (valueH << 8) | valueL ); 

  return IMU_SUCCESS;
}

int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int main(int argc, char *argv[])
{
  LSM303C myIMU;
  struct  timeval tvBegin, tvEnd,tvDiff;
  int startInt  = mymillis();
  printf("This is my main\n");
    if (myIMU.begin() != IMU_SUCCESS)
  {
    printf("Failed setup.\n");
    _exit(1);
  }
  
  gettimeofday(&tvBegin, NULL);
  
  while(1) {
  startInt = mymillis();
    //Get all parameters
  printf("\nAccelerometer:\n");
  printf(" X = ");
  printf("%f\n",myIMU.readAccelX());
  printf(" Y = ");
  printf("%f\n",myIMU.readAccelY());
  printf(" Z = ");
  printf("%f\n",myIMU.readAccelZ());

  // Not supported by hardware, so will return NAN
  printf("\nGyroscope:\n");
  printf(" X = ");
  printf("%f",myIMU.readGyroX());
  printf(" Y = ");
  printf("%f",myIMU.readGyroY());
  printf(" Z = ");
  printf("%f",myIMU.readGyroZ());

  printf("\nMagnetometer:\n");
  printf(" X = ");
  printf("%f",myIMU.readMagX());
  printf(" Y = ");
  printf("%f",myIMU.readMagY());
  printf(" Z = ");
  printf("%f",myIMU.readMagZ());

  printf("\nThermometer:\n");
  printf(" Degrees C = ");
  printf("%f",myIMU.readTempC());
  printf(" Degrees F = ");
  printf("%f",myIMU.readTempF());
  
	//Each loop should be at least 20ms.
        while(mymillis() - startInt < 200)
        {
            usleep(100);
        }

	printf("Loop Time %d\t", mymillis()- startInt);
  }
  
}
