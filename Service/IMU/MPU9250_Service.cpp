#include "MPU9250_Service.hpp"
#include "MPU9250_HAL.hpp"
#include <cmath>
#include "pico/stdlib.h"

IMUService::IMUService(MPU9250_HAL &hal) // better to but in magic file 
: hal_(hal),
  //Physical_Value = Raw_Value × Scale_Factor
  accelScale_(1.0f / 16384.0f),   
  gyroScale_(1.0f / 131.0f),      
  tempScale_(1.0f / 333.87f),     
  magScale_(0.15f)                
{}

bool IMUService::IMUInit(uint sda_pin, uint scl_pin, uint32_t baudrate_hz) 
{ 

    if (!hal_.init(sda_pin, scl_pin, baudrate_hz))
    {
        return false;
    }
    
    return true;
}

AccelData IMUService::getAccelerometer() 
{
    //Edit create struct 
    int16_t ax, ay, az;

    if (!hal_.readAccelRaw(ax, ay, az))
    {
        return {0, 0, 0};
    }

    return 
    {
        ax * accelScale_,
        ay * accelScale_,
        az * accelScale_
    };
}

GyroData IMUService::getGyroscope()
{
    int16_t gx, gy, gz;
    if (!hal_.readGyroRaw(gx, gy, gz))
    {
        return {0, 0, 0};
    }

    return 
    {
        gx * gyroScale_,
        gy * gyroScale_,
        gz * gyroScale_
    };
}

TempData IMUService::getTemperature() 
{
    int16_t tempRaw;
    if (!hal_.readTempRaw(tempRaw))
    {
        return {0};
    }

    float temp_c = (tempRaw * tempScale_) + 21.0f;
    {
        return { temp_c };
    }
}

/*
MagData IMUService::getMagnetometer() 
{
    int16_t mx, my, mz;

    if (!hal_.readMagRaw(mx, my, mz))
        return {0,0,0};

    return {
        mx * magScale_,   
        my * magScale_,
        mz * magScale_
    };
}
*/

IMUData IMUService::getAll() 
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t tempRaw;
    //int16_t mx, my, mz;

    hal_.readAllRaw(ax,ay,az,
                    gx,gy,gz,
                    tempRaw);

    IMUData data;

    data.accel= 
    {
        ax * accelScale_,
        ay * accelScale_,
        az * accelScale_
    };

    data.gyro = 
    {
        gx * gyroScale_,
        gy * gyroScale_,
        gz * gyroScale_
    };

    data.temp = 
    {
        (tempRaw * tempScale_) + 21.0f
    };

    /*data.mag = 
    {
        mx * magScale_,
        my * magScale_,
        mz * magScale_
    };*/

    return data;
}