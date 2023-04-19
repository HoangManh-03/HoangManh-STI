#ifndef _BNO055_IMU_
#define _BNO055_IMU_

#include "BNO055_support.h"     //Contains the bridge code between the API and Arduino
#include <Wire.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001

#define ACCEL_SCALE 1 / 100 // LSB/g
#define GYRO_SCALE 1 / 16 // LSB/(deg/s)
#define MAG_SCALE 1 / 16 // uT/LSB

struct bno055_t myBNO;

bool initIMU()
{

  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
    BNO_Init(&myBNO); //Assigning the structure to hold information about the device
    // bno055_set_operation_mode(OPERATION_MODE_ACCGYRO);
    bno055_set_operation_mode(OPERATION_MODE_NDOF);

    return true;
}

geometry_msgs::Quaternion readQuaternion()
{
    geometry_msgs::Quaternion quat;
    bno055_quaternion quat_;
    bno055_read_quaternion_wxyz(&quat_);

    quat.x = (double)quat_.x;
    quat.y = (double)quat_.y;
    quat.z = (double)quat_.z;
    quat.w = (double)quat_.w;

    return quat;
}

geometry_msgs::Vector3 readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    bno055_accel acc_;
    bno055_read_accel_xyz(&acc_);

    accel.x = (double)acc_.x * (double) ACCEL_SCALE * G_TO_ACCEL ;
    accel.y = (double)acc_.y * (double) ACCEL_SCALE * G_TO_ACCEL ;
    accel.z = (double)acc_.z * (double) ACCEL_SCALE * G_TO_ACCEL ;

    return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
    geometry_msgs::Vector3 gyro;
    bno055_gyro my_gyr;
    bno055_read_gyro_xyz(&my_gyr);

    gyro.x = (double)my_gyr.x * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = (double)my_gyr.y * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = (double)my_gyr.z * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
	bno055_mag my_mag;
    geometry_msgs::Vector3 mag;
    bno055_read_mag_xyz(&my_mag);

    mag.x = (double)my_mag.x * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = (double)my_mag.y * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = (double)my_mag.z * (double) MAG_SCALE * UTESLA_TO_TESLA;
    // mag.x = 0;
    // mag.y = 0;
    // mag.z = 0;

    return mag;
}

#endif
