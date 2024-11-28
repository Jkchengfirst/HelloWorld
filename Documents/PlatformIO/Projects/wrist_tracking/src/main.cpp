#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>

// 创建 BNO055 传感器对象，I2C 地址为 0x28 或 0x29
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // 开启串口调试输出
  Serial.begin(9600);
  delay(1000);  // 等待串口准备好
  
  // 初始化 BNO055
  if (!bno.begin()) {
    Serial.print("无法初始化 BNO055，检查连接！");
    while (1);
  }

  // 启用外部晶振，用于提高姿态跟踪的精度
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 初始化成功！");
}

void loop() {
  // 获取加速度计、陀螺仪和磁力计的数据并融合
  sensors_event_t accelData, gyroData, magData;
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Quaternion quat = bno.getQuat();
  /* // 打印加速度计数据
  Serial.print("加速度 (m/s^2): ");
  Serial.print("X: "); Serial.print(accelData.acceleration.x);
  Serial.print(" Y: "); Serial.print(accelData.acceleration.y);
  Serial.print(" Z: "); Serial.println(accelData.acceleration.z);

  // 打印陀螺仪数据
  Serial.print("角速度 (rad/s): ");
  Serial.print("X: "); Serial.print(gyroData.gyro.x);
  Serial.print(" Y: "); Serial.print(gyroData.gyro.y);
  Serial.print(" Z: "); Serial.println(gyroData.gyro.z);

  // 打印磁力计数据
  Serial.print("磁场强度 (uT): ");
  Serial.print("X: "); Serial.print(magData.magnetic.x);
  Serial.print(" Y: "); Serial.print(magData.magnetic.y);
  Serial.print(" Z: "); Serial.println(magData.magnetic.z);

  // 打印融合后的四元数
  imu::Quaternion quat = bno.getQuat();
  Serial.print("四元数: ");
  Serial.print("W: "); Serial.print(quat.w());
  Serial.print(" X: "); Serial.print(quat.x());
  Serial.print(" Y: "); Serial.print(quat.y());
  Serial.print(" Z: "); Serial.println(quat.z()); */

  // 将四元数转换为欧拉角
  float roll = atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
  float pitch = asin(2.0 * (quat.w() * quat.y() - quat.z() * quat.x()));
  float yaw = atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z())); 

/*   // 打印通过四元数计算的欧拉角
  Serial.print("计算得到的姿态 (欧拉角度): ");
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Yaw: "); Serial.println(yaw); 

  // 延迟100毫秒，避免输出过快
  delay(100); */
  

  // 将四元数数据通过串口发送 (格式: qw,qx,qy,qz)
  Serial.print(quat.w(), 4); // 四元数精度限制为4位小数
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  Serial.print(",");

  // 发送加速度数据（格式: ax,ay,az）
  // 从BNO055读取的四元数和加速度数据
float qw = quat.w();
float qx = quat.x();
float qy = quat.y();
float qz = quat.z();
float ax = accelData.acceleration.x;
float ay = accelData.acceleration.y;
float az = accelData.acceleration.z;

// 计算重力加速度矢量
float gravity_x = 2 * (qx * qz - qw * qy) * 9.794;
float gravity_y = 2 * (qw * qx + qy * qz) * 9.794;
float gravity_z = (qw * qw - qx * qx - qy * qy + qz * qz) * 9.794;

// 计算去除重力后的加速度
float dynamic_ax = ax - gravity_x;
float dynamic_ay = ay - gravity_y;
float dynamic_az = az - gravity_z;

// 输出加速度数据，保留4位小数
Serial.print(dynamic_ax, 4);
Serial.print(",");
Serial.print(dynamic_ay, 4);
Serial.print(",");
Serial.print(dynamic_az, 4);
Serial.print(",");

  
Serial.print(pitch,4);Serial.print(",");
Serial.print(roll,4);Serial.print(",");
Serial.print(yaw,4);
Serial.println();
   // 打印陀螺仪数据
  
  /* Serial.print(gyroData.gyro.x,4);Serial.print(",");
  Serial.print(gyroData.gyro.y,4);Serial.print(",");
  Serial.print(gyroData.gyro.z,4);Serial.print(","); */
  
  // 换行
  

  delay(100); // 延迟100毫秒，避免输出过快
}
