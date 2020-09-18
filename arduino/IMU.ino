#include <MPU9250.h>

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
float gx, gy, gz;
float thetaM, phiM;
float thetaFOld = 0, thetaFNew, phiFOld = 0, phiFNew;
float thetaG = 0.0, phiG = 0.0;
unsigned long millisOld = 0;
float dt;
float theta = 0.0, phi = 0.0;
float thetaRad, phiRad;
float psi;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  status = IMU.calibrateAccel();
  status = IMU.calibrateGyro();
//  IMU.setGyroBiasX_rads(0.02);
//  IMU.setGyroBiasY_rads(0);
//  IMU.setGyroBiasZ_rads(0.03);
  status = IMU.calibrateMag();
}

void readSensor() {
  IMU.readSensor();

  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  
  float ax = IMU.getAccelX_mss();
  float ay = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();
  float mx = IMU.getMagX_uT();
  float my = IMU.getMagY_uT();
  float mz = IMU.getMagZ_uT();

  // Use accelerometer calculate pitch, roll
  thetaM = -atan2(ax / 9.8, az / 9.8) / PI * 180;
  phiM = atan2(ay / 9.8, az / 9.8) / PI * 180;
  // Move smooth
  thetaFNew = .95 * thetaFOld + .05 * thetaM;
  phiFNew = .95 * phiFOld + .05 * phiM;

  thetaFOld = thetaFNew;
  phiFOld = phiFNew;

  // Use gyro
//  Serial.print(millis());
//  Serial.print(",");
  dt = (millis() - millisOld) / 1000.0;
//  Serial.print(dt);
//  Serial.print(",");
  
  millisOld = millis();

  // Fusion accelerometer and gyro with no filter
  theta = (theta + gy * dt) * 0.95 + thetaM * 0.05;
  phi = (phi + gx * dt) * 0.95 + phiM * 0.05;

  // Use gyro
  thetaG = thetaG + gx * dt;
  phiG = phiG + gy * dt;

  // Compass
  thetaRad = theta / 180 * PI;
  phiRad = phi / 180 * PI;

//  Serial.print(mx);
//  Serial.print(",");
//  Serial.print(my);
//  Serial.print(",");
//  Serial.print(mz);
//  Serial.print(",");
  
  mx = -mx * cos(thetaRad) + my * sin(phiRad) * sin(thetaRad) - mz * cos(phiRad) * sin(thetaRad);
  my = -my * cos(phiRad) - mz * sin(phiRad);
  psi = atan2(my, mx) / PI * 180;
  

//  Serial.print(gx);
//  Serial.print(",");
//  Serial.print(gy);
//  Serial.print(",");
//  Serial.print(gz);
//  Serial.print(",");
//  Serial.print(thetaM);
//  Serial.print(",");
//  Serial.print(phiM);
//  Serial.print(",");
  
//  Serial.print(ax);
//  Serial.print(",");
//  Serial.print(ay);
//  Serial.print(",");
//  Serial.print(az);
//  Serial.print(",");
//  Serial.print(thetaG);
//  Serial.print(",");
//  Serial.println(phiG);
//  Serial.print(thetaM);
//  Serial.print(",");
//  Serial.println(phiM);

  // Print the final theta(pitch), phi(roll), psi(yaw)
  Serial.print(theta);
  Serial.print(",");
  Serial.print(phi);
  Serial.print(",");
  Serial.println(psi);
}

void loop() {
  readSensor();
  
  delay(50);
}
