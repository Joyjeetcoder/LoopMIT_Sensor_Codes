#include "DFRobot_ICG20660L.h"
#include <SPI.h>

#define SPI_CS_PIN 5
DFRobot_ICG20660L_SPI icg(SPI_CS_PIN, &SPI);

// Calibration offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Init SPI
  SPI.begin(18, 19, 23, SPI_CS_PIN);

  Serial.println("Initializing ICG20660L...");
  while (icg.begin(icg.eRegMode) != 0) {
    Serial.println("ICG init failed. Check wiring.");
    delay(1000);
  }
  Serial.println("ICG ready!");

  icg.enableSensor(icg.eAxisAll);
  icg.configAccel(icg.eFSR_A_4G, icg.eAccel_DLPF_99_1KHZ);
  icg.configGyro(icg.eFSR_G_500DPS, icg.eGyro_DLPF_92_1KHZ);

  // Run calibration
  calibrateIMU();
}

void calibrateIMU() {
  const int samples = 500;
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;

  Serial.println("Calibrating... Keep IMU still!");

  for (int i = 0; i < samples; i++) {
    sumAx += icg.getAccelDataX();
    sumAy += icg.getAccelDataY();
    sumAz += icg.getAccelDataZ();

    sumGx += icg.getGyroDataX();
    sumGy += icg.getGyroDataY();
    sumGz += icg.getGyroDataZ();

    delay(5);
  }

  accelOffsetX = sumAx / samples;
  accelOffsetY = sumAy / samples;
  // Subtract 1g (gravity) from Z axis
  accelOffsetZ = (sumAz / samples) - 1.0;

  gyroOffsetX = sumGx / samples;
  gyroOffsetY = sumGy / samples;
  gyroOffsetZ = sumGz / samples;

  Serial.println("Calibration complete!");
  Serial.print("Accel Offsets -> X: "); Serial.print(accelOffsetX, 4);
  Serial.print(", Y: "); Serial.print(accelOffsetY, 4);
  Serial.print(", Z: "); Serial.println(accelOffsetZ, 4);

  Serial.print("Gyro Offsets -> X: "); Serial.print(gyroOffsetX, 4);
  Serial.print(", Y: "); Serial.print(gyroOffsetY, 4);
  Serial.print(", Z: "); Serial.println(gyroOffsetZ, 4);
}

void loop() {
  // Read calibrated data
  float ax = icg.getAccelDataX() - accelOffsetX;
  float ay = icg.getAccelDataY() - accelOffsetY;
  float az = icg.getAccelDataZ() - accelOffsetZ;

  float ax_r = ax*9.80665;
  float ay_r = ay*9.80665;
  float az_r = az*9.80665;

  float gx = icg.getGyroDataX() - gyroOffsetX;
  float gy = icg.getGyroDataY() - gyroOffsetY;
  float gz = icg.getGyroDataZ() - gyroOffsetZ;

  Serial.print("Accel (g): X="); Serial.print(ax_r, 3);
  Serial.print(", Y="); Serial.print(ay_r, 3);
  Serial.print(", Z="); Serial.print(az_r, 3);
  Serial.println("");
  
  /*Serial.print(" | Gyro (°/s): X="); Serial.print(gx, 3);
  Serial.print(", Y="); Serial.print(gy, 3);
  Serial.print(", Z="); Serial.println(gz, 3);*/

  delay(200);
}
