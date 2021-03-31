#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define seaLevelPressure_hPa 1024
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
Adafruit_MPU6050 mpu;

float altitude, acceleration;

float q = 0.0001;

// The system dynamics
BLA::Matrix<3, 3> A = {1.0, 0.05, 0.00125,
                        0, 1.0, 0.05,
                        0, 0, 1};

// Relationship between measurement and states
BLA::Matrix<2, 3> H = {1.0, 0, 0,
                        0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                        0, 1, 0, 
                        0, 0, 1};


// Measurement error covariance
BLA::Matrix<2, 2> R = {35.8229, 0,
                        0, 0.012};

// Process noise covariance
BLA::Matrix<3, 3> Q = {q, 0, 0,
                        0, q, 0, 
                        0, 0, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};

BLA::Matrix<3, 1> x_hat = {0.0,
                            0.0,
                            0.0};


BLA::Matrix<3, 1> Y = {0.0,
                       0.0,
                       0.0};



void setup(void) {
  Serial.begin(115200);
  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");

  Serial.println("");
  delay(100);

    Serial.println(F("BMP280 test"));
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

    altitude = bmp.readAltitude(seaLevelPressure_hPa);
  acceleration = a.acceleration.z;


    BLA::Matrix<2, 1> Z = {altitude,
                        acceleration};

    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;

    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;

    BLA::Matrix<3, 2> K  = P_minus * (~H) * ((H * P_minus * (~H) + R)).Inverse();

    x_hat = x_hat_minus + K * (Z - (H * x_hat_minus));

    P = (I - K * H) * P_minus;
    
    // Y = Z - H * x_hat_minus;
    
    float s,v,ac;
    
    s = x_hat(0);
    v = x_hat(1);
    ac = x_hat(2);
    
    Serial.print(s);Serial.print("\t");
    Serial.print(v);Serial.print("\t");
    Serial.println(ac);Serial.println("\t");

    delay(500);


}

