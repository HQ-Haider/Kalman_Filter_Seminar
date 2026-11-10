#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== Screen Settings ===== It depends on the screen being used.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_MPU6050 mpu;

// ===== Kalman Filter Variables =====
float kalmanRoll = 0;   
float uncertaintyRoll = 2;
float kalmanPitch = 0;   
float uncertaintyPitch = 2;
float Q = 0.01;   
float R = 0.5;    

// ===== Calibration Variables =====
float gyroBiasX = 0;
float gyroBiasY = 0;
float accelOffsetRoll = 0;
float accelOffsetPitch = 0;

unsigned long lastTime = 0;

float kalman1D(float measuredAngle, float gyroRate, float dt, float &angle, float &uncertainty) {
  angle = angle + dt * gyroRate;
  uncertainty = uncertainty + dt * dt * Q;
  float gain = uncertainty / (uncertainty + R);
  angle = angle + gain * (measuredAngle - angle);
  uncertainty = (1 - gain) * uncertainty;
  return angle;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(5, 6);  // SDA=5, SCL=6 It depends on the controller being used.

  // ===== Initialize Display =====
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  display.clearDisplay();
  display.setTextSize(1);      // Small font
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Init...");
  display.display();
  delay(500);

  // ===== Initialize Sensor =====
  if (!mpu.begin(0x68, &Wire)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("MPU Error!");
    display.display();
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ===== Calibration =====
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Calibrating...");
  display.println("Keep still");
  display.display();
  
  delay(1000);

  float sumGx = 0, sumGy = 0;
  int samples = 1000;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumGx += g.gyro.x * 180.0 / PI;
    sumGy += g.gyro.y * 180.0 / PI;
    delay(2);
  }
  gyroBiasX = sumGx / samples;
  gyroBiasY = sumGy / samples;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  
  accelOffsetRoll = atan2(ay, az) * 180.0 / PI;
  accelOffsetPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  
  kalmanRoll = 0;   
  kalmanPitch = 0; 
  
  lastTime = millis();
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Ready!");
  display.display();
  delay(500);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float rawRoll = (atan2(ay, az) * 180.0 / PI) - accelOffsetRoll;
  float rawPitch = (atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI) - accelOffsetPitch;

  float rateRoll = (g.gyro.x * 180.0 / PI) - gyroBiasX;
  float ratePitch = (g.gyro.y * 180.0 / PI) - gyroBiasY;

  kalman1D(rawRoll, rateRoll, dt, kalmanRoll, uncertaintyRoll);
  kalman1D(rawPitch, ratePitch, dt, kalmanPitch, uncertaintyPitch);

  // ===== Display Data on Small Screen =====
  display.clearDisplay();
  
  // First row: Title
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Kalman Filter");
  
  // Second row: Roll
  display.setCursor(0, 12);
  display.print("R: ");
  display.print(kalmanRoll, 1); // One decimal place
  display.print((char)247);     // Degree symbol
  
  // Third row: Pitch
  display.setCursor(0, 24);
  display.print("P: ");
  display.print(kalmanPitch, 1);
  display.print((char)247);

  display.display(); 
  delay(50); // Slower update to avoid flickering
}
