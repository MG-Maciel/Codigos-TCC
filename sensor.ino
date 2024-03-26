#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

/* -----------------------------------------------------*/

#define MPU_ADDR 0x68
#define STOP_OFFSET 300

/* -----------------------------------------------------*/

uint8_t mpu6050_init();
void mpu6050_offset(int16_t cal_ac_x, int16_t cal_ac_y, int16_t cal_ac_z);
void mpu6050_write(uint8_t address, int16_t val, uint8_t size);
int16_t mpu6050_read(uint8_t address, uint8_t size);

unsigned long amostra = 0;
unsigned long parado = 0;
float distancia = 0;

/* -------------------- VOID, LOOP, ETC.---------------------------------*/

void setup() {
  Serial.begin(9600);
  uint8_t mpu6050_init();
  amostra = millis();

}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);
}
/* -----------------------------------------------------*/

float calculo(float dist, float acel, float vel, unsigned long tempo){
    static float last_acel = 0.0;
    static float last_vel = 0.0;
    float t = (float)tempo/1000000.0;

   if(dist == 0.0){
      last_vel = 0.0;
     last_acel = 0.0;
  }

//De onde vem esse 'tempo'?

  vel = last_vel + (last_acel + acel) * t / 2.0;
  dist = dist + (last_vel + vel) * t / 2.0;

  last_acel = acel;
  last_vel = vel;

  return distancia;
}

uint8_t mpu6050_init(){
  Wire.begin();
  uint8_t id = mpu6050_read(0x75, 1);

  mpu6050_write(0x6B, 4, 1);

  return 1;
}

void mpu6050_offset(int16_t acalcx, int16_t acalcy, int16_t acalcz){
  mpu6050_write(0x06, acalcx, 2);
  mpu6050_write(0x08, acalcy, 2);
  mpu6050_write(0x0A, acalcz, 2);
}

void mpu6050_write(uint8_t address, int16_t val, uint8_t size){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ADDR);
  Wire.write(address);

  if(size == 2){
    Wire.write(val >> 8);
  }

  Wire.write(val & 0xFF);
  Wire.endTransmission();
}

int16_t mpu6050_read(uint8_t address, uint8_t size){
  int16_t data = 0;

  Wire.beginTransmission(MPU_ADDR);
  
  Wire.write(address);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (float) size);

  if(size == 2){
    data = Wire.read() << 8;
  }

  data |= Wire.read();

  return data;
}