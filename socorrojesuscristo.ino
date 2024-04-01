#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MPU6050.h>

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
bool t_parado = true;

/* -------------------- VOID --------------------*/

void setup() {
  Serial.begin(9600);
  uint8_t mpu6050_init();
  amostra = millis();
  mpu6050_offset(-2267, -1111, 504); //calibração do sensor
  
}

/* -------------------- LOOP --------------------*/

void loop() {

  sensors_event_t event;
  accel.getEvent(&event);
  int16_t ac_y = mpu6050_read(0x3D, 2);
  float aux_ac = float(ac_y);


  if(aux_ac < STOP_OFFSET && millis() - parado > 50){
    parado = true;
    amostra = micros();
  }

  else if((aux_ac) >= STOP_OFFSET)
  {
    parado = millis();
    parado = false;
  }


  //Conversão do valor de aceleração
  aux_ac = ((aux_ac + 32768.0) * 4.0/65536.0 - 2.0) * 9.81;

 
  // ----- Calculo da distancia deslocada -----
  //Se está parado
  if (parado)
  {
    if(distancia != 0.0)
    {
      Serial.println("Distância deslocada (em y): ");
      // A função retorna o valor em metros,
      // então multiplica por 100 para converter para centímetros
      Serial.print(distancia*100, 2);
      Serial.println(" cms em y.");
    }
    distancia = 0.0;
  }
  // Está se movendo
  else
  {
    // Calcula o tempo percorrido
    amostra = millis() - amostra;
 
    //distancia = calculo(distancia, aux_ac, amostra);
   
    amostra = millis();
  }
}


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


/* ---------------------- COMUNICAÇÃO MPU6050 ------------------------------*/


uint8_t mpu6050_init(){
  Wire.begin();
  uint8_t id = mpu6050_read(0x75, 1);

  mpu6050_write(0x6B, 4, 1);

  return 1;
}


void mpu6050_offset(int16_t acalcx, int16_t acalcy, int16_t acalcz){
  mpu6050_write(0x06, acalcx, 2); //eixo X
  mpu6050_write(0x08, acalcy, 2); //eixo Y
  mpu6050_write(0x0A, acalcz, 2); //eixo Z
}

void mpu6050_write(uint8_t address, int16_t val, uint8_t size){
  Wire.beginTransmission(MPU_ADDR);
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