#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Listo");// inicialización puerto serie
  
  if (!mpu.begin()) { // inicialización mpu
  Serial.println("MPU falló");
  while (1)
    yield();
  }
  Serial.println("MPU encontrado");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);// resolución del acelerometro 2g
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // resolución del giroscopio 250 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);// filtro pasa bajas 5Hz delay de lectura 20ms
  
}
float thetaZf, thetaZi=0.0, degThetaZ;
unsigned long int tiempoAnt=0;
int del = 30;//ms
void loop() {
  // put your main code here, to run repeatedly:
  if(millis()-tiempoAnt >= del){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    thetaZf = thetaZi + (g.gyro.z * (del/1000.0));//poceso
    degThetaZ = thetaZf * (180.0/3.14159);
    Serial.println(degThetaZ);
    tiempoAnt = millis();//reset del proceso
    thetaZi = thetaZf;
    }
}
