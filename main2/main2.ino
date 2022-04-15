const int pin1MD = 13;
const int pin2MD = 14;
const int pin1MI = 2;
const int pin2MI = 4;// pines de los motores
const int tiempoMuestreo = 100; //100ms
// caracteristicas PWM 
#define PWM0 2
#define PWM1 3
#define PWM2 4
#define PWM3 5
#define PWM_res 10
#define PWM_freq 1000

// encoders
struct Encoders{
  const int PIN; 
  int CONT;
  int REV;
  };
  
Encoders encD = {35, 0, 0};//Rueda derecha
Encoders encI = {21, 0, 0};//Rueda Izquierda

void IRAM_ATTR isrEncD(){// funcion del encoder Derecho
  encD.CONT += 1;
//  if(encD.CONT == 20){
//    encD.REV++;
//    encD.CONT = 0; 
//    }
  }

void IRAM_ATTR isrEncI(){// funcion del encoder Izquiedo
  encI.CONT += 1;
//  if(encI.CONT == 20){
//    encI.REV++;
//    encI.CONT = 0; 
//    }
  }

// fin encoders
// inicio Timer

hw_timer_t * timer = NULL;

float RPM_D, RPM_I;
volatile int contTiempo = 0;
volatile bool banderaTimer = false;

void IRAM_ATTR onTimer(){
  if(contTiempo >= 0 && contTiempo <= tiempoMuestreo){
    contTiempo++;
    banderaTimer = true;
    }
}

// Fin timer

// Filtros:
float KDerecho[61]={0.000429533101548133,0.000488471044816538,0.000584472567929729,0.000725789504505098,0.000920508762129306,0.00117625991359895,0.00149991662498511,0.0018973020523189,0.00237290864268536,0.00292964270765806,0.00356860368890976,0.00428890721340366,0.00508755985908534,0.00595939205561938,0.00689705377543647,0.00789107568642941,0.00892999630653132,0.0100005534963966,0.0110879364278175,0.0121760920518237,0.0132480781392375,0.014286453250569,0.0152736925766565,0.016192617531295,0.0170268263147843,0.0177611124312723,0.018381858346064,0.0188773921088957,0.0192382958267399,0.019457656310931,0.01953125,0.019457656310931,0.0192382958267399,0.0188773921088957,0.018381858346064,0.0177611124312723,0.0170268263147843,0.016192617531295,0.0152736925766565,0.014286453250569,0.0132480781392375,0.0121760920518237,0.0110879364278175,0.0100005534963966,0.00892999630653132,0.00789107568642941,0.00689705377543647,0.00595939205561938,0.00508755985908534,0.00428890721340366,0.00356860368890976,0.00292964270765806,0.00237290864268536,0.0018973020523189,0.00149991662498511,0.00117625991359895,0.000920508762129305,0.000725789504505098,0.000584472567929729,0.000488471044816538,0.000429533101548133};
float derecho[61]={};// muestras pasadas del sensor derecho


float filtroDerecho(float);
//

//velocidades en rpm
float velRPM(int);

void setup() {
  Serial.begin(115200);// Comunicacion serial
  pinMode(pin1MD, OUTPUT);
  pinMode(pin2MD, OUTPUT);
  pinMode(pin1MI, OUTPUT);
  pinMode(pin2MI, OUTPUT);//Pines de salida de los motores

  ledcAttachPin(pin1MI, PWM0);
  ledcSetup(PWM0, PWM_res, PWM_freq);

  ledcAttachPin(pin1MD, PWM1);
  ledcSetup(PWM1, PWM_res, PWM_freq);

  ledcAttachPin(pin2MI, PWM2);
  ledcSetup(PWM2, PWM_res, PWM_freq);

  ledcAttachPin(pin2MD, PWM3);
  ledcSetup(PWM3, PWM_res, PWM_freq);// PWM para los motores

  pinMode(encD.PIN, INPUT_PULLUP);// Pin 32 como entrada
  pinMode(encI.PIN, INPUT_PULLUP);// Pin 21 como entrada

  attachInterrupt(encD.PIN, isrEncD, RISING);// enlace con la funcion de interrupccion
  attachInterrupt(encI.PIN, isrEncI, RISING);// enlace con la funcion de interrupccion

  // configuracion del timer 0 a 200ms
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);//timer a 1ms
  timerAlarmEnable(timer);

  ledcWrite(PWM0, 200);
  ledcWrite(PWM1, 200);
}


int cont = 0;
void loop() {
  // put your main code here, to run repeatedly:
//  if(Serial.available() > 0){
//    int valorSerial = Serial.parseInt();
//    ledcWrite(PWM0, valorSerial);
//    ledcWrite(PWM1, valorSerial);
//    }
  if(banderaTimer){// si la bandera está activa se ejecuta la orden del timer
    if(contTiempo == tiempoMuestreo ){
      Serial.print(velRPM(encD.CONT));
      Serial.print(",");
      Serial.println(velRPM(encI.CONT));
      encD.CONT = 0;
      encI.CONT = 0;
      contTiempo = 0;
    }
    banderaTimer = false;// Se desactiva la bandera para evitar el rebote. La bandera solo la activa el timer0
    }
    
}
float velRPM(int rev){
  float t = tiempoMuestreo/1000.0;
  float rpm = ((rev/20.0)/t)*2*3.1416;
  return rpm;
  }
  
float filtroDerecho(float RPM){
  float mulDerecho = 0.0;
  float sumDerecho = 0.0;
  derecho[60] = RPM;
  for(int i=0; i<61; i++){
    mulDerecho = KDerecho[i]*derecho[60-i];
    sumDerecho = sumDerecho + mulDerecho;
    }
  for(int j=60; j>=0; j--){
    if(j-1 >= 0){
      derecho[j-1] = derecho[j];
      }
    }
    return sumDerecho;
  }
