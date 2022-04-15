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
float radD = 0.0;
float radFilterD = radD;
float alphaD = 0.04;

float radI = 0.0;
float radFilterI = radI;
float alphaI = 0.04;

//

//velocidades en rad
float velRad(int);


//controlador de bajo nivel
struct PID{
  float P;
  float I;
  float D;
  };
PID pidRight = {2, 0.0, 0.0};
int controlRight(float);
int entradaD = 320;
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

  ledcWrite(PWM0, 0);
  ledcWrite(PWM1, entradaD);
}


int cont = 0;

float errorRight = 0.0, errorLeft = 0.0;
void loop() {

  

  if(banderaTimer){// si la bandera está activa se ejecuta la orden del timer
    if(contTiempo == tiempoMuestreo ){
      radD = velRad(encD.CONT);
      radFilterD = alphaD*radD + (1.0-alphaD)*radFilterD;
      
      radI = velRad(encI.CONT);
      radFilterI = alphaI*radI + (1.0-alphaI)*radFilterI;

      errorRight = entradaD - radFilterD;
      Serial.print(radFilterD);
      Serial.print(",");
      Serial.print(errorRight);
      Serial.print(",");
      Serial.println(controlRight(entradaD));
      ledcWrite(PWM1, controlRight(entradaD));
      encD.CONT = 0;
      encI.CONT = 0;
      contTiempo = 0;
    }
    banderaTimer = false;// Se desactiva la bandera para evitar el rebote. La bandera solo la activa el timer0
    }
    
}

float velRad(int rev){
  float t = tiempoMuestreo/1000.0;
  float rad_s = ((rev/20.0)/t)*2*PI;
  return rad_s;
  }

//controldor motor derecho

float tm = tiempoMuestreo/1000.0;
float T0d = pidRight.P + (pidRight.D/tm) + ((pidRight.I*tm)/2);
float T1d = -pidRight.P - (((2*pidRight.D)/tm)) + ((pidRight.I*tm)/2);
float T2d = ((pidRight.D)/tm);
float Ukd_1 = 0.0; //señales de control
float Ekd_1 = 0.0, Ekd_2 = 0.0; // errores;

int controlRight(float e){
  float UK;
  UK = T0d*e + T1d*Ekd_1 + T2d*Ekd_2 + Ukd_1;
  Ekd_2 = Ekd_1;
  Ekd_1 = e;
  Ukd_1 = UK;

  return round(UK);  

  }
