 const int pin1MD = 13;
const int pin2MD = 12;
const int pin1MI = 5;
const int pin2MI = 18;// pines de los motores
const int tiempoMuestreo = 10; //10ms
// caracteristicas PWM 
#define PWM0 2
#define PWM1 3
#define PWM2 4
#define PWM3 5
#define PWM_res 12
#define PWM_freq 1000

// encoders
struct Encoders{
  const int PIN; 
  int CONT;
  int REV;
  };
  
Encoders encD = {35, 0, 0};//Rueda derecha
Encoders encI = {34, 0, 0};//Rueda Izquierda

void IRAM_ATTR isrEncD(){// funcion del encoder Derecho
  encD.CONT += 1;
  }

void IRAM_ATTR isrEncI(){// funcion del encoder Izquiedo
  encI.CONT += 1;
  }

// fin encoders

// Filtros:
float radD = 0.0;
float radFilterD = radD;
float alphaD = 0.01;

float radI = 0.0;
float radFilterI = radI;
float alphaI = 0.01;

//

// inicio Timer

hw_timer_t * timer = NULL;

float RPM_D, RPM_I;
volatile bool banderaTimer = false;

void IRAM_ATTR onTimer(){
  banderaTimer = true;
}

// Fin timer

//velocidades en rad
float velRad(int);


//controlador de bajo nivel
struct PID{
  float P;
  float I;
  float D;
  };
PID pidRight = {3000.0, 5.0, 100.0};//50 1250
PID pidLeft = {0.0, 0.0, 0.0};
int controlRight(float);
int controlLeft(float);
float entradaD = 0.0, entradaI = 0.0;
void setup() {
  Serial.begin(115200);// Comunicacion serial
  pinMode(pin1MD, OUTPUT);
  pinMode(pin2MD, OUTPUT);
  pinMode(pin1MI, OUTPUT);
  pinMode(pin2MI, OUTPUT);//Pines de salida de los motores

  ledcAttachPin(pin1MD, PWM0);
  ledcAttachPin(pin2MD, PWM1);
  ledcAttachPin(pin1MI, PWM2);
  ledcAttachPin(pin2MI, PWM3);

  ledcSetup(PWM0, PWM_res, PWM_freq);
  ledcSetup(PWM1, PWM_res, PWM_freq);
  ledcSetup(PWM2, PWM_res, PWM_freq);
  ledcSetup(PWM3, PWM_res, PWM_freq);// PWM para los motores

  pinMode(encD.PIN, INPUT_PULLUP);// Pin 32 como entrada
  pinMode(encI.PIN, INPUT_PULLUP);// Pin 21 como entrada

  attachInterrupt(encD.PIN, isrEncD, RISING);// enlace con la funcion de interrupccion
  attachInterrupt(encI.PIN, isrEncI, RISING);// enlace con la funcion de interrupccion

  // configuracion del timer 0 a 200ms
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);//timer a 10ms
  timerAlarmEnable(timer);

  ledcWrite(PWM2, 0);
}
 

int contMuestras = 0;

float errorRight = 0.0, errorLeft = 0.0;
void loop() {
  if(banderaTimer){// si la bandera está activa se ejecuta la orden del timer
    radD = velRad(encD.CONT);
    radFilterD = alphaD*radD + (1.0-alphaD)*radFilterD;
    
    radI = velRad(encI.CONT);
    radFilterI = alphaI*radI + (1.0-alphaI)*radFilterI;
    errorRight = abs(entradaD) - radFilterD;
    errorLeft = abs(entradaI) - radFilterI;
    
    Serial.print(radI);
    Serial.print(",");
    Serial.print(radFilterI);
    Serial.print(",");
    Serial.println(0);
    
    if(entradaD > 0){
      ledcWrite(PWM0, controlRight(errorRight));
      ledcWrite(PWM1, 0);
      }else{
        ledcWrite(PWM0, 0);
        ledcWrite(PWM1, controlRight(errorRight));
        }
        
//    if(entradaI > 0){
//      ledcWrite(PWM2, controlLeft(errorLeft));
//      ledcWrite(PWM3, 0);
//      }else{
//        ledcWrite(PWM2, 0);
//        ledcWrite(PWM3, controlLeft(errorLeft));
//        }
    
    encD.CONT = 0;
    encI.CONT = 0;
    banderaTimer = false;// Se desactiva la bandera para evitar el rebote. La bandera solo la activa el timer0
    }
}

float velRad(int rev){
  float t = tiempoMuestreo/1000.0;
  float rad_s = ((rev/20.0)/t);//*2*PI
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
  if(abs(entradaD) > 0){
    if(UK >= 400 && UK <= 4095){
    return round(UK);
    }else if(UK < 400){
      return 400;
      }else{
        return 4095;
        }
    }else{
      return 0;
      }
  }

float T0i = pidLeft.P + (pidLeft.D/tm) + ((pidLeft.I*tm)/2);
float T1i = -pidLeft.P - (((2*pidLeft.D)/tm)) + ((pidLeft.I*tm)/2);
float T2i = ((pidLeft.D)/tm);
float Uki_1 = 0.0; //señales de control
float Eki_1 = 0.0, Eki_2 = 0.0; // errores;

int controlLeft(float e){
  float UK;
  UK = T0i*e + T1i*Eki_1 + T2i*Eki_2 + Uki_1;
  Eki_2 = Eki_1;
  Eki_1 = e;
  Uki_1 = UK;
  if(abs(entradaI) > 0){
    if(UK >= 400 && UK <= 4095){
    return round(UK);
    }else if(UK < 400){
      return 400;
      }else{
        return 4095;
        }
    }else{
      return 0;
      }
  }
