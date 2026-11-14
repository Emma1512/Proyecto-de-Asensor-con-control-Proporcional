// Puente H
const int VELOCIDAD = 9;
const int IN1 = 7;
const int IN2 = 8;
// Sensor
const uint8_t TRIG = 12;
const uint8_t ECHO = 3;
// Botones
const int PISO1 = 4;
const int PISO2 = 6;
const int BTN_EMERG = 13;

// Período de muestreo (el HC-SR04 pide ~60 ms entre mediciones)
const unsigned long PERIODO_MS = 100;

// ---- Parámetros del control proporcional ----
// Modo de control:
//   false -> P clásico:     PWM = Kp * error              (se recorta entre mín y máx)
//   true  -> P con offset:  PWM = PWM_MIN + Kp * (|error| - zona muerta)
const bool USAR_OFFSET = true;

// Kp para P clásico (PWM por cm)
const float KP_SUBIR = 18.0;
const float KP_BAJAR = 12.0;

// Kp para P con offset (PWM por cm). Cubre todo el recorrido sin saturar:
//   Kp = (PWM_MAX - PWM_MIN) / (error_máximo - ZONA_MUERTA)
const float KP_SUBIR_OFS = 1.25;   // (210-185)/(21-1)
const float KP_BAJAR_OFS = 2.25;   // (185-140)/(21-1)

// PWM distinto según el sentido (subir vs. bajar)
const int PWM_MAX_SUBIR = 210;
const int PWM_MIN_SUBIR = 180;
const int PWM_MAX_BAJAR = 165;
const int PWM_MIN_BAJAR = 125;
const float ZONA_MUERTA = 1.5;     // [cm] si |error| <= esto, se considera que llegó

// Variables globales
const float tope = 27.0;
const float topeMinimo = 4.0;
int pisoDeseado = 0;              // 0 = todavía no se eligió piso (no se mueve)
bool emergencia = false;
unsigned long tMuestra = 0;       // instante programado de la próxima muestra

enum TipoEstado{
  Detenido,
  Subiendo,
  Bajando,
  EMERGENCIA
};
// Estado del ascensor
TipoEstado estado = Detenido;

const char* nombreEstado(TipoEstado e){
  switch(e){
    case Detenido: return "Detenido";
    case Subiendo: return "Subiendo";
    case Bajando: return "Bajando";
    case EMERGENCIA: return "EMERGENCIA";
    default: return "Desconocido";
  }
}

// Medicion de distancia con el Sensor (mide siempre que se la llama;
// el tiempo entre muestras lo garantiza el loop)
float medicion(){
  digitalWrite(TRIG, HIGH);
  unsigned long t0 = micros();
  while(micros() - t0 < 10){}
  digitalWrite(TRIG, LOW);

  unsigned long duracion = pulseIn(ECHO, HIGH, 3500UL);
  if(duracion == 0){ return -1; }   // error: no hubo eco
  return duracion / 58.3;
}

void parar(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(VELOCIDAD, 0);
  estado = Detenido;
}

// Ahora el PWM lo decide el controlador
void subir(int pwm){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(VELOCIDAD, pwm);
  estado = Subiendo;
}

void bajar(int pwm){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(VELOCIDAD, pwm);
  estado = Bajando;
}

float objetivo(int piso){
  if(piso == 1){
    return 4.5;
  }else{
    return 26.5;
  }
}

// Se ejecuta UNA vez por período de muestreo.
// Orden: A = medir, B = decidir acción de control, C = informar.
void irAPiso(int piso){
  unsigned long t = millis();       // instante de la muestra
  float u = 0;                      // acción de control sin saturar (con signo)
  int pwmAplicado = 0;              // PWM que realmente recibe el motor (con signo)

  // Paso A: medir
  float h = medicion();

  // Paso B: decidir la acción de control
  if(emergencia){
    parar();
    estado = EMERGENCIA;
  }
  else if(piso == 0 || h < 0){
    // sin piso elegido o medición inválida: no se cambia la acción de control
  }
  else{
    float error = objetivo(piso) - h;

    if(abs(error) <= ZONA_MUERTA){
      parar();                      // llegó al piso
      u = 0;
    }
    else{
      float exceso = abs(error) - ZONA_MUERTA;   // error "útil", sin la zona muerta

      if(error > 0){
        int pwm;
        if(USAR_OFFSET){
          u = KP_SUBIR_OFS * exceso;             // parte proporcional sobre el mínimo
          pwm = PWM_MIN_SUBIR + (int)u;
        } else {
          u = KP_SUBIR * error;
          pwm = (int)u;
        }
        pwm = constrain(pwm, PWM_MIN_SUBIR, PWM_MAX_SUBIR);
        subir(pwm);
        pwmAplicado = pwm;
      }
      else{
        int pwm;
        if(USAR_OFFSET){
          u = -KP_BAJAR_OFS * exceso;            // u < 0 al bajar
          pwm = PWM_MIN_BAJAR + (int)abs(u);
        } else {
          u = KP_BAJAR * error;                  // u < 0 al bajar
          pwm = (int)abs(u);
        }
        pwm = constrain(pwm, PWM_MIN_BAJAR, PWM_MAX_BAJAR);
        bajar(pwm);
        pwmAplicado = -pwm;
      }
    }
  }

  // Paso C: informar (tiempo, altura, piso deseado, acción de control, estado)
  Serial.print("Tiempo: ");
  Serial.print(t);
  Serial.print('\t');
  Serial.print("Altura: ");
  Serial.print(h);                  // -1 si hubo error de medición
  Serial.print('\t');
  Serial.print("Piso: ");
  Serial.print(piso);
  Serial.print('\t');
  Serial.print("u: ");
  Serial.print(u);
  Serial.print('\t');
  Serial.print("PWM: ");
  Serial.print(pwmAplicado);
  Serial.print('\t');
  Serial.print("Estado: ");
  Serial.println(nombreEstado(estado));
}

void setup() {
  Serial.begin(115200);
  // Puente H
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(VELOCIDAD, OUTPUT);
  // Sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  // Botones
  pinMode(PISO1, INPUT_PULLUP);
  pinMode(PISO2, INPUT_PULLUP);
  pinMode(BTN_EMERG, INPUT_PULLUP);

  tMuestra = millis();
}

void loop() {
  // Botones: se leen en cada vuelta para no perder pulsaciones
  if(digitalRead(PISO1) == LOW){ pisoDeseado = 1; }
  if(digitalRead(PISO2) == LOW){ pisoDeseado = 2; }
  if(digitalRead(BTN_EMERG) == LOW){
    emergencia = true;
    parar();                        // frena enseguida, sin esperar a la próxima muestra
    estado = EMERGENCIA;
  }

  // Muestreo a período fijo: medir / decidir / informar solo cada PERIODO_MS
  if(millis() - tMuestra >= PERIODO_MS){
    tMuestra += PERIODO_MS;
    irAPiso(pisoDeseado);
  }
}
