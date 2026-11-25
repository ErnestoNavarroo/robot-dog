// =====================================================================
//  Pata 3-DOF con mecanismo de cinco barras · Día 1 · COORYC Robotics
//  ESP32 DevKit V1 -> placa Feetech FE-URT-1 (lado TTL) -> 3 servos STS3125
//  Librería: SCServo (clase SMS_STS)
//
//  Uso: abrir el monitor serie a 115200 baudios y escribir "x y z" en mm
//  (por ejemplo: 80 280 0). El pie se mueve a ese punto si es alcanzable.
//  Otras órdenes: r reposo, m montaje (servos en 2048), p leer posiciones.
// =====================================================================
#include <SCServo.h>

// ---------------------------------------------------------------------
// Conexión
//   ESP32 GPIO17 (TX2) ->  RX del conector TTL de la URT-1
//   ESP32 GPIO16 (RX2) <-  TX del conector TTL de la URT-1 (ver nota)
//   ESP32 GND          ->  GND de la URT-1 (masa común con la fuente)
//   Fuente 7,4 V       ->  entrada de alimentación de servos de la URT-1
// El USB del ESP32 (Serial) queda libre para el monitor serie y los
// servos van por Serial2, un puerto hardware propio.
// Nota: el ESP32 trabaja a 3,3 V. Medir a qué tensión sale el TX de la
// URT-1: si son 5 V, poner un divisor (1 kΩ en serie y 2 kΩ a GND).
// Los servos llevan IDs 1, 2 y 3 (se ponen una vez con el programa FD
// de Feetech) y el bus a 1 Mbps, la velocidad de fábrica.
// ---------------------------------------------------------------------
const int  PIN_RX = 16;
const int  PIN_TX = 17;
const long BAUD_SERVOS = 1000000;
SMS_STS servos;

// Posición de cada articulación en todos los arrays
const int ABD = 0, FEM = 1, MAN = 2;  // abducción, fémur, manivela
uint8_t ID[3] = {1, 2, 3};

// ---------------------------------------------------------------------
// Geometría en mm: medir en el CAD 
// Puntos: H hombro (origen), M eje de la manivela, B manivela-biela,
//         R rodilla, E enganche de la biela en la tibia, P pie.
// Ejes: x hacia delante, y hacia abajo, z hacia el lado.
// ---------------------------------------------------------------------
const float Lf   = 120;              // fémur: H -> R
const float Lt   = 240;              // tibia: R -> P
const float Le   = 75;               // tibia: R -> E
const float ALFA = 21.011 * DEG_TO_RAD;  // ángulo de la tibia entre RP y RE
const float Lm   = 55;               // manivela: M -> B
const float Lb   = 175;              // biela: B -> E
const float S    = 35;               // separación entre H y M

// ---------------------------------------------------------------------
// Calibración: con cada servo en 2048 la pata está en la postura de
// montaje (abducción vertical, fémur abajo, manivela adelante).
// ---------------------------------------------------------------------
const float TH_MONTAJE[3] = {0, 90, 0};     // grados en la postura de montaje
const float P_REPOSO[3]   = {80, 280, 0};   // reposo: la foto de la portada
const int   SENTIDO[3]    = {1, -1, 1};     // el fémur va con el eje hacia dentro
const int   AJUSTE[3]     = {0, 0, 0};      // corrección fina en pasos
const int   POS_MIN[3]    = {1024, 1024, 1024};  // topes en pasos
const int   POS_MAX[3]    = {3072, 3072, 3072};

const int   VELOCIDAD = 1000;  // pasos/s
const int   ACEL      = 50;
const float PASOS_POR_RAD = 4096 / (2 * PI);

// Cuadrado (práctica 3, aún sin usar): lado de 130 mm
const float CUADRADO_X0   = 80;   // una esquina es el reposo
const float CUADRADO_Y0   = 280;
const float CUADRADO_LADO = 130;
const float PASO_MM       = 2;    // el pie avanza 2 mm...
const int   PAUSA_MS      = 40;   // ...cada 40 ms: unos 50 mm/s

float thRef[3];        // TH_MONTAJE en radianes
bool informar = true;  // false: moverPie no escribe nada

// ---------------------------------------------------------------------
// Cinemática inversa: los tres pasos de las diapositivas
// ---------------------------------------------------------------------
bool calcularIK(float x, float y, float z, float th[3]) {
  // Paso 1: abducción
  th[ABD] = atan2(z, y);
  float h = sqrt(y * y + z * z);

  // Paso 2: triángulo H-R-P
  float dP = sqrt(x * x + h * h);
  if (dP > Lf + Lt || dP < fabs(Lf - Lt)) return false;
  th[FEM] = atan2(h, x) + acos((Lf * Lf + dP * dP - Lt * Lt) / (2 * Lf * dP));

  // Paso 3: triángulo M-B-E
  float Rx = Lf * cos(th[FEM]), Ry = Lf * sin(th[FEM]);
  float dir = atan2(h - Ry, x - Rx) + ALFA;
  float Ex = Rx + Le * cos(dir) - S, Ey = Ry + Le * sin(dir);  // E medido desde M
  float dE = sqrt(Ex * Ex + Ey * Ey);
  float k  = (Lm * Lm + dE * dE - Lb * Lb) / (2 * Lm * dE);
  if (k < -1 || k > 1) return false;
  th[MAN] = atan2(Ey, Ex) - acos(k);
  return true;
}

// ---------------------------------------------------------------------
// Enviar los tres servos a la vez (una sola trama)
// ---------------------------------------------------------------------
void enviarServos(s16 pos[3]) {
  u16 vel[3] = {VELOCIDAD, VELOCIDAD, VELOCIDAD};
  u8  acc[3] = {ACEL, ACEL, ACEL};
  servos.SyncWritePosEx(ID, 3, pos, vel, acc);
}

// ---------------------------------------------------------------------
// Mover el pie: pensar (IK) -> comprobar (topes) -> enviar
// Si algo falla no se mueve ningún servo.
// ---------------------------------------------------------------------
bool moverPie(float x, float y, float z) {
  float th[3];
  if (!calcularIK(x, y, z, th)) {
    Serial.println("Fuera de alcance");
    return false;
  }
  s16 pos[3];
  for (int i = 0; i < 3; i++) {
    pos[i] = 2048 + AJUSTE[i] + SENTIDO[i] * (th[i] - thRef[i]) * PASOS_POR_RAD;
    if (pos[i] < POS_MIN[i] || pos[i] > POS_MAX[i]) {
      Serial.printf("Tope superado en el servo %d\n", ID[i]);
      return false;
    }
  }
  enviarServos(pos);
  if (informar) {
    for (int i = 0; i < 3; i++) {
      Serial.printf("  servo %d = %.1f grados -> %d\n", ID[i], degrees(th[i]), pos[i]);
    }
  }
  return true;
}

// ---------------------------------------------------------------------
// Órdenes de una letra
// ---------------------------------------------------------------------
void reposo() {
  Serial.println("Reposo");
  moverPie(P_REPOSO[0], P_REPOSO[1], P_REPOSO[2]);
}

void modoMontaje() {
  s16 centro[3] = {2048, 2048, 2048};
  enviarServos(centro);
  Serial.println("Servos en 2048: abduccion vertical, femur abajo, manivela adelante");
}

void leerPosiciones() {
  for (int i = 0; i < 3; i++) {
    int pos = servos.ReadPos(ID[i]);  // -1 si el servo no responde
    Serial.printf("  servo %d: %d\n", ID[i], pos);
  }
}

bool lineaRecta(float x0, float y0, float x1, float y1) {
int pasos = ceil(max(fabs(x1 - x0), fabs(y1 - y0)) / PASO_MM);
for (int i = 1; i <= pasos; i++) {
float t = (float)i / pasos; // de 0 a 1
if (!moverPie(x0 + t * (x1 - x0), y0 + t * (y1 - y0), 0)) return false;
delay(PAUSA_MS);
}
return true;
}


void cuadrado() {
float x0 = CUADRADO_X0, x1 = CUADRADO_X0 + CUADRADO_LADO;
float y0 = CUADRADO_Y0, y1 = CUADRADO_Y0 - CUADRADO_LADO;
float esq[5][2] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
Serial.println("Cuadrado");
reposo();
delay(1000);
informar = false;
for (int i = 0; i < 4; i++) {
if (!lineaRecta(esq[i][0], esq[i][1], esq[i + 1][0], esq[i + 1][1])) {
Serial.println("Cuadrado interrumpido");
break;
}
}
informar = true;
Serial.println("Fin del cuadrado");

}

void idaYVuelta() {
float x0 = CUADRADO_X0, x1 = CUADRADO_X0 + CUADRADO_LADO;
float y0 = CUADRADO_Y0; // la línea del reposo
Serial.println("Ida y vuelta");
reposo();
delay(1000);
informar = false;
bool ok = lineaRecta(x0, y0, x1, y0) && lineaRecta(x1, y0, x0, y0);
informar = true;
Serial.println(ok ? "Fin de la ida y vuelta" : "Ida y vuelta interrumpida");

}
  

// ---------------------------------------------------------------------
// setup: una vez al encender. loop: se repite sin parar.
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                                    // USB: monitor serie
  Serial2.begin(BAUD_SERVOS, SERIAL_8N1, PIN_RX, PIN_TX);  // bus de servos
  servos.pSerial = &Serial2;

  for (int i = 0; i < 3; i++) thRef[i] = TH_MONTAJE[i] * DEG_TO_RAD;

  delay(500);
  Serial.println("Listo. Escribe: x y z (mm), r, m o p");
  reposo();
}

void loop() {
  if (!Serial.available()) return;  // nada escrito: no hacer nada

  char letra = Serial.peek();        // mirar el primer carácter sin sacarlo
  if (isAlpha(letra)) {
    while (Serial.available()) Serial.read();  // vaciar la línea
    if      (letra == 'r') reposo();
    else if (letra == 'm') modoMontaje();
    else if (letra == 'p') leerPosiciones();
    else if (letra == 'i') lineaRecta(80,280,210,190);
    else if (letra == 'c') cuadrado();
    else Serial.println("Orden desconocida");
    return;
  }

  float x = Serial.parseFloat();     // leer tres números
  float y = Serial.parseFloat();
  float z = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  Serial.printf("Pie a (%.1f, %.1f, %.1f)\n", x, y, z);
  moverPie(x, y, z);
}
