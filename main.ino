#include <SCServo.h>

const int PIN_RX = 16;
const int PIN_TX = 17;
const long BAUD_SERVOS = 1000000;
SMS_STS servos;

//Posición de cad articulación en todos los arrays
const int ABD = 0, FEM = 1, MAN = 2; //abducción, fémur, manivela
uint8_t ID[3] = {1, 2, 3};

const float Lf = 120; //fémur: H -> R
const float Lt = 240; //tibia: R -> P
const float Le = 75; // tibia: R -> E
const float ALFA = 21.011 * DEG_TO_RAD; //ángulo de la tibia entre RP y RE
const float Lm = 55;  //manivela: M -> B
const float Lb = 175; //biela: B -> E
const float S = 35; //separación entre H y M

const float TH_MONTAKE[3] = {0, 90, 0}; // grados en la postura de montaje
const float P_REPOSO[3] = {80, 280, 0}; //reposo: la foto en la portada
const int SENTIDO[3] = {1, -1, 1}; //el fémur va con el eje hacia dentro
const int AJUSTE[3] = {0, 0, 0}; //corrección fina en pasos
const int POS_MIN[3] = {1024, 1024, 1024}; //topes en pasos
const int POS_MAX[3] = {3072, 3072, 3072};

const int VELOCIDAD = 1000; //pasos/s
const int ACEL = 50;
const float PASOS_POR_RAD = 4096 /(2*PI);

float thRef[3];
bool informar = true;

void enviarServos(s16 pos[3]) {
    u16 vel[3] = {VELOCIDAD, VELOCIDAD, VELOCIDAD};
    u8 acc[3] = {ACEL, ACEL, ACEL};
    servos.SyncWritePosEx(ID, 3, pos, vel, acc);
}

void modoMontaje() {
    s16 centro[3] = {2048, 2048, 2048};
    enviarServos(centro);
    Serial.println("Servos en 2048");
}

void leerPosiciones() {
    for (int i = 0; i < 3; i++) {
        int pos = servos.ReadPos(ID[i]); // -1 si no responde
        Serial.printf(" servo %d: %d\n", ID[i], pos);
    }
}

bool calcularIK(float x, float y, float z, float th[3]) {
    //Paso 1: Abducción
    th[ABD] = atan2(z, y);
    float h = sqrt(y * y + z * z );

    //Paso 2: triángulo H-R-P
    float dP = sqrt(x * x + h * h);
    if (dP > Lf + Lt || dP < fabs(Lf - Lt)) return false;
    th[FEM] = atan2(h, x) + acos((Lf * Lf + dP * dP - Lt * Lt) / (2 * Lf * dP));

    //Paso 3: Triángulo M-B-E
    float Rx = Lf * cos(th[FEM]), Ry = Lf * sin(th[FEM]);
    float dir = atan2(h - Ry, x - Rx) + ALFA;
    float Ex = Rx + Le * cos(dir) - S, Ey = Ry + Le * sin(dir);
    float dE = sqrt(Ex * Ex + Ey * Ey);
    float k = (Lm * Lm + dE * dE - Lb * Lb) / (2 * Lm * dE);
    if (k < -1 || k > 1) return false;
    th[MAN] = atan2(Ey,Ex) - acos(k);
    return true;
}

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
            Serial.println(pos[i]);
            Serial.printf("Tope superado en el servo %d\n", ID[i]);
            return false;
        }
    }

    enviarServos(pos);
    if (informar) {
        for (int i = 0; i < 3; i++) {
            Serial.printf("  servo %d = %.1f grafos -> %d\n", ID[i], degrees(th[i]), pos[i]);
        }
    }
    return true;
}

void reposo() {
    Serial.println("Reposo");
    moverPie(P_REPOSO[0], P_REPOSO[1], P_REPOSO[2]);
}
// Provisionales: en la práctica 2 se cambian
void setup() {
    Serial.begin(115200);
    Serial2.begin(BAUD_SERVOS, SERIAL_8N1, PIN_RX, PIN_TX);
    servos.pSerial = &Serial2;

    for (int i = 0; i < 3; i++) thRef[i] = TH_MONTAKE[i] * DEG_TO_RAD;
    delay(500);
    Serial.println("Listo. Escribe x y z (mm), r, c, m o p");
}

void loop() {
    if(!Serial.available()) return; // nada escrito, no hacer nada

    char letra = Serial.peek(); //mirar el primer carácter escrito sin sacarlo
    if (isAlpha(letra)) {
        while (Serial.available()) Serial.read();
        if (letra == 'r') reposo();
        //else if (letra == 'c') cuadrado();
        else if (letra =='m') modoMontaje();
        else if (letra == 'p') leerPosiciones();
        else Serial.println("Orden desconocida");
        return;
    }
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    Serial.printf("Pie a (%.1f, %.1f, %.1f)\n", x, y, z);
    moverPie(x, y ,z);
}
