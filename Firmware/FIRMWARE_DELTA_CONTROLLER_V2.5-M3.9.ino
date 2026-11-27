#include <Arduino.h>
#include <FastAccelStepper.h>


HardwareSerial InterfaceSerial(2);

#define UART2_RX 22
#define UART2_TX 23


// ============================================================
// DELTA CONTROLLER V2.5.2-M3.7
// M3.6: PAUSE/RESUME reversível + emergência física GPIO21 + HOME físico GPIO18 + RESET físico GPIO19.
// A parada por ENA+/ENA- permanece independente do firmware.
// ARQUITETURA NÃO BLOQUEANTE + PROTOCOLO M2
// ============================================================
//
// Comandos:
//
// Protocolo M2:
// CMD|ID|COMANDO|PARAMETROS
//
// Comandos estruturados:
// CMD|0001|RESET
// CMD|0002|HOME
// CMD|0003|STOP
// CMD|0004|STATUS
// CMD|0005|MOVEJ|0|0|-600|OFF
// CMD|0006|PAUSE
// CMD|0007|RESUME
//
// Compatibilidade de bancada: comandos antigos também aceitos.
//
// Exemplo:
//
// MOVEJ 0 0 -600 OFF
//
// ============================================================


// M3.9: HOME e RESET físicos removidos. RESET/REARME passa a ser
// supervisório pela HMI. A emergência física permanece obrigatória e
// bloqueia RESET enquanto estiver eletricamente ativa.

// ============================================================
// PINOS
// ============================================================

const int STEP_1 = 33;
const int DIR_1  = 25;

const int STEP_2 = 16;
const int DIR_2  = 17;

const int STEP_3 = 4;
const int DIR_3  = 2;

const int VACUUM_PIN = 26;
const int PUMP_PIN   = 14;

const int LIMIT_X = 34;
const int LIMIT_Y = 35;
const int LIMIT_Z = 32;

// Emergência física oficial: contato NF alimentado por 3V3 e resistor
// externo de 10 kOhm entre GPIO21 e GND.
// Normal = HIGH; emergência/contato aberto/fio interrompido = LOW.
// HOME e RESET físicos removidos na M3.9; operação/rearme pela HMI.
const int EMERGENCY_PIN = 19;//ERA 21, MUDEI PRA 19


// ============================================================
// LIMITES ANGULARES
// ============================================================

const float THETA_MIN = -35.0;
const float THETA_MAX = 75.0;


// ============================================================
// HOMING
// ============================================================

const long HOME_RECOIL_STEPS = 120;

const uint32_t HOMING_SPEED = 100;

const unsigned long HOMING_TIMEOUT = 5000;


// ============================================================
// MOTOR
// ============================================================

const uint32_t MAX_OPERATING_SPEED = 100;

const uint32_t MOTOR_ACCELERATION = 100;


// ============================================================
// CONVERSÃO
// ============================================================

const float STEPS_PER_DEGREE =
    1200.0 / 360.0;


// ============================================================
// ESTADOS
// ============================================================

enum RobotState {

    ROBOT_NOT_HOMED,
    ROBOT_HOMING,
    ROBOT_READY,
    ROBOT_MOVING,
    ROBOT_PAUSED,
    ROBOT_STOPPED,
    ROBOT_EMERGENCY,
    ROBOT_ERROR
};

RobotState robotState =
    ROBOT_NOT_HOMED;


// ============================================================
// PROTOCOLO M2
// ============================================================

const char* getRobotStateName();

extern FastAccelStepper *stepper1;
extern FastAccelStepper *stepper2;
extern FastAccelStepper *stepper3;

const size_t PROTOCOL_RX_BUFFER_SIZE = 192;

char protocolRxBuffer[PROTOCOL_RX_BUFFER_SIZE];
size_t protocolRxIndex = 0;

uint32_t activeCommandId = 0;
String activeCommandName = "";
Stream* activeProtocolStream = &Serial;

// ============================================================
// CONTEXTO DE PAUSA / RESUME
// ============================================================
bool pausedMotionValid = false;
long pausedTarget1 = 0;
long pausedTarget2 = 0;
long pausedTarget3 = 0;
char pausedTool[10] = "OFF";
uint32_t pendingPauseId = 0;
Stream* pendingPauseStream = nullptr;
bool pausePending = false;


void clearPausedMotion() {
    pausedMotionValid = false;
    pausedTarget1 = 0;
    pausedTarget2 = 0;
    pausedTarget3 = 0;
    strncpy(pausedTool, "OFF", sizeof(pausedTool));
    pausedTool[sizeof(pausedTool) - 1] = '\0';
    pendingPauseId = 0;
    pendingPauseStream = nullptr;
    pausePending = false;
}

void clearActiveCommand() {

    activeCommandId = 0;
    activeCommandName = "";
    activeProtocolStream = &Serial;
}


void setActiveCommand(
    uint32_t id,
    const char* command,
    Stream& out
) {

    activeCommandId = id;
    activeCommandName = command;
    activeProtocolStream = &out;
}


void sendAck(Stream& out, uint32_t id) {
    out.printf("ACK|%04lu|RECEIVED\n", (unsigned long)id);
}

void sendOk(Stream& out, uint32_t id, const char* command) {
    out.printf("OK|%04lu|%s\n", (unsigned long)id, command);
}

void sendProtocolError(Stream& out, uint32_t id, const char* code) {
    out.printf("ERROR|%04lu|%s\n", (unsigned long)id, code);
}

void sendProtocolError(Stream& out, uint32_t id, const char* code, const char* p1, float value) {
    out.printf("ERROR|%04lu|%s|%s|%.2f\n", (unsigned long)id, code, p1, value);
}

void sendProtocolStatus(Stream& out, uint32_t id) {
    float theta1 = stepper1->getCurrentPosition() / STEPS_PER_DEGREE;
    float theta2 = stepper2->getCurrentPosition() / STEPS_PER_DEGREE;
    float theta3 = stepper3->getCurrentPosition() / STEPS_PER_DEGREE;

    out.printf("STATUS|%04lu|%s|T1=%.2f|T2=%.2f|T3=%.2f\n",
               (unsigned long)id, getRobotStateName(), theta1, theta2, theta3);
}

void sendStateEvent() {
    Stream& out = *activeProtocolStream;
    if (activeCommandId != 0) {
        out.printf("STATUS|%04lu|%s\n",
                   (unsigned long)activeCommandId,
                   getRobotStateName());
    } else {
        out.printf("STATUS|%s\n", getRobotStateName());
    }
}

// ============================================================
// HOMING INTERNO
// ============================================================

bool homeXDone = false;
bool homeYDone = false;
bool homeZDone = false;

bool homingRecoil = false;

unsigned long homingStartTime = 0;


// ============================================================
// STEPPERS
// ============================================================

FastAccelStepperEngine engine =
    FastAccelStepperEngine();

FastAccelStepper *stepper1 = nullptr;
FastAccelStepper *stepper2 = nullptr;
FastAccelStepper *stepper3 = nullptr;


// ============================================================
// CINEMÁTICA
// ============================================================

const float pi = 3.141592653;

const float sin120 = 0.8660254;
const float cos120 = -0.5;
const float tan30 = 0.577350269;

float e  = 63.0;
float f  = 183.0;
float re = 620.0;
float rf = 225.0;


// ============================================================
// ESTADO → TEXTO
// ============================================================

const char* getRobotStateName() {

    switch (robotState) {

        case ROBOT_NOT_HOMED:
            return "NOT_HOMED";

        case ROBOT_HOMING:
            return "HOMING";

        case ROBOT_READY:
            return "READY";

        case ROBOT_MOVING:
            return "MOVING";

        case ROBOT_PAUSED:
            return "PAUSED";

        case ROBOT_STOPPED:
            return "STOPPED";

        case ROBOT_EMERGENCY:
            return "EMERGENCY";

        case ROBOT_ERROR:
            return "ERROR";
    }

    return "UNKNOWN";
}


// ============================================================
// FINS DE CURSO
// ============================================================

bool limitXTriggered() {

    return digitalRead(LIMIT_X) == HIGH;
}


bool limitYTriggered() {

    return digitalRead(LIMIT_Y) == HIGH;
}


bool limitZTriggered() {

    return digitalRead(LIMIT_Z) == HIGH;
}


bool anyLimitTriggered() {

    return
        limitXTriggered() ||
        limitYTriggered() ||
        limitZTriggered();
}


// ============================================================
// M3.9 — REARME SUPERVISÓRIO + GERENCIAMENTO PNEUMÁTICO
// ============================================================
//
// Política:
//   NOT_HOMED / HOMING             -> bomba OFF, válvula OFF
//   READY / MOVING / PAUSED        -> bomba ON; válvula preserva TOOL
//   STOPPED / EMERGENCY / ERROR    -> bomba OFF, válvula OFF
//
// A bomba é automática e não depende do operador/rotina.
// A válvula continua sendo comandada por TOOL ON/OFF durante
// operação normal. Em STOP/EMERGENCY/ERROR, a segurança tem
// prioridade e a peça é liberada.
// ============================================================

void updatePneumaticState() {

    switch (robotState) {

        case ROBOT_READY:
        case ROBOT_MOVING:
        case ROBOT_PAUSED:
            // Operação normal: bomba automática ligada.
            // Não alteramos VACUUM_PIN aqui para preservar o
            // último TOOL ON/OFF, inclusive durante PAUSE.
            digitalWrite(PUMP_PIN, HIGH);
            break;

        case ROBOT_NOT_HOMED:
        case ROBOT_HOMING:
        case ROBOT_STOPPED:
        case ROBOT_EMERGENCY:
        case ROBOT_ERROR:
        default:
            // Estado não operacional/seguro: desliga geração de
            // vácuo e força a válvula a liberar a peça.
            digitalWrite(PUMP_PIN, LOW);
            digitalWrite(VACUUM_PIN, LOW);
            break;
    }
}


// ============================================================
// EMERGÊNCIA FÍSICA
// ============================================================

bool emergencyActive = false;
bool emergencyLastRaw = false;
unsigned long emergencyChangeTime = 0;
const unsigned long EMERGENCY_DEBOUNCE_MS = 30;

// ============================================================
// BOTÕES FÍSICOS HOME / RESET
// ============================================================


bool readEmergencyInput() {
    // Circuito oficial M3.9:
    // NF fechado conecta 3V3 ao GPIO21 => HIGH = condição normal.
    // Emergência acionada abre o NF; resistor externo 10 kOhm puxa
    // GPIO21 para GND => LOW = emergência. Fio interrompido também tende a LOW.
    return digitalRead(EMERGENCY_PIN) == LOW;
}


void updateEmergencyState() {

    bool raw = readEmergencyInput();

    if (raw != emergencyLastRaw) {
        emergencyLastRaw = raw;
        emergencyChangeTime = millis();
    }

    if ((millis() - emergencyChangeTime) < EMERGENCY_DEBOUNCE_MS)
        return;

    if (raw == emergencyActive)
        return;

    emergencyActive = raw;

    if (emergencyActive) {

        stopAllMotors();

        if (activeCommandId != 0) {
            sendProtocolError(
                *activeProtocolStream,
                activeCommandId,
                "EMERGENCY_STOP"
            );
            clearActiveCommand();
        }

        clearPausedMotion();
        robotState = ROBOT_EMERGENCY;

        Serial.println("EMERGENCY|ACTIVE");
        InterfaceSerial.println("STATUS|EMERGENCY");
    }
    else {

        // Liberar a botoeira não rearma o robô.
        // O retorno operacional exigirá RESET e depois HOME.
        Serial.println("EMERGENCY|RELEASED");
        InterfaceSerial.println("STATUS|EMERGENCY_RELEASED");
    }
}


// ============================================================
// STOP DOS MOTORES
// ============================================================

void stopAllMotors() {

    if (stepper1)
        stepper1->forceStop();

    if (stepper2)
        stepper2->forceStop();

    if (stepper3)
        stepper3->forceStop();
}


// ============================================================
// CINEMÁTICA
// ============================================================

int delta_calcAngleYZ(
    float x0,
    float y0,
    float z0,
    float &theta
) {

    float y1 =
        -0.5 * tan30 * f;

    y0 -=
        0.5 * tan30 * e;


    float a =
        (
            x0 * x0 +
            y0 * y0 +
            z0 * z0 +
            rf * rf -
            re * re -
            y1 * y1
        )
        /
        (2 * z0);


    float b =
        (y1 - y0) / z0;


    float d =
        -(a + b * y1) *
        (a + b * y1)
        +
        rf *
        (
            b * b * rf +
            rf
        );


    if (d < 0) {

        return -1;
    }


    float yj =
        (
            y1 -
            a * b -
            sqrt(d)
        )
        /
        (b * b + 1);


    float zj =
        a +
        b * yj;


    theta =
        atan(
            -zj /
            (y1 - yj)
        )
        *
        180.0 /
        pi;


    if (yj > y1) {

        theta += 180.0;
    }


    return 0;
}


// ============================================================
// CINEMÁTICA INVERSA
// ============================================================

int delta_calcInverse(
    float x0,
    float y0,
    float z0,
    float &t1,
    float &t2,
    float &t3
) {

    int s =
        delta_calcAngleYZ(
            x0,
            y0,
            z0,
            t1
        );


    if (s == 0) {

        s =
            delta_calcAngleYZ(
                x0 * cos120 +
                y0 * sin120,

                y0 * cos120 -
                x0 * sin120,

                z0,

                t2
            );
    }


    if (s == 0) {

        s =
            delta_calcAngleYZ(
                x0 * cos120 -
                y0 * sin120,

                y0 * cos120 +
                x0 * sin120,

                z0,

                t3
            );
    }


    return s;
}


// ============================================================
// LIMITES ANGULARES
// ============================================================

bool validateThetaLimits(
    float t1,
    float t2,
    float t3,
    uint32_t commandId = 0,
    Stream* out = nullptr
) {

    if (
        t1 < THETA_MIN ||
        t1 > THETA_MAX
    ) {

        if (commandId != 0)
            sendProtocolError(out ? *out : Serial, commandId, "JOINT_LIMIT", "J1", t1);
        else if (activeCommandId != 0)
            sendProtocolError(*activeProtocolStream, activeCommandId, "JOINT_LIMIT", "J1", t1);
        else
            Serial.printf("ERROR|JOINT_LIMIT|J1|%.2f\n", t1);

        return false;
    }


    if (
        t2 < THETA_MIN ||
        t2 > THETA_MAX
    ) {

        if (commandId != 0)
            sendProtocolError(out ? *out : Serial, commandId, "JOINT_LIMIT", "J2", t2);
        else if (activeCommandId != 0)
            sendProtocolError(*activeProtocolStream, activeCommandId, "JOINT_LIMIT", "J2", t2);
        else
            Serial.printf("ERROR|JOINT_LIMIT|J2|%.2f\n", t2);

        return false;
    }


    if (
        t3 < THETA_MIN ||
        t3 > THETA_MAX
    ) {

        if (commandId != 0)
            sendProtocolError(out ? *out : Serial, commandId, "JOINT_LIMIT", "J3", t3);
        else if (activeCommandId != 0)
            sendProtocolError(*activeProtocolStream, activeCommandId, "JOINT_LIMIT", "J3", t3);
        else
            Serial.printf("ERROR|JOINT_LIMIT|J3|%.2f\n", t3);

        return false;
    }


    return true;
}


// ============================================================
// THETA → PASSOS
// ============================================================

long thetaToSteps(float theta) {

    return (long)(
        theta *
        STEPS_PER_DEGREE
    );
}


// ============================================================
// MOVIMENTO INTERPOLADO
// ============================================================

void moverInterpolado(
    long target1,
    long target2,
    long target3
) {

    long current1 =
        stepper1->getCurrentPosition();

    long current2 =
        stepper2->getCurrentPosition();

    long current3 =
        stepper3->getCurrentPosition();


    long delta1 =
        labs(target1 - current1);

    long delta2 =
        labs(target2 - current2);

    long delta3 =
        labs(target3 - current3);


    long maxDelta =
        delta1;


    if (delta2 > maxDelta)
        maxDelta = delta2;


    if (delta3 > maxDelta)
        maxDelta = delta3;


    if (maxDelta == 0)
        return;


    float speed1 =
        ((float)delta1 / maxDelta)
        *
        MAX_OPERATING_SPEED;


    float speed2 =
        ((float)delta2 / maxDelta)
        *
        MAX_OPERATING_SPEED;


    float speed3 =
        ((float)delta3 / maxDelta)
        *
        MAX_OPERATING_SPEED;


    if (
        speed1 < 1.0 &&
        delta1 > 0
    )
        speed1 = 1.0;


    if (
        speed2 < 1.0 &&
        delta2 > 0
    )
        speed2 = 1.0;


    if (
        speed3 < 1.0 &&
        delta3 > 0
    )
        speed3 = 1.0;


    if (delta1 > 0)
        stepper1->setSpeedInHz(
            (uint32_t)speed1
        );


    if (delta2 > 0)
        stepper2->setSpeedInHz(
            (uint32_t)speed2
        );


    if (delta3 > 0)
        stepper3->setSpeedInHz(
            (uint32_t)speed3
        );


    stepper1->moveTo(target1);
    stepper2->moveTo(target2);
    stepper3->moveTo(target3);
}


// ============================================================
// INICIA HOMING
// ============================================================

void startHoming(uint32_t commandId = 0, Stream& out = Serial) {

    if (
        robotState == ROBOT_MOVING ||
        robotState == ROBOT_HOMING
    ) {

        Serial.println(
            "ERROR|HOME_NOT_ALLOWED"
        );

        return;
    }


    robotState =
        ROBOT_HOMING;


    homingRecoil = false;


    homeXDone =
        limitXTriggered();

    homeYDone =
        limitYTriggered();

    homeZDone =
        limitZTriggered();


    homingStartTime =
        millis();


    if (commandId != 0) {
        setActiveCommand(commandId, "HOME", out);
        sendAck(out, commandId);
        sendStateEvent();
    }
    else {
        out.println("ACK|HOME");
        out.println("STATUS|HOMING");
    }


    stepper1->setSpeedInHz(
        HOMING_SPEED
    );

    stepper2->setSpeedInHz(
        HOMING_SPEED
    );

    stepper3->setSpeedInHz(
        HOMING_SPEED
    );


    if (!homeXDone)
        stepper1->runBackward();


    if (!homeYDone)
        stepper2->runBackward();


    if (!homeZDone)
        stepper3->runBackward();
}


// ============================================================
// ATUALIZA HOMING
// ============================================================

void updateHoming() {

    if (
        robotState != ROBOT_HOMING
    ) {

        return;
    }


    // ========================================================
    // FASE 1 - PROCURAR SENSORES
    // ========================================================

    if (!homingRecoil) {

        // ----------------------------------------------------
        // TIMEOUT
        // ----------------------------------------------------

        if (
            millis() -
            homingStartTime >
            HOMING_TIMEOUT
        ) {

            stopAllMotors();

            robotState =
                ROBOT_ERROR;

            if (activeCommandId != 0) {
                sendProtocolError(*activeProtocolStream, activeCommandId, "HOMING_TIMEOUT");
                clearActiveCommand();
            }
            else {
                Serial.println("ERROR|HOMING_TIMEOUT");
            }

            return;
        }


        // ----------------------------------------------------
        // SENSOR X
        // ----------------------------------------------------

        if (
            !homeXDone &&
            limitXTriggered()
        ) {

            stepper1->forceStop();

            homeXDone = true;
        }


        // ----------------------------------------------------
        // SENSOR Y
        // ----------------------------------------------------

        if (
            !homeYDone &&
            limitYTriggered()
        ) {

            stepper2->forceStop();

            homeYDone = true;
        }


        // ----------------------------------------------------
        // SENSOR Z
        // ----------------------------------------------------

        if (
            !homeZDone &&
            limitZTriggered()
        ) {

            stepper3->forceStop();

            homeZDone = true;
        }


        // ----------------------------------------------------
        // TODOS OS SENSORES?
        // ----------------------------------------------------

        if (
            homeXDone &&
            homeYDone &&
            homeZDone
        ) {

            // Aguarda os motores realmente pararem.

            if (
                stepper1->isRunning() ||
                stepper2->isRunning() ||
                stepper3->isRunning()
            ) {

                return;
            }


            // --------------------------------------------
            // INICIA RECUO
            // --------------------------------------------

            stepper1->setSpeedInHz(
                HOMING_SPEED
            );

            stepper2->setSpeedInHz(
                HOMING_SPEED
            );

            stepper3->setSpeedInHz(
                HOMING_SPEED
            );


            stepper1->move(
                HOME_RECOIL_STEPS
            );

            stepper2->move(
                HOME_RECOIL_STEPS
            );

            stepper3->move(
                HOME_RECOIL_STEPS
            );


            homingRecoil = true;
        }


        return;
    }


    // ========================================================
    // FASE 2 - RECUO
    // ========================================================

    if (homingRecoil) {

        if (
            stepper1->isRunning() ||
            stepper2->isRunning() ||
            stepper3->isRunning()
        ) {

            return;
        }


        // ----------------------------------------------------
        // DEFINE REFERÊNCIA
        // ----------------------------------------------------

        stepper1->setCurrentPosition(0);

        stepper2->setCurrentPosition(0);

        stepper3->setCurrentPosition(0);


        robotState =
            ROBOT_READY;


        homingRecoil = false;


        if (activeCommandId != 0) {
            uint32_t completedId = activeCommandId;
            sendOk(*activeProtocolStream, completedId, "HOME");
            sendProtocolStatus(*activeProtocolStream, completedId);
            clearActiveCommand();
        }
        else {
            Serial.println("STATUS|READY");
            Serial.println("HOME|OK");
        }
    }
}


// ============================================================
// ATUALIZA MOVIMENTO
// ============================================================

void updateMotion() {

    if (robotState != ROBOT_MOVING) {
        return;
    }

    if (anyLimitTriggered()) {
        stopAllMotors();
        robotState = ROBOT_ERROR;

        if (activeCommandId != 0) {
            sendProtocolError(*activeProtocolStream, activeCommandId, "LIMIT_SWITCH");
            clearActiveCommand();
        }
        else {
            Serial.println("ERROR|LIMIT_SWITCH");
        }

        clearPausedMotion();
        return;
    }

    if (!stepper1->isRunning() &&
        !stepper2->isRunning() &&
        !stepper3->isRunning()) {

        if (pausePending) {
            uint32_t pauseId = pendingPauseId;
            Stream* pauseStream = pendingPauseStream;

            pausePending = false;
            pendingPauseId = 0;
            pendingPauseStream = nullptr;
            robotState = ROBOT_PAUSED;

            if (pauseStream) {
                pauseStream->printf("STATUS|%04lu|PAUSED\n", (unsigned long)pauseId);
                sendOk(*pauseStream, pauseId, "PAUSE");
            } else {
                Serial.println("STATUS|PAUSED");
                Serial.println("OK|PAUSE");
            }
            return;
        }

        robotState = ROBOT_READY;

        if (activeCommandId != 0) {
            uint32_t completedId = activeCommandId;
            sendOk(*activeProtocolStream, completedId, "MOVEJ");
            sendProtocolStatus(*activeProtocolStream, completedId);
            clearActiveCommand();
        } else {
            Serial.println("OK|MOVEJ");
        }

        clearPausedMotion();
    }
}


// ============================================================
// MOVEJ
// ============================================================

void executeMoveJ(
    float x,
    float y,
    float z,
    const char* tool,
    uint32_t commandId = 0,
    Stream& out = Serial
) {

    if (
        robotState != ROBOT_READY
    ) {

        if (commandId != 0)
            sendProtocolError(out, commandId, "ROBOT_NOT_READY");
        else
            Serial.println("ERROR|ROBOT_NOT_READY");

        return;
    }


    // ========================================================
    // CINEMÁTICA
    // ========================================================

    float t1;
    float t2;
    float t3;


    int result =
        delta_calcInverse(
            x,
            y,
            z,
            t1,
            t2,
            t3
        );


    if (
        result != 0
    ) {

        if (commandId != 0)
            sendProtocolError(out, commandId, "KINEMATICS");
        else
            Serial.println("ERROR|KINEMATICS");

        return;
    }


    // ========================================================
    // DEBUG
    // ========================================================

    Serial.printf(
        "DEBUG|IK|T1=%.2f|T2=%.2f|T3=%.2f\n",
        t1,
        t2,
        t3
    );


    // ========================================================
    // LIMITES
    // ========================================================

    if (
        !validateThetaLimits(
            t1,
            t2,
            t3,
            commandId,
            &out
        )
    ) {

        return;
    }


    // ========================================================
    // PASSOS
    // ========================================================

    long target1 =
        thetaToSteps(t1);

    long target2 =
        thetaToSteps(t2);

    long target3 =
        thetaToSteps(t3);


    Serial.printf(
        "DEBUG|TARGET|%ld|%ld|%ld\n",
        target1,
        target2,
        target3
    );


    // ========================================================
    // CONTEXTO DO MOVIMENTO
    // ========================================================
    pausedTarget1 = target1;
    pausedTarget2 = target2;
    pausedTarget3 = target3;
    strncpy(pausedTool, tool, sizeof(pausedTool));
    pausedTool[sizeof(pausedTool) - 1] = '\0';
    pausedMotionValid = true;
    pausePending = false;
    pendingPauseId = 0;
    pendingPauseStream = nullptr;

    // ========================================================
    // FERRAMENTA
    // ========================================================

    if (
        strcasecmp(
            tool,
            "ON"
        ) == 0
    ) {

        digitalWrite(
            VACUUM_PIN,
            HIGH
        );
    }


    else if (
        strcasecmp(
            tool,
            "OFF"
        ) == 0
    ) {

        digitalWrite(
            VACUUM_PIN,
            LOW
        );
    }


    // ========================================================
    // INICIA MOVIMENTO
    // ========================================================

    moverInterpolado(
        target1,
        target2,
        target3
    );


    robotState =
        ROBOT_MOVING;

    if (commandId != 0) {
        setActiveCommand(commandId, "MOVEJ", out);
        sendAck(out, commandId);
        sendStateEvent();
    }
    else {
        Serial.println("ACK|MOVEJ");
        Serial.println("STATUS|MOVING");
    }
}


// ============================================================
// PAUSE
// ============================================================

void requestPause(uint32_t commandId, Stream& out) {
    if (robotState != ROBOT_MOVING ||
        activeCommandId == 0 ||
        !pausedMotionValid) {
        sendAck(out, commandId);
        sendProtocolError(out, commandId, "PAUSE_NOT_ACTIVE");
        return;
    }

    sendAck(out, commandId);
    pendingPauseId = commandId;
    pendingPauseStream = &out;
    pausePending = true;
    stopAllMotors();
}

// ============================================================
// RESUME
// ============================================================

void requestResume(uint32_t commandId, Stream& out) {
    if (robotState != ROBOT_PAUSED ||
        activeCommandId == 0 ||
        !pausedMotionValid) {
        sendAck(out, commandId);
        sendProtocolError(out, commandId, "RESUME_NOT_ALLOWED");
        return;
    }

    sendAck(out, commandId);
    moverInterpolado(pausedTarget1, pausedTarget2, pausedTarget3);
    robotState = ROBOT_MOVING;
    out.printf("STATUS|%04lu|MOVING\n", (unsigned long)commandId);
    sendOk(out, commandId, "RESUME");
}


// ============================================================
// STATUS
// ============================================================

void sendStatus(uint32_t commandId = 0, Stream& out = Serial) {

    float theta1 =
        stepper1->getCurrentPosition()
        /
        STEPS_PER_DEGREE;


    float theta2 =
        stepper2->getCurrentPosition()
        /
        STEPS_PER_DEGREE;


    float theta3 =
        stepper3->getCurrentPosition()
        /
        STEPS_PER_DEGREE;


    if (commandId != 0) {
        out.printf(
            "STATUS|%04lu|%s|T1=%.2f|T2=%.2f|T3=%.2f\n",
            (unsigned long)commandId,
            getRobotStateName(),
            theta1,
            theta2,
            theta3
        );
    }
    else {
        out.printf(
            "STATUS|%s|T1=%.2f|T2=%.2f|T3=%.2f\n",
            getRobotStateName(),
            theta1,
            theta2,
            theta3
        );
    }
}


// ============================================================
// RESET
// ============================================================

void resetController(uint32_t commandId = 0, Stream& out = Serial) {

    //stopAllMotors();


    robotState =
        ROBOT_NOT_HOMED;


    homingRecoil =
        false;


    homeXDone =
        false;

    homeYDone =
        false;

    homeZDone =
        false;

    clearPausedMotion();

    if (commandId != 0) {
        sendAck(out, commandId);
    }
    else {
        out.println("ACK|RESET");
    }

    if (activeCommandId != 0) {
        uint32_t interruptedId = activeCommandId;
        sendProtocolError(
            *activeProtocolStream,
            interruptedId,
            "RESET_BY_OPERATOR"
        );
    }

    clearActiveCommand();

    if (commandId != 0) {
        sendOk(out, commandId, "RESET");
        out.println("STATUS|NOT_HOMED");
    }
    else {
        Serial.println("STATUS|NOT_HOMED");
    }
}


// ============================================================
// SETUP
// ============================================================

void setup() {


    InterfaceSerial.begin(
    115200,
    SERIAL_8N1,
    UART2_RX,
    UART2_TX
);

    Serial.begin(115200);

    delay(500);


    // ========================================================
    // FINS DE CURSO
    // ========================================================

    // GPIO34 e GPIO35:
    // usar resistores externos de aproximadamente 10k
    // para 3.3V.

    pinMode(
        LIMIT_X,
        INPUT
    );

    pinMode(
        LIMIT_Y,
        INPUT
    );

    pinMode(
        LIMIT_Z,
        INPUT_PULLUP
    );

    // Emergência M3.9: NF para 3V3 + pull-down externo de 10 kOhm.
    // Não usar INPUT_PULLUP: o estado seguro é definido pelo circuito externo.
    pinMode(EMERGENCY_PIN, INPUT);

    emergencyLastRaw = readEmergencyInput();
    emergencyActive = emergencyLastRaw;
    emergencyChangeTime = millis();


    // ========================================================
    // SAÍDAS
    // ========================================================

    pinMode(
        VACUUM_PIN,
        OUTPUT
    );

    pinMode(
        PUMP_PIN,
        OUTPUT
    );


    // M3.9: boot ocorre em NOT_HOMED; sistema pneumático inicia seguro.
    digitalWrite(
        PUMP_PIN,
        LOW
    );

    digitalWrite(
        VACUUM_PIN,
        LOW
    );


    // ========================================================
    // STEPPERS
    // ========================================================

    engine.init();


    stepper1 =
        engine.stepperConnectToPin(
            STEP_1
        );


    stepper2 =
        engine.stepperConnectToPin(
            STEP_2
        );


    stepper3 =
        engine.stepperConnectToPin(
            STEP_3
        );


    if (
        !stepper1 ||
        !stepper2 ||
        !stepper3
    ) {

        robotState =
            ROBOT_ERROR;


        Serial.println(
            "ERROR|STEPPER_INIT"
        );

        return;
    }


    // ========================================================
    // DIREÇÃO
    // ========================================================

    stepper1->setDirectionPin(
        DIR_1,
        false
    );

    stepper2->setDirectionPin(
        DIR_2,
        false
    );

    stepper3->setDirectionPin(
        DIR_3,
        false
    );


    // ========================================================
    // ACELERAÇÃO
    // ========================================================

    stepper1->setAcceleration(
        MOTOR_ACCELERATION
    );

    stepper2->setAcceleration(
        MOTOR_ACCELERATION
    );

    stepper3->setAcceleration(
        MOTOR_ACCELERATION
    );


    robotState =
        ROBOT_NOT_HOMED;


    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "DELTA CONTROLLER V2.5-M3.9"
    );

    if (emergencyActive) {
        robotState = ROBOT_EMERGENCY;
        Serial.println("Estado: EMERGENCY");
        Serial.println("EMERGENCY|ACTIVE");
    }
    else {
        Serial.println(
            "Estado: NOT_HOMED"
        );
    }

    Serial.println(
        "================================"
    );
}


// ============================================================
// LOOP
// ============================================================

// ============================================================
// PROCESSAMENTO DE COMANDOS
// ============================================================

bool extractProtocolCommand(
    const String& input,
    uint32_t& id,
    String& command,
    String& parameters
) {

    if (!input.startsWith("CMD|"))
        return false;

    int p1 = input.indexOf('|', 4);

    if (p1 < 0)
        return false;

    int p2 = input.indexOf('|', p1 + 1);

    // ID do protocolo: exatamente 4 dígitos (0001..9999).
    // O campo de parâmetros é opcional.
    // Portanto, comandos sem parâmetros como:
    // CMD|0001|STATUS
    // CMD|0002|HOME
    // CMD|0003|STOP
    // CMD|0004|RESET
    // também são frames válidos.
    String idText = input.substring(4, p1);

    if (p2 < 0) {
        command = input.substring(p1 + 1);
        parameters = "";
    }
    else {
        command = input.substring(p1 + 1, p2);
        parameters = input.substring(p2 + 1);
    }

    idText.trim();
    command.trim();
    parameters.trim();

    if (idText.length() != 4 || command.length() == 0)
        return false;

    for (size_t i = 0; i < idText.length(); i++) {
        if (!isDigit(idText[i]))
            return false;
    }

    id = (uint32_t)idText.toInt();

    // 0000 é reservado para erros de frame sem ID válido.
    if (id == 0)
        return false;

    command.toUpperCase();

    return true;
}


bool parseMoveJParameters(
    const String& parameters,
    float& x,
    float& y,
    float& z,
    char* tool,
    size_t toolSize
) {

    String p = parameters;
    p.trim();

    int p1 = p.indexOf('|');
    int p2 = p.indexOf('|', p1 + 1);
    int p3 = p.indexOf('|', p2 + 1);

    if (p1 < 0 || p2 < 0)
        return false;

    String sx = p.substring(0, p1);
    String sy = p.substring(p1 + 1, p2);
    String sz;
    String st = "OFF";

    if (p3 >= 0) {
        sz = p.substring(p2 + 1, p3);
        st = p.substring(p3 + 1);
    }
    else {
        sz = p.substring(p2 + 1);
    }

    sx.trim();
    sy.trim();
    sz.trim();
    st.trim();

    if (sx.length() == 0 || sy.length() == 0 || sz.length() == 0)
        return false;

    char* endPtr = nullptr;

    x = strtof(sx.c_str(), &endPtr);
    if (endPtr == sx.c_str() || *endPtr != '\0') return false;

    y = strtof(sy.c_str(), &endPtr);
    if (endPtr == sy.c_str() || *endPtr != '\0') return false;

    z = strtof(sz.c_str(), &endPtr);
    if (endPtr == sz.c_str() || *endPtr != '\0') return false;

    if (st.length() == 0 || st.length() >= toolSize)
        return false;

    st.toUpperCase();
    st.toCharArray(tool, toolSize);

    if (strcmp(tool, "ON") != 0 && strcmp(tool, "OFF") != 0)
        return false;

    return true;
}


void handleProtocolCommand(
    const String& input,
    Stream& out
) {

    uint32_t commandId = 0;
    String command;
    String parameters;

    if (!extractProtocolCommand(
        input,
        commandId,
        command,
        parameters
    )) {

        out.println("ERROR|0000|INVALID_FRAME");
        return;
    }


    // ========================================================
    // STATUS — pode ser consultado a qualquer momento.
    // ========================================================

    if (command == "STATUS") {

        sendAck(out, commandId);
        sendStatus(commandId, out);
        sendOk(out, commandId, "STATUS");
        return;
    }


    // ========================================================
    // PAUSE — parada reversível, preservando o MOVEJ ativo.
    // ========================================================
    if (command == "PAUSE") {
        requestPause(commandId, out);
        return;
    }

    // ========================================================
    // RESUME — retoma o MOVEJ interrompido por PAUSE.
    // ========================================================
    if (command == "RESUME") {
        requestResume(commandId, out);
        return;
    }

    // ========================================================
    // STOP — comando de prioridade durante movimento.
    // ========================================================

    if (command == "STOP") {

        sendAck(out, commandId);

        if (robotState == ROBOT_MOVING ||
            robotState == ROBOT_PAUSED) {

            stopAllMotors();
            robotState = ROBOT_STOPPED;

            if (activeCommandId != 0) {
                uint32_t interruptedId = activeCommandId;
                sendProtocolError(
                    *activeProtocolStream,
                    interruptedId,
                    "STOPPED_BY_OPERATOR"
                );
                clearActiveCommand();
            }

            clearPausedMotion();

            sendOk(out, commandId, "STOP");
            out.println("STATUS|STOPPED");
        }
        else {
            sendProtocolError(out, commandId, "STOP_NOT_ACTIVE");
        }

        return;
    }


    // ========================================================
    // RESET — limpa a execução e exige HOME novamente.
    // ========================================================

    if (command == "RESET") {

        if (emergencyActive) {
            sendProtocolError(out, commandId, "EMERGENCY_ACTIVE");
            return;
        }

        // M3.9: rearme supervisório. RESET remoto é permitido em STOPPED
        // somente quando a emergência física NÃO está ativa. A confirmação
        // consciente do operador é responsabilidade da HMI.
        resetController(commandId, out);
        return;
    }


    // ========================================================
    // HOME
    // ========================================================

    if (command == "HOME") {

        if (emergencyActive) {
            sendProtocolError(out, commandId, "EMERGENCY_ACTIVE");
            return;
        }

        if (
            robotState == ROBOT_MOVING ||
            robotState == ROBOT_HOMING ||
            robotState == ROBOT_PAUSED ||
            robotState == ROBOT_STOPPED
        ) {
            sendProtocolError(out, commandId, "HOME_NOT_ALLOWED");
            return;
        }

        startHoming(commandId, out);
        return;
    }


    // ========================================================
    // MOVEJ
    // ========================================================

    if (command == "MOVEJ") {

        float x;
        float y;
        float z;
        char tool[10] = "OFF";

        if (!parseMoveJParameters(
            parameters,
            x,
            y,
            z,
            tool,
            sizeof(tool)
        )) {

            sendProtocolError(out, commandId, "INVALID_MOVEJ");
            return;
        }

        executeMoveJ(
            x,
            y,
            z,
            tool,
            commandId,
            out
        );

        return;
    }


    // ========================================================
    // DESCONHECIDO
    // ========================================================

    sendProtocolError(out, commandId, "UNKNOWN_COMMAND");
}


void processProtocolStream(Stream& in, bool allowLegacy) {

    while (in.available()) {

        char c = (char)in.read();

        if (c == '\r')
            continue;

        if (c == '\n') {

            protocolRxBuffer[protocolRxIndex] = '\0';

            if (protocolRxIndex > 0) {

                String input = String(protocolRxBuffer);
                input.trim();

                if (input.startsWith("CMD|")) {
                    handleProtocolCommand(input, in);
                }
                else if (allowLegacy) {
                    // ------------------------------------------------ com comandos antigos de bancada.
                    // ------------------------------------------------
                    if (input.equalsIgnoreCase("RESET")) {
                        if (emergencyActive) {
                            Serial.println("ERROR|EMERGENCY_ACTIVE");
                        }
                        else {
                            resetController();
                        }
                    }
                    else if (input.equalsIgnoreCase("STOP")) {
                        if (robotState == ROBOT_MOVING) {
                            stopAllMotors();
                            robotState = ROBOT_STOPPED;
                            Serial.println("ACK|STOP");
                            Serial.println("STOP|OK");
                            Serial.println("STATUS|STOPPED");
                        }
                        else {
                            Serial.println("ERROR|STOP_NOT_ACTIVE");
                        }
                    }
                    else if (
                        input.equalsIgnoreCase("HOME") ||
                        input.equalsIgnoreCase("HOMING")
                    ) {
                        if (robotState == ROBOT_STOPPED) {
                            Serial.println("ERROR|HOME_NOT_ALLOWED");
                        }
                        else {
                            startHoming();
                        }
                    }
                    else if (input.equalsIgnoreCase("STATUS")) {
                        sendStatus();
                    }
                    else if (input.startsWith("MOVEJ")) {

                        float x;
                        float y;
                        float z;
                        char tool[10] = "OFF";

                        int parsed = sscanf(
                            input.c_str(),
                            "MOVEJ %f %f %f %9s",
                            &x,
                            &y,
                            &z,
                            tool
                        );

                        if (parsed < 3) {
                            Serial.println("ERROR|INVALID_MOVEJ");
                        }
                        else {
                            executeMoveJ(x, y, z, tool);
                        }
                    }
                    else {
                        Serial.println("ERROR|UNKNOWN_COMMAND");
                    }
                }
            }

            protocolRxIndex = 0;
            continue;
        }

        if (protocolRxIndex < PROTOCOL_RX_BUFFER_SIZE - 1) {
            protocolRxBuffer[protocolRxIndex++] = c;
        }
        else {
            protocolRxIndex = 0;
            in.println("ERROR|0|LINE_TOO_LONG");
        }
    }
}


// ============================================================
// LOOP
// ============================================================

void loop() {

    // Supervisão contínua da emergência física.
    // ENA+/ENA- continua sendo a parada de hardware independente.
    updateEmergencyState();

    processProtocolStream(InterfaceSerial, false);
    processProtocolStream(Serial, true);

    updateHoming();
    updateMotion();

    // M3.9: aplica continuamente a política pneumática do estado atual.
    updatePneumaticState();
}