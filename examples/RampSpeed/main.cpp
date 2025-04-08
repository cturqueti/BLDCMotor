#include <Arduino.h>
#include <BLDCMotor.h>

BLDCMotor motor;

void setup()
{
    Serial.begin(115200);

    // Define os pinos usando a struct Pins
    BLDCMotor::Pins motorPins = {
        .pwm = 5,
        .fwr = 18,
        .en = 19,
        .brk = 21,
        .spd = 22,
        .alm = 23};

    // Define as características do motor
    BLDCMotor::Characteristics motorCharacteristics = {
        .wheelDiameter = 80.0, // em mm
        .ppr = 2               // pulsos por rotação
    };

    // Inicializa o motor
    motor.begin(motorPins, motorCharacteristics);
    motor.setDirection(BLDCMotor::Direction::FORWARD);

    LOG_INFO("Rampando até velocidade 200");
    motor.rampToSpeed(200, 3000); // 3 segundos

    delay(2000);

    LOG_INFO("Rampando até velocidade 100");
    motor.rampToSpeed(100, 2000); // 2 segundos

    delay(2000);

    LOG_INFO("Parando");
    motor.setStop();
}

void loop()
{
    // Nada aqui
}
