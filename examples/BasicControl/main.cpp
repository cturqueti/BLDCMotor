#include <Arduino.h>
#include <BLDCMotor.h>

BLDCMotor motor;

void setup()
{
    Serial.begin(115200);

    // Definindo os pinos usando a struct Pins
    BLDCMotor::Pins motorPins = {
        .pwm = 5,
        .fwr = 18,
        .en  = 19,
        .brk = 21,
        .spd = 22,
        .alm = 23
    };

    // Características do motor
    BLDCMotor::Characteristics motorCharacteristics = {
        .wheelDiameter = 80.0, // em mm
        .ppr = 2               // pulsos por rotação
    };

    // Inicializa o motor com os parâmetros
    motor.begin(motorPins, motorCharacteristics);

    Serial.println("Motor iniciado em modo FORWARD");
    motor.setDirection(BLDCMotor::Direction::FORWARD, 128);
}

void loop()
{
    delay(5000);
    Serial.println("Mudando para REVERSE");
    motor.setDirection(BLDCMotor::Direction::REVERSE, 128);
    
    delay(5000);
    Serial.println("Parando (BRAKE)");
    motor.setDirection(BLDCMotor::Direction::BRAKE);
    
    delay(3000);
    Serial.println("Rodando livre (COAST)");
    motor.setDirection(BLDCMotor::Direction::COAST);
    
    delay(3000);
}
