#include <Arduino.h>
#include <BLDCMotor.h>

BLDCMotor motor;

uint8_t motorPins[] = {
    5,  // PWM
    18, // FWR
    19, // EN
    21, // BRK
    22, // SPD
    23  // ALM
};

void setup()
{
    Serial.begin(115200);
    motor.begin(motorPins, 80, 2); // diâmetro em mm, ppr (pulsos por rotação)

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
