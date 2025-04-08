#include <Arduino.h>
#include <BLDCMotor.h>

BLDCMotor motor;

uint8_t motorPins[] = {
    5, 18, 19, 21, 22, 23};

void setup()
{
    Serial.begin(115200);
    motor.begin(motorPins, 80, 2);
    motor.setDirection(BLDCMotor::Direction::FORWARD);

    Serial.println("Rampando até velocidade 200");
    motor.rampToSpeed(200, 3000); // 3 segundos

    delay(2000);

    Serial.println("Rampando até velocidade 100");
    motor.rampToSpeed(100, 2000); // 2 segundos

    delay(2000);

    Serial.println("Parando");
    motor.setStop();
}

void loop()
{
    // Nada aqui
}
