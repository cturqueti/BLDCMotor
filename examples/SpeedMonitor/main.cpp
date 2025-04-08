#include <Arduino.h>
#include <BLDCMotor.h>

BLDCMotor motor;

uint8_t motorPins[] = {
    5, 18, 19, 21, 22, 23};

void setup()
{
    Serial.begin(115200);
    motor.begin(motorPins, 80, 2);
    motor.setDirection(BLDCMotor::Direction::FORWARD, 180);
}

void loop()
{
    motor.setSpeed(180); // Mantém a velocidade
    delay(1000);

    Serial.print("Velocidade: ");
    Serial.print(motor.getSpeedMPS());
    Serial.print(" m/s | ");
    Serial.print(motor.getRPM());
    Serial.print(" RPM");

    if (motor.isFault())
    {
        Serial.println(" | Falha detectada!");
    }
    else
    {
        Serial.println(" | Sem falhas.");
    }
}
