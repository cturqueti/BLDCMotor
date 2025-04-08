#include <Arduino.h>
#include <BLDCMotor.h>

BLDCMotor motor;

void setup()
{
    Serial.begin(115200);

    // Define os pinos com a nova struct
    BLDCMotor::Pins motorPins = {
        .pwm = 5,
        .fwr = 18,
        .en = 19,
        .brk = 21,
        .spd = 22,
        .alm = 23};

    // Define as características do motor
    BLDCMotor::Characteristics motorCharacteristics = {
        .wheelDiameter = 80.0, // mm
        .ppr = 2               // Pulsos por rotação
    };

    // Inicializa o motor com a nova API
    motor.begin(motorPins, motorCharacteristics);

    // Direção inicial e velocidade
    motor.setDirection(BLDCMotor::Direction::FORWARD, 180);
}

void loop()
{
    // Mantém a velocidade (opcional se já setado no setup)
    motor.setSpeed(180);

    // Exibe os dados no monitor serial
    LOG_INFO("Velocidade: %f m/s | %f RPM", motor.getSpeedMPS(), motor.getRPM());

    // Verifica falha
    if (motor.isFault())
    {
        LOG_ERROR("Falha detectada!");
    }
    else
    {
        Serial.println(" | Sem falhas.");
        LOG_INFO("Sem falhas.");
    }

    delay(1000);
}
