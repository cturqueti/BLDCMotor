#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#define NC 255 // No connected

// #include "esp_log.h"
#include <Arduino.h> // Se for usar funções do Arduino (digitalWrite, etc.)
#include <LogLibrary.h>

class BLDCMotor
{
public:
    struct Pins
    {
        uint8_t pwm; // Controle de velocidade
        uint8_t fwr; // Direção
        uint8_t en;  // Enable
        uint8_t brk; // Freio
        uint8_t spd; // Sensor de velocidade
        uint8_t alm; // Alarme
    };

    struct Characteristics
    {
        float wheelDiameter; // Diametro da roda em mm
        uint8_t ppr;         // Pulos por revolução
    };

    enum class Direction
    {
        FORWARD,
        REVERSE,
        BRAKE,
        COAST
    };

    BLDCMotor();
    ~BLDCMotor();
    void begin(const Pins &pins, const Characteristics &characteristics); // Inicializa os pinos
    void setSpeed(int speed);                                             // Controla velocidade (0-255)
    void setDirection(Direction dir, int8_t speed = 255);
    float getSpeedMPS() const;
    float getRPM() const;
    bool isFault() const;
    void setStop();
    void rampToSpeed(uint8_t target, uint16_t duration);
    void updateSpeed();

private:
    Pins _pins;
    Characteristics _characteristics;
    float _speed_mps, _speed_rps;
    uint8_t _speed;
    static volatile uint32_t _speed_pulse;

    static void handleSpeedInterrupt();
};

#endif
