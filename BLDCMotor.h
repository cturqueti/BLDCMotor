#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#define NC 255 // No connected

// #include "esp_log.h"
#include <Arduino.h> // Se for usar funções do Arduino (digitalWrite, etc.)
#include <LogLibrary.h>

class BLDCMotor
{
public:
    enum class Direction
    {
        FORWARD,
        REVERSE,
        BRAKE,
        COAST
    };

    enum class ControlPins
    {
        PWM,
        FWR,
        EN,
        BRK
    };

    enum class SensorPins
    {
        SPD,
        ALM
    };

    BLDCMotor();
    ~BLDCMotor();
    void begin(uint8_t pins[6], uint8_t diameter, uint8_t ppr); // Inicializa os pinos
    void setSpeed(int speed);                                   // Controla velocidade (0-255)
    void setDirection(Direction dir, int8_t speed = 255);
    float getSpeedMPS() const;
    float getRPM() const;
    bool isFault() const;
    void setStop();
    void rampToSpeed(uint8_t target, uint16_t duration);
    // static void motInt();

private:
    uint8_t _pinControl[4]; // PWM, FWR, EN, BRK
    uint8_t _pinSensor[2];  // SPD, ALM
    uint8_t _pinPWM, _pinFWR, _pinEN, _pinBRK;
    uint8_t _pinSPD, _pinALM;
    float _speed_mps, _speed_rps;
    uint8_t _speed;
    uint8_t _diameter, _ppr;
    static volatile uint32_t _speed_pulse;

    static void motInt();
    void calculateSpeed();
};

#endif
