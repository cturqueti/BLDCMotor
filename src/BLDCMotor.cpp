#include "BLDCMotor.h"

// Inicialização da variável estática
volatile uint32_t BLDCMotor::_speed_pulse = 0;

BLDCMotor::BLDCMotor() : _speed_mps(0), _speed_rps(0), _speed(0)
{
    // Inicializa todos os pinos como NC (Não Conectado)
    _pins = {NC, NC, NC, NC, NC, NC};
    _characteristics = {0, 0};
}

BLDCMotor::~BLDCMotor()
{
    // Para o motor ao destruir o objeto
    setStop();
}

void BLDCMotor::begin(const Pins &pins, const Characteristics &characteristics)
{
    // Atribui os pinos de controle
    _pins.pwm = pins.pwm;
    _pins.fwr = pins.fwr;
    _pins.en = pins.en;
    _pins.brk = pins.brk;
    // Atribui os pinos de sensor
    _pins.spd = pins.spd;
    _pins.alm = pins.alm;

    // Configura os pinos de controle
    pinMode(_pins.pwm, OUTPUT);
    pinMode(_pins.fwr, OUTPUT);
    pinMode(_pins.en, OUTPUT);
    pinMode(_pins.brk, OUTPUT);

    // Configura os pinos de sensor
    if (_pins.spd != NC)
    {
        pinMode(_pins.spd, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(_pins.spd),
                        handleSpeedInterrupt, RISING);
    }

    if (_pins.alm != NC)
    {
        pinMode(_pins.alm, INPUT_PULLUP);
    }

    // Armazena os parâmetros do motor
    _characteristics.wheelDiameter = characteristics.wheelDiameter;
    _characteristics.ppr = characteristics.ppr;

    // Inicializa o motor parado
    setStop();
}

void BLDCMotor::setSpeed(int speed)
{
    // Limita a velocidade entre 0 e 255
    _speed = constrain(speed, 0, 255);

    // Aplica a velocidade ao pino PWM
    if (_pins.pwm != NC)
    {
        analogWrite(_pins.pwm, _speed);
    }
}

void BLDCMotor::setDirection(Direction dir, int8_t speed)
{
    if (speed >= 0)
    {
        setSpeed(speed);
    }

    switch (dir)
    {
    case Direction::FORWARD:
        digitalWrite(_pins.fwr, HIGH);
        digitalWrite(_pins.en, HIGH);
        digitalWrite(_pins.brk, LOW);
        break;

    case Direction::REVERSE:
        digitalWrite(_pins.fwr, LOW);
        digitalWrite(_pins.en, HIGH);
        digitalWrite(_pins.brk, LOW);
        break;

    case Direction::BRAKE:
        digitalWrite(_pins.en, LOW);
        digitalWrite(_pins.brk, HIGH);
        setSpeed(0);
        break;

    case Direction::COAST:
        digitalWrite(_pins.en, LOW);
        digitalWrite(_pins.brk, LOW);
        setSpeed(0);
        break;
    }
}

float BLDCMotor::getSpeedMPS() const
{
    return _speed_mps;
}

float BLDCMotor::getRPM() const
{
    return _speed_rps * 60.0f; // Converte de rotações por segundo para RPM
}

bool BLDCMotor::isFault() const
{
    if (_pins.alm == NC)
    {
        return false;
    }
    return digitalRead(_pins.alm) == LOW;
}

void BLDCMotor::setStop()
{
    setDirection(Direction::BRAKE);
    _speed = 0;
}

void BLDCMotor::rampToSpeed(uint8_t target, uint16_t duration)
{
    if (duration == 0)
    {
        setSpeed(target);
        return;
    }

    uint8_t start = _speed;
    uint16_t steps = duration / 10; // Ajuste o passo conforme necessário
    float increment = (target - start) / (float)steps;

    for (uint16_t i = 0; i < steps; i++)
    {
        setSpeed(start + (increment * i));
        delay(10); // Ajuste o delay conforme necessário
    }

    setSpeed(target); // Garante que chegamos no valor exato
}

void BLDCMotor::handleSpeedInterrupt()
{
    _speed_pulse = _speed_pulse + 1;
}

void BLDCMotor::calculateSpeed()
{
    static uint32_t last_pulse = 0;
    static unsigned long last_time = millis();

    uint32_t current_pulse = _speed_pulse;
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - last_time;

    if (elapsed > 0)
    {
        float pulses_per_second = (current_pulse - last_pulse) * 1000.0f / elapsed;
        _speed_rps = pulses_per_second / _characteristics.ppr;
        _speed_mps = _speed_rps * (PI * _characteristics.wheelDiameter / 1000.0f); // Converte diâmetro de mm para m

        last_pulse = current_pulse;
        last_time = current_time;
    }
}