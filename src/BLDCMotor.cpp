#include "BLDCMotor.h"

// Inicialização da variável estática
volatile uint32_t BLDCMotor::_speed_pulse = 0;

BLDCMotor::BLDCMotor() : _speed_mps(0), _speed_rps(0), _speed(0), _diameter(0), _ppr(0)
{
    // Inicializa todos os pinos como NC (Não Conectado)
    for (uint8_t i = 0; i < 4; i++)
    {
        _pinControl[i] = NC;
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        _pinSensor[i] = NC;
    }
}

BLDCMotor::~BLDCMotor()
{
    // Para o motor ao destruir o objeto
    setStop();
}

void BLDCMotor::begin(uint8_t pins[6], uint8_t diameter, uint8_t ppr)
{
    // Atribui os pinos de controle
    _pinControl[static_cast<uint8_t>(ControlPins::PWM)] = pins[0];
    _pinControl[static_cast<uint8_t>(ControlPins::FWR)] = pins[1];
    _pinControl[static_cast<uint8_t>(ControlPins::EN)] = pins[2];
    _pinControl[static_cast<uint8_t>(ControlPins::BRK)] = pins[3];

    // Atribui os pinos de sensor
    _pinSensor[static_cast<uint8_t>(SensorPins::SPD)] = pins[4];
    _pinSensor[static_cast<uint8_t>(SensorPins::ALM)] = pins[5];

    // Configura os pinos
    pinMode(_pinControl[static_cast<uint8_t>(ControlPins::PWM)], OUTPUT);
    pinMode(_pinControl[static_cast<uint8_t>(ControlPins::FWR)], OUTPUT);
    pinMode(_pinControl[static_cast<uint8_t>(ControlPins::EN)], OUTPUT);
    pinMode(_pinControl[static_cast<uint8_t>(ControlPins::BRK)], OUTPUT);

    // Configura os pinos de sensor
    if (_pinSensor[static_cast<uint8_t>(SensorPins::SPD)] != NC)
    {
        pinMode(_pinSensor[static_cast<uint8_t>(SensorPins::SPD)], INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(_pinSensor[static_cast<uint8_t>(SensorPins::SPD)]),
                        motInt, RISING);
    }

    if (_pinSensor[static_cast<uint8_t>(SensorPins::ALM)] != NC)
    {
        pinMode(_pinSensor[static_cast<uint8_t>(SensorPins::ALM)], INPUT_PULLUP);
    }

    // Armazena os parâmetros do motor
    _diameter = diameter;
    _ppr = ppr;

    // Inicializa o motor parado
    setStop();
}

void BLDCMotor::setSpeed(int speed)
{
    // Limita a velocidade entre 0 e 255
    _speed = constrain(speed, 0, 255);

    // Aplica a velocidade ao pino PWM
    if (_pinControl[static_cast<uint8_t>(ControlPins::PWM)] != NC)
    {
        analogWrite(_pinControl[static_cast<uint8_t>(ControlPins::PWM)], _speed);
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
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::FWR)], HIGH);
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::EN)], HIGH);
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::BRK)], LOW);
        break;

    case Direction::REVERSE:
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::FWR)], LOW);
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::EN)], HIGH);
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::BRK)], LOW);
        break;

    case Direction::BRAKE:
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::EN)], LOW);
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::BRK)], HIGH);
        _speed = 0;
        break;

    case Direction::COAST:
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::EN)], LOW);
        digitalWrite(_pinControl[static_cast<uint8_t>(ControlPins::BRK)], LOW);
        _speed = 0;
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
    if (_pinSensor[static_cast<uint8_t>(SensorPins::ALM)] == NC)
    {
        return false;
    }
    return digitalRead(_pinSensor[static_cast<uint8_t>(SensorPins::ALM)]) == LOW;
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

void BLDCMotor::motInt()
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
        _speed_rps = pulses_per_second / _ppr;
        _speed_mps = _speed_rps * (PI * _diameter / 1000.0f); // Converte diâmetro de mm para m

        last_pulse = current_pulse;
        last_time = current_time;
    }
}