#include "BLDCMotor.h"

// Inicialização da variável estática
volatile uint32_t BLDCMotor::_speed_pulse = 0;

/**
 * @brief Construtor padrão do motor.
 *
 * Inicializa todos os pinos como NC (Não Conectado) e as características do motor como 0.
 */
BLDCMotor::BLDCMotor() : _speed_mps(0), _speed_rps(0), _speed(0)
{
    // Inicializa todos os pinos como NC (Não Conectado)
    _pins = {NC, NC, NC, NC, NC, NC};
    _characteristics = {0, 0};
}

/**
 * @brief Destrutor do motor.
 *
 * Para o motor ao destruir o objeto e remove as configurações
 * dos pinos de controle e sensor.
 */
BLDCMotor::~BLDCMotor()
{
    // Para o motor ao destruir o objeto
    setStop();
}

/**
 * @brief Inicializa o motor com os parâmetros de controle e características.
 *
 * Inicializa os pinos de controle e sensor, configura os pinos de controle como saídas
 * e os pinos de sensor como entradas com pull-up. Além disso, armazena os parâmetros do motor
 * e inicializa o motor parado.
 *
 * @param pins Parâmetros de controle do motor.
 * @param characteristics Parâmetros de características do motor.
 */
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

/**
 * @brief Controla a velocidade do motor.
 *
 * Este método limita a velocidade entre 0 e 255 e aplica a
 * velocidade ao pino PWM do motor.
 *
 * @param speed Velocidade do motor de 0 a 255.
 */
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

/**
 * @brief Controla a direção do motor e opcionalmente a velocidade.
 *
 * @param dir Direção do motor: FORWARD, REVERSE, BRAKE ou COAST.
 * @param speed Opcional. Velocidade do motor de 0 a 255.
 *
 * Caso a velocidade seja positiva, o motor é setado para a velocidade
 * especificada. A direção do motor é então setada para a direção
 * especificada. Caso a velocidade seja negativa, a velocidade não é alterada.
 */
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

/**
 * @brief Retorna a velocidade atual do motor em metros por segundo.
 *
 * Este método fornece a velocidade linear do motor com base no diâmetro da roda
 * e nos cálculos de velocidade realizados. A velocidade é atualizada
 * periodicamente por outro método que lida com os pulsos do sensor de velocidade.
 *
 * @return A velocidade do motor em metros por segundo.
 */
float BLDCMotor::getSpeedMPS() const
{
    return _speed_mps;
}

/**
 * @brief Retorna a velocidade atual do motor em rotações por minuto (RPM).
 *
 * Este método fornece a velocidade angular do motor com base na velocidade
 * linear calculada. A velocidade é atualizada periodicamente por outro
 * método que lida com os pulsos do sensor de velocidade.
 *
 * @return A velocidade do motor em rotações por minuto.
 */
float BLDCMotor::getRPM() const
{
    return _speed_rps * 60.0f; // Converte de rotações por segundo para RPM
}

/**
 * @brief Verifica se o motor está em estado de falha.
 *
 * Este método verifica o estado do pino de alarme do motor. Se o pino
 * estiver em nível baixo, o motor está em estado de falha. Se o pino
 * estiver desconectado (NC), o motor não está em estado de falha.
 *
 * @return True se o motor estiver em estado de falha, false caso contrário.
 */
bool BLDCMotor::isFault() const
{
    if (_pins.alm == NC)
    {
        return false;
    }
    return digitalRead(_pins.alm) == LOW;
}

/**
 * @brief Para o motor.
 *
 * Muda a direção do motor para BRAKE e define a velocidade para 0.
 */
void BLDCMotor::setStop()
{
    setDirection(Direction::BRAKE, 0);
}

/**
 * @brief Rampa a velocidade do motor de forma suave.
 *
 * @param target Velocidade alvo (0-255)
 * @param duration Duração da rampa em milissegundos
 *
 * O método começa a partir da velocidade atual e vai gradualmente
 * aumentando ou diminuindo a velocidade até que a velocidade alvo seja
 * alcançada. Caso a duração não seja de 0, a velocidade é alterada imediatamente.
 */
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

/**
 * @brief Trata o pulso do sensor de velocidade.
 *
 * Incrementa um contador de pulsos toda vez que o sensor de velocidade
 * detecta um pulso. Este contador é utilizado para calcular a velocidade
 * do motor em metros por segundo e rotações por segundo.
 */
void BLDCMotor::handleSpeedInterrupt()
{
    _speed_pulse = _speed_pulse + 1;
}

/**
 * @brief Atualiza a velocidade do motor em metros por segundo e rotações por segundo.
 *
 * Este método calcula a velocidade atual do motor com base nos pulsos do sensor de velocidade
 * desde a última atualização. A velocidade em rotações por segundo é calculada dividindo
 * os pulsos por segundo pelo número de pulsos por rotação (ppr). A velocidade em metros por
 * segundo é calculada usando o diâmetro da roda. O método também armazena o tempo e pulsos
 * atuais para a próxima atualização.
 */
void BLDCMotor::updateSpeed()
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