# BLDCMotor Library  
## Biblioteca de Controle para Motores BLDC  
![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-brightgreen)
![Licenca](https://img.shields.io/badge/License-MIT-green)
![Versao](https://img.shields.io/badge/Version-1.0.0-blue)

Biblioteca completa para PlatformIO com controle de motores BLDC e sistema integrado de logging usando LogLibrary.

## 📦 Funcionalidades  
🚀 Controle completo de motor BLDC (PWM, direção, frenagem e neutro)

📈 Aceleração/desaceleração suave com rampToSpeed()

📊 Monitoramento em tempo real (m/s e RPM)

⚠️ Sistema de detecção de falhas

📝 Logging integrado via LogLibrary

🔄 Suporte a encoder para controle em malha fechada

## 📥 Instalação  
Usando PlatformIO  
Adicione no seu platformio.ini:  

```ini
lib_deps =
    https://github.com/cturqueti/BLDCMotor.git
    https://github.com/cturqueti/LogLibrary.git
```
Ou instale via CLI:  

```bash
pio pkg install --library "cturqueti/BLDCMotor@^1.0.0"
pio pkg install --library "cturqueti/LogLibrary@^1.0.0"
```
---
## 🛠️ Estrutura da Classe

```cpp
BLDCMotor motor;
motor.begin(pins, characteristics);
motor.setDirection(BLDCMotor::Direction::FORWARD);
motor.setSpeed(128);
```
### Estrutura 'Pins'
```cpp
BLDCMotor::Pins {
  uint8_t pwm; // Pino PWM (velocidade)
  uint8_t fwr; // Direção
  uint8_t en;  // Enable
  uint8_t brk; // Freio
  uint8_t spd; // Sensor de velocidade (opcional)
  uint8_t alm; // Alarme (opcional)
};
```
### Estrutura 'Characteristics'
```cpp
BLDCMotor::Characteristics {
  float wheelDiameter; // Diâmetro da roda em mm
  uint8_t ppr;         // Pulsos por rotação
};
```
## ⚙️ Configuração de Hardware  
Diagrama de pinos  
|Função	| Tipo do Pino	| Descrição |
|-------------|-------------|-------------|
|PWM	| Saída (PWM)	| Controle de velocidade |
|FWR	| Saída	| Controle de direção |
|EN	| Saída	| Habilitação do motor |
|BRK	| Saída	| Controle de freio |
|SPD	| Entrada	| Sensor de velocidade |
|ALM	| Entrada	| Detecção de falhas |

## 🚀 Começo Rápido  
```cpp
#include <BLDCMotor.h>
#include <LogLibrary.h>

BLDCMotor motor;

void setup() {
    BLDCMotor::Pins pins = {
      .pwm = 23,
      .fwr = 19,
      .en = 18,
      .brk = 5,
      .spd = NC,
      .alm = NC
    };
    BLDCMotor::Characteristics characteristics = {65.0, 20};

    motor.begin(pins, characteristics);
    motor.setDirection(BLDCMotor::Direction::FORWARD);
    motor.setSpeed(150);
}

void loop() {
    // Nada aqui
}

```
## 📚 Referência da API  
### Controle do Motor  
```cpp
void begin(const Pins &pins, const Characteristics &characteristics);
void setSpeed(uint8_t speed);  // 0-255
void setDirection(Direction dir, int8_t speed = -1);
void rampToSpeed(uint8_t target, uint16_t duration);
void setStop();
```
### Telemetria
```cpp
float getSpeedMPS() const;  // Velocidade em m/s
float getRPM() const;       // Velocidade em RPM
bool isFault() const;       // Status de falha
void calculateSpeed();      // Atualiza cálculos de velocidade
```
### Logging Integrado
```cpp
// Use estas macros no seu código:
LOG_DEBUG("Mensagem de debug");
LOG_INFO("Velocidade: %.2f m/s", motor.getSpeedMPS());
LOG_WARN("Aviso importante");
LOG_ERROR("Código de erro: %d", error);
```
## 🔧 Configurações  
### Logging  
Configure no arquivo principal:  

```cpp
#define CURRENT_LOG_LEVEL LOG_LEVEL_DEBUG  // DEBUG, INFO, WARNING, ERROR

// Configurações opcionais
Log.enableColors(true);    // Saída colorida
Log.enableNewline(true);   // Quebras de linha automáticas
```
### Parâmetros do Motor  
Ajuste no begin():

```cpp
motor.begin(pins, characteristics);
```
diameter: Diâmetro da roda em mm

ppr: Pulsos por revolução do encoder

## 📊 Exemplos Práticos  
1. Monitoramento de Velocidade
```cpp
void loop() {
  static uint32_t lastUpdate = 0;
  if(millis() - lastUpdate > 100) {
    motor.calculateSpeed();
    LOG_INFO("RPM: %.1f | Velocidade: %.2f m/s", motor.getRPM(), motor.getSpeedMPS());
    lastUpdate = millis();
  }
}
```
2. Sistema de Recuperação de Falhas
```cpp
void loop() {
  if(motor.isFault()) {
    LOG_ERROR("Falha detectada! Tentando recuperação...");
    motor.setStop();
    delay(1000);
    motor.setDirection(BLDCMotor::Direction::FORWARD, 100);
  }
}
```
## ⚠️ Solução de Problemas  
| Problema	| Solução |  
|-------------|-------------|
| Motor não responde	| Verifique alimentação e pino EN |
| Leituras de velocidade incorretas	| Confira valor PPR e conexões do encoder |
| Logs não aparecem	| Verifique CURRENT_LOG_LEVEL |
| Saída serial ilegível	| Confira baud rate (normalmente 115200) |  
## 📜 Licença  
MIT License - Veja LICENSE para detalhes.  

---
💡 Dica Profissional: Para produção, defina CURRENT_LOG_LEVEL como LOG_LEVEL_WARNING ou superior para reduzir overhead. Use LOG_DEBUG apenas durante desenvolvimento.