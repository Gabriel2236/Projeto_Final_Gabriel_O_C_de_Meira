# Projeto_Final_Gabriel_O_C_de_MEIRA# Projeto de Monitoramento de Temperatura com RP2040

Este projeto implementa um sistema de monitoramento de temperatura usando o microcontrolador RP2040, sensores analógicos e um display OLED. O sistema exibe a temperatura em tempo real e aciona alertas visuais e sonoros quando a temperatura excede um limite pré-determinado.

## Componentes Utilizados

- Microcontrolador: RP2040
- Sensores: Joystick analógico
- Display: OLED via I2C
- Alerta: LED vermelho e buzzer
- Comunicação: UART para envio de alertas

## Definição dos Pinos

- `led_r`: Pino 13, LED vermelho para indicação de alerta
- `JOY_X`: Pino 26, eixo X do joystick
- `JOY_Y`: Pino 27, eixo Y do joystick
- `BUZZER`: Pino 10, buzzer para alerta sonoro
- `UART_TX_PIN`: Pino 0, transmissão UART
- `UART_RX_PIN`: Pino 1, recepção UART
- `I2C_SDA`: Pino 14, linha de dados SDA do I2C
- `I2C_SCL`: Pino 15, linha de clock SCL do I2C

## Configuração do Sistema

1. **UART**:
   - `UART_ID`: `uart0`
   - Taxa de baud: `115200`

2. **ADC e Temperatura**:
   - `DEADZONE`: `700` (zona morta do joystick)
   - `ADC_MAX`: `4096` (resolução do ADC, 12 bits)
   - `VREF`: `3.3V` (tensão de referência do ADC)
   - `TEMP_THRESHOLD`: `50.0°C` (temperatura limite para alerta)

3. **Display OLED via I2C**:
   - `I2C_PORT`: `i2c1`
   - Endereço: `0x3C`

## Funcionalidades

- Leitura da temperatura usando o sensor interno do RP2040.
- Ajuste da temperatura usando joystick.
- Exibição da temperatura no display OLED.
- Envio de alertas via UART e USB quando a temperatura excede o limite.
- Acionamento de alertas visuais (LED) e sonoros (buzzer).

## Funções Principais

- `setup_buzzer_pwm(uint freq)`: Configura o PWM para o buzzer.
- `stop_buzzer()`: Desativa o buzzer.
- `read_adc_average(uint input, int samples)`: Lê o ADC e calcula a média.
- `read_temperature()`: Lê a temperatura do sensor interno do RP2040.
- `buzzer_alert()`: Ativa o alerta de temperatura no buzzer e LED.
- `read_joystick_and_update_temperature()`: Lê o joystick e ajusta a temperatura.
- `enviar_mensagem(const char *mensagem)`: Envia mensagem por UART e USB.
- `ssd1306_update_display(ssd1306_t *ssd)`: Atualiza o display OLED.

## Como Executar

1. Faça o clone do repositório.
2. Compile o código utilizando o ambiente de desenvolvimento apropriado para o RP2040.
3. Conecte os componentes conforme a definição dos pinos.
4. Carregue o código no microcontrolador RP2040.
5. Execute o sistema e monitore a temperatura no display OLED e na interface UART.

## Resultados dos Testes

Os testes realizados demonstraram que o projeto é funcional. A leitura da temperatura apresentou uma leve imprecisão devido ao uso do sensor interno do RP2040. No entanto, os alertas visuais e sonoros foram acionados corretamente quando necessário. A comunicação via UART e USB funcionou conforme esperado, garantindo que os alertas fossem transmitidos. O display OLED forneceu uma interface visual clara para o usuário, permitindo a monitorização em tempo real da temperatura.

## Considerações Finais

Este projeto atende aos objetivos propostos, apesar da leve imprecisão nas leituras de temperatura. Para melhorar a precisão, recomenda-se o uso de sensores externos adicionais e calibrações adicionais.
