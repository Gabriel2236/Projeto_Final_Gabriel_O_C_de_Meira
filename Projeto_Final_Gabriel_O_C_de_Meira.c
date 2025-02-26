#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "inc/ssd1306.h"

// Definição dos pinos
#define led_r 13
#define JOY_X 26  // Pino do eixo X do joystick
#define JOY_Y 27  // Pino do eixo Y do joystick
#define BUZZER 10 // Pino do buzzer

// Configuração da UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Definições para o ADC e temperatura
#define DEADZONE 700   // Zona morta do joystick
#define ADC_MAX 4096   // Resolução do ADC (12 bits)
#define VREF 3.3       // Tensão de referência do ADC
#define TEMP_THRESHOLD 50.0  // Temperatura limite para alerta

// Configuração do display OLED via I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
ssd1306_t ssd;

// Variáveis para temperatura
float temperature_offset = 0.0;
float temperature = 25.0;

// Configura o PWM para o buzzer
void setup_buzzer_pwm(uint freq) {
    gpio_set_function(BUZZER, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER);
    
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider16 = clock / (freq * 125);
    pwm_set_clkdiv(slice_num, divider16 / 16.0);
    pwm_set_wrap(slice_num, 125);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 62);
    pwm_set_enabled(slice_num, true);
}

// Desativa o buzzer
void stop_buzzer() {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER);
    pwm_set_enabled(slice_num, false);
    gpio_set_function(BUZZER, GPIO_FUNC_NULL);
    gpio_init(BUZZER);
    gpio_put(BUZZER, 0);
}

// Lê o ADC e calcula a média
float read_adc_average(uint input, int samples) {
    adc_select_input(input);
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += adc_read();
        sleep_ms(5);
    }
    return sum / samples;
}

// Lê a temperatura do sensor interno do RP2040
float read_temperature() {
    float raw_avg = read_adc_average(4, 10);
    float voltage = (raw_avg * VREF) / ADC_MAX;
    return 27 - (voltage - 0.706) / 0.001721;
}

// Ativa o alerta de temperatura no buzzer e LED
void buzzer_alert() {
    if (temperature > TEMP_THRESHOLD) {
        setup_buzzer_pwm(300);
        gpio_put(led_r, true);
        sleep_ms(200);
        gpio_put(led_r, false);
        stop_buzzer();
        sleep_ms(300);
    }
}

// Lê o joystick e ajusta a temperatura
void read_joystick_and_update_temperature() {
    float x_value = read_adc_average(1, 5);
    float y_value = read_adc_average(0, 5);

    float x_voltage = x_value * VREF / ADC_MAX;
    float y_voltage = y_value * VREF / ADC_MAX;

    int16_t joy_x_offset = (x_voltage - 2.048) * 1000;

    if (joy_x_offset > DEADZONE) {
        temperature_offset += 1.0;
    } else if (joy_x_offset < -DEADZONE) {
        temperature_offset -= 1.0;
    }

    printf("Joystick X: %.2fV, Y: %.2fV, Temperature Offset: %.2f\n", x_voltage, y_voltage, temperature_offset);
}

// Envia mensagem por UART e USB
void enviar_mensagem(const char *mensagem) {
    printf("%s\n", mensagem);
    uart_puts(UART_ID, mensagem);
    uart_puts(UART_ID, "\n");
}

// Atualiza o display OLED
void ssd1306_update_display(ssd1306_t *ssd) {
    i2c_write_blocking(ssd->i2c_port, ssd->address, ssd->ram_buffer, ssd->bufsize, false);
}

int main() {
    // Inicializa entrada e saída padrão
    stdio_init_all();

    // Configuração do LED vermelho
    gpio_init(led_r);
    gpio_set_dir(led_r, GPIO_OUT);
    gpio_put(led_r, 0);

    // Inicialização do barramento I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicialização do display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Configuração do ADC e sensores
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(JOY_X);
    adc_gpio_init(JOY_Y);

    // Configuração do buzzer
    gpio_set_function(BUZZER, GPIO_FUNC_PWM);

    // Loop principal
    while (true) {
        // Lê a temperatura e aplica o offset
        temperature = read_temperature() + temperature_offset;
        
        // Lê o joystick e atualiza a temperatura
        read_joystick_and_update_temperature();
        
        // Ativa alerta se a temperatura estiver acima do limite
        buzzer_alert();

        // Envia alerta por UART e USB se necessário
        if (temperature > TEMP_THRESHOLD) {
            enviar_mensagem("⚠️ ALERTA: Temperatura acima de 50°C!");
        }

        // Atualiza o display OLED com a temperatura
        char temp_str[16];
        sprintf(temp_str, "Temp: %.2fC", temperature);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, temp_str, 10, 30);
        ssd1306_send_data(&ssd);

        // Exibe a temperatura no terminal
        printf("Simulated Temperature: %.2f C\n", temperature);
        
        // Aguarda 500ms antes da próxima leitura
        sleep_ms(500);
    }
}
