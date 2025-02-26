#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "inc/ssd1306.h"

// Definições dos pinos
#define JOY_X 26  // Eixo X (Esquerda/Direita)
#define JOY_Y 27  // Eixo Y (Cima/Baixo)
#define BUZZER 10 // Pino do buzzer

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define DEADZONE 700  // Zona morta para evitar pequenas variações
#define ADC_MAX 4096  // Resolução do ADC (12 bits)
#define VREF 3.3      // Tensão de referência do ADC
#define TEMP_THRESHOLD 50.0  // Temperatura limite para ativar o buzzer

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
ssd1306_t ssd;

// Variáveis para controle da temperatura
float temperature_offset = 0.0;
float temperature = 25.0;

// Função para configurar o PWM no pino do buzzer
void setup_buzzer_pwm(uint freq) {
    gpio_set_function(BUZZER, GPIO_FUNC_PWM);  // Configura o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(BUZZER); // Obtém o slice do PWM para o pino
    
    // Define o período para obter a frequência desejada
    uint32_t clock = clock_get_hz(clk_sys);  // Obtém a frequência do sistema (normalmente 125 MHz)
    uint32_t divider16 = clock / (freq * 125);  // Calcula o divisor de frequência
    pwm_set_clkdiv(slice_num, divider16 / 16.0);  // Define o divisor de clock
    pwm_set_wrap(slice_num, 125);  // Define o período do PWM
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 62); // Define 50% de duty cycle
    pwm_set_enabled(slice_num, true);  // Liga o PWM
}

// Função para desligar o buzzer
void stop_buzzer() {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER);
    pwm_set_enabled(slice_num, false);  // Desativa o PWM
    gpio_set_function(BUZZER, GPIO_FUNC_NULL);  // Configura o pino como pino normal (sem PWM)
    gpio_init(BUZZER);  // Inicializa o pino como saída digital
    gpio_put(BUZZER, 0);  // Garante que o pino tenha nível baixo
}

// Função para ler o ADC e calcular a média de várias leituras
float read_adc_average(uint input, int samples) {
    adc_select_input(input);
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += adc_read();
        sleep_ms(5);
    }
    return sum / samples;
}

// Função para ler a temperatura do sensor interno do RP2040
float read_temperature() {
    float raw_avg = read_adc_average(4, 10);
    float voltage = (raw_avg * VREF) / ADC_MAX;
    return 27 - (voltage - 0.706) / 0.001721;
}

// Função para ativar bipes intermitentes no buzzer se a temperatura ultrapassar o limite
void buzzer_alert() {
    if (temperature > TEMP_THRESHOLD) {
        setup_buzzer_pwm(300);  // Define a frequência para 300 Hz (som mais grave)
        sleep_ms(200);  // Som ativo por 200ms
        stop_buzzer();  // Desliga o som
        sleep_ms(300);  // Pausa de 300ms antes do próximo bip
    }
}

// Função para ler o joystick e ajustar a temperatura
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


void enviar_mensagem(const char *mensagem) {
    printf("%s\n", mensagem);  // Envia para USB (printf redireciona para USB no Pico SDK)
    uart_puts(UART_ID, mensagem);  // Envia para UART
    uart_puts(UART_ID, "\n");
}

void ssd1306_update_display(ssd1306_t *ssd) {
    i2c_write_blocking(ssd->i2c_port, ssd->address, ssd->ram_buffer, ssd->bufsize, false);
}


int main() {

    stdio_init_all();

    
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Define a função GPIO para I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Define a função GPIO para I2C
    gpio_pull_up(I2C_SDA); // Puxa a linha de dados
    gpio_pull_up(I2C_SCL); // Puxa a linha de relógio

     // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    adc_gpio_init(JOY_X);
    adc_gpio_init(JOY_Y);

    gpio_set_function(BUZZER, GPIO_FUNC_PWM);

    while (true) {
        temperature = read_temperature() + temperature_offset;

        read_joystick_and_update_temperature();

        buzzer_alert();  // Emite bipes se a temperatura passar de 50°C

        if (temperature > TEMP_THRESHOLD) {
            enviar_mensagem("⚠️ ALERTA: Temperatura acima de 50°C!");
        }

        // Exibe a temperatura no display OLED
        char temp_str[16];
        sprintf(temp_str, "Temp: %.2fC", temperature);
        ssd1306_fill(&ssd, false); // Limpa o display
        ssd1306_draw_string(&ssd, temp_str, 10, 30); // Desenha a string no display
        ssd1306_send_data(&ssd); // Atualiza o display

        printf("Simulated Temperature: %.2f C\n", temperature);

        sleep_ms(500);  // Aguarda 500ms antes da próxima leitura
    }
}
