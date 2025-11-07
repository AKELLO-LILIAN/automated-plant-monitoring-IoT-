#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// Pin definitions (assuming Arduino Uno / ATmega328P)
#define DHTPIN PD2          // Digital pin 2 (Port D, bit 2)
#define LED_PIN PB5         // Digital pin 13 (Port B, bit 5)
#define MOISTURE_PIN 0      // Analog pin A0 (ADC0)
#define RELAY_PIN PD7       //

// Thresholds
#define TEMP_THRESHOLD 100 //250  // 25.0°C * 10 (stored as integer)
#define HUM_THRESHOLD 900   // 60.0% * 10 (stored as integer)
#define MOISTURE_THRESHOLD 90   //600  //

#define EXTREME_HIGH_MOISTURE 900

// DHT11 timing constants (microseconds)
#define DHT_START_SIGNAL 18000  // 18ms low signal
#define DHT_WAIT_RESPONSE 40    // Wait 40us for response
#define DHT_TIMEOUT 255

// Function prototypes
void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_print_int(int16_t num);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
uint8_t dht11_read(int16_t *temperature, int16_t *humidity);
void delay_us(uint16_t us);

// UART functions for debugging
void uart_init(void) {
    // Set baud rate to 9600 (for 16MHz clock: UBRR = F_CPU/16/BAUD-1)
    //#define BAUD 9600
    //#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)
    
    //UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    //UBRR0L = (uint8_t)UBRR_VALUE;
    UBRR0H = 0x00;
    UBRR0L = 0x67;
    
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

void uart_print_int(int16_t num) {
    char buffer[7];
    sprintf(buffer, "%d", num);
    uart_puts(buffer);
}

// ADC initialization for soil moisture sensor
void adc_init(void) {
    // Set reference voltage to AVCC with external capacitor at AREF
    ADMUX = (1 << REFS0);
    
    // Enable ADC, set prescaler to 128 (16MHz/128 = 125kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
    // Select ADC channel (0-7)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    return ADC;
}

// Microsecond delay function
void delay_us(uint16_t us) {
    while (us--) {
        _delay_us(1);
    }
}

// DHT11 read function
uint8_t dht11_read(int16_t *temperature, int16_t *humidity) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t i, j;
    uint8_t timeout;
    
    // Send start signal: LOW for 18ms
    DDRD |= (1 << DHTPIN);   // Set as output
    PORTD &= ~(1 << DHTPIN); // Pull low
    _delay_ms(18);
    
    PORTD |= (1 << DHTPIN);  // Pull high
    delay_us(30);
    
    // Switch to input
    DDRD &= ~(1 << DHTPIN);  // Set as input
    PORTD |= (1 << DHTPIN);  // Enable pull-up
    
    // Wait for DHT response (low then high)
    timeout = DHT_TIMEOUT;
    while ((PIND & (1 << DHTPIN)) && timeout--) delay_us(1);
    if (timeout == 0) return 0;
    
    timeout = DHT_TIMEOUT;
    while (!(PIND & (1 << DHTPIN)) && timeout--) delay_us(1);
    if (timeout == 0) return 0;
    
    timeout = DHT_TIMEOUT;
    while ((PIND & (1 << DHTPIN)) && timeout--) delay_us(1);
    if (timeout == 0) return 0;


    
    // Read 40 bits (5 bytes)
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            // Wait for low to go high
            timeout = DHT_TIMEOUT;
            while (!(PIND & (1 << DHTPIN)) && timeout--) delay_us(1);
            if (timeout == 0) return 0;
            
            delay_us(40);
            
            // If still high after 40us, it's a '1'
            if (PIND & (1 << DHTPIN)) {
                data[i] |= (1 << (7 - j));
            }
            
            // Wait for pin to go low
            timeout = DHT_TIMEOUT;
            while ((PIND & (1 << DHTPIN)) && timeout--) delay_us(1);
            if (timeout == 0) return 0;
        }
    }
    
    // Verify checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        return 0;
    }
    
    // DHT11 returns integer values (data[0] = humidity, data[2] = temperature)
    *humidity = data[0] * 10;    // Convert to tenths
    *temperature = data[2] * 10; // Convert to tenths
    
    return 1;
}

int main(void) {
    int16_t temperature, humidity;
    uint16_t moisture;
    
    // Initialize UART for debugging
    uart_init();
    
    // Initialize ADC for moisture sensor
    adc_init();
    
    // Set LED pin as output
    DDRB |= (1 << LED_PIN);
    DDRD |= (1 << RELAY_PIN);       // PD7 as OUTPUT (Relay)
    PORTD |= (1 << RELAY_PIN);      // Relay OFF (active LOW)
    PORTB &= ~(1 << LED_PIN); // LED off initially
    
    uart_puts("Irrigation System Started\n");
    
    while (1) {
        // Read DHT11 sensor
        if (dht11_read(&temperature, &humidity)) {
            // Read soil moisture
            moisture = adc_read(MOISTURE_PIN);
            
            // Debug output
            uart_puts("Temperature: ");
            uart_print_int(temperature / 10);
            uart_putc('.');
            uart_print_int(temperature % 10);
            uart_puts(" C | Humidity: ");
            uart_print_int(humidity / 10);
            uart_putc('.');
            uart_print_int(humidity % 10);
            uart_puts(" % | Soil Moisture: ");
            uart_print_int(moisture);
            uart_putc('\n');
            
            // Check irrigation conditions
            if (temperature > TEMP_THRESHOLD && 
                humidity < HUM_THRESHOLD && 
                moisture > MOISTURE_THRESHOLD) {
                PORTB |= (1 << LED_PIN);  // LED ON
                // Pump ON (relay active LOW)
                PORTD &= ~(1 << RELAY_PIN);
                uart_puts("Irrigation triggered! LED ON\n");
            } 
                else if ( 
                moisture >EXTREME_HIGH_MOISTURE) {
                PORTB |= (1 << LED_PIN);  // LED ON
                // Pump ON (relay active LOW)
                PORTD &= ~(1 << RELAY_PIN);
                uart_puts("Irrigation triggered! LED ON\n");
                uart_puts("Soil Too dry\n");
            }
            else {
                PORTB &= ~(1 << LED_PIN); // LED OFF
                uart_puts("Conditions not met. LED OFF\n");
                PORTD |= (1 << RELAY_PIN);      // Relay OFF (active LOW)
            }
        } else {
            uart_puts("Failed to read from DHT sensor!\n");
        }
        
        _delay_ms(2000); // Wait 2 seconds
    }
    
    return 0;
}