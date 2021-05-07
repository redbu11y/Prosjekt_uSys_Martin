/*
 * Prosjekt_uSys_Martin.c
 *
 * Created: 06.05.2021 10:21:03
 * Author : Martin Selsoyvold
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"
#include <avr/interrupt.h>


//#define LCD_PORTB PORTB
//#define LCD_PORTD PORTD
//#define LCD_PINB DDRB
//#define RS PD0
//#define E PD1

int distance;
int timeout_timer;

static volatile uint32_t first_reading = 0;
static volatile uint32_t second_reading = 0;
static volatile uint32_t duty_cycle = 0;

void setup(){
	cli();
	sei();
	//Setup ADC
	DDRD |= (1<<6);
	DDRB &= ~(1<<DDB0);
	//Timer/Counter Control Register A
	TCCR0B |= (1<<CS01); // Devided by 8 prescaler
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00); //Set OC0A on Compare Match, clear OC0A at BOTTOM,(inverting mode). Fast PWM
	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11);
	TIMSK1 |= (1<<ICIE1);
	OCR0A = 235; //Set PMW pulse 10us
	//USART
}

int ultrasonic_measure(){
	
	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	int range = ((float)duty_cycle * 32768 / 65536) * 0.034 / 2;;
	return range;
}


int main(void)
{
	initUSART();
	I2C_Setup();
	_delay_ms(1000);
	printString("USART Initialized \n");

	//setup();
	HCSR04_Init();
    while (1) 
    {
		int range = ultrasonic_measure();
		printString("range cm: ");
		printWord(range);
		//printString(range);
		printString("\n");
		I2C_Start();
		//printString("I2C START \n");
		I2C_Send(0x10);
		I2C_Send("test");
		//printString("I2C SEND \n");
		I2C_Stop();
		//printString("I2C STOP \n");
		_delay_ms(2000);
		
		

    }
}



void I2C_Setup(){
	TWBR = 32;
	TWCR |= (1 << TWEN);
}
void I2C_Start(){
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  // Step 1: Start TWI/I2C (Clear TWINT flag)
	//while (!(TWCR & (1<<TWINT))){} //Step 2: Wait for TWINT to be set
	loop_until_bit_is_set(TWCR, TWINT);
}
int I2C_Read_High(){
	TWCR = (1<<(TWINT) | (1<<TWEN) | (1<<TWEA));//TWI Enable Acknowledge Bit
	//while (!(TWCR & (1<<TWINT))){}
	loop_until_bit_is_set(TWCR, TWINT);
	return (TWDR);
}
int I2C_Read_Low(){
	TWCR = (1<<(TWINT) | (1<<TWEN));//TWI Enable Acknowledge Bit
	//while (!(TWCR & (1<<TWINT))){}
	loop_until_bit_is_set(TWCR, TWINT);
	return (TWDR);
}
void I2C_Send(int DATA){
	TWDR = DATA;
	TWCR = (1<<TWINT) | (1<<TWEN);
	//while (!(TWCR & (1<<TWINT))){}
	loop_until_bit_is_set(TWCR, TWINT);
}
void I2C_Stop(){
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); //Step 8: Stop I2C
}


int read_ADC(){
	return 1;
}



ISR(TIMER1_CAPT_vect){
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1)){
		first_reading = ICR1;
	}
	else{
		second_reading = ICR1;
	}
	
	if (first_reading != 0 && second_reading != 0){
		duty_cycle = second_reading - first_reading;
		first_reading = 0;
		second_reading = 0;
	}
	
	TCCR1B ^= (1<<ICES1); //toggle edge detection bit
	TIFR1 = (1<<ICF1);//clear Input Capture Flag
}
