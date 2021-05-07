/*
 * Prosjekt_uSys_Martin.c
 *
 * Created: 06.05.2021 10:21:03
 * Author : Martin Selsoyvold
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define LCD_PORTD PORTD 
#define LCD_PIND DDRD
#define RX PD0
#define TX PD1

int distance;
int timeout_timer;

void setup(){
	//Setup timer
	//Setup ADC
	//Setup SPI/I2C
	
	//USART
}

int main(void)
{
    while (1) 
    {

    }
}


int read_ADC(){
	return 1;
}

int read_ultrasonic_i2c(){
	return 1;
}
