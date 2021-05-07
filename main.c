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

#define LCD_PORTD PORTD
#define LCD_PIND DDRD
#define RS PD2
#define E PD3

void setup(){
	initUSART();
	lcd_setup();
	clear_lcd();
	
	
	//Interrupt on P0 to clear LCD
	DDRB = 0b00000010;
	PORTB = (1<<PORTB0); //Pullup on B0
	EIMSK = (1<<INT0); // Enable INT0
	PCICR = (1<<PCIE0); // Enable pin change mask 0 reg
	PCMSK0 = (1<<PCINT0); // Enable Int on B0 in Pin change mask 0 register
	EICRA = (1<<ISC01) | (1<<ISC00); // EICRA sense control, positive flank
	

	//ADC
	ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
	ADMUX = (1<<REFS0); //ADC on 0000
	//ADMUX = (0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0); FOR ADC 2
	
	// timer prescaler
	TCCR1A|=(1<<COM1A0)|(0<<WGM10); //CTC
	TCCR1B|=(1<<CS12)|(0<<CS11)|(1<<CS10)|(0<<WGM13)|(1<<WGM12); //CTC and 1024 prescaler
	
	//Enable interrupts
	sei();
}



int main(void)
{
	setup();
	printString("setup complete \n");
	write_to_lcd("setup complete");	
	_delay_ms(1000);
	int toggle = 0;
	
    while (1) 
    {
		clear_lcd();
		_delay_ms(2);
		//clear_lcd();
		
		char adcVAL [16];

		write_to_lcd("ADC Value 1: ");
		printString("ADC Value 1: ");

		int val = read_ADC();
		itoa (val,adcVAL,10);
		
		//Where going to use a temp sensor for a second ADC comperator, but i broke it and a normal potmeter has taken it's place as a backup.
		//While not shown here, it's switched too by changing the ADMUX address bits.

		lcd_func(0xC0);	

		write_to_lcd(itoa(val));
		printWord(val);
		_delay_ms(100);
		printString("\n");
		
		//LED CTC styrt av ADC. 2hz at max ADC (1023)
		ICR1 = (F_CPU / val /2);
		OCR1A = (F_CPU / val /2);
		
		_delay_ms(1000);

		
		
		//Output
		//ADC_LED(leds);

	}
}

//Simple function to read the ADC
int read_ADC(){
	//ADC
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	int val = ADC;
	return val;
}


//This interrupt will interrupt the program and run main in case the LCD starts showing gibberish. Triggered by PB0
ISR(INT0_vect){
	if(PINB & (0 << PINB0)){
	}
}


//Setup LCD in 4 bit mode.
void lcd_setup(){
	LCD_PIND = 0xFF;
	_delay_ms(15);
	lcd_func(0x02);		//4-Bit Control
	lcd_func(0x28);     //Control Matrix @ 4-Bit
	lcd_func(0x0c);     //Disable Cursor
	lcd_func(0x06);     //Move Cursor
	clear_lcd();
	_delay_ms(2);
}

void lcd_func(unsigned char action){
	LCD_PORTD = (LCD_PORTD & 0x0F) | (action & 0xF0);
	LCD_PORTD &= ~(1<<RS);
	lcd_helper(action);
}

void write_to_lcd(char *str){
	int i;
	for(i=0; str[i]!=0; i++)
	{
		LCD_PORTD = (LCD_PORTD & 0x0F) | (str[i] & 0xF0);
		LCD_PORTD |= (1<<RS);
		lcd_helper(str[i]);
	}
}

void lcd_helper(unsigned char _char){
	LCD_PORTD |= (1<<E);
	_delay_us(1);
	LCD_PORTD &= ~(1<<E);
	_delay_us(200);
	LCD_PORTD = (LCD_PORTD & 0x0F) | (_char << 4); //Using 4 bits of LCD controller
	LCD_PORTD |= (1<<E);
	_delay_us(1);
	LCD_PORTD &= ~ (1<<E);
	_delay_ms(2);
	//https://www.aeq-web.com/atmega328-4bit-16x2-lcd-display-amtel-studio-c/?lang=en
}

void clear_lcd(){
	lcd_func(0x01); //Clear
	_delay_ms(2);
	lcd_func(0x80); //move to start
}