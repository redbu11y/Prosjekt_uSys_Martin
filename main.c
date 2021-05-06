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
	lcd_setup();
    /* Replace with your application code */
	write_to_lcd("test");	//Begin writing at Line 1, Position 1
    while (1) 
    {
			char showruntime [16];
			itoa (timeout_timer,showruntime,10);
			lcd_func(0xC0);		//Go to Line 2, Position 1
			write_to_lcd("RUNTIME (s): ");
			write_to_lcd(showruntime);
			_delay_ms(1000);
			timeout_timer++;
    }
}

void lcd_setup(){
	LCD_PIND = 0xFF;
	_delay_ms(15);
	lcd_func(0x02);		//4-Bit Control
	lcd_func(0x28);     //Control Matrix @ 4-Bit
	//lcd_func(0x0c);     //Disable Cursor
	//lcd_func(0x06);     //Move Cursor
	clear_lcd();
	_delay_ms(2);
}

void lcd_func(unsigned char action){
	LCD_PORTD = (LCD_PORTD & 0x0F) | (action & 0xF0);
	LCD_PORTD &= ~(1<<RX);
	lcd_helper(action);
}

void write_to_lcd(char *str){
		int i;
		for(i=0; str[i]!=0; i++)
		{
			LCD_PORTD = (LCD_PORTD & 0x0F) | (str[i] & 0xF0);
			LCD_PORTD |= (1<<RX);
			lcd_helper(str[i]);
		}
}

void lcd_helper(unsigned char _char){
		LCD_PORTD |= (1<<TX);
		_delay_us(1);
		LCD_PORTD &= ~(1<<TX);
		_delay_us(200);
		LCD_PORTD = (LCD_PORTD & 0x0F) | (_char << 4); //Using 4 bits of LCD controller
		LCD_PORTD |= (1<<TX);
		_delay_us(1);
		LCD_PORTD &= ~ (1<<TX);
		_delay_ms(2);
		//https://www.aeq-web.com/atmega328-4bit-16x2-lcd-display-amtel-studio-c/?lang=en
}

void clear_lcd(){
	lcd_func(0x01); //Clear
	_delay_ms(2);
	lcd_func(0x80); //move to start
}


int read_ADC(){
	return 1;
}

int read_ultrasonic_i2c(){
	return 1;
}
