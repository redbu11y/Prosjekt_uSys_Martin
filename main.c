/*
 * Prosjekt_uSys_Martin.c
 *
 * Created: 06.05.2021 10:21:03
 * Author : Martin Selsoyvold
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>


//LCD
#define LCD_PORTD PORTD 
#define LCD_PIND DDRD
#define RX PD0
#define TX PD1
//I2C ultra sonic
//#define trig PB4  //SCL
//#define echo PB5 //SDL
#define HC_SR04_Address 0x57
//Other
#define Button PB1
#define R_LED PB2
#define G_LED PB3
#define POTMETER PB6


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
	clock_prescale_set(clock_div_1); 
	lcd_setup();
	I2C_Setup();
	
	write_to_lcd("test");	//Begin writing at Line 1, Position 1
	
	
    while (1) 
    {
		I2C_Start();
		I2C_Send(HC_SR04_Address);
		int HighByte = I2C_Read_High();
		int LowByte = I2C_Read_Low();
		lcd_func(0x80);		//Go to Line 1 Pos 1
		write_to_lcd(LowByte);
		lcd_func(0xC0);		//Go to Line 2, Position 1
		write_to_lcd(HighByte);
		I2C_Stop();

		_delay_ms(1000);
		timeout_timer++;
    }
}


/*
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
*/

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
