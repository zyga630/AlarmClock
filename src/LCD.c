#include "LCD.h"
#include <stdlib.h>
#include <string.h>

#define LENGTH 2

void lcd_init(void){
	DATA_PORT->ODR = 0;
	lcd_func(SET_FUNCTION, 1, 1, 0);
	lcd_func(DISPLAY_CONTROL, 1, 0, 0);
	lcd_func(ENTRY_MODE, 1, 0, 0);
	lcd_clear();
	lcd_return();

}

static char tab[LENGTH] = {0};

char* int_to_char(int number){
	
	int n = number;
	
	for(int i = 0; i < LENGTH; i++){
			tab[LENGTH-i] = (n % 10) + 48;
			n /= 10;
	}
	//tab[10] = tab[9];
	//tab[9] = '.';
	return tab;
}

void lcd_clear(void){
	lcd_func(CLEAR, 0, 0, 0);
}

void lcd_return(void){
	lcd_func(RETURN_HOME, 0, 0, 0);
}

void lcd_func(DisplayFunction func,int arg0,int arg1,int arg2){
	int _param0 = ((arg0 > 0) ? 1 : 0);
	int _param1 = ((arg1 > 0) ? 1 : 0);
	int _param2 = ((arg2 > 0) ? 1 : 0);
	int data = 0;
	
	switch(func){
		case CLEAR:
			data = DB0;
		break;
		case RETURN_HOME:
			data = DB1;
		break;
		case ENTRY_MODE:
			data = DB2 | _param0*DB1 | _param1*DB0;
		break;
		case DISPLAY_CONTROL:
			data = DB3 | _param0*DB2 | _param1*DB1 | _param2*DB0;
		break;
		case CURSOR_SHIFT:
			data = DB4 | _param0*DB3 | _param1*DB2;
		break;
		case SET_FUNCTION:
			data = DB5 | _param0*DB4 | _param1*DB3 |_param2;
		break;
	}
	HAL_GPIO_WritePin(OP_PORT, RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OP_PORT, EN, GPIO_PIN_SET);
	DATA_PORT->ODR = data;
	HAL_Delay(1);
	HAL_GPIO_WritePin(OP_PORT, EN, GPIO_PIN_RESET);
	HAL_Delay(1);
	
};

void lcd_CursorShift(int dir, int n){
	for (int i = 0; i < n; i++)
		lcd_func(CURSOR_SHIFT, 0, dir, 0);
}
void lcd_setCGRAM(uint8_t address){
	int addr = address & 63;
	
	HAL_GPIO_WritePin(OP_PORT, EN, GPIO_PIN_SET);
	DATA_PORT->ODR = DB6 | addr;
	HAL_Delay(1);
	HAL_GPIO_WritePin(OP_PORT, EN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(OP_PORT, RS, GPIO_PIN_SET);
}

void lcd_write(const char* text){	
	if (text!=NULL){
		int length = strlen(text);
		HAL_GPIO_WritePin(OP_PORT, RS, GPIO_PIN_SET);
		for (int i = 0; i < length; i++){
			HAL_GPIO_WritePin(OP_PORT, EN, GPIO_PIN_SET);
			DATA_PORT->ODR =*(text+i);
			for(int i = 0; i < 450; i++);
			HAL_GPIO_WritePin(OP_PORT, EN, GPIO_PIN_RESET);
			for(int i = 0; i < 100; i++);
		}

	}
}
