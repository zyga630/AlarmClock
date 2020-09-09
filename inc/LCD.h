#include "stm32f1xx_hal.h"

#define OP_PORT 	GPIOB
#define DATA_PORT GPIOA
#define DB0 			GPIO_PIN_0
#define DB1 			GPIO_PIN_1
#define DB2 			GPIO_PIN_2
#define DB3 			GPIO_PIN_3
#define DB4 			GPIO_PIN_4
#define DB5 			GPIO_PIN_5
#define DB6			 	GPIO_PIN_6
#define DB7 			GPIO_PIN_7
#define RS				GPIO_PIN_11
#define EN				GPIO_PIN_10



#define RIGHT 	0
#define LEFT 		1


typedef enum{
	CLEAR, 
	RETURN_HOME,
	ENTRY_MODE,
	DISPLAY_CONTROL, 
	CURSOR_SHIFT,
	SET_FUNCTION
}DisplayFunction;



//Instruction table
//_____________________________________________________________________
// INST	|	RS	|	R/NW	|	DB7	|	DB6	|	DB5	|	DB4	|	DB3	|	DB2	|	DB1	|	DB0	|
//---------------------------------------------------------------------
// Clear|	 0	|		0		|	 0 	|	 0	|	 0	|	 0	|	 0	|	 0	|	 0	|	 1	|
//			|			|				|			|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
//Return|	 0	|		0 	|	 0	|	 0	|	 0	|	 0	|	 0 	|	 0	|	 1 	|	 *	|
// home	|			|				|	 		|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
// Entry|	 0 	|		0 	|	 0	|	 0 	|	 0	|	 0	|	 0	|	 1	|	I/D	|	 S	|
// Mode |			|				|			|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
// Disp |	 0	|		0		|	 0	|  0  |	 0	|  0	|	1 	|	 D 	|	 C	|	 B	|
// ctrl |			|				|			|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
//Cursor|	 0	|		0		|	 0	|	 0	|	 0	|	 1	|	S/C	|	R/L	|	 *	|	 *	|
//shift |			|				|			|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
//Func	|	 0	|		0		|	 0	|	 0	|	 1	|	DL	|	 N	|	 F	|	 *	|	 *	|
//Set		|			|				|			|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
//Set	  |			|				|			|			|			|			|			|			|			|			|
//CGRAM	|	 0	|		0		|	 0	|	 1	|	 A 	|	 A	|	 A	|	 A	|	 A	|	 A	|
//Addr	|			|				|			|			|			|			|			|			|			|			|
//---------------------------------------------------------------------
// Note:  *  = Don't care
//			 I/D = Increment/Decrement
//			  S  = Shift entire display to right(I/D = 0) or left (I/D = 1)
//				D	 = Display on or off
//				C  = Show cursor
//				B	 = Blink indicated character
//			 S/C = shift screen(S) or cursor(C)
//			 R/L = shitf right(R) or left(L)
//			  DL = set interface data lenght. When DL = 0 length = 4, when DL = 1 length = 8
//			  N  = Set the number of display lines
//				F  = sets the character font

void lcd_func(DisplayFunction func,int arg0,int arg1,int arg2);

void lcd_clear(void);

void lcd_return(void);

void lcd_setEntryMode(int dir, int shift_to_edge);  

void lcd_DisplayControl(int display, int cursor, int blink);

//shift Right => dir = 1, shift left => dir = 0
// n => number of shifts
void lcd_CursorShift(int dir, int n);

void lcd_setFunction(int dataLength, int NofLines, int setFont);

void lcd_write(const char* text);

void lcd_init(void);

char* int_to_char(int number);
