/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_LCD_X8_H
#define	XC_LCD_X8_H
#define _XTAL_FREQ   4000000UL 
#include <xc.h> // include processor files - each processor file is guarded..  


#define LCD_TYPE 2

#define LCD_LINE_TWO 0x40    // LCD RAM address for the second line
#define LCD_LINE_SIZE 16

//unsigned char  LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};

struct lcd_pin_map {
    // This structure is overlayed

    unsigned un1 : 1; //unused on to an I/O port to gain, should be cleared
    unsigned rs : 1; // rd1 access to the LCD pins.          
    unsigned rw : 1; // low order up.  ENABLE will
    unsigned enable : 1; // The bits are allocated from
    unsigned data : 4; // be pins d0:,1,2,3

} lcd __at(0xF83); //PORTD   // this is new syntax
//@ 0x0F83; //   ; PORTD     // old Syntax

#define lcd_output_enable(x) PORTEbits.RE1 = x //for simulator
//lcd.enable = x // use this for actual hardware
#define lcd_output_rs(x) PORTEbits.RE2 = x //for simulator
//lcd.rs = x //for simulator
void delay_cycles(unsigned char n);
void delay_ms(unsigned int n);
void lcd_send_nibble(unsigned char n);
void lcd_send_byte(unsigned char cm_data, unsigned char n);
void lcd_init(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_putc(char c);
void lcd_puts(char *s);
void Lcd_Shift_Right(void);
void Lcd_Shift_Left(void);


#endif	/* XC_LCD_X8_H */



