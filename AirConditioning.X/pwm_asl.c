/*
 * File:   pwm_asl.c
 * Author: raed
 * PWM + ADC + SERIAL + LCD
 * Created on March 30, 2019, 1:05 PM
 * LCD is set to work on the simulator, must be fixed to work with real
 */


#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
//function prototypes
#define STARTVALUE  3036

unsigned int RPS_count = 0;

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

// This function is needed for measuring speed
void initTimers01(void) {
    T0CON = 0;
    //T0CONbits.T0CS = 0;
    //T0CONbits.PSA = 0;
    //T0CONbits.T08BIT = 1;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);

    T1CONbits.TMR1CS = 1; //external clock ,emasuring the speed of the fan in RPS
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;


    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}

// used also for measuring speed
//void interrupt high_priority highIsr(void)//old syntax
void __interrupt(high_priority) highIsr(void)//new syntax
{
    RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L); //

    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.T0IF = 0;

}

void main(void) {
    //ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    char Buffer[32]; // for sprintf
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float voltage;
    setupPorts();
    setupSerial();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    //PORTCbits.RC4 =1;
    PORTCbits.RC5 = 1;
    send_string_no_lib((unsigned char *) "\r\rReading AN0, AN1, AN2\r\r");
    lcd_putc('\f'); //clears the display
    unsigned char RecvedChar = 0;
    unsigned char SendToSerial = 0;
    int RPS;
     initTimers01();   // These will be used to measure the speed
     TRISCbits.RC0 = 1; //Timer1 clock
    while (1) {
        CLRWDT(); // no need for this inside the delay below
        PORTCbits.RC5 = !PORTCbits.RC5;
        delay_ms(200); //read ADC AN0,AN1, AN2 every 2 seconds
        for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
            voltage = read_adc_voltage((unsigned char) channel);
            AN[channel] = voltage; // store in array AN0--AN2
            if (is_byte_available()) { // Read serial, if receive S
                RecvedChar = read_byte_no_lib(); // Then start sending to serial
                if (RecvedChar == 'S') SendToSerial = 1;
                else if (RecvedChar == 'N') SendToSerial = 0;// Stop sending to serialif N is recived
                else {
                    /*No Change */
                }
            }
            if (SendToSerial) {
                // If Sending to Serial ( after receiving S, send values)
                sprintf(Buffer, "V%d:%6.2f volt\r", channel, voltage);
                send_string_no_lib(Buffer);
            }
        }
        raw_val = read_adc_raw_no_lib(0); // read raw value for POT1 
        set_pwm1_raw(raw_val);  // set the Pwm to that value 0--1023
        lcd_gotoxy(1, 1);
        sprintf(Buffer, "V0=%4.2fV\nV1=%4.2fV", AN[0], AN[1]);
        lcd_puts(Buffer);
        lcd_gotoxy(1, 3);
        RPS = RPS_count;
       /// sprintf(Buffer, "V2=%7.4fV\n", AN[2]);
        sprintf(Buffer, "Speed=%6.2f RPS\n", RPS/7.0); // Display Speed
        lcd_puts(Buffer);       // speed = Revolution per second
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "D=%5d,%6.2f", raw_val, (raw_val * 100.0) / 1023.0);
        lcd_puts(Buffer); // Above displays duty 0--1023, and also as percentage
        lcd_gotoxy(15, 4);
        lcd_putc('%');
        if (SendToSerial) {
            send_string_no_lib((unsigned char *) "----------------------------\r\r");
        }

    }
}
