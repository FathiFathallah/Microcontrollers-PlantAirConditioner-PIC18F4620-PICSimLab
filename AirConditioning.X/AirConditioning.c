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

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "my_adc.h"
#include "lcd_x8.h"
#include "my_pwm.h"
#include "my_ser.h"
#include <xc.h>

#define WINTER_T 40
#define SUMMER_T 60
#define STARTVALUE  63036
#define STARTVALUE_SPEED  3036

//DISPLAY GLOBAL VALUES
char PERCENT[6];
char HSS[4];
char HC[3];
char LCD[64];
char MODEDISPLAY[64];
////////////////
float SP;
float OT;
float RT;
int mode = -1;
bool heater = false;
bool cooler = false;
/////////////////////
int PercentHeatCounter = 0;
int counterTimer3 = 0;
///////////////////////////
int PercentCoolCounter = 0;
/////////////////////////////////
float R = 0;
int HS = 0;
///////////////////////////////
unsigned int RPS_count = 0;
bool SendToSerial = false;

void reloadTimer3(void) {
    TMR3H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR3L = (unsigned char) (STARTVALUE & 0x00FF);
}

void Timer3_isr(void) {
    PIR2bits.TMR3IF = 0;

    if (counterTimer3 >= PercentHeatCounter) {
        PORTCbits.RC5 = 0;
    } else PORTCbits.RC5 = 1;
    if (counterTimer3 == 20) {
        if (PercentHeatCounter != 0) PORTCbits.RC5 = 1;
        counterTimer3 = 0;
    }
    counterTimer3++;
    reloadTimer3();
}

//HEATING WITH PERCENTAGE
void set_fakePWM_Heater(float percent) {
    cooler = false;
    heater = true;
    PercentHeatCounter = percent / 100.0 * 20;
    counterTimer3 = 0;
    reloadTimer3();
    T3CONbits.TMR3ON = 1;
}

//TURNING THE HEATER OFF
void Heater_OFF(void) {
    heater = false;
    PercentHeatCounter = 0;
    counterTimer3 = 0;
    T3CONbits.TMR3ON = 0;
    PORTCbits.RC5 = 0;
}

//RB0 / INT0
void EXT_Int0_isr(void) {
    INTCONbits.INT0IF = 0;
    if (mode == 2) {
        mode = 0;
    } else {
        mode++;
    }
    switch (mode) {
        case 0:
            cooler = false;
            set_pwm1_percent(0);
            PORTCbits.RC2 = 0;
            heater = true;
            PercentHeatCounter = 0;
            counterTimer3 = 0;
            reloadTimer3();
            T3CONbits.TMR3ON = 1;
            break;
        case 1:
            cooler = true;
            heater = false;
            T3CONbits.TMR3ON = 0;
            PORTCbits.RC5 = 0;
            PercentCoolCounter = 0;
            break;
        case 2:
            HS = 0;
            Heater_OFF();
            cooler = false;
            PORTCbits.RC2 = 0;
            set_pwm1_percent(0);
            break;
    }
}

void EXT_Int1_isr(void) {
    INTCON3bits.INT1IF = 0;
    //HEATER DECREMENT
    if (mode == 0) {
        if (PercentHeatCounter > 0) {
            PercentHeatCounter = PercentHeatCounter - 1;
        }
    }
        //COOLER DECREMENT
    else if (mode == 1) {
        if (PercentCoolCounter > 0) {
            PercentCoolCounter = PercentCoolCounter - 1;
            set_pwm1_percent(PercentCoolCounter * 5.0);
        }
    }        //AUTO HC DECREMENT
    else if (mode == 2) {
        if (HS > 0) {
            HS--;
        }
    }
}

void EXT_Int2_isr(void) {
    INTCON3bits.INT2IF = 0;
    //HEATER INCREMENT
    if (mode == 0) {
        if (PercentHeatCounter < 20) {
            PercentHeatCounter = PercentHeatCounter + 1;
        }
    }        //COOLER INCREMENT
    else if (mode == 1) {
        if (PercentCoolCounter < 20) {
            PercentCoolCounter = PercentCoolCounter + 1;
            set_pwm1_percent(PercentCoolCounter * 5.0);
        }
    }        //AUTO HC INCREMENT
    else if (mode == 2) {
        if (HS < 4) {
            HS++;
        }
    }
}

//THIS WILL GO OFF (RB3 PRESSED OR SERIAL COMMUNICATION 'S' )
void go_Off_Mode(void) {
    mode = -1;
    cooler = false;
    heater = false;
    T3CONbits.TMR3ON = 0; // TIMER FOR HEATING IS OFF
    PORTCbits.RC5 = 0;
    PORTCbits.RC2 = 0;
    set_pwm1_percent(0);
    HS = 0;
}

//SERIAL INTURRUPT
void Serial_IntS(void) {
    if (RCSTAbits.FERR || RCSTAbits.OERR)//check for error
    {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    char RecvedChar = read_byte_no_lib(); // Then start sending to serial
    if (RecvedChar == 'M') SendToSerial = true;
    else if (RecvedChar == 'S') {
        go_Off_Mode();
        SendToSerial = false; // Stop sending to serialif N is recived
    } else {
        /*No Change */
    }
    PIR1bits.RCIF = 0;
}



//TO CALCULATE FAN SPEER
void FanSpeed(void) {

    RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L); //
    TMR0H = (unsigned char) ((STARTVALUE_SPEED >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE_SPEED & 0x00FF);
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.T0IF = 0;
}


void __interrupt(high_priority)highIsr(void) {
    if(PIR2bits.TMR3IF)Timer3_isr();
    else if (INTCONbits.INT0IF)EXT_Int0_isr();
    else if (INTCON3bits.INT1IF)EXT_Int1_isr();
    else if (INTCON3bits.INT2IF)EXT_Int2_isr();
    else if (INTCONbits.T0IF)FanSpeed();
    else if (PIR1bits.RCIF)Serial_IntS();   
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

void delay_ms(unsigned int n) {
    int i;
    for (i = 0; i < n; i++) {
        __delaywdt_ms(1);
    }
}

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; // Because We need AN2/AN1/AN0, Then we need 3 analog inputs
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // outputs
    PORTC = 0;
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void display(void) {


    switch (mode) {
        case -1:
            sprintf(MODEDISPLAY, "MD:OFF    ");
            break;

        case 0:
            sprintf(MODEDISPLAY, "MD:Heat   ");
            break;
        case 1:
            sprintf(MODEDISPLAY, "MD:Cool   ");
            break;
        case 2:
            sprintf(MODEDISPLAY, "MD:Auto HC");
            break;
    }


    //////////////////////////////////////////////////////
    lcd_gotoxy(1, 1);
    sprintf(LCD, "RT:%5.1fC  ", RT);
    lcd_puts(LCD);

    lcd_gotoxy(14, 1);
    sprintf(LCD, "H C");
    lcd_puts(LCD);
    /////////////////////////////////////////////////////
    lcd_gotoxy(1, 2);
    sprintf(LCD, "SP:%5.1fC  ", SP);
    lcd_puts(LCD);

    if (heater && cooler)
        sprintf(HC, "Y Y");
    else if (!heater && !cooler)
        sprintf(HC, "N N");
    else if (heater && !cooler)
        sprintf(HC, "Y N");
    else if (!heater && cooler)
        sprintf(HC, "N Y");

    lcd_gotoxy(14, 2);
    lcd_puts(HC);
    /////////////////////////////////////////////////////
    lcd_gotoxy(1, 3);
    sprintf(LCD, "OT:%5.1fC  ", OT);
    lcd_puts(LCD);
    //OFF
    if (mode == -1) {
        lcd_gotoxy(11, 3);
        sprintf(PERCENT, "R:%4.1f", R);
        lcd_puts(PERCENT);
        lcd_gotoxy(13, 4);
        sprintf(HSS, "    ");
        lcd_puts(HSS);
    }        //MODE = HEAT
    else if (mode == 0) {
        lcd_gotoxy(11, 3);
        sprintf(PERCENT, "H:%3d%c ", PercentHeatCounter * 5, '%');
        lcd_puts(PERCENT);
        lcd_gotoxy(13, 4);
        sprintf(HSS, "    ");
        lcd_puts(HSS);
    }        //MODE = COOLER
    else if (mode == 1) {
        lcd_gotoxy(11, 3);
        sprintf(PERCENT, "C:%3d%c ", PercentCoolCounter * 5, '%');
        lcd_puts(PERCENT);
        lcd_gotoxy(13, 4);
        sprintf(HSS, "    ");
        lcd_puts(HSS);
    } else if (mode == 2) {
        lcd_gotoxy(11, 3);
        sprintf(PERCENT, "R:%4.1f", R);
        lcd_puts(PERCENT);
        delay_ms(10);
        lcd_gotoxy(13, 4);
        sprintf(HSS, "HS:%d", HS);
        lcd_puts(HSS);
    }
    /////////////////////////////////////   
    lcd_gotoxy(1, 4);
    lcd_puts(MODEDISPLAY);
    /////////////////////////////////////////
}

void main(void) {
    char Buffer[64];
    setupPorts();
    setupSerial();
    init_adc_no_lib();
    init_pwm1();

    
    //Timer-0 Initialization | Without starting
    T0CON = 0;
    //T0CONbits.TMR0ON = 0; // We don't want the timer to run yet
    T0CONbits.T08BIT = 0; // We want Timer-0 = 16 bit
    T0CONbits.T0CS = 0;
    T0CONbits.T0SE = 0;
    T0CONbits.PSA = 0; //IF PSA IS ONE, THEN pre-scaler = 1, Regardless of T0PS2:T0PS0
    T0CONbits.T0PS2 = 0;
    T0CONbits.T0PS1 = 1; // 011 for pre-scaler = 16
    T0CONbits.T0PS0 = 1;

    
    //Timer-3 Initialization | Without starting
    T3CON = 0;
    T3CONbits.TMR3ON = 0; // OFF
    T3CONbits.TMR3CS = 0;
    T3CONbits.T3CKPS0 = 1;
    T3CONbits.T3CKPS1 = 1;
    PIE2 = 0;
    PIR2 = 0;
    PIE2bits.TMR3IE = 1; // TIMER INTERRUPT ENABLE
    PIR2bits.TMR3IF = 0;
    

    //Interrupts Initialization
    RCONbits.IPEN = 0; // disable Priority

    INTCON = 0; // disable interrupts first, then enable the ones u want
    INTCONbits.INT0IE = 1; //INT0 Enable, for Mode Increment
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIEH = 1; // enable global interrupt bits
    INTCONbits.GIEL = 1;

    INTCON2 = 0;
    INTCON2bits.INTEDG0 = 1;
    INTCON2bits.INTEDG1 = 1; //FOR ALL INT, WE CHOOSE RISING EDGE
    INTCON2bits.INTEDG2 = 1;

    INTCON3 = 0;
    INTCON3bits.INT1IE = 1; //INT1 Enable, To START (RESUME) The Cooker
    INTCON3bits.INT2IE = 1; //INT2 Enable, to STOP (PAUSE) the cooker



    PIE1 = 0;
    PIR1 = 0;
    IPR1 = 0;

    IPR2 = 0;
    PORTB = 0;
    
    PIE1bits.RCIE = 1;//ENABLE INTERRUPT WITH SERIAL RECEPTION
    

    lcd_init();
    lcd_send_byte(0, 1);
    int RPS;
    TRISCbits.RC0 = 1; //Timer1 clock
    initTimers01();   // These will be used to measure the speed
    while (1) {

        CLRWDT();
        if(!PORTBbits.RB3) {
            go_Off_Mode();
        }
        delay_ms(200);
        SP = (read_adc_voltage(0))*20.0; //Read the value from Channel 0 => AN0 (The Value Will be from 0-5 volt) | Pt.1
        // We want to scale the 0-5 Volt we have to 0-200C, So we multiply by 40
        OT = (read_adc_voltage(1))*20.0;
        
        RT = (read_adc_voltage(2))*100.0; //Read the value from Channel 2 => AN2 (The Value Will be from 0-5 volt) |

        if (mode == 2) {
            if (OT > SUMMER_T) {
                float CoolerError = RT - (SP);
                if (CoolerError > 0) {
                    Heater_OFF();
                    cooler = true;
                    float PWMValue = CoolerError * 100.0 / 10.0;
                    if (PWMValue < 25.0) PWMValue = 25.0;
                    if (PWMValue > 100.0)PWMValue = 100.0;
                    set_pwm1_percent(PWMValue);
                } else if (RT < (SP - HS)) {
                    heater = true;
                    cooler = false;
                    PORTCbits.RC2 = 0;
                    set_pwm1_percent(0);
                    set_fakePWM_Heater(50.0);
                }
            }
            else if(OT < WINTER_T){
                cooler = false;
                PORTCbits.RC2 = 0;
                set_pwm1_percent(0);
                float HeatError = SP - (RT);
                if(HeatError > 0){
                    heater = true;
                    PercentHeatCounter = 2 * HeatError ;
                    if(PercentHeatCounter > 20) PercentHeatCounter = 20;
                    else if(PercentHeatCounter < 10) PercentHeatCounter = 10;
                    else if(SP > 52) PercentHeatCounter = 20;
                    set_fakePWM_Heater(PercentHeatCounter*5);
                }
                else if(RT > (SP + HS)){
                    PercentHeatCounter = 0;
                    cooler = false;
                    heater = false;
                    PORTCbits.RC2 = 0;
                    set_pwm1_percent(0);
                }
            }
        }
        RPS = RPS_count;
        R = RPS / 7.0;
        delay_ms(200);
        display();


        if (heater && cooler)
            sprintf(HC, "Y Y");
        else if (!heater && !cooler)
            sprintf(HC, "N N");
        else if (heater && !cooler)
            sprintf(HC, "Y N");
        else if (!heater && cooler)
            sprintf(HC, "N Y");
        
        if (SendToSerial) {
            sprintf(Buffer, "RT:%5.1fC | SP:%5.1fC | OT:%5.1fC\r\n", RT, SP,OT);
            send_string_no_lib(Buffer);
            sprintf(Buffer, "%s| HEATER:%c  | COOLER:%c\r\n", MODEDISPLAY,HC[0],HC[2]);
            send_string_no_lib(Buffer);
            sprintf(Buffer, "========================================\r\n");
            send_string_no_lib(Buffer);
        }
        
    }
    return;
}