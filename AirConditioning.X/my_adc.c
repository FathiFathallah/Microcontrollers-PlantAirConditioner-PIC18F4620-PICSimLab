/*
 * File:   my_adc.c
 * Author: pc
 *
 * Created on April 8, 2022, 3:35 PM
 */


#include "my_adc.h"

void init_adc_no_lib(void) {
    ADCON0 = 0;
    ADCON0bits.ADON = 1; // turn ADC on 
    ADCON2 = 0b10001001; // ADFM= 1 right justified 10 bits, 2 Tad, Fosc/8
}

int read_adc_raw_no_lib(unsigned char channel) {
    int raw_value;
    ADCON0bits.CHS = channel; //select the channel, 4 bit value
    //start conversion
    ADCON0bits.GO = 1; // start conversion
    while (ADCON0bits.GO) {
        CLRWDT();
    }; // We wait until conversion is done
    raw_value = ADRESH << 8 | ADRESL; // 10 bit, need to shift the bits right, this puts the result in one 16 bit variable
    return raw_value; // 16 bit , it is actually 10 bit 0 ---1023
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;
}

