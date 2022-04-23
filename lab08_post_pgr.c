/*
 * lab08_post
 * File:   lab08_post_pgr.c
 * Author: Gerardo Paz - 20173
 * Potenciómetro en RA0 y RA1
 * DAC en puerto C
 * 7 segmentos en puerto D
 * 
 * Created on April 5, 2022, 8:52 PM
 */

#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>         // registros del PIC
#include <stdio.h>
#include <stdlib.h>

/*------------ CONSTANTES -----------------*/
#define _XTAL_FREQ 4000000  // oscilador 
#define _tmr0_value 236   // TMR0 value

/*------------- VARIABLES GLOBALES ------------------*/
uint8_t cod = 0, selector = 0;
int contador = 0, vector[3] = {0,0,0};

/*--------------- FUNCIONES DE INTERRUPCIONES -----------------*/
void codificar_7seg(uint8_t valor){
    if(selector == 4){
        switch(valor){
            case 0:
                PORTD = 0b10111111;
                break;
            case 1:
                PORTD = 0b10000110;
                break;
            case 2:
                PORTD = 0b11011011;
                break;
            case 3:
                PORTD = 0b11001111;
                break;   
            case 4:
                PORTD = 0b11100110;
                break;
            case 5:
                PORTD = 0b11101101;
                break;
            case 6:
                PORTD = 0b11111101;
                break;
            case 7:
                PORTD = 0b10000111;
                break;   
            case 8:
                PORTD = 0b11111111;
                break;
            case 9:
                PORTD = 0b11101111;
                break;
            default:
                PORTD = 0b10000000;
                break;
        }   
    }
    else{
        switch(valor){
            case 0:
                PORTD = 0b00111111;
                break;
            case 1:
                PORTD = 0b00000110;
                break;
            case 2:
                PORTD = 0b01011011;
                break;
            case 3:
                PORTD = 0b01001111;
                break;   
            case 4:
                PORTD = 0b01100110;
                break;
            case 5:
                PORTD = 0b01101101;
                break;
            case 6:
                PORTD = 0b01111101;
                break;
            case 7:
                PORTD = 0b00000111;
                break;   
            case 8:
                PORTD = 0b01111111;
                break;
            case 9:
                PORTD = 0b01101111;
                break;
            default:
                PORTD = 0b00000000;
                break;
        }
    }
    return;
}

void multiplexado(){
    if(selector < 4)
    {
        if(selector == 0)
            selector++;         // siempre tiene que estar encendido un bit
        else
            selector *= 2;      // encender bits de uno en uno 
    }
    else
        selector = 1;       // al hacer overflow de los selectores, se coloca en 1

    switch(selector){
        case 1:{
            codificar_7seg(vector[0]);  //mostrar valor
            PORTB = selector;
            break;
        }
        case 2:{
            codificar_7seg(vector[1]);  //mostrar valor
            PORTB = selector;
            break;
        }
        case 4:{
            codificar_7seg(vector[2]);  //mostrar valor
            PORTB = selector;
            break;
        }
        default:
            break;
    }      
    return;
}
/*---------------- FUNCIONES PRINCIPALES ---------------*/

void setup(void){
    ANSEL = 0b00000011;      // AN0 y AN1
    ANSELH = 0;
    
    TRISA = 0b00000011;      // A in
    TRISB = 0;      // B out
    TRISC = 0;      // C out
    TRISD = 0;      // C out
    
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    PORTC = 0;      // limpiar
    return;
}

void separar(int valor, int* vector){
    int temp = 0;
    
    temp = valor;
    vector[2] = temp/100;
    temp-= vector[2]*100;
    vector[1] = temp/10;
    temp-= vector[1]*10;
    vector[0] = temp; 
    return;
}

void timer0(void){
    OPTION_REGbits.T0CS = 0;    // temporizador
    OPTION_REGbits.PSA = 0;     // asignar prescaler
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;     // ps 1:256
    INTCONbits.T0IF = 0;
    TMR0 = _tmr0_value;
    return;
}

void clk(void){
    OSCCONbits.IRCF = 0b0110;   // Tiempo
    OSCCONbits.SCS = 1;         // oscilador interno
    return;
}

void setup_adc(void){
    ADCON1bits.ADFM = 0;    // justificado izquierda
    ADCON1bits.VCFG0 = 0;   // VDD
    ADCON1bits.VCFG1 = 0;   // VSS
    
    ADCON0bits.ADCS = 0b01; // Fosc/8
    ADCON0bits.CHS = 0;     // AN0
    ADCON0bits.ADON = 1;    // habilitar módulo ADC
    __delay_us(50);         // tiempo para que cargue el capacitor
    return;
}

void setup_int(void){
    INTCONbits.GIE = 1;     // globales
    INTCONbits.PEIE = 1;    // periféricas
    INTCONbits.T0IE = 1;    // timer 0
    PIE1bits.ADIE = 1;      // ADC
    
    INTCONbits.T0IF = 0;    // bandera timer0
    PIR1bits.ADIF = 0;      // ADC bandera
    return;
}

/*------------ CÓDIGO PRINCIPAL ---------------*/
void __interrupt() isr (void){
    int temp = 0;
    if(ADIF){
        PIR1bits.ADIF = 0;      // limpiar bandera
        if(ADCON0bits.CHS == 0)
            PORTC = ADRESH;         // canal 0 en C
        else {
            temp = 100*ADRESH/51;   // conversion 0-255 a 0-500
            separar(temp, vector);
        }           
    }
    if(T0IF){
        TMR0 = _tmr0_value;
        PORTB = 0;          // siempre hay que limpiar el puerto del selector para que no haya prolemas
        multiplexado();     // comienza el multiplexado
        T0IF = 0;           // limpiar bandera
    }   
    return;
}

void main(void){
    setup();
    clk();
    timer0();
    setup_adc();
    setup_int();
    ADCON0bits.GO = 1;  // iniciar conversión
    while(1){   // principal loop
        if(ADCON0bits.GO == 0){
            if(ADCON0bits.CHS == 0)     // cambiar entre dos canales
                ADCON0bits.CHS = 1;
            else
                ADCON0bits.CHS = 0;
            
            __delay_us(50);     
            ADCON0bits.GO = 1;  // comenzar otra conversión
        }   
    }
}

