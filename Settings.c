//
//  Settings.c
//  Speakers
//
//  Created by Francisco de Villalobos on 10/6/13.
//  Copyright (c) 2013 Francisco de Villalobos. All rights reserved.
//

#include <stdio.h>
#include "lib_functions.h"

void init(void){
    m_clockdivide(0);

    
/******************************************************************************************/
/*****************                    Initialization                      *****************/
/******************************************************************************************/
    
    //Encoder Definition
	set(PCICR,PCIE0);		// enable pin-change interrupts
	set(PCMSK0,PCINT0);		// demask B0
	set(PCMSK0,PCINT1);		// demask B1
	set(PCMSK0,PCINT2);		// demask B2
	set(PCMSK0,PCINT3);		// demask B3
    mx_encoder_zero(1);     // Reset Encoder 1
    mx_encoder_zero(2);     // Reset Encoder 2
    clear(DDRB,0);          // Encoder 1, Quadrature 1
    clear(DDRB,1);          // Encoder 1, Quadrature 2
    clear(DDRB,2);          // Encoder 2, Quadrature 1
    clear(DDRB,3);          // Encoder 2, Quadrature 2
    
    // Wheel Direction Pins
    set(DDRB, 4);           // Left Wheel Dir 1
    set(DDRC, 6);           // Left Wheel Dir 2
    
    set(DDRB, 5);           // Right Wheel Dir 1
    set(DDRC, 7);           // Right Wheel Dir 2
    
    
/******************************************************************************************/
    // Define Timer 1 Settings
    // We are going to use Timer 1 as the PWM modulated signal to generate
    // the velocity control of the back motors (To enable Channel).
    
    // Define Prescaler as 8.
    clear(TCCR1B,CS12);
    set(TCCR1B,CS11);
    clear(TCCR1B,CS10);
    
    // Set Mode 15: UP to OCR1A, PWM mode
    set(TCCR1B,WGM13);
    set(TCCR1B,WGM12);
    set(TCCR1A,WGM11);
    set(TCCR1A,WGM10);
    
    // Toggle Mode on OC1B, which is multiplexed to B6
    set(DDRB,6);
    set(TCCR1A,COM1B1);
    clear(TCCR1A,COM1B0);
    
    // Toggle Mode on OC1C, which is multiplexed to B7
    set(DDRB,7);
    set(TCCR1A,COM1C1);
    clear(TCCR1A,COM1C0);
    
    // Set the OCR1A, OCR1B and OCR1C. (Only Initial values. Will Change dynamically)
    OCR1A = 0x00FF;  //255
    OCR1B = 0x0000;
    OCR1C = OCR1B;
    
    // Set Interrupt in Overflow
    // set(TIMSK1,TOIE1);
    

/******************************************************************************************/
    // Timer 0 for mili seconds
    
    // 1024 Prescaler   ------> 16000000 / 1024 = 15.625 counts/sec
    set(TCCR0B, CS02);
    clear(TCCR0B, CS01);
    set(TCCR0B, CS00);
    
    // UP to OCR0A
    set(TCCR0B, WGM02);
    clear(TCCR0A, WGM01);
    clear(TCCR0A, WGM00);
    OCR0A = 156;         // 156counts / 15.625counts/s =~ 0.01s = 10ms
    
    // No Change Pin B7
    clear(TCCR0A, COM0A1);
    clear(TCCR0A, COM0A0);
    
    // Enable OverFlow Interrupt
    set(TIMSK0, OCIE0A);
    
    
}