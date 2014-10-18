//
//  Functions.c
//  Test_1
//
//  Created by Francisco de Villalobos on 9/25/13.
//  Copyright (c) 2013 Francisco de Villalobos. All rights reserved.
//

#include <stdio.h>
#include "Functions.h"
#include "Settings.h"
#include "lib_functions.h"


void usb_com(void){
    
    int in_key;
    
    /******************************************************************************************/
    /*****************                    Keyboard Control                    *****************/
    /******************************************************************************************/
    // Uses the keyboard inputs to do things.
    if(m_usb_rx_available())
    {
        in_key = m_usb_rx_char();
        m_usb_tx_string(" ");
        m_usb_tx_int(in_key);          // Echo key
        m_usb_tx_string(" ");
        switch(in_key){
            case key_a:
                
                break;
            case key_b:
                m_usb_tx_string("                                                          ");
                break;
            case key_c:
                m_usb_tx_int(m_adc(D4));
                break;
            case key_d:
                m_usb_tx_int(m_adc(D6));
                break;
            case key_e:
                m_gpio_out(E6, OFF);
                break;
            case key_f:
                m_gpio_out(B5, ON);
                break;
            case key_g:
                m_gpio_out(B5, OFF);
                break;
            case key_h:
                
                break;
            case key_i:
                
                break;
            case key_j:
                
                break;
            case key_k:
                
                break;
            case key_l:
                
                break;
            case key_m:
                
                break;
            case key_n:
                
                break;
            case key_o:
                
                break;
            case key_p:
                
                break;
            case key_q:
                
                break;
            case key_r:
                
                break;
            case key_s:
                
                break;
            case key_t:
                
                break;
            case key_u:
                
                break;
            case key_v:
                
                break;
            case key_w:
                
                break;
            case key_x:
                
                break;
            case key_y:
                
                break;
            case key_z:
                
                break;
            default:
                m_usb_tx_string("Invalid Input");
                break;
        }                                           // End Switch Case.
        m_usb_tx_string("      \r");
    }
    
    /******************************************************************************************/
    // Display information for the SCREEN.
    /*
     //m_usb_tx_string("\r");      // Carriage Return
     //m_usb_tx_string("\n");      // Enter
     //m_usb_tx_string("\f");      // Form Feed
     //m_usb_tx_string("\f");      // Tab
     //m_usb_tx_string("\v");      // Vertical Tab
     */
    /******************************************************************************************/
    
}



/******************************************************************************************/
/*****************                Open-Loop Motor Control                 *****************/
/******************************************************************************************/
// Inputs, Motor Direction, Desired Speed and Motor Number.

// Takes:
//      dir: 0 for Brake, 1 for FWD and 2 for REV
//      speed:
//      motor_num: 1 for Left Motor, 2 for Right Motor.

char motor_control(char dir, int speed, char motor_num){
    
    switch (motor_num) {
        case 1:                 // Motor 1
            
            switch (dir) {
                case 0:             // Brake Motor
                    set(PORTB, 4);
                    set(PORTC, 6);
                    break;
                case 1:             // Motor FWD
                    set(PORTB, 4);
                    clear(PORTC, 6);
                    break;
                case 2:             // Motor REV
                    clear(PORTB, 4);
                    set(PORTC, 6);
                    break;
                    
                default:
                    break;
            }
            OCR1B = speed;
            break;
            
        case 2:                 // Motor 2
            switch (dir) {
                case 0:             // Brake Motor
                    set(PORTB, 5);
                    set(PORTC, 7);
                    break;
                case 1:             // Motor FWD
                    set(PORTB, 5);
                    clear(PORTC, 7);
                    break;
                case 2:             // Motor REV
                    clear(PORTB, 5);
                    set(PORTC, 7);
                    break;
                    
                default:
                    break;
            }
            OCR1C = speed;
            break;
            
        default:
            return 0;
            break;
    }
    
    return 1;
}

/******************************************************************************************/
/*****************                PID Controller For Angle                *****************/
/******************************************************************************************/
// Inputs, Desired Angle, Actual Angle

// Takes:
//
//
//
volatile float kp1 = 0.5;
volatile float ki1 = 0;
volatile float kd1 = 0.1;

int angle_pid(int setting, int angle){
    
    
    
    float err;
    float e_dot;
    volatile float old_e;
    volatile float integral_E;
    int output;
    
    err = angle - setting;
    e_dot = err - old_e;
    integral_E += err;                                      // Integral of the Error.
    output = kp1 * err + ki1 * integral_E + kd1 * e_dot;       // Controller
    old_e = err;
    
    return output;
    
}


/******************************************************************************************/
/*****************             PID Controller Implementation              *****************/
/******************************************************************************************/
// Inputs, Motor Direction, Desired Speed and Motor Number.

// Takes:
//      dir: 0 for Brake, 1 for FWD and 2 for REV
//      speed:
//      motor_num: 1 for Left Motor, 2 for Right Motor.

int pid(int setting, int encoder){
    
#define kp      0.001
#define ki      0
#define kd      0
    
    float err;
    float e_dot;
    float inst_speed;
    volatile float old_e;
    volatile float integral_E;
    volatile int old_counts;
    int output;
    
    inst_speed = (encoder - old_counts) * 613 * 4 * 10;          //Instantaneous Speed. Delta t = 4ms. In rad/s.
    err = setting - inst_speed;
    e_dot = err - old_e;
    integral_E += err;                                      // Integral of the Error.
    output = kp * err + ki * integral_E + kd * e_dot;       // Controller
    old_e = err;
    old_counts = encoder;
    
    return output;

}





/******************************************************************************************/
/*****************                 PID Parameter Settings                 *****************/
/******************************************************************************************/
// Increases or decreases kp, kd and ki for the angle controller.

void Increment_Parameter(int Param)
{
	switch (Param)
	{
		case 0:
			kp1 += 0.1;
			
			break;
		case 1:
			ki1 += 0.1;
			break;
		case 2:
			kd1 += 0.1;
			break;
	}
	m_usb_tx_string(" kp: ");
	m_usb_tx_int(kp1*100);
	m_usb_tx_string(" ki: ");
	m_usb_tx_int(ki1*100);
	m_usb_tx_string(" kd: ");
	m_usb_tx_int(kd1*100);
	m_usb_tx_string("\n");
}

void Decrement_Parameter(int Param)
{
	switch (Param)
	{
		case 0:
			kp1 -= 0.1;
			break;
		case 1:
			ki1 -= 0.1;
			break;
		case 2:
			kd1 -= 0.1;
			break;
	}
	m_usb_tx_string(" kp: ");
	m_usb_tx_int(kp1*100);
	m_usb_tx_string(" ki: ");
	m_usb_tx_int(ki1*100);
	m_usb_tx_string(" kd: ");
	m_usb_tx_int(kd1*100);
	m_usb_tx_string("\n");
}

