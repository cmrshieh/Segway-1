/* Name: main.c
 * Author: Francisco de Villalobos
 * Copyright: Francisco de Villalobos
 * License: Francisco de Villalobos
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include "lib_functions.h"
#include "Functions.h"
#include "Settings.h"
#include "math.h"


#define Brake           0
#define FWD             1
#define REV             2
#define Left            1
#define Right           2
#define Diam            70      // 70mm diameter
#define counts_right    4095
#define counts_left     1023
#define Beta            0.94    // Infinite Horizon Digital Filter
#define Beta_High       0.94
#define kp_p            0
#define ki_p            1
#define kd_p            2
#define angle_setting   63


int validation;
int gyro[10];
char start;
char flag;
float send[10];
char recieve[10];
int buffer[9];
float cmd_speed;
int* data = &buffer;

int v_left = 0xA0;
int v_right = 0xA0;
float act_speed;
int old_counts;
int buffer_last[9];
float ang_rad;
float ang_deg;

int in_key;

int main(void)
{
/******************************************************************************************/
/*****************                    Initialization                      *****************/
/******************************************************************************************/
    
    init();                             // Run Initialization Code for Ports and Timers.
    m_init();                           // 16MHz and Disable JTAG.
    m_imu_init(0, 0);                   // Initialize iMu with determined Settings.
	m_usb_init();                       // Initialize the USB connection.
    sei();                              // Enable Interrupts
    
    m_rf_open(1, 0x3A, 10);             // Open Channel 1, Address 0x3A for this M2. 10 Packet Lenght.
    
    
//    // Initialization in FWD Direction Motor Left
//    clear(PORTB, 4);
//    set(PORTC, 6);
//    v_left = 0x30;
    

    while(1){

/******************************************************************************************/
/*****************                       Main Code                       *****************/
/******************************************************************************************/
// Checks the value of the conversion and stores it in the correct variable
        
        m_imu_raw(buffer);
        //        send[0] = (int)(Beta * (float)send[0] + (1.0-Beta) * (float)data[0]);                                   // ax
        send[1] = (Beta * (float)send[1] + (1.0-Beta) * (float)data[1]);                                   // ay
        send[2] = (Beta * (float)send[2] + (1.0-Beta) * (float)data[2]);                                   // az
        send[3] = (Beta_High * (float)send[3] + (1.0-Beta_High) * (float)(data[3]-buffer_last[3]));        // gx
        //        send[4] = (int)(Beta_High * (float)send[4] + (1.0-Beta_High) * (float)(data[4]-buffer_last[4]));        // gy
        //        send[5] = (int)(Beta_High * (float)send[5] + (1.0-Beta_High) * (float)(data[5]-buffer_last[5]));        // gz
        //        buffer_last[0] = data[0];
        buffer_last[1] = data[1];
        buffer_last[2] = data[2];
        buffer_last[3] = data[3];
        //        buffer_last[4] = data[4];
        //        buffer_last[5] = data[5];
        
        ang_rad = atan2f(-send[2], send[1]);
        ang_deg = ang_rad * 180 / M_PI * 10;
        
        cmd_speed = angle_pid(angle_setting, ang_deg);
        
        
        // DeadBand for OCR1B
        OCR1B = abs((int)cmd_speed) * 255 / 80;
        if (OCR1B > 0xFF) {
            OCR1B = 0xFF;
        }
        else if (OCR1B < 0xA5 && OCR1B > 0x30){
            OCR1B = 0xA5;
        }
        OCR1C = OCR1B;
        
        
        // Motor Control and Direction
        
        if (ang_deg > 350 || ang_deg < -350) {
            motor_control(Brake, 0, Left);
            motor_control(Brake, 0, Right);
        }
        else {
            if (ang_deg > angle_setting + 3) {
                motor_control(FWD, OCR1B, Left);
                motor_control(FWD, OCR1C, Right);
            }
            else if (ang_deg < angle_setting - 3){
                motor_control(REV, OCR1B, Left);
                motor_control(REV, OCR1C, Right);
            }
        }

    
        
        m_usb_tx_string("   ay: ");
//        m_usb_tx_int(send[0]);
//        m_usb_tx_string("    ");
        m_usb_tx_int(send[1]);
        m_usb_tx_string("    az: ");
        m_usb_tx_int(send[2]);
        m_usb_tx_string("    ");
        m_usb_tx_string("    gx: ");
        m_usb_tx_int(send[3]);
        m_usb_tx_string("    ");
        m_usb_tx_string("    0:  ");
        m_usb_tx_int(ang_deg);
        m_usb_tx_string("    Speed:  ");
        m_usb_tx_int(OCR1B);
        m_usb_tx_string("              \r");
        
        
        
        

        

/******************************************************************************************/
/*****************                   Rotation Direction                   *****************/
/******************************************************************************************/
// Changes Direction According to the brightness of the LEDs.
        
/******************************************************************************************/
/*****************                     Keyboard Inputs                    *****************/
/******************************************************************************************/
// Stops Motors When After Some turns, it finds the maximum.
//
        if(m_usb_rx_available())
        {
            in_key = m_usb_rx_char();
            m_usb_tx_string(" ");
            //m_usb_tx_int(in_key);          // Echo key
            //m_usb_tx_string(" ");
            switch(in_key){
                case key_a:
                    m_imu_raw(buffer);
                    send[0] = data[0];
                    send[1] = data[1];
                    send[2] = data[2];
                    send[3] = data[3];
                    send[4] = data[4];
                    send[5] = data[5];
                    m_usb_tx_string(" Val: ");
                    m_usb_tx_int(validation);
                    m_usb_tx_string("   Accel ");
                    m_usb_tx_int(data[0]);
                    m_usb_tx_string("    ");
                    m_usb_tx_int(data[1]);
                    m_usb_tx_string("    ");
                    m_usb_tx_int(data[2]);
                    m_usb_tx_string("    ");
                    m_usb_tx_string("Gyro ");
                    m_usb_tx_int(data[3]);
                    m_usb_tx_string("    ");
                    m_usb_tx_int(data[4]);
                    m_usb_tx_string("    ");
                    m_usb_tx_int(data[5]);
                    m_usb_tx_string("                           ");
                    break;
                case key_b:
                    m_usb_tx_string("                                                          ");
                    break;
                case key_c:
                    m_usb_tx_string(" OCR1B: ");
                    m_usb_tx_int(OCR1B);
                    m_usb_tx_string("   Setting: ");
                    m_usb_tx_int(v_left);
                    m_usb_tx_string("    Encoder: ");
                    m_usb_tx_int(mx_encoder(2));
                    m_usb_tx_string("    Old Error: ");
                    //m_usb_tx_int(old_e);
                    m_usb_tx_string("    Error Integral");
                    //m_usb_tx_int(integral_E);
                    m_usb_tx_string("                           ");
                    break;
                case key_d:
                    m_usb_tx_string(" Actual Speed ");
                    m_usb_tx_long(act_speed);
                    m_usb_tx_string(" rad/s ");
                    break;
                case key_e:
                    motor_control(Brake, v_left, Left);
                    m_usb_tx_string("Brake Left Motor                           ");
                    break;
                case key_f:
                    motor_control(FWD, v_left, Left);
                    m_usb_tx_string("Left Motor FWD         ");
                    m_usb_tx_int(v_left);
                    break;
                case key_g:
                    motor_control(REV, v_left, Left);
                    m_usb_tx_string("Left Motor REV                          ");
                    m_usb_tx_int(v_left);
                    break;
                case key_h:
                    motor_control(Brake, v_right, Right);
                    m_usb_tx_string("Brake Right Motor                           ");
                    break;
                case key_i:
                    motor_control(FWD, v_right, Right);
                    m_usb_tx_string("Right Motor FWD                          ");
                    m_usb_tx_int(v_right);
                    break;
                case key_j:
                    motor_control(REV, v_right, Right);
                    m_usb_tx_string("Right Motor REV                          ");
                    m_usb_tx_int(v_right);
                    break;
                case key_k:
                    m_usb_tx_string(" Encoder 1: ");
                    m_usb_tx_int(mx_encoder(1));
                    break;
                case key_l:
                    m_usb_tx_string(" Encoder 2: ");
                    m_usb_tx_int(mx_encoder(2));
                    break;
                case key_m:
                    Increment_Parameter(kp_p);
                    break;
                case key_n:
                    Decrement_Parameter(kp_p);
                    break;
                case key_o:
                    Increment_Parameter(ki_p);
                    break;
                case key_p:
                    Decrement_Parameter(ki_p);
                    break;
                case key_q:
                    Increment_Parameter(kd_p);
                    break;
                case key_r:
                    Decrement_Parameter(kd_p);
                    break;
                case key_s:
                    
                    break;
                case key_t:
                    
                    break;
                case key_u:
                    
                    break;
                case key_v:
                    m_rf_send(0x3F, send, 10);
                    m_usb_tx_string(" Packet Sent");
                    break;
                case key_w:
                    if (v_left > 0xFB) {
                        v_left += 1;
                    }
                    break;
                case key_x:
                    if (v_left < 0x05) {
                        v_left -= 1;
                    }
                    break;
                case key_y:
                    if (v_right > 0xFB) {
                        v_right += 1;
                    }
                    break;
                case key_z:
                    if (v_right < 0x05) {
                        v_right -= 1;
                    }
                    break;
                case key_up:
                    
                    break;
                case key_down:
                    
                    break;
                case key_left:
                    
                    break;
                case key_right:
                    
                    break;
                    
                default:
                    m_usb_tx_string(" Invalid Input");
                    break;
            }                                           // End Switch Case.
            m_usb_tx_string("                \r");
        }
        
/******************************************************************************************/
        
    }
    return 0;                       /* never reached */

}


/******************************************************************************************/
/*****************                     WiFi Interrupt                     *****************/
/******************************************************************************************/
// Every time PIN D2 goes LOW. From the bus communications (D0-D2).

//ISR(INT2_vect){
//    m_rf_read(recieve, 10);
//    m_red(TOGGLE);
//}


/******************************************************************************************/
/*****************                    Timer 0 OverFlow                    *****************/
/******************************************************************************************/
//

ISR(TIMER0_COMPA_vect){
//    OCR1B = pid(1, mx_encoder(2));
//    
//    if (OCR1B > 0xFE) {
//        OCR1B = 0xFE;
//    }
//    else if (OCR1B < 0x02){
//        OCR1B = 0x02;
//    }
    
    
//    act_speed = (float)(mx_encoder(2) - old_counts) * 0.6142;           // Interrupts once every 10 ms.
//    old_counts = mx_encoder(2);
    
    

}