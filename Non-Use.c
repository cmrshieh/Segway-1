//
//  Non-Use.c
//  Segway
//
//  Created by Francisco on 11/9/13.
//  Copyright (c) 2013 Francisco de Villalobos. All rights reserved.
//

#include <stdio.h>



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