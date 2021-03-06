/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 *  Vilnius A/D module
 *  ==================
 *
 *  Copyright (C) 2015-2020-https://github.com/grodansparadis/can4vscp_paris/releases/download/v1.1.6/paris_relay_pic18f26k80_1_1_6_relocated.hex Ake Hedman, Grodans Paradis AB
 *                          http://www.grodansparadis.com
 *                          <akhe@grodansparadis.com>
 *
 *  This work is licensed under the Creative Common 
 *  Attribution-NonCommercial-ShareAlike 3.0 Unported license. The full
 *  license is available in the top folder of this project (LICENSE) or here
 *  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
 *  It is also available in a human readable form here 
 *  http://creativecommons.org/licenses/by-nc-sa/3.0/
 * 
 *	This file is part of VSCP - Very Simple Control Protocol 	
 *	http://www.vscp.org
 *
 * ******************************************************************************
 */

#include "vscp_compiler.h"
#include "vscp_projdefs.h"

#include <xc.h>
#include <timers.h>
#include <delays.h>
#include <inttypes.h>
#include <string.h>
#include <ECAN.h>
#include <vscp-firmware.h>
#include <vscp_class.h>
#include <vscp_type.h>
#include "vilnius.h"
#include "main.h"
#include "version.h"

#if defined(_18F2580) 

#if defined(RELEASE)

#pragma config WDT = ON, WDTPS = 128
#pragma config OSC = HSPLL
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 0             // 4.6V
#pragma config LVP = ON
#pragma config CPB = ON
#pragma config BBSIZ = 2048
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#else

#pragma config WDT = OFF
#pragma config OSC = HSPLL
#pragma config PWRT = ON
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 0             // 4.6V
#pragma config LVP = OFF
#pragma config CPB = OFF
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#endif


#else if defined(_18F25K80) || defined(_18F26K80) || defined(_18F45K80) || defined(_18F46K80) || defined(_18F65K80) || defined(_18F66K80)


// CONFIG1L
#pragma config SOSCSEL = DIG    // RC0/RC is I/O
#pragma config RETEN = OFF      // Ultra low-power regulator is Disabled (Controlled by REGSLP bit).
#pragma config INTOSCSEL = HIGH // LF-INTOSC in High-power mode during Sleep.
#pragma config XINST = OFF      // No extended instruction set

// CONFIG1H
#pragma config FOSC = HS2       // Crystal 10 MHz
#pragma config PLLCFG = ON      // 4 x PLL

// CONFIG2H
#pragma config WDTPS = 1048576  // Watchdog prescaler
#pragma config BOREN = SBORDIS  // Brown out enabled
#pragma config BORV  = 0        // 3V

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN TX and RX pins are located on RB2 and RB3, respectively.
#pragma config MSSPMSK = MSK7   // 7 Bit address masking mode.
#pragma config MCLRE = ON       // MCLR Enabled, RE3 Disabled.

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset enabled
#pragma config BBSIZ = BB2K     // Boot block size 2K

#ifdef DEBUG
#pragma config WDTEN = OFF      // WDT disabled in hardware; SWDTEN bit disabled.
#else
#pragma config WDTEN = ON       // WDT enabled in hardware; 
#endif


#endif

// Calculate and st required filter and mask
// for the current decision matrix
void calculateSetFilterMask( void );

// The device URL (max 32 characters including null termination)
const uint8_t vscp_deviceURL[] = "www.eurosource.se/vilnius_1.xml";

volatile unsigned long measurement_clock_sec;  // Clock for one second work

uint16_t sendTimer;         // Timer for CAN send
uint8_t seconds;            // Counter for seconds
uint8_t minutes;            // Counter for minutes
uint8_t hours;              // Counter for hours

uint16_t analog_value[4];   // A/D values

// Set to true when an alarm condition is met. Will be reseted
// when alarm condition is not met.
BOOL bLowAlarm[ 4 ];
BOOL bHighAlarm[ 4 ];

// Report interval counters
volatile uint16_t valueReports[ 4 ];
uint16_t measurementReports[ 4 ];

// IO states
BOOL IOsavebit0;
BOOL IOsavebit1;

//__EEPROM_DATA(0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88);

// This table translates registers in page 0 to EEPROM locations
const uint8_t reg2eeprom_pg0[] = {
    /* REG0_VILNIUS_ZONE                    */          VSCP_EEPROM_END + 0,
    /* REG0_VILNIUS_SUBZONE                 */          VSCP_EEPROM_END + 1,
    /* REG0_VILNIUS_CH0_ZONE                */          VSCP_EEPROM_END + 2,
    /* REG0_VILNIUS_CH0_SUBZONE             */          VSCP_EEPROM_END + 3,
    /* REG0_VILNIUS_CH1_ZONE                */          VSCP_EEPROM_END + 4,
    /* REG0_VILNIUS_CH1_SUBZONE             */          VSCP_EEPROM_END + 5,
    /* REG0_VILNIUS_CH2_ZONE                */          VSCP_EEPROM_END + 6,
    /* REG0_VILNIUS_CH2_SUBZONE             */          VSCP_EEPROM_END + 7,
    /* REG0_VILNIUS_CH3_ZONE                */          VSCP_EEPROM_END + 8,
    /* REG0_VILNIUS_CH3_SUBZONE             */          VSCP_EEPROM_END + 9,
    /* REG0_VILNIUS_IO0_ZONE                */          VSCP_EEPROM_END + 10,
    /* REG0_VILNIUS_IO0_SUBZONE             */          VSCP_EEPROM_END + 11,
    /* REG0_VILNIUS_IO1_ZONE                */          VSCP_EEPROM_END + 12,
    /* REG0_VILNIUS_IO1_SUBZONE             */          VSCP_EEPROM_END + 13,
    /* REG0_VILNIUS_CONTROL_MODULE          */          VSCP_EEPROM_END + 14,
    /* REG0_VILNIUS_CONTROL_CHANNEL0        */          VSCP_EEPROM_END + 15,
    /* REG0_VILNIUS_CONTROL_CHANNEL1        */          VSCP_EEPROM_END + 16,
    /* REG0_VILNIUS_CONTROL_CHANNEL2        */          VSCP_EEPROM_END + 17,
    /* REG0_VILNIUS_CONTROL_CHANNEL3        */          VSCP_EEPROM_END + 18,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL0_MSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL0_LSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL1_MSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL1_LSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL2_MSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL2_LSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL3_MSB   */          0xff,
    /* REG0_VILNIUS_AD_VALUE_CHANNEL3_LSB   */          0xff,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_MSB*/   VSCP_EEPROM_END + 19,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_LSB*/   VSCP_EEPROM_END + 20,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL1_MSB*/   VSCP_EEPROM_END + 21,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL1_LSB*/   VSCP_EEPROM_END + 22,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL2_MSB*/   VSCP_EEPROM_END + 23,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL2_LSB*/   VSCP_EEPROM_END + 24,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL3_MSB*/   VSCP_EEPROM_END + 25,
    /* REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL3_LSB*/   VSCP_EEPROM_END + 26,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL0_MSB  */          VSCP_EEPROM_END + 27,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL0_LSB  */          VSCP_EEPROM_END + 28,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL1_MSB  */          VSCP_EEPROM_END + 29,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL1_LSB  */          VSCP_EEPROM_END + 30,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL2_MSB  */          VSCP_EEPROM_END + 31,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL2_LSB  */          VSCP_EEPROM_END + 32,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL3_MSB  */          VSCP_EEPROM_END + 33,
    /* REG0_VILNIUS_ALARM_LOW_CHANNEL3_LSB  */          VSCP_EEPROM_END + 34,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL0_MSB */          VSCP_EEPROM_END + 35,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL0_LSB */          VSCP_EEPROM_END + 36,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL1_MSB */          VSCP_EEPROM_END + 37,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL1_LSB */          VSCP_EEPROM_END + 38,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL2_MSB */          VSCP_EEPROM_END + 39,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL2_LSB */          VSCP_EEPROM_END + 40,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL3_MSB */          VSCP_EEPROM_END + 41,
    /* REG0_VILNIUS_ALARM_HIGH_CHANNEL3_LSB */          VSCP_EEPROM_END + 42,
    /* REG0_VILNIUS_HYSTERESIS_CH0_MSB      */          VSCP_EEPROM_END + 43,
    /* REG0_VILNIUS_HYSTERESIS_CH1_MSB      */          VSCP_EEPROM_END + 44,
    /* REG0_VILNIUS_HYSTERESIS_CH2_MSB      */          VSCP_EEPROM_END + 45,
    /* REG0_VILNIUS_HYSTERESIS_CH3_MSB      */          VSCP_EEPROM_END + 46,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH0_MSB */  VSCP_EEPROM_END + 47,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH0_LSB */  VSCP_EEPROM_END + 48,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH1_MSB */  VSCP_EEPROM_END + 49,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH1_LSB */  VSCP_EEPROM_END + 50,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH2_MSB */  VSCP_EEPROM_END + 51,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH2_LSB */  VSCP_EEPROM_END + 52,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH3_MSB */  VSCP_EEPROM_END + 53,
    /* REG0_VILNIUS_MESURMENT_REPORT_INTERVAL_CH3_MSB */  VSCP_EEPROM_END + 54,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_MSB    */      VSCP_EEPROM_END + 55,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_LSB    */      VSCP_EEPROM_END + 56,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL1_MSB    */      VSCP_EEPROM_END + 57,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL1_LSB    */      VSCP_EEPROM_END + 58,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL2_MSB    */      VSCP_EEPROM_END + 59,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL2_LSB    */      VSCP_EEPROM_END + 60,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL3_MSB    */      VSCP_EEPROM_END + 61,
    /* REG0_VILNIUS_ABSOLUT_LOW_CHANNEL3_LSB    */      VSCP_EEPROM_END + 62,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_MSB   */      VSCP_EEPROM_END + 63,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_LSB   */      VSCP_EEPROM_END + 64,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL1_MSB   */      VSCP_EEPROM_END + 65,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL1_LSB   */      VSCP_EEPROM_END + 66,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL2_MSB   */      VSCP_EEPROM_END + 67,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL2_LSB   */      VSCP_EEPROM_END + 68,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL3_MSB   */      VSCP_EEPROM_END + 69,
    /* REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL3_LSB   */      VSCP_EEPROM_END + 70,
    /* REG0_VILNIUS_CONTROL_IO                  */      VSCP_EEPROM_END + 71,
    /* REG0_VILNIUS_IO_STATE                    */      0xff,
};

// This table translates registers in page 1 to EEPROM locations
const uint8_t reg2eeprom_pg1[] = {
    /* REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0 */  VSCP_EEPROM_END + 72,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH0   */  VSCP_EEPROM_END + 73,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH0    */  VSCP_EEPROM_END + 74,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH1 */  VSCP_EEPROM_END + 75,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH1   */  VSCP_EEPROM_END + 76,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH1    */  VSCP_EEPROM_END + 77,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH2 */  VSCP_EEPROM_END + 78,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH2   */  VSCP_EEPROM_END + 79,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH2    */  VSCP_EEPROM_END + 80,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH3 */  VSCP_EEPROM_END + 81,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH3   */  VSCP_EEPROM_END + 81,
    /* REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH3    */  VSCP_EEPROM_END + 82,
    /* REG1_VILNIUS_CH0_LINEARIZATION_K_0           */  VSCP_EEPROM_END + 83,
    /* REG1_VILNIUS_CH0_LINEARIZATION_K_1           */  VSCP_EEPROM_END + 84,
    /* REG1_VILNIUS_CH0_LINEARIZATION_K_2           */  VSCP_EEPROM_END + 85,
    /* REG1_VILNIUS_CH0_LINEARIZATION_K_3           */  VSCP_EEPROM_END + 86,
    /* REG1_VILNIUS_CH1_LINEARIZATION_K_0           */  VSCP_EEPROM_END + 87,
    /* REG1_VILNIUS_CH1_LINEARIZATION_K_1           */  VSCP_EEPROM_END + 88,
    /* REG1_VILNIUS_CH1_LINEARIZATION_K_2           */  VSCP_EEPROM_END + 89,
    /* REG1_VILNIUS_CH1_LINEARIZATION_K_3           */  VSCP_EEPROM_END + 90,
    /* REG1_VILNIUS_CH2_LINEARIZATION_K_0           */  VSCP_EEPROM_END + 91,
    /* REG1_VILNIUS_CH2_LINEARIZATION_K_1           */  VSCP_EEPROM_END + 92,
    /* REG1_VILNIUS_CH2_LINEARIZATION_K_2           */  VSCP_EEPROM_END + 93,
    /* REG1_VILNIUS_CH2_LINEARIZATION_K_3           */  VSCP_EEPROM_END + 94,
    /* REG1_VILNIUS_CH3_LINEARIZATION_K_0           */  VSCP_EEPROM_END + 95,
    /* REG1_VILNIUS_CH3_LINEARIZATION_K_1           */  VSCP_EEPROM_END + 96,
    /* REG1_VILNIUS_CH3_LINEARIZATION_K_2           */  VSCP_EEPROM_END + 97,
    /* REG1_VILNIUS_CH3_LINEARIZATION_K_3           */  VSCP_EEPROM_END + 98,
    /* REG1_VILNIUS_CH0_LINEARIZATION_M_0           */  VSCP_EEPROM_END + 99,
    /* REG1_VILNIUS_CH0_LINEARIZATION_M_1           */  VSCP_EEPROM_END + 100,
    /* REG1_VILNIUS_CH0_LINEARIZATION_M_2           */  VSCP_EEPROM_END + 101,
    /* REG1_VILNIUS_CH0_LINEARIZATION_M_3           */  VSCP_EEPROM_END + 102,
    /* REG1_VILNIUS_CH1_LINEARIZATION_M_0           */  VSCP_EEPROM_END + 103,
    /* REG1_VILNIUS_CH1_LINEARIZATION_M_1           */  VSCP_EEPROM_END + 104,
    /* REG1_VILNIUS_CH1_LINEARIZATION_M_2           */  VSCP_EEPROM_END + 105,
    /* REG1_VILNIUS_CH1_LINEARIZATION_M_3           */  VSCP_EEPROM_END + 106,
    /* REG1_VILNIUS_CH2_LINEARIZATION_M_0           */  VSCP_EEPROM_END + 107,
    /* REG1_VILNIUS_CH2_LINEARIZATION_M_1           */  VSCP_EEPROM_END + 108,
    /* REG1_VILNIUS_CH2_LINEARIZATION_M_2           */  VSCP_EEPROM_END + 109,
    /* REG1_VILNIUS_CH2_LINEARIZATION_M_3           */  VSCP_EEPROM_END + 110,
    /* REG1_VILNIUS_CH3_LINEARIZATION_M_0           */  VSCP_EEPROM_END + 111,
    /* REG1_VILNIUS_CH3_LINEARIZATION_M_1           */  VSCP_EEPROM_END + 112,
    /* REG1_VILNIUS_CH3_LINEARIZATION_M_2           */  VSCP_EEPROM_END + 113,
    /* REG1_VILNIUS_CH3_LINEARIZATION_M_3           */  VSCP_EEPROM_END + 114,
};

#define DECISION_MATRIX_EEPROM_START                    VSCP_EEPROM_END + 115     // always one after above

///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine
//      	- Services Timer0 Overflow
//      	- Services GP3 Pin Change
//////////////////////////////////////////////////////////////////////////////

void interrupt low_priority  interrupt_at_low_vector( void )
{
    // Clock
    if ( INTCONbits.TMR0IF ) { // If a Timer0 Interrupt, Then...

        // Reload value for 1 ms resolution
        WriteTimer0(TIMER0_RELOAD_VALUE);
        
        vscp_timer++;
        vscp_configtimer++;
        measurement_clock_sec++;
        sendTimer++;
        valueReports[ 0 ]++;
        valueReports[ 1 ]++;
        valueReports[ 2 ]++;
        valueReports[ 3 ]++;
        measurementReports[ 0 ]++;
        measurementReports[ 1 ]++;
        measurementReports[ 2 ]++;
        measurementReports[ 3 ]++;

        // Check for init. button
        if ( INIT_BUTTON ) {
            vscp_initbtncnt = 0;
        } 
        else {
            // Active
            vscp_initbtncnt++;
        }

        // Status LED
        vscp_statuscnt++;
        if ( ( VSCP_LED_BLINK1 == vscp_initledfunc ) &&
                ( vscp_statuscnt > 100 ) ) {

            if ( STATUS_LED ) {
                STATUS_LED = 0;
            }
            else {
                STATUS_LED = 1;
            }

            vscp_statuscnt = 0;

        }
        else if (VSCP_LED_ON == vscp_initledfunc) {
            STATUS_LED = 1;
            vscp_statuscnt = 0;
        }
        else if (VSCP_LED_OFF == vscp_initledfunc) {
            STATUS_LED = 0;
            vscp_statuscnt = 0;
        }

        INTCONbits.TMR0IF = 0; // Clear Timer0 Interrupt Flag

    }
    
    // Check ADC
    if ( PIR1bits.ADIF ) {

        switch (0x3C & ADCON0) {

            case SELECT_ADC0:
                // Read conversion
                analog_value[ 3 ] = ADRES;

                // Start new conversion
                ADCON0 = SELECT_ADC1 + 1;
                break;

            case SELECT_ADC1:
                // Read conversion
                analog_value[ 2 ] = ADRES;

                // Start new conversion
                ADCON0 = SELECT_ADC2 + 1;
                break;

            case SELECT_ADC2:
                // Read conversion
                analog_value[ 1 ] = ADRES;

                // Start new conversion
                ADCON0 = SELECT_ADC3 + 1;
                break;

            case SELECT_ADC3:
                // Read conversion
                analog_value[ 0 ] = ADRES;
                
                // Start new conversion
                ADCON0 = SELECT_ADC0 + 1;
                break;

            default:
                // Start new conversion
                ADCON0 = SELECT_ADC0 + 1;
                break;
        }

        // Start conversion
        ConvertADC();

        PIR1bits.ADIF = 0; // Reset interrupt flag

    } // ADC
    
    return;
}

///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine
//      	- External interrupt 
//      	
//////////////////////////////////////////////////////////////////////////////

/*void interrupt high_priority  interrupt_at_high_vector( void )
{
    
    return;
}*/


//***************************************************************************
// Main() - Main Routine
//***************************************************************************


void main()
{
    init();         // Initialize Micro controller
    
    // Check VSCP persistent storage and
    // restore if needed
    if ( !vscp_check_pstorage() ) {

        // Spoiled or not initialized - reinitialize
        init_app_eeprom();

    }
    
    // Initialize data
    init_app_ram();
    
    // Initialize the VSCP functionality
    vscp_init();    
    
    // Set DM filters
    calculateSetFilterMask();
            
    while ( 1 ) {   // Loop Forever

        ClrWdt();   // Feed the dog

        if ( ( vscp_initbtncnt > 2500 ) &&
             ( VSCP_STATE_INIT != vscp_node_state ) ) {

            // Init. button pressed
            vscp_nickname = VSCP_ADDRESS_FREE;
            eeprom_write( VSCP_EEPROM_NICKNAME, VSCP_ADDRESS_FREE );
            vscp_init();
            
        }

        // Check for a valid  event
        vscp_imsg.flags = 0;
        vscp_getEvent();

        switch ( vscp_node_state ) {

            case VSCP_STATE_STARTUP: // Cold/warm reset

                // Get nickname from EEPROM
                if (VSCP_ADDRESS_FREE == vscp_nickname) {
                    // new on segment need a nickname
                    vscp_node_state = VSCP_STATE_INIT;
                } 
                else {
                    // been here before - go on
                    vscp_node_state = VSCP_STATE_ACTIVE;
                    vscp_goActiveState();
                }
                break;

            case VSCP_STATE_INIT: // Assigning nickname
                vscp_handleProbeState();
                break;

            case VSCP_STATE_PREACTIVE:  // Waiting for host initialization
                vscp_goActiveState();
                break;

            case VSCP_STATE_ACTIVE:     // The normal state

                // Check for incoming event?
                if (vscp_imsg.flags & VSCP_VALID_MSG) {

                    if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class  ) {

                        // Handle protocol event
                        vscp_handleProtocolEvent();

                    }
                    else {    

                        doDM();
                        
                        // Check for sync request
                        if ( ( VSCP_CLASS1_CONTROL == vscp_imsg.vscp_class ) &&
                                ( VSCP_TYPE_CONTROL_SYNC == vscp_imsg.vscp_type ) ) {
                            
                            handleSyncRequest( vscp_imsg.data[ 0 ], 
                                                    vscp_imsg.data[ 1 ], 
                                                    vscp_imsg.data[ 2 ] );
                            
                        }
                        
                    }
                    
                }               
                break;

            case VSCP_STATE_ERROR: // Everything is *very* *very* bad.
                vscp_error();
                break;

            default: // Should not be here...
                vscp_node_state = VSCP_STATE_STARTUP;
                break;

        }

        // do a measurement if needed
        if ( measurement_clock_sec > 1000 ) {

            measurement_clock_sec = 0;
 
            // Do VSCP one second jobs
            vscp_doOneSecondWork();

            if ( VSCP_STATE_ACTIVE == vscp_node_state ) {
                // Do VSCP one second jobs
                doApplicationOneSecondWork();
            }

        }

        // Timekeeping
        if ( seconds > 59 ) {

            seconds = 0;
            minutes++;

            if ( minutes > 59 ) {
                minutes = 0;
                hours++;
            }

            if ( hours > 23 ) hours = 0;

        }

        // Do loop work
        doWork();

    } // while
}

///////////////////////////////////////////////////////////////////////////////
// Init. - Initialisation Routine
//  

void init()
{   
    //uint8_t msgdata[ 8 ];

    // Initialise the uP
    
    // PORTA
    // RA0/AN0 - input
    // RA1/AN1 - input
    // RA2/AN2 - input
    // RA3/AN3 - input
    // RA4 input (VCAP for PIC18F26K80))
    TRISA = 0x1F;
    
    // RC0 - Input  - Init. button
    // RC1 - Output - Status LED - Default off
    // RC2 - Output - Not used = output..
    // RC3 - Output - Not used = output.
    // RC4 - Input - I/O channel 0.
    // RC5 - Input - I/O channel 1.
    // RC6 - Output - Not used = output.
    // RC7 - Output - Not used = output.
    TRISC = 0b00000001;
    PORTC = 0x00;   // Default off

    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8 );
    WriteTimer0( TIMER0_RELOAD_VALUE );
    
#if defined(_18F2580)     
    OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD,
            ADC_CH0 & ADC_INT_ON & ADC_11ANA &
            ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
            15);
#else if defined(_18F25K80) || defined(_18F26K80) || defined(_18F45K80) || defined(_18F46K80) || defined(_18F65K80) || defined(_18F66K80)
    OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD,
                ADC_CH0 & ADC_INT_ON,
                15);
#endif   

    // Initialize CAN
    ECANInitialize();

    // Must be in config. mode to change many of settings.
    //ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

    // Return to Normal mode to communicate.
    //ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

    /*
        msgdata[ 0 ] = 1;
            msgdata[ 1 ] = 2;
            msgdata[ 2 ] = 3;
            msgdata[ 3 ] = 4;

            if ( vscp18f_sendMsg( 0x123,  msgdata, 4, CAN_TX_PRIORITY_0 & CAN_TX_XTD_FRAME & CAN_TX_NO_RTR_FRAME ) ) {
                    ;
            }

     */
        
    // Enable global interrupt
    INTCONbits.GIE = 1;

    ConvertADC();

    return;
}

///////////////////////////////////////////////////////////////////////////////
// init_app_ram
//

void init_app_ram( void )
{    
    measurement_clock_sec = 0;      // start a new measurement cycle
    
    seconds = 0;
    minutes = 0;
    hours = 0;
    
    memset( analog_value, 0, sizeof( analog_value ) );
    
    memset( bLowAlarm, 0, sizeof( bLowAlarm ) );
    memset( bHighAlarm, 0, sizeof( bHighAlarm ) );
    
    memset( valueReports, 0, sizeof( valueReports ) );
    memset( measurementReports, 0, sizeof( measurementReports ) );
    
    IOsavebit0 = FALSE;
    IOsavebit1 = FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
//

void init_app_eeprom(void)
{
    unsigned char i, j;

    // Module zone/sub zone
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ZONE ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_SUBZONE ], 0 );
    
    // Channel zone/sub zone
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH0_ZONE ], 1 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH0_SUBZONE ], 1 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH1_ZONE ], 2 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH1_SUBZONE ], 2 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH2_ZONE ], 3 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH2_SUBZONE ], 3 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH3_ZONE ], 4 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CH3_SUBZONE ], 4 );
    
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_IO0_ZONE ], 5 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_IO0_SUBZONE ], 5 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_IO1_ZONE ], 6 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_IO1_SUBZONE ], 6 );
    
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_MODULE ], DEFAULT_VILNIUS_CONTROL_MODULE );
            
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_CHANNEL0 ], DEFAULT_VILNIUS_CONTROL_CHANNEL0 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_CHANNEL1 ], DEFAULT_VILNIUS_CONTROL_CHANNEL0 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_CHANNEL2 ], DEFAULT_VILNIUS_CONTROL_CHANNEL0 );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_CHANNEL3 ], DEFAULT_VILNIUS_CONTROL_CHANNEL0 );
            
    for ( int i=0; i<8; i++ ) {
        eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH0_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_LOW_CHANNEL0_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_HIGH_CHANNEL0_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_MSB + i ], 0 );
    }
    
    for ( int i=0; i<8; i++ ) {
        eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_MSB + i ], 0xFF );
    }
    
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_HYSTERESIS_CH0_MSB ], DEFAULT_AD0_HYSTERESIS );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_HYSTERESIS_CH1_MSB ], DEFAULT_AD1_HYSTERESIS );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_HYSTERESIS_CH2_MSB ], DEFAULT_AD2_HYSTERESIS );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_HYSTERESIS_CH3_MSB ], DEFAULT_AD3_HYSTERESIS );
        
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ], DEFAULT_IO_CONTROL );
    eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_IO_STATE ], 0 );
    
    // Page 1
    
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0 ], DEFAULT_LINEARIZATION_EVENT_SETTING );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH1 ], DEFAULT_LINEARIZATION_EVENT_SETTING );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH2 ], DEFAULT_LINEARIZATION_EVENT_SETTING );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH3 ], DEFAULT_LINEARIZATION_EVENT_SETTING );
    
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH0 ], DEFAULT_LINEARIZATION_EVENT_CLASS );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH1 ], DEFAULT_LINEARIZATION_EVENT_CLASS );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH2 ], DEFAULT_LINEARIZATION_EVENT_CLASS );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH3 ], DEFAULT_LINEARIZATION_EVENT_CLASS );
    
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH0 ], DEFAULT_LINEARIZATION_EVENT_TYPE );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH1 ], DEFAULT_LINEARIZATION_EVENT_TYPE );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH2 ], DEFAULT_LINEARIZATION_EVENT_TYPE );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH3 ], DEFAULT_LINEARIZATION_EVENT_TYPE );
    
    // We set k = 1 and m = 0
    for ( int i=REG1_VILNIUS_CH0_LINEARIZATION_K_0;
            i<=REG1_VILNIUS_CH3_LINEARIZATION_M_LSB;
            i++ ) {
        eeprom_write( reg2eeprom_pg1[ i ], 0 );
    }
    
    // 32-t floating point little endian 1 = 0x00, 0x00, 0x80, 0x3f
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_K_1 ], 0x80 );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_K_0 ], 0x3f );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH1_LINEARIZATION_K_1 ], 0x80 );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH1_LINEARIZATION_K_0 ], 0x3f );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH2_LINEARIZATION_K_1 ], 0x80 );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH2_LINEARIZATION_K_0 ], 0x3f );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH3_LINEARIZATION_K_1 ], 0x80 );
    eeprom_write( reg2eeprom_pg1[ REG1_VILNIUS_CH3_LINEARIZATION_K_0 ], 0x3f );
    
    // * * * Decision Matrix * * *
    
    // All elements disabled.
    for ( i = 0; i<DESCION_MATRIX_ROWS; i++ ) {
        for ( j = 0; j < 8; j++ ) {
            eeprom_write( DECISION_MATRIX_EEPROM_START + 8*i  + j, 0 );
        }
    }
    
    // Set DM filters
    calculateSetFilterMask();

}

///////////////////////////////////////////////////////////////////////////////
// doWork
//
// The actual work is done here.
//

void doWork(void)
{
    uint16_t report_interval;
    
    // For each analog channel
    for ( int i=0; i<4; i++ ) {
        
        //*********************************************************************
        // Check if this is the lowest reading ever
        //*********************************************************************
        
        if ( analog_value[ i ] < 
                construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_MSB + 2*i ] ),
                                                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_LSB + 2*i ] ) ) ) {
            // Store new lowest value
            eeprom_write( reg2eeprom_pg0[REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_MSB + 2*i ], 
                            ( analog_value[ i ] >> 8 ) );
            eeprom_write( reg2eeprom_pg0[REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_LSB + 2*i ], 
                            ( analog_value[ i ] & 0xff ) );
            
        }

        //*********************************************************************
        // Check if this is the highest reading ever
        //*********************************************************************
        
        if (analog_value[ i ] > 
                construct_unsigned16( eeprom_read( reg2eeprom_pg0[REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_MSB + 2*i ] ), 
                                                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_LSB + 2*i ] ) ) ) {
            // Store new lowest value
            eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_MSB + 2*i ], 
                            ( analog_value[ i ] >> 8 ) );
            eeprom_write( reg2eeprom_pg0[ REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_LSB + 2*i ], 
                         ( analog_value[ i ] & 0xff ) );
        }
        
        //*********************************************************************
        // Periodic events
        //*********************************************************************
        
        report_interval = 
                construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_MSB + 2*i ] ),
                                                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_LSB + 2*i ] ) );
        
        // Must be enabled  )
        if ( report_interval && ( valueReports[ i ] > report_interval ) ) {
            
                valueReports[ i ] = 0;  // Reset
                SendDataEvent( VSCP_TYPE_DATA_AD, i, analog_value[ i ] );
                
        }
        
        //*********************************************************************
        // Low alarm
        //*********************************************************************
        
        // Must be enabled
        if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_CHANNEL0 + i ] ) & 
                                                CONFIG_AD_LOW_ALARM_ENABLE ) {
            
            // No alarm
            if ( !bLowAlarm[ i ] ) {
             
                if ( analog_value[ i ] <
                        construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_LOW_CHANNEL0_MSB + 2*i ] ),
                                              eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_LOW_CHANNEL0_LSB + 2*i ] ) ) ) {
                    bLowAlarm[ i ] = TRUE;                  // Mark alarm condition
                    vscp_alarmstatus |= VSCP_ALARM_LOW;     // Set VSCP alarm bit
                    SendAlarmEvent( i );
                }
                
            }
            // Already alarm
            else {
                
                // Check if we should reset alarm
                if ( analog_value[ i ] >
                        ( construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_LOW_CHANNEL0_MSB + 2*i ] ),
                                                eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_LOW_CHANNEL0_LSB + 2*i ] ) ) +
                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_HYSTERESIS_CH0_MSB + i ] ) ) ) {
                    
                    bLowAlarm[ i ] = FALSE;   // Reset alarm condition
                    
                }
                
            }
            
        }
        
        //*********************************************************************
        // High alarm
        //*********************************************************************
        
        // Must be enabled
        if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_CHANNEL0 + i ] ) & 
                                                CONFIG_AD_HIGH_ALARM_ENABLE ) {
            
            // No alarm
            if ( !bHighAlarm[ i ] ) {
             
                if ( analog_value[ i ] >
                        construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_HIGH_CHANNEL0_MSB + 2*i ] ),
                                              eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_HIGH_CHANNEL0_LSB + 2*i ] ) ) ) {
                    bHighAlarm[ i ] = TRUE;                  // Mark alarm condition
                    vscp_alarmstatus |= VSCP_ALARM_HIGH;     // Set VSCP alarm bit
                    SendAlarmEvent( i + 0x80 );              // Bit 7 marks high alarm
                }
                
            }
            // Already alarm
            else {
                
                // Check if we should reset alarm
                if ( analog_value[ i ] <
                        ( construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_HIGH_CHANNEL0_MSB + 2*i ] ),
                                                eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ALARM_HIGH_CHANNEL0_LSB + 2*i ] ) ) -
                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_HYSTERESIS_CH0_MSB + i ] ) ) ) {
                    
                    bHighAlarm[ i ] = FALSE;   // Reset alarm condition
                    
                }
                
            }            
            
        }
        
        //*********************************************************************
        // Periodic measurement events
        //*********************************************************************
        
        report_interval = 
                construct_unsigned16( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH0_MSB + 2*i ] ),
                                                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH0_LSB + 2*i ] ) );
        
        // Must be enabled >0 
        if ( report_interval && ( measurementReports[ i ] > report_interval ) ) {
        
            sendMeasurementEvent( i );
            
        }
        
    }
    
    //*********************************************************************
    // Check if I/O-0 should be checked for changes
    //*********************************************************************
    
    // Must be enabled
    if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) & 
                                            CONFIG_IO0_EVENT_ON_CHANGE ) {
        
        // Must be input (Bit == 1)
        if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) & 
                                            CONFIG_IO0_DIRECTION ) {
            
            // Was set last check
            if ( IOsavebit0 ) {
                
                // If not set now send events 
                if ( !IO0_INPUT ) {
                    
                    if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) &  
                                            CONFIG_IO0_EVENT_SELECT ) {
                        
                        SendInformationEvent( 0, 
                                                VSCP_CLASS1_CONTROL,
                                                VSCP_TYPE_CONTROL_TURNOFF );
                    }   
                    else {
                        SendInformationEvent( 0, 
                                                VSCP_CLASS1_INFORMATION,
                                                VSCP_TYPE_INFORMATION_OFF );
                    }
                    
                }
                
            }
            // was cleared last check
            else {
                
                // If set now send events 
                if ( IO0_INPUT ) {
                    
                    if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) &  
                                            CONFIG_IO0_EVENT_SELECT ) {
                        
                        SendInformationEvent( 0, 
                                                VSCP_CLASS1_CONTROL,
                                                VSCP_TYPE_CONTROL_TURNON );
                    }   
                    else {
                        SendInformationEvent( 0, 
                                                VSCP_CLASS1_INFORMATION,
                                                VSCP_TYPE_INFORMATION_ON );
                    }
                    
                }
                
            }
            
            // Save state
            IOsavebit0 = IO0_INPUT;
            
        }
        
    }
        
    //*********************************************************************
    // Check if I/O-1 should be checked for changes
    //*********************************************************************
        
    // Must be enabled
    if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) & 
                                            CONFIG_IO1_EVENT_ON_CHANGE ) {
        
        // Must be input (Bit == 1)
        if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) & 
                                            CONFIG_IO1_DIRECTION ) {
            
            // Was set last check
            if ( IOsavebit1 ) {
                
                // If not set now send events 
                if ( !IO1_INPUT ) {
                    
                    if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) &  
                                            CONFIG_IO1_EVENT_SELECT ) {
                        
                        SendInformationEvent( 1, 
                                                VSCP_CLASS1_CONTROL,
                                                VSCP_TYPE_CONTROL_TURNOFF );
                    }   
                    else {
                        SendInformationEvent( 1, 
                                                VSCP_CLASS1_INFORMATION,
                                                VSCP_TYPE_INFORMATION_OFF );
                    }
                    
                }
                
            }
            // was cleared last check
            else {
                
                // If set now send events 
                if ( IO1_INPUT ) {
                    
                    if ( eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CONTROL_IO ] ) &  
                                            CONFIG_IO1_EVENT_SELECT ) {
                        
                        SendInformationEvent( 1, 
                                                VSCP_CLASS1_CONTROL,
                                                VSCP_TYPE_CONTROL_TURNON );
                    }   
                    else {
                        SendInformationEvent( 1, 
                                                VSCP_CLASS1_INFORMATION,
                                                VSCP_TYPE_INFORMATION_ON );
                    }
                    
                }
                
            }
            
            // Save state
            IOsavebit1 = IO1_INPUT;
            
        }
        
    }
        
        
}

///////////////////////////////////////////////////////////////////////////////
// doApplicationOneSecondWork
//

void doApplicationOneSecondWork(void)
{
    ;
}

double calculateMeasurement( uint8_t sendoridx )
{
    uint8_t floatBuf[4];
            
    floatBuf[ 0 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_K_3 + 4*sendoridx ] );
    floatBuf[ 1 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_K_2 + 4*sendoridx ] );
    floatBuf[ 2 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_K_1 + 4*sendoridx ] );
    floatBuf[ 3 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_K_0 + 4*sendoridx ] );
    double k = *((double *)floatBuf);
            
    floatBuf[ 0 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_M_3 + 4*sendoridx ] );
    floatBuf[ 1 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_M_2 + 4*sendoridx ] );
    floatBuf[ 2 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_M_1 + 4*sendoridx ] );
    floatBuf[ 3 ] = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_CH0_LINEARIZATION_M_0 + 4*sendoridx ] );
    double m = *((double *)floatBuf);
                            
    // Do calculation                
    return  (k*analog_value[ sendoridx ]*MODULE_VOLTAGE)/MODULE_AD_MAX_VALUE + m;
}

///////////////////////////////////////////////////////////////////////////////
// SendInformationEvent
//

void SendInformationEvent( unsigned char channel,
                            unsigned char eventClass,
                            unsigned char eventTypeId )
{
    uint8_t data[3];

    data[ 0 ] = channel; // Register
    data[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_IO0_ZONE + 2*( channel & 0x03 ) ] );
    data[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_IO0_ZONE + 2*( channel & 0x03 ) + 1 ] );
    sendVSCPFrame( eventClass,
                    eventTypeId,
                    vscp_nickname,
                    VSCP_PRIORITY_MEDIUM,
                    3,
                    data );
}


///////////////////////////////////////////////////////////////////////////////
// SendDataEvent
//

void SendDataEvent( uint8_t eventType, uint8_t sensoridx, uint16_t val )
{
    uint8_t data[5];

    data[ 0 ] = 0b01100000 + sensoridx; // Coding integer, default unit, channel
    data[ 1 ] = 0; // Send as 32-bit integer to take care of sign - always positive.
    data[ 2 ] = 0;        
    data[ 3 ] = ( val >> 8 ) & 0xff;
    data[ 4 ] = ( val & 0xff );
    sendVSCPFrame( VSCP_CLASS1_DATA,
                    eventType,
                    vscp_nickname,
                    VSCP_PRIORITY_MEDIUM,
                    5,
                    data );
}

///////////////////////////////////////////////////////////////////////////////
// SendAlarmEvent
//

void SendAlarmEvent( uint8_t channel  )
{
    uint8_t data[3];

    data[ 0 ] = channel;    // Channel 0-3 for counter, channel 4-6 for frequency low, channel 7-9 for 
                            // frequency high
    data[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CH0_ZONE + ( channel & 0x03 ) ] );
    data[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_CH0_SUBZONE + ( channel & 0x03 ) ] );
    sendVSCPFrame( VSCP_CLASS1_ALARM,
                    VSCP_TYPE_ALARM_ALARM,
                    vscp_nickname,
                    VSCP_PRIORITY_HIGH,
                    3,
                    data );
}

///////////////////////////////////////////////////////////////////////////////
// Send measurement event for selected channel
//

void sendMeasurementEvent( uint8_t channel )
{
    uint8_t data[8];
    double val;
            
    measurementReports[ channel ] = 0;    // New period
            
    val = calculateMeasurement( channel );
            
    // Floating point value 0b10100000
    // unit from control settings (bits 3,4)
    // Channel as index
    data[ 0 ] = 
            ( 0b10100000 | ( eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0 + 3*channel ] ) & 
                                                        MEASUREMENT_CTRL_UNIT_MASK ) ) + channel; 
            
    // Set data
    uint8_t *p = (uint8_t *)&val;
    data[ 4 ] = p[ 3 ];
    data[ 3 ] = p[ 2 ];
    data[ 2 ] = p[ 1 ];
    data[ 1 ] = p[ 0 ];
            
    uint16_t vscpclass = (( eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0 + 3*channel ] ) & 
                                                    MEASUREMENT_CTRL_CLASS_BIT_8 ) ? 512 : 0 ) +
                                                    eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH0 + 3*channel ] );
    uint8_t vscptype = eeprom_read( reg2eeprom_pg1[ REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH0 + 3*channel ] );
                    
    sendVSCPFrame( vscpclass,
                    vscptype,
                    vscp_nickname,
                    VSCP_PRIORITY_NORMAL,
                    5,
                    data );
}

///////////////////////////////////////////////////////////////////////////////
// Handle sync request
//
// if sensoridx is 0xff send info for all sensors
//      otherwise coded as  type:sensors (each four bits)
//      type = 0 - Lower bits are A/D channels
//      type = 1 - Lower bits are measurements
//      type = 2 - Lower bits are I/O
// if zone is 0xff send for all zones
// if zubzone is 0xff send for all sub zones

void handleSyncRequest( uint8_t sensoridx, uint8_t zone, uint8_t subzone )
{
    // sensoridx coding: type [7,6,5,4], sensors [3,2,1,0]
    BOOL bADChannels = FALSE;
    BOOL bMeasurements = FALSE;
    BOOL bIO = FALSE;
    
    // if zone is 0xff send for all zones if not module zone must match
    if ( ( 0xff != zone ) && ( zone != eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ZONE ] ) ) ) return;
    
    // if subzone is 0xff send for all sub zones if not module sub zone must match
    if ( ( 0xff != subzone ) && ( subzone != eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_SUBZONE ] ) ) ) return;
    
    // Should all sensors be reported
    if ( 0xf0 == ( sensoridx & 0xf0 ) ) {
        // All
        bADChannels = TRUE;
        bMeasurements = TRUE;
        bIO = TRUE;
    }
    
    // A/D channels
    if ( bADChannels ) {     
        for ( int i=0; i<4; i++ ) {            
            if ( sensoridx & ( 1 << i ) ) {
                SendDataEvent( VSCP_TYPE_DATA_AD, i, analog_value[ i ] );
            }            
        }        
    }
    
    // Measurements
    if ( bMeasurements ) {
        for ( int i=0; i<4; i++ ) {
            if ( sensoridx & ( 1 << i ) ) {
                sendMeasurementEvent( i );
            }
        }        
    }
    
    // I/O channels
    if ( bIO ) {     
        uint8_t data[3];

        data[ 0 ] = 0b0110000; // Coding integer, default unit, channel = 0
        data[ 1 ] = 0; // Send as 16-bit integer to take care of sign - always positive.
        data[ 2 ] = IO0_INPUT | ( IO1_INPUT << 1 );        
        sendVSCPFrame( VSCP_CLASS1_DATA,
                    VSCP_TYPE_DATA_IO,
                    vscp_nickname,
                    VSCP_PRIORITY_MEDIUM,
                    3,
                    data );
    }
}


///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char getMajorVersion()
{
    return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char getMinorVersion()
{
    return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char getSubMinorVersion()
{
    return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get GUID from EEPROM
//

#ifdef ENABLE_WRITE_2PROTECTED_LOCATIONS
void vscp_setGUID(uint8_t idx, uint8_t data ) {
    if ( idx>15 ) return;
    eeprom_write(VSCP_EEPROM_REG_GUID + idx, data);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

#ifdef ENABLE_WRITE_2PROTECTED_LOCATIONS
void vscp_setManufacturerId( uint8_t idx, uint8_t data ) {
    if ( idx>7 ) return;
    eeprom_write(VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx, data);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

unsigned char getBootLoaderAlgorithm(void)
{
    return VSCP_BOOTLOADER_PIC1;
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

unsigned char getBufferSize(void)
{
    return 8; // Standard CAN frame
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_readNicknamePermanent
//

uint8_t vscp_readNicknamePermanent(void)
{
    return eeprom_read( VSCP_EEPROM_NICKNAME );
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_writeNicknamePermanent
//

void vscp_writeNicknamePermanent(uint8_t nickname)
{
    eeprom_write( VSCP_EEPROM_NICKNAME, nickname );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getZone
//

uint8_t vscp_getZone(void)
{
    return eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ZONE ] );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getSubzone
//

uint8_t vscp_getSubzone(void)
{
    return eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_SUBZONE ] );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_readAppReg
//

uint8_t vscp_readAppReg(uint8_t reg)
{    
    uint8_t rv;

    rv = 0x00; // default read
    
    // * * *  Page 0  * * *
    if ( 0 == vscp_page_select ) {

        // Channel (sub)zones & control registers
        if ( ( reg >= REG0_VILNIUS_ZONE ) && 
                    ( reg <= REG0_VILNIUS_CONTROL_CHANNEL3 ) ) {
            rv = eeprom_read( reg2eeprom_pg0[ reg ] );
        }
        // A/D reading channel 0 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL0_MSB ) {
            rv = ( analog_value[ 0 ] >> 8 ) & 0xff;            
        }
        // A/D reading channel 0 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL0_LSB ) {
            rv = analog_value[ 0 ] & 0xff;
        }
        // A/D reading channel 1 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL1_MSB ) {
            rv = ( analog_value[ 1 ] >> 8 ) & 0xff;
        }
        // A/D reading channel 1 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL1_LSB ) {
            rv = analog_value[ 1 ] & 0xff;
        }
        // A/D reading channel 2 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL2_MSB ) {
            rv = ( analog_value[ 2 ] >> 8 ) & 0xff;
        }
        // A/D reading channel 2 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL2_LSB ) {
            rv = analog_value[ 2 ] & 0xff;
        }
        // A/D reading channel 3 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL3_MSB ) {
            rv = ( analog_value[ 3 ] >> 8 ) & 0xff;
        }
        // A/D reading channel 3 MSB
        else if ( reg == REG0_VILNIUS_AD_VALUE_CHANNEL3_LSB ) {
            rv = analog_value[ 3 ] & 0xff;
        }
        // 
        else if ( ( reg >= REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_MSB ) && 
                    ( reg <= REG0_VILNIUS_CONTROL_IO ) ) {
            rv = eeprom_read( reg2eeprom_pg0[ reg ] );
        }
        // IO status register
        else if ( reg == REG0_VILNIUS_IO_STATE ) {
            rv = IO0_INPUT | ( IO1_INPUT << 1 );
        }
        
    } // page 0
    
    // * * *  Page 1  * * *
    else if ( 1 == vscp_page_select ) {
        
        if ( ( reg >= REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0 ) && 
                    ( reg <= REG1_VILNIUS_CH3_LINEARIZATION_M_LSB ) ) {
            rv = eeprom_read( reg2eeprom_pg1[ reg ] );
        }
        else if ( ( reg >= REG1_VILNIUS_CH0_MEASUREMENT_VALUE_MSB ) &&
            ( reg <= REG1_VILNIUS_CH0_MEASUREMENT_VALUE_LSB ) ) {
            
            double val;           
            val = calculateMeasurement( 0 );
            
            // Set data
            uint8_t *p = (uint8_t *)&val;
            rv = p[ reg - REG1_VILNIUS_CH0_MEASUREMENT_VALUE_MSB ];
            
        }
        else if ( ( reg >= REG1_VILNIUS_CH1_MEASUREMENT_VALUE_MSB ) &&
            ( reg <= REG1_VILNIUS_CH1_MEASUREMENT_VALUE_LSB ) ) {
            
            double val;            
            val = calculateMeasurement( 1 );
            
            // Set data
            uint8_t *p = (uint8_t *)&val;
            rv = p[ reg - REG1_VILNIUS_CH1_MEASUREMENT_VALUE_MSB ];
            
        }
        else if ( ( reg >= REG1_VILNIUS_CH2_MEASUREMENT_VALUE_MSB ) &&
            ( reg <= REG1_VILNIUS_CH2_MEASUREMENT_VALUE_LSB ) ) {
            
            double val;            
            val = calculateMeasurement( 2 );
            
            // Set data
            uint8_t *p = (uint8_t *)&val;
            rv = p[ reg - REG1_VILNIUS_CH2_MEASUREMENT_VALUE_MSB ];
            
        }
        else if ( ( reg >= REG1_VILNIUS_CH3_MEASUREMENT_VALUE_MSB ) &&
            ( reg <= REG1_VILNIUS_CH3_MEASUREMENT_VALUE_LSB ) ) {
            
            double val;            
            val = calculateMeasurement( 3 );
            
            // Set data
            uint8_t *p = (uint8_t *)&val;
            rv = p[ reg - REG1_VILNIUS_CH3_MEASUREMENT_VALUE_MSB ];
            
        }
        
    } // page 1
    
    // * * *  Page 2  * * *
    else if ( 2 == vscp_page_select ) {
        
        if ( reg < ( REG_DESCION_MATRIX + 8*DESCION_MATRIX_ROWS ) ) {
            rv = eeprom_read( DECISION_MATRIX_EEPROM_START + reg );
        }
        
    } // page 2
    
    return rv;

}

///////////////////////////////////////////////////////////////////////////////
// vscp_writeAppReg
//

uint8_t vscp_writeAppReg( uint8_t reg, uint8_t val )
{
    uint8_t rv;

    rv = ~val; // error return
    
    // * * *  Page 0  * * *
    if ( 0 == vscp_page_select ) {

        // Zone
        if ( ( reg >= REG0_VILNIUS_ZONE ) && 
                       ( reg <= REG0_VILNIUS_CONTROL_CHANNEL3 ) ) {
            eeprom_write( reg2eeprom_pg0[ reg ], val );
            rv = eeprom_read( reg2eeprom_pg0[ reg ] );
        }
        else if ( ( reg >= REG0_VILNIUS_AD_VALUE_CHANNEL0_MSB ) && 
                       ( reg <= REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL3_LSB ) ) {            
            eeprom_write( reg2eeprom_pg0[ reg ], val );
            rv = eeprom_read( reg2eeprom_pg0[ reg ] );
        }
        // IO control register
        else if ( reg == REG0_VILNIUS_CONTROL_IO ) {
            IO0_DIR = ( val & 1);
            IO1_DIR = ( (val >> 1) & 1);
            eeprom_write( reg2eeprom_pg0[ reg ], val );
            rv = eeprom_read( reg2eeprom_pg0[ reg ] );
        }
        // IO status register
        else if ( reg == REG0_VILNIUS_IO_STATE ) {
            
            IO0_OUTPUT = ( val & 0x01 ) ? 1 : 0;
            IO1_OUTPUT = ( val & 0x02 ) ? 1 : 0;
            
            rv =  ( IO1_INPUT << 1 ) | IO0_INPUT;
            
        }
        
    } // page 0
    
    // * * *  Page 1  * * *
    else if ( 1 == vscp_page_select ) {
    
        
        if ( ( reg >= REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0 ) && 
                    ( reg <= REG1_VILNIUS_CH3_LINEARIZATION_M_LSB ) ) {
            eeprom_write( reg2eeprom_pg1[ reg ], val );
            rv = eeprom_read( reg2eeprom_pg1[ reg ] );
        }
        
    } // page 1
    
    // * * *  Page 2  * * *
    else if ( 3 == vscp_page_select ) {
    
        if ( reg < (REG_DESCION_MATRIX + 8 * DESCION_MATRIX_ROWS  ) ) {
            eeprom_write( DECISION_MATRIX_EEPROM_START + reg, val );
            calculateSetFilterMask();  // Calculate new hardware filter
            rv = eeprom_read( DECISION_MATRIX_EEPROM_START + reg );
        }
        
    } // page 2
        
    // --------------------------------------------------------------------------

    return rv;

}

///////////////////////////////////////////////////////////////////////////////
// Send Decision Matrix Information
//

void sendDMatrixInfo(void)
{
    vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
    vscp_omsg.flags = VSCP_VALID_MSG + 2;
    vscp_omsg.vscp_class = VSCP_CLASS1_PROTOCOL;
    vscp_omsg.vscp_type = VSCP_TYPE_PROTOCOL_GET_MATRIX_INFO_RESPONSE;

    vscp_omsg.data[ 0 ] = DESCION_MATRIX_ROWS;
    vscp_omsg.data[ 1 ] = REG_DESCION_MATRIX;

    vscp_sendEvent(); // Send data
}






///////////////////////////////////////////////////////////////////////////////
// Do decision Matrix handling
// 
// The routine expects vscp_imsg to contain a valid incoming event
//

void doDM(void)
{
    unsigned char i;
    unsigned char dmflags;
    unsigned short class_filter;
    unsigned short class_mask;
    unsigned char type_filter;
    unsigned char type_mask;

    // Don't deal with the protocol functionality
    if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class ) return;

    for (i = 0; i<DESCION_MATRIX_ROWS; i++) {

        // Get DM flags for this row
        dmflags = eeprom_read( DECISION_MATRIX_EEPROM_START + (8 * i) );

        // Is the DM row enabled?
        if ( dmflags & VSCP_DM_FLAG_ENABLED ) {

            // Should the originating id be checked and if so is it the same?
            if ( ( dmflags & VSCP_DM_FLAG_CHECK_OADDR ) &&
                    ( vscp_imsg.oaddr != eeprom_read( DECISION_MATRIX_EEPROM_START + (8 * i) ) ) ) {
                continue;
            }

            // Check if zone should match and if so if it match
            if ( dmflags & VSCP_DM_FLAG_CHECK_ZONE ) {
                if ( 255 != vscp_imsg.data[ 1 ] ) {
                    if ( vscp_imsg.data[ 1 ] != eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ZONE ] ) ) {
                        continue;
                    }
                }
            }

            // Check if sub zone should match and if so if it match
            if ( dmflags & VSCP_DM_FLAG_CHECK_SUBZONE ) {
                if ( 255 != vscp_imsg.data[ 2 ] ) {
                    if ( vscp_imsg.data[ 2 ] != eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_SUBZONE ] ) ) {
                        continue;
                    }
                }
            }
            
            class_filter = ( ( dmflags & VSCP_DM_FLAG_CLASS_FILTER) << 8 ) +
                    eeprom_read( DECISION_MATRIX_EEPROM_START +
                                    (8 * i) +
                                    VSCP_DM_POS_CLASSFILTER );
            
            class_mask = ( ( dmflags & VSCP_DM_FLAG_CLASS_MASK ) << 7 ) +
                    eeprom_read( DECISION_MATRIX_EEPROM_START +
                                    (8 * i) +
                                    VSCP_DM_POS_CLASSMASK);
            
            type_filter = eeprom_read( DECISION_MATRIX_EEPROM_START +
                                        (8 * i) +
                                        VSCP_DM_POS_TYPEFILTER);
            
            type_mask = eeprom_read( DECISION_MATRIX_EEPROM_START +
                                        (8 * i) +
                                        VSCP_DM_POS_TYPEMASK);
                           
            if ( !( ( class_filter ^ vscp_imsg.vscp_class ) & class_mask ) &&
                    !( ( type_filter ^ vscp_imsg.vscp_type ) & type_mask ) ) {

                // OK Trigger this action
                switch ( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTION ) ) {

                    case VILNIUS_ACTION_NOOP:
                        break;
                        
                    case VILNIUS_ACTION_SET:
                        actionSet( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTIONPARAM ) );
                        break;
        
                    case VILNIUS_ACTION_CLEAR:
                        actionClear( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTIONPARAM ) );
                        break;

                    case VILNIUS_ACTION_TOGGLE:
                        actionToggle( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTIONPARAM ) );
                        break;
                        
                    case VILNIUS_ACTION_STATUS:
                        actionStatus( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTIONPARAM ) );
                        break;
                        
                    case VILNIUS_ACTION_STATUSALL:
                        actionStatusAll( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTIONPARAM ) );
                        break;
                        
                } // case
                
            } // Filter/mask
            
        } // Row enabled
        
    } // for each row
    
}

///////////////////////////////////////////////////////////////////////////////
// actionSet
//

void actionSet( uint8_t param )
{
    if ( param & 1 ) {
        IO0_OUTPUT = 1;
    }
    
    if ( param & 2 ) {
        IO1_OUTPUT = 1;
    }
}

///////////////////////////////////////////////////////////////////////////////
// actionClear
//

void actionClear( uint8_t param )
{
    if ( param & 1 ) {
        IO0_OUTPUT = 0;
    }
    
    if ( param & 2 ) {
        IO1_OUTPUT = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////
// actionToggle
//

void actionToggle( uint8_t param )
{
    if ( param & 1 ) {
        IO0_OUTPUT = IO0_OUTPUT ? 0 : 1;
    }
    
    if ( param & 2 ) {
        IO1_OUTPUT = IO1_OUTPUT ? 0 : 1;
    }
}

///////////////////////////////////////////////////////////////////////////////
// actionStatus
//

void actionStatus( uint8_t param )
{
    handleSyncRequest( param, 
                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ZONE ] ), 
                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_SUBZONE ] ) );
}

///////////////////////////////////////////////////////////////////////////////
// actionStatusAll
//

void actionStatusAll( uint8_t param )
{
    handleSyncRequest( 0xff, 
                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_ZONE ] ), 
                        eeprom_read( reg2eeprom_pg0[ REG0_VILNIUS_SUBZONE ] ) );
}



///////////////////////////////////////////////////////////////////////////////
//                        VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char vscp_getMajorVersion()
{
    return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char vscp_getMinorVersion()
{
    return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char vscp_getSubMinorVersion()
{
    return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// getVSCP_GUID
//
// Get GUID from EEPROM
//

uint8_t vscp_getGUID(uint8_t idx)
{
    return eeprom_read( VSCP_EEPROM_REG_GUID + idx );
}


///////////////////////////////////////////////////////////////////////////////
// getDeviceURL
//
// Get device URL from EEPROM
//

uint8_t vscp_getMDF_URL(uint8_t idx)
{
    return vscp_deviceURL[ idx ];
}

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getUserID(uint8_t idx)
{
    return eeprom_read( VSCP_EEPROM_REG_USERID + idx );
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPUserID
//

void vscp_setUserID(uint8_t idx, uint8_t data)
{
    eeprom_write( idx + VSCP_EEPROM_REG_USERID, data );
}

///////////////////////////////////////////////////////////////////////////////
// getVSCPManufacturerId
// 
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getManufacturerId(uint8_t idx)
{
    return eeprom_read( VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx );
}

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

uint8_t vscp_getBootLoaderAlgorithm(void)
{
    return VSCP_BOOTLOADER_PIC1;
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

uint8_t vscp_getBufferSize(void)
{
    return 8; // Standard CAN frame
}


///////////////////////////////////////////////////////////////////////////////
//  getNickname
//

uint8_t vscp_getNickname(void)
{
    return eeprom_read(VSCP_EEPROM_NICKNAME);
}

///////////////////////////////////////////////////////////////////////////////
//  setNickname
//

void vscp_setNickname(uint8_t nickname)
{
    eeprom_write(VSCP_EEPROM_NICKNAME, nickname);
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPControlByte
//

void vscp_setControlByte( uint8_t idx, uint8_t ctrl)
{
    if ( idx > 1) return;
    eeprom_write(VSCP_EEPROM_CONTROL0 + idx, ctrl);
}


///////////////////////////////////////////////////////////////////////////////
//  getVSCPControlByte
//

uint8_t vscp_getControlByte( uint8_t idx )
{
    return eeprom_read( VSCP_EEPROM_CONTROL0 + idx );
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_init_pstorage
//

void vscp_init_pstorage( void )
{
    init_app_eeprom();
}


///////////////////////////////////////////////////////////////////////////////
//  vscp_getEmbeddedMdfInfo
//

void vscp_getEmbeddedMdfInfo(void)
{
    // No embedded DM so we respond with info about that

    vscp_omsg.priority = VSCP_PRIORITY_NORMAL;
    vscp_omsg.flags = VSCP_VALID_MSG + 3;
    vscp_omsg.vscp_class = VSCP_CLASS1_PROTOCOL;
    vscp_omsg.vscp_type = VSCP_TYPE_PROTOCOL_RW_RESPONSE;

    vscp_omsg.data[ 0 ] = 0;
    vscp_omsg.data[ 1 ] = 0;
    vscp_omsg.data[ 2 ] = 0;

    // send the event
    vscp_sendEvent();
}

///////////////////////////////////////////////////////////////////////////////
// vscp_goBootloaderMode
//

void vscp_goBootloaderMode( uint8_t algorithm )
{
    if ( VSCP_BOOTLOADER_PIC1 != algorithm  ) return;

    // OK, We should enter boot loader mode
    // 	First, activate bootloader mode
    eeprom_write(VSCP_EEPROM_BOOTLOADER_FLAG, VSCP_BOOT_FLAG);

    // Reset processor
    Reset();
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getMatrixInfo
//

void vscp_getMatrixInfo(char *pData)
{
    pData[ 0 ] = DESCION_MATRIX_ROWS;  // Number of matrix rows
    pData[ 1 ] = REG_DESCION_MATRIX;   // Matrix start offset
    pData[ 2 ] = 0;                    // Matrix start page
    pData[ 3 ] = DESCION_MATRIX_PAGE;
    pData[ 4 ] = 0;                    // Matrix end page
    pData[ 5 ] = DESCION_MATRIX_PAGE;
    pData[ 6 ] = 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getFamilyCode
//

uint32_t vscp_getFamilyCode() {
    return 0L;
}


///////////////////////////////////////////////////////////////////////////////
// vscp_getFamilyType
//

uint32_t vscp_getFamilyType() {
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_restoreDefaults
//

void vscp_restoreDefaults() {
    init_app_eeprom();
    init_app_ram();
}


///////////////////////////////////////////////////////////////////////////////
// vscp_getRegisterPagesUsed
//

uint8_t vscp_getRegisterPagesUsed( void )
{
    return NUMBER_OF_REGISTER_PAGES; // One pages used
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPFrame
//

int8_t sendVSCPFrame( uint16_t vscpclass,
                        uint8_t vscptype,
                        uint8_t nodeid,
                        uint8_t priority,
                        uint8_t size,
                        uint8_t *pData )
{
    uint32_t id = ( (uint32_t)priority << 26 ) |
                    ( (uint32_t)vscpclass << 16 ) |
                    ( (uint32_t)vscptype << 8 ) |
                    nodeid; // node address (our address)

    if ( !sendCANFrame( id, size, pData ) ) {
        return FALSE;
    }

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// getVSCPFrame
//

int8_t getVSCPFrame(uint16_t *pvscpclass,
                        uint8_t *pvscptype,
                        uint8_t *pNodeId,
                        uint8_t *pPriority,
                        uint8_t *pSize,
                        uint8_t *pData)
{
    uint32_t id;

    if ( !getCANFrame(&id, pSize, pData) ) {
        return FALSE;
    }

    *pNodeId = id & 0x0ff;
    *pvscptype = (id >> 8) & 0xff;
    *pvscpclass = (id >> 16) & 0x1ff;
    *pPriority = (uint16_t) (0x07 & (id >> 26));

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// sendCANFrame
//

int8_t sendCANFrame(uint32_t id, uint8_t dlc, uint8_t *pdata)
{
    uint8_t rv = FALSE;
    sendTimer = 0;

    while ( sendTimer < 1000 ) {
        if ( ECANSendMessage( id, pdata, dlc, ECAN_TX_XTD_FRAME ) ) {
            rv = TRUE;
            break;
        }
    }

    vscp_omsg.flags = 0;

    return rv;
}

///////////////////////////////////////////////////////////////////////////////
// getCANFrame
//

int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata)
{
    ECAN_RX_MSG_FLAGS flags;

    // Don't read in new event if there already is a event
    // in the input buffer
    if (vscp_imsg.flags & VSCP_VALID_MSG) return FALSE;

    if ( ECANReceiveMessage( pid, pdata, pdlc, &flags) ) {

        // RTR not interesting
        if (flags & ECAN_RX_RTR_FRAME) return FALSE;

        // Must be extended frame
        if (!(flags & ECAN_RX_XTD_FRAME)) return FALSE;

        return TRUE;

    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// calculateSetFilterMask
//
// Calculate and set required filter and mask
// for the current decision matrix
//

void calculateSetFilterMask( void )
{
    uint8_t i,j;
    uint8_t lastOID;
    uint32_t rowmask;
    uint32_t rowfilter;

    // Reset filter masks
    uint32_t mask = 0xffffffff; // Just id 0x00000000 will come true
    uint32_t filter = 0x00000000;

    // Go through all DM rows
    for ( i=0; i < DESCION_MATRIX_ROWS; i++ ) {

        // No need to check not active DM rows
        if ( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_ENABLED ) {

            // build the mask
            // ==============
            // We receive
            //  - all priorities
            //  - hardcoded and not hardcoded
            //  - from all nodes

            rowmask =
                    // Bit 9 of class mask
                    ( (uint32_t)( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_CLASS_MASK ) << 23 ) |
                    // Rest of class mask
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_CLASSMASK ) << 16 ) |
                    // Type mask
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_TYPEMASK ) << 8 ) |
                    // Hardcoded bit
                    //( ( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_HARDCODED ) << 20 ) |   
					// OID  - handle later
					0xff;
                    

            // build the filter
            // ================

            rowfilter =
                    // Bit 9 of class filter
                    ( (uint32_t)( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_CLASS_FILTER ) << 24 ) |
                    // Rest of class filter
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_CLASSFILTER ) << 16 ) |
                    // Type filter
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_TYPEFILTER ) << 8 ) |
                    // OID Mask cleared if not same OID for all or one or more
                    // rows don't have OID check flag set.
                    eeprom_read( VSCP_EEPROM_END + 8*i );

            if ( 0 == i ) filter = rowfilter;   // Hack for first iteration loop

            // Form the mask - if one mask have a don't care (0)
            // the final mask should also have a don't care on that position
            mask &= rowmask;

            // Check the calculated filter and the current
            // filter to see if the bits are the same. If they
            // are not then clear the mask at that position
            for ( j=0; j<32; j++ ) {
                // If two bits are different we clear the mask bit
                if ( ( ( filter >> j ) & 1 ) != ( ( rowfilter >> j ) & 1 ) ) {
                    mask &= ~(1<<j);
                }
            }

            // Form the filter
            // if two bits are not the same they will be zero
            // All zeros will be zero
            // All ones will be one
            filter &= rowfilter;

            // Not check OID?
            if ( !eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_CHECK_OADDR ) {
                // No should not be checked for this position
                // This mean that we can't filter on a specific OID
                // so mask must be a don't care
                mask &= ~0xff;
            }

            if ( i ) {
                // If the current OID is different than the previous
                // we accept all
                for (j = 0; j < 8; j++) {
                    if ((lastOID >> i & 1)
                            != (eeprom_read(VSCP_EEPROM_END + 8*i ) >> i & 1)) {
                        mask &= (1 << i);
                    }
                }

                lastOID = eeprom_read(VSCP_EEPROM_END + 8*i );

            } 
            else {
                // First round we just store the OID
                lastOID = eeprom_read(VSCP_EEPROM_END + 8*i );
            }

        }
    }
    
    // Must be in Config mode to change settings.
    ECANSetOperationMode( ECAN_OP_MODE_CONFIG );

    //Set mask 1
    ECANSetRXM1Value( mask, ECAN_MSG_XTD );

    // Set filter 1
    ECANSetRXF1Value( filter, ECAN_MSG_XTD );

    // Return to Normal mode to communicate.
    ECANSetOperationMode( ECAN_OP_MODE_NORMAL );
  
}

