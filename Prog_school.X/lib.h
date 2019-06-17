#include "pragma.h"
#include "define.h"

#include <xc.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>



void lcd_clk(void); /*генерация импульса на вход EN*/
void lcd_command(unsigned char outbyte); /*отправить команду (4-битный режим работы) */
void lcd_putc(char outbyte); /* отправить данные (4-битная операция) */

/*
 * @brief   Show string on LCD 
 */
void lcd_puts(uint8_t line,const uint8_t *p); // *вывод строки на экран*

/*
 * @brief   Show integer digits on LCD 
 */
void inttolcd(uint8_t posi, int32_t value); //вывод на экран значений пере-менных

/*
 * @brief   Initialization of LCD-unit 
 */
void lcd_init(); // инициализация LCD-дисплея

/*
 * @brief   Initialization of ADC-unit 
 */
void Adc_init();    // инициализация модуля АЦП

/*
 * @brief   Read data from ADC 
 */
int16_t read_Adc(int16_t channel);  

/*
 * @brief   Initialization of High-level-interrupt 
 *          from external source (button RB0/INT0)
 */
void high_interrupt_init( void );

/*
 * @brief   Initialization of Low-level-interrupt 
 *          from internal source (Timer 0)
 */
void low_interrupt_init( );

/*
 * @brief   Initialization of motor-unit 
 */
void motor_init();   

/*
 * @brief   Control speed of Motor A
 *          Speed [-124; 124]
 *          forward : 0 - 124
 *          backward: -124 - 0   
 */
void motor_a_change_Speed (int8_t speed); 

/*
 * @brief   Control speed of Motor B
 *          Speed [-124; 124]
 *          forward : 0 - 124
 *          backward: -124 - 0   
 */
void motor_b_change_Speed (int8_t speed);  // управление скоростью вращенения 

/*
 * @brief   Initialization of all lld-units in robot
 */
void init_all_units( void ); 


/*****************************************/
/***************** TESTS *****************/
/*****************************************/

/*
 * @brief   Blinking LED (RB3)
 *          frequency 2 Hz (500 ms)
 *          write on lcd "LED is OK"
 * @note    Include delay 500 ms !!!! 
 */
void testLEDRoutine( void );

/*
 * @brief   If button is pressed, LED is on 
 *          Not require simultaneous pressing both buttons 
 */
void testButtonRoutine( void ); 

/*
 * @brief   WARNING! Noise is unstoppable  
 *          Turn off the power to stop it
 *          frequency is 15 kHz 
 * @note    Include addition delay ( 500 ms ) !!!!
 */
void testNoiseRoutine( void ); 

/*
 * @brief   Press the button to change sensor
 *          On lcd you will see:
 *              1st line : NUMBER of Sensor and NAME
 *              2nd line : VALUE of Sensor  
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 100 ms ) !!!!
 */
void testAllSensorsRoutine( void ); 

/*
 * @brief   Check ref-sensors (bottom sensors)
 *          On lcd you will see:
 *              1st line : REF1: VALUE1 | REF2:
 *              2nd line : REF2: VALUE3 | VALUE2 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testRefSensors( void );

/*
 * @brief   Check distance sensor
 *          On lcd you will see:
 *              1st line : IR-sensor
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testDistanceSensorRoutine( void );

/*
 * @brief   Check light sensor
 *          On lcd you will see:
 *              1st line : Light sensor
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testLightSensorRoutine( void );

/*
 * @brief   Check noise sensor
 *          On lcd you will see:
 *              1st line : Noise sensor
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testNoiseSensorRoutine( void );

/*
 * @brief   Check temperature sensor
 *          On lcd you will see:
 *              1st line : Temperature
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 100 ms ) !!!!
 */
void testTemperatureSensorRoutine( void );

/*
 * @brief   Check potentiometr
 *          On lcd you will see:
 *              1st line : Potentiometr
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 100 ms ) !!!!
 */
void testPotentiometrRoutine( void );

/*
 * @brief   Check motors
 *          Press the button RB0:
 *              0 - stop
 *              1 - move forward; 
 *              2 - move backward;
 *              3 - stop. 
 *          On lcd you will see:
 *              1st line : Button pressed
 *              2nd line : NUMBER OF PRESS  
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 */
void testMotors( void );