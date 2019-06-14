#include "lib.h"

//int sensor_1 = 0, sensor_2 = 0, sensor_3 = 0;
void interrupt low_priority LIisr (void) 
{ 
    if (INTCONbits.TMR0IF) 
    { 
        INTCONbits.TMR0IF=0; 
        
        LATBbits.LATB3=!LATBbits.LATB3; 
    } 
} 


void main(void) 
{
    init_all_units( );
   
    
    low_interrupt_init( );

    TRISBbits.RB3=0;// configure the RB3 as output 
    while(1)
    {
        
    } 
}
