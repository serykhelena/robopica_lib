#define lcd_clear() lcd_command(1)
#define lcd_origin() lcd_command(2)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 10000000 // определение тактовой частоты  10 ћ√ц  дл€ функции __delay_ms
#endif
#define E_pulse_with 50 //длительность тактовых импульсов LCD в мкс
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 LATD // инициализаци€ портов D4-D7 LCD


#define button_A4_pressed PORTAbits.RA4
#define button_B0_pressed PORTBbits.RB0



