#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_black_line = 0;
unsigned char Center_black_line = 0;
unsigned char Right_black_line = 0;
const int range = 25;
int left_count, White = 0;
//int nb = 6;
int blocks [] = {2,4,5,7,9,11};
char path_to_follow[62] = {'s','s','l','v','s','l','u','r','s','s','s','s','w','s','s','s','s','l','u','r','s','s','s','s','s','r','x','s','r','s','s','s','u','s','s','s','r','y','l','s','s','s','u','s','a','s','l','s','z','s','s','s','s','r','u','l','s','q','s','r'};
int not;

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;        //Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;      //Setting PORTC 3 logic low to turnoff buzzer
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void port_init()
{
    motion_pin_config();
    buzzer_pin_config();
    adc_pin_config ();
    lcd_port_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
    TCCR5B = 0x00;  //Stop
    TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
    TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
    OCR5AH = 0x00;  //Output compare register high value for Left Motor
    OCR5AL = 0xFF;  //Output compare register low value for Left Motor
    OCR5BH = 0x00;  //Output compare register high value for Right Motor
    OCR5BL = 0xFF;  //Output compare register low value for Right Motor
    OCR5CH = 0x00;  //Output compare register high value for Motor C1
    OCR5CL = 0xFF;  //Output compare register low value for Motor C1
    TCCR5A = 0xA9;  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
                      For Overriding normal port functionality to OCRnA outputs.
                      {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
    
    TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
    ADCSRA = 0x00;
    ADCSRB = 0x00;      //MUX5 = 0
    ADMUX = 0x20;       //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
    ACSR = 0x80;
    ADCSRA = 0x86;      //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
    unsigned char a;
    if(Ch>7)
    {
        ADCSRB = 0x08;
    }
    Ch = Ch & 0x07;             
    ADMUX= 0x20| Ch;            
    ADCSRA = ADCSRA | 0x40;     //Set start conversion bit
    while((ADCSRA&0x10)==0);    //Wait for conversion to complete
    a=ADCH;
    ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
    ADCSRB = 0x00;
    return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
    
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, coloumn, ADC_Value, 3);
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
    OCR5AL = (unsigned char)left_motor;
    OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;         // removing upper nibbel for the protection
 PortARestore = PORTA;      // reading the PORTA original status
 PortARestore &= 0xF0;      // making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore;      // executing the command
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}

void init_devices (void)
{
    cli(); //Clears the global interrupts
    port_init();
    adc_init();
    timer5_init();
    sei();   //Enables the global interrupts
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}
/*
//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
// UBRR0L = 0x47; //11059200 Hz
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}THIS IS TTHE CCOOODE FOR THE XBEE MODULE IT IS TO BE SEEN AFTERWARDS*/

void readsensors ()
{
    Left_black_line = ADC_Conversion(3);    //Getting data of Left WL Sensor
    Center_black_line = ADC_Conversion(2);  //Getting data of Center WL Sensor
    Right_black_line = ADC_Conversion(1);   //Getting data of Right WL Sensor
}

void slight_forward ()
{
    forward();
    velocity(200,200);
    _delay_ms(500);
}

void straight ()
{
    forward();
    velocity(200,200);
}

void turn_left ()
{
    left();
    velocity(200, 200);
    _delay_ms(200);
    Left_black_line = ADC_Conversion(3);
    while (Left_black_line < range)
    {
        left();
        velocity(200,200);
        Left_black_line = ADC_Conversion(3);
    }
}

void turn_right ()
{
    right();
    velocity(200, 200);
    _delay_ms(200);
    Right_black_line = ADC_Conversion(1);
    while (Right_black_line < range)
    {
        right();
        velocity(200,200);
        Right_black_line = ADC_Conversion(1);
    }
}

void startpositright ()
{
    right();
    velocity(200,200);
    _delay_ms(700);
    readsensors();
    while(Center_black_line<range)
    {
        right();
        velocity(200,200);
        readsensors();
    }
}

int center_reached = 0;

void straightlinefollowing ()
{
    flag=0;
    forward();
    if(Center_black_line>range)
    {
        velocity(200,200);
        center_reached = 1;
    }

    if (Left_black_line > range)
    {
        velocity(100,200);
        left_count = 1;
        center_reached = 0;
    }

    if (Right_black_line > range)
    {
        velocity(200,100);
        left_count = 0;
        center_reached = 0;
    }

    if(Center_black_line < range && Left_black_line < range && Right_black_line < range)
    {
        if (left_count){
            if (center_reached)
                velocity(200, 175);
            else
                velocity(175, 200);
        }
        else{
            if (center_reached)
                velocity(175,200);
            else
                velocity(200,175);
        }
                    
    }
}

void priintsensorvalue ()
{
    print_sensor(1,1,3);    //Prints value of White Line Sensor1
    print_sensor(1,5,2);    //Prints Value of White Line Sensor2
    print_sensor(1,9,1);    //Prints Value of White Line Sensor3
}

void countnodes ()
{
    if ((Center_black_line>range && Left_black_line>range) || (Center_black_line>range && Right_black_line>range) || (Center_black_line>range && Left_black_line>range && Right_black_line>range))
    {
        White += 1;
        switch(path_to_follow[White])
        {
            case 'r':
                slight_forward();
                turn_right();
                break;
            // double right
            case 'a':
                slight_forward();
                turn_right();
                turn_right();
                break;
            case 'l':
                slight_forward();
                turn_left();
                break;
            // double left
            case 'b':
                slight_forward();
                turn_left();
                turn_left();
                break;
            case 's':
                slight_forward();
                break;
            case 'u':
                right();
                velocity(200, 200);
                _delay_ms(600);
                turn_right();
                break;
            case 'v':
                slight_forward();
                turn_right();
                // velocity(200,200);
                // _delay_ms(850);
                stop();
                _delay_ms(300);
                turn_left();
                // velocity(200,200);
                // _delay_ms(900);
                break;
            case 'w':
                slight_forward();
                turn_left();
                // velocity(200,200);
                // _delay_ms(850);
                stop();
                _delay_ms(300);
                turn_left();
                // velocity(200,200);
                // _delay_ms(1500);
                break;
            case 'x':
                slight_forward();
                turn_left();
                // velocity(200,200);
                // _delay_ms(850);
                stop();
                _delay_ms(300);
                turn_right();
                // velocity(200,200);
                // _delay_ms(780);
                break;
            case 'y':
                slight_forward();
                turn_left();
                // velocity(200,200);
                // _delay_ms(850);
                stop();
                _delay_ms(300);
                turn_left();
                // velocity(200,200);
                // _delay_ms(850);
                break;
            case 'z':
                slight_forward();
                turn_right();
                // velocity(200,200);
                // _delay_ms(850);
                stop();
                _delay_ms(300);
                turn_right();
                // velocity(200,200);
                // _delay_ms(850);
                break;
            case 'q':
                slight_forward();
                turn_right();
                // velocity(200,200);
                // _delay_ms(850);
                stop();
                _delay_ms(300);
                turn_right();
                // velocity(200,200);
                // _delay_ms(850);
                break;
        }   
    }
}

/*void gotostepr ()
{
    turn_left();
    straightlinefollowing();
    
}
*/

int main()
{
    init_devices();
    lcd_set_4bit();
    lcd_init(); 
    
    // startpositright();
    turn_right();
    turn_right();
    readsensors();

    while(1)
    {
        readsensors();
        priintsensorvalue();
        straightlinefollowing();
        countnodes();
        lcd_print(2,2,White,3);
        
    }
}
