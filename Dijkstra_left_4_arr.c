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

unsigned char data;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_black_line = 0;
unsigned char Center_black_line = 0;
unsigned char Right_black_line = 0;
const int range = 25;
int left_count, center_reached = 0, White = -1, point = -1;
int blocks [] = {2,4,5,7,9,11};
char color [] = {'r','b','b','r','y','g'};
char dropobj[] = {'m','n','o'};
char path_to_follow[] = {'s','l','A','s','l','u','r','s','s','s','s','D','s','s','s','s','l','u','r','s','s','s','s','s','r','E','s','r','s','s','s','u','s','s','s','r','G','l','s','s','s','u','s',']','s','l','s','I','s','s','s','s','r','u','l','s','K','s','r','0'};


void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

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
 DDRC = DDRC | 0x08;        //Setting PORTC 3 as output
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
    servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
    servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
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

void timer1_init(void)
{
    TCCR1B = 0x00; //stop
    TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
    TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
    OCR1AH = 0x03; //Output compare Register high value for servo 1
    OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
    OCR1BH = 0x03; //Output compare Register high value for servo 2
    OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
    OCR1CH = 0x03; //Output compare Register high value for servo 3
    OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
    ICR1H  = 0x03; 
    ICR1L  = 0xFF;
    TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
                        For Overriding normal port functionality to OCRnA outputs.
                      {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
    TCCR1C = 0x00;
    TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
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
// void print_sensor(char row, char coloumn,unsigned char channel)
// {
    
//     ADC_Value = ADC_Conversion(channel);
//     lcd_print(row, coloumn, ADC_Value, 3);
// }

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

void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}

void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void object_grip_and_lift (void) // grip the object
{
 servo_2 (120);
 _delay_ms(1000);
 servo_1 (20);
 _delay_ms(1000);
 servo_2 (50);
 _delay_ms(1000);
 point += 1;
}

void object_ungrip (void) // relase the object
{
 servo_1 (90);
 _delay_ms(1000);
 servo_2(20);
 _delay_ms(1000);
}
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
}

void transmit(char data)
{
    UDR0 = data;
}

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
    _delay_ms(750);
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
    Right_black_line = ADC_Conversion(3);
    while (Right_black_line < range)
    {
        left();
        velocity(200,200);
        Right_black_line = ADC_Conversion(3);
    }
}

void turn_right ()
{
    right();
    velocity(200, 200);
    _delay_ms(200);
    Left_black_line = ADC_Conversion(1);
    while (Left_black_line < range)
    {
        right();
        velocity(200,200);
        Left_black_line = ADC_Conversion(1);
    }
}

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

// void priintsensorvalue ()
// {
//     print_sensor(1,1,3);    //Prints value of White Line Sensor1
//     print_sensor(1,5,2);    //Prints Value of White Line Sensor2
//     print_sensor(1,9,1);    //Prints Value of White Line Sensor3
// }

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
            case ']':
                slight_forward();
                turn_right();
                turn_right();
                break;
            case 'l':
                slight_forward();
                turn_left();
                break;
            case '[':
                slight_forward();
                turn_left();
                turn_left();
                break;
            case 's':
                slight_forward();
                break;
            case 'u':
                stop();
                _delay_ms(1000);
                object_ungrip();
                right();
                velocity(200, 200);
                _delay_ms(600);
                turn_right();
                transmit('d');
                break;
            case 'A':                       //1
                slight_forward();
                turn_right();
                right();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_left();
                transmit(color[point]);
                transmit(dropobj[0]);
                break;
            case 'B':                       //2                     
                slight_forward();
                turn_right();
                right();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_left();
                transmit(color[point]);
                transmit(dropobj[0]);
                break;
            case 'C':                       //3
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_left();
                transmit(color[point]);
                transmit(dropobj[0]);
                break;
            case 'D':                       //4
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_left();
                transmit(color[point]);
                transmit(dropobj[0]);
                break;
            case 'E':                       //5
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_right();
                transmit(color[point]);
                transmit(dropobj[1]);
                break;
            case 'F':                       //6
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_right();
                transmit(color[point]);
                transmit(dropobj[1]);
                break;
            case 'G':                       //7
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(215);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_left();
                transmit(color[point]);
                transmit(dropobj[1]);
                break;
            case 'H':                       //8
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(215);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_left();
                transmit(color[point]);
                transmit(dropobj[1]);
                break;
            case 'I':                       //9
                slight_forward();
                turn_right();
                right();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_right();
                transmit(color[point]);
                transmit(dropobj[2]);
                break;
            case 'J':                       //10
                slight_forward();
                turn_left();
                left();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_right();
                transmit(color[point]);
                transmit(dropobj[2]);
                break;
            case 'K':                       //11
                slight_forward();
                turn_right();
                right();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_right();
                transmit(color[point]);
                transmit(dropobj[2]);
                break;
            case 'L':                       //12
                slight_forward();
                turn_right();
                right();
                velocity(200 ,200);
                _delay_ms(200);
                stop();
                _delay_ms(1000);
                object_grip_and_lift();
                turn_right();
                transmit(color[point]);
                transmit(dropobj[2]);
                break;
            case '0':
                stop();
                _delay_ms(1000);
                object_ungrip();
                transmit(dropobj[2]);
                buzzer_on();
                stop();
                _delay_ms(10000);
                return 0;
            break;
        }   
    }
}

void init_devices (void)
{
    cli(); //Clears the global interrupts
    port_init();
    uart0_init(); //Initailize UART1 for serial communiaction
    adc_init();
    timer5_init();
    timer1_init();
    sei();   //Enables the global interrupts
}

int main()
{
    init_devices();
    turn_right();
    turn_right();
    readsensors();
    while(1)
    {
        readsensors();
        straightlinefollowing();
        countnodes();
    }
    stop();
    _delay_ms(10000);
}
