/*
 * Team Id		   : SB#811
 * Author list	   : Yashas H S
 *
 *Filename		   : 
 *Theme			   : Supply Bot  
 *Functions		   : uart_initialize(),USART_Receive(),USART_Flush(),port_init(),adc_init(),timer0_init(),timer1_init(),ADC_Conversion(unsigned char),
					 send_data(unsigned char),init_devices(),set_pwm(int,int),move_forward(),move_backward().move_left(), move_right(),move_backright(),move_backleft(),
					 stop()
 *Global VAriables : None
 * 
 */ 
#define F_CPU 16000000UL //crystal frequency of 16MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define servo1 PB1
#define servo2 PB2

#define en1 PD6
#define en2 PD5
#define m1 PB4
#define m2 PB5
#define m3 PD2
#define m4 PD3
#define PIN_ADC0 PC0	
#define PIN_ADC1 PC1
#define PIN_ADC2 PC2
#define BUZZER PD7

/*
* Function name: uart_initialize
*Input : None
*Output : Initializes the uart communication 
*Logic  : enables the RX and TX communication
*
*Example Call : uart_initialize()
*/
void uart_initialize() //function to initialize UART Communication
{
	unsigned int barte = 103; //103 for setting baudrate for 9600
	
	PRR &= ~(1<<PRUSART0); 
	UCSR0B |= ((1 << RXEN0 ) | (1<< TXEN0)); //enabling RX and Tx transmission
	UCSR0C |= ((1<< UCSZ01) | (1 << UCSZ00)) ; //8-bit data transmission
	//setting baud rate for 9600
	UBRR0H = (unsigned char) (barte >> 8); 
	UBRR0L = (unsigned char) (barte); 
}

/*
* Function name: USART_Receive
*Input : None
*Output : when there in any data available in the Rx buffer, reads it and returns the value
*Logic  : The function continously checks if RXC0 flag is set and if set, returns the content of UDR0 register
*
*Example Call : USART_Receive()
*/

unsigned char USART_Receive(void)
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)))
	;
	/* Get and return received data from buffer */
	return UDR0;
}

/*
* Function name: USART_Flush
*Input : None
*Output : Cleares the content of Rx buffer
*Logic  : copies the contents of RX buffer into a dummy variable
*
*Example Call :USART_Flush()
*/

void USART_Flush(void)
{
	//to flush any rubbish data present inside buffer for fresh recieval
	unsigned char dummy;
	while (UCSR0A & (1<<RXC0)) dummy = UDR0;
}

/*
* Function name: port_init
*Input : None
*Output : Initializes all the required ports as correspondingly input and output
*Logic  : DDRx register if set high, makes that pin as output and as input when set to 0
*
*Example Call : port_init()
*/

void port_init() //initializing ports
{
	DDRC &= ~((1 << PIN_ADC0)|(1 << PIN_ADC1)|(1 << PIN_ADC2)); //set specific pins of PORTC as input
	PORTC &= ~((1 << PIN_ADC0)|(1 << PIN_ADC1)|(1 << PIN_ADC2)); //set PORTC pins floating
	DDRD    |= ((1 << en1)|(1 << en2)|(1<<m3)|(1<<m4)|(1<<BUZZER));//set specific pins of portD as output 
	PORTD   |= ((1 << en1)|(1<<en2));
	PORTD   &= ~((1<<m4)|(1<<m3)|(1<<m1)|(1<<m2));
	DDRB    |= ((1<<m1)|(1<<m2)); //setting POrtb pins as output
	PORTB   &= ~((1<<m2)|(1<<m1));
	DDRB |= ((1<<servo1)|(1 << servo2));
}

/*
* Function name: adc_init
*Input : None
*Output : Initializes the Analog to Digital Conversion
*Logic  : ADC is turned on by writing to ADCSRA register
*
*Example Call : adc_init()
*/

void adc_init(){
	//PRR &= ~(1<< PRADC);
	ACSR = (1 << ACD);   	// Analog Comparator Disable; else ADC wont work
	ADMUX = (1 << ADLAR);	//left adjust output
	// (turn ADC ON) | (set prescalar to 64 110)
	ADCSRA = ((1 << ADEN) |  (1 << ADPS2 | 1 << ADPS1)) ;
}

/*
* Function name: timer0_init
*Input : None
*Output : Initializes timer 0 with appropriate counting condition and value to generate PWM for motors
*Logic  : Initialize makes PWM pin go low, when timer value equal to OCROa and OCR0B registers, then 8-bit Phase correct pwm mode for motor control.Prescalar is set to 1024
*
*Example Call : timer0_init()
*/

void timer0_init()
{
	cli(); //disable all interrupts
	
	TCCR0B = 0x00;	//Stop
	
	TCNT0 = 0xFF;	//Counter higher 8-bit value to which OCR2A value is compared with
	OCR0B = 0XFF;
	OCR0A = 0xFF;	//Output compare register low value for Led
	
	//  Clear OC2A and OC2B, on compare match (set output to low level)
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);
	
	TCCR0A |= (1 << COM0B1);
	TCCR0A &= ~(1 << COM0B0);
	
	// PHASE CORRECT PWM 8-bit Mode for motor speed control
	TCCR0A |= (1 << WGM00);
	TCCR0A &= ~(1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	
	// Set Prescalar to 1024
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);
	
	sei(); //re-enable interrupts
}

/*
* Function name: timer1_init
*Input : None
*Output : Initializes the timer1 to run, and generates a PWM waveform on pins OCCR1x
*Logic  : initializes timer1, then the output is made to go low upon set by setting COM1A1 high anf COM1A0 low, then 16-bit fast pwm mode is selected to dupply pwm for servo
*
*Example Call : timer1_init()
*/

void timer1_init() //initializing timer 1 for generating 16-bit PWM for Servo motors
{
	cli(); //disable all interrupts
	
	TCCR1B = 0x00;	//Stop
	TCCR1A = 0x00;
	TCNT1 = 0xFFFF;	//Counter higher 16-bit value to which OCR2A value is compared with
	OCR1B = 0XFFFF;
	OCR1A = 0xFFFF;	
	
	//  Clear OC1A and OC2A, on compare match (set output to low level)
	TCCR1A |= (1 << COM1A1);
	TCCR1A &= ~(1 << COM1A0);
	
	TCCR1A |= (1 << COM1B1);
	TCCR1A &= ~(1 << COM1B0);
	
	// FAST PWM 16-bit Mode with TOP value equal to ICR1 value
	TCCR1A &= ~(1 << WGM10);
	TCCR1A |= (1 << WGM11);
	TCCR1B |= ((1 << WGM12)|(1<<WGM13)|(1 << CS11));
	TCCR1B &= ~((1 << CS12)|( 1 << CS10));
	
	TCCR1C = 0x00;
	
	//For driving servo, we need a PWM pulse of time period 20ms, the corresponding ICR1 value for 16MHz crystal is 39999
	ICR1 = 39999;
	sei(); //re-enable interrupts
}

/*
* Function name: ADC_Conversion
*Input : ch -> channel through which ADC conversion has to be initiated. 
*Output : returns the analog rto digital conv erted value.
*Logic  : Starts conversion by enabling ADSC bit of ADCSRA, and then ADIF flag is set indicating end of conversion, reads the value.
*
*Example Call : ADC_Conversion(2)
*/

unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned int a;
	// Extract Last 3 bits from Ch for ADMUX
	Ch = Ch & 0b00000111; //0x07
	ADMUX = 0x60 | Ch; // (Left Adjusted Output with AVcc as reference) | (ADMUX4:0)
	//-----------------------------------------------------------------
	
	// ADMUX |= Ch;	// *** does not work if ADLAR is set in adc_init()
	//-----------------------------------------------------------------
	//-----------------------------------------------------------------
	ADCSRA |= (1 << ADSC);		//Set start conversion bit
	// Wait for ADC conversion to complete; ADIF = 0, conversion going on; ADIF = 1, conversion complete
	while((ADCSRA & (1 << ADIF) ) == 0);
	// store ADC value in variable are return it.
	a = ADCH;
	//-----------------------------------------------------------------
	//-----------------------------------------------------------------
	// ADIF is set when ADC conversion is complete and Data Registers are updated
	// ADIF needs to be cleared before starting next conversion
	ADCSRA |= (1 << ADIF); // IMP: clear ADIF (ADC Interrupt Flag) by writing 1 to it
	//-----------------------------------------------------------------
	return a;
}

/*
* Function name: send_data
*Input : data -> The character which has to be sent over UART
*Output : sends the data over UART
*Logic  : Waits untill the transmit buffer is empty, by polling UDRE0 flag and then when Tx buffer is ready to recieve, data is copied to Tx buffer
*
*Example Call : send_data('a')
*/

void send_data(unsigned char data) //sending data over UART
{
	while(!(UCSR0A & (1<<UDRE0))); //poll untill transmit buffer is empty
	UDR0 = data; //send data once transmit buffer is ready to recieve
}

/*
* Function name: init_devices
*Input : None
*Output : initialises all the required processes
*Logic  : calls the initialisation functions defined earlier
*
*Example Call : init_devices()
*/

void init_devices() //function to initialize all devices together
{
	port_init();
	adc_init();
	timer0_init();
	uart_initialize();
	timer1_init();
}

/*
* Function name: set_pwm
*Input : one-> value that has to be copied to OCR0A register
		 two -> value that has to be copied to OCR0B register

*Output : Sets value of OCR0A and OCR0B register to control pwm of motor driver enable pins
*Logic  : copies the 255 - incoming value to OCCR0A and OCCR0B register
*
*Example Call : uart_initialize()
*/

void set_pwm (unsigned char one, unsigned char two) //function to set pwm values for motor-driver pins enable1 and enable2 
{
	OCR0A = 255 - (unsigned char)one;
	OCR0B = 255 - (unsigned char)two;
}

/*
* Function name: move_forward
*Input : None
*Output : sets the m1,m2,m3,m4 values to make bot move forwards
*Logic  : both side wheels move forward
*
*Example Call : move_forward()
*/

void move_forward() //function to move the bot forward
{
	PORTD |= (1 << m3);
	PORTD &= ~(1 << m4);
	PORTB   |= (1<<m1);
	PORTB   &= ~(1<<m2);
}

/*
* Function name: move_backward
*Input : None
*Output : sets the m1,m2,m3,m4 values to make bot move backwards
*Logic  : both side wheels move backward
*
*Example Call : uart_initialize()
*/

void move_backward() //function to move the bot backward
{
	PORTD |= (1 << m4);
	PORTD &= ~(1 << m3);
	PORTB   |= (1<<m2);
	PORTB   &= ~(1<<m1);
}

/*
* Function name: move_left
*Input : None
*Output : sets the m1,m2,m3,m4 values to make bot move left
*Logic  : right wheels move forwards, and left wheels move backwards
*
*Example Call : uart_initialize()
*/

void move_left() //function to move the bot leftwards
{
		PORTD &= ~(1 << m3);
		PORTD |= (1 << m4);
		PORTB   |= (1<<m1);
		PORTB   &= ~(1<<m2);
}

/*
* Function name: move_right
*Input : None
*Output : sets value of m1,m2,m3,m4
*Logic  : the robot turns right, left side motors move backwards, right side motors move forward
*
*Example Call : move_right()
*/

void move_right() //function to move the bot rightwards
{
	PORTB &= ~(1 << m1); 
	PORTB |= (1 << m2);
	PORTD   |= (1<<m3);
	PORTD   &= ~(1<<m4);
}

/*
* Function name: move_backleft
*Input : None
*Output : sets appropriate values to m1,m2,m3,m4 values.
*Logic  : the robot turns backleft (left side motors stop rotating and right side motors move with a certain speed backwards)
*
*Example Call : uart_initialize()
*/

void move_backleft() //function to move the bot backward left
{
	
	PORTD &= ~(1 << m3);
	PORTD &= ~(1 << m4);
	PORTB   |= (1<<m2);
	PORTB   &= ~(1<<m1);
}

/*
* Function name: move_backright
*Input : None
*Output : sets appropriate values to m1,m2,m3,m4 values.
*Logic  : the robot turns back_right (right side motors stop rotating and left side motors move with a certain speed backwards)
*
*Example Call : move_backright()
*/

void move_backright() //function to move the bot backward right
{
	PORTB &= ~(1 << m1);
	PORTB &= ~(1 << m2);
	PORTD   |= (1<<m4);
	PORTD   &= ~(1<<m3);
}

/*
* Function name: stop
*Input : None
*Output : stops the robot
*Logic  : passes low to all m1,m2,m3,m4 pins
*
*Example Call : stop()
*/

void stop() //function to stop the robot
{
	PORTB &= ~((1<<m1) |(1<<m2));
	PORTD &= ~((1<<m3)|(1<<m4));
}

int main(void)
{
	init_devices();
	
	/*
	our designed hitting mechanism includes two servos, with a spring mechanism. once servo is used to pull the spring backwards whereas,
	 another one for releasing the strecthed spring
	
	OCR2B pwm pin is used for controlling the servo used to pull back the spring
	OCR1B PWM pin is used for controlling the servo used to release the string
	*/
	
	OCR1B = 1050;  //hold the spring
	//_delay_ms(10000);
	//_delay_ms(10000);
	OCR1A = 2000;  //move the servo used to pull back the spring to its initial position
	//_delay_ms(10000);
	_delay_ms(10000);
	
	//variables to store white line sensor values
	unsigned int sensor1; 
	unsigned int sensor2;
	unsigned int sensor3;
	
	/*
	unsigned char reading1;
	unsigned int msb;
	unsigned int msb1;
	unsigned int lsb;
	*/
	unsigned char data;
	unsigned char acknowledgment = 'D';
    while(1)
    {
		if(((UCSR0A &= (1 << FE0))== 0x10)|((UCSR0A &= (1 << DOR0))== 0x08)|((UCSR0A &= (1 << UPE0))== 0x04)) //if any error flags are set while communication flush the buffer
		{
			USART_Flush();
		}
		data = USART_Receive();
	
	//store sensor values in variables
    sensor1 = ADC_Conversion(0); //sensor 1 holds value of leftmost sensor
    sensor2 = ADC_Conversion(1); //sensor 2 holds value of middle sensor
    sensor3 = ADC_Conversion(2); //sensor 3 holds value of rightmost sensor
    
	//analyze data recieved
	
	if(data == 'F') //if character recieved is 'F', move forward by following the white line
	{
		
		if(sensor2 < 150) //if middle sensor is on white line
		{
			if(sensor1 > 150 && sensor3 > 150) //if sensor1 and sensor3 are on black region
			{
				set_pwm(190,190);
				move_forward(); //move forward 
			}
		}
		if(sensor1 < 150 && sensor3 > 150)//if leftmost sensor is on white line and rightmost sensor is on black region move left
		{
			set_pwm(10,220); //slower left motors and faster right motors
			move_forward();
		}
		if (sensor3 < 150 && sensor1 > 150)//if rightmost sensor is on white line and leftmost sensor is on black region move right
		{
			set_pwm(220,10); //slower right motors and faster left motors
			move_forward();
		}
		if((sensor1 > 150) && (sensor2 > 150) && (sensor3 > 150))
		{ //if all three sensors are on black region turn right
			set_pwm(150,150); 
			move_right();
		}
		if ((sensor1 > 150 ) && (sensor2 > 150)) //if middle sensor and left sensor are on black regions turn right
		{
			set_pwm(220,30);
			move_forward();
		}
		if (( sensor3 > 150 ) && (sensor2 > 150)) //if middle sensor and right sensor are on black regions turn left
		{
			set_pwm(30,220);
			move_forward();
		}
	}
	else if (data == 'B')
	{
		if(sensor2 < 150) //if middle sensor is on white line
		{
			if(sensor1 > 150 && sensor3 > 150) //if sensor1 and sensor3 are on black region
			{
				set_pwm(190,190);
				move_forward(); //move forward
			}
		}
		if(sensor1 < 150 && sensor3 > 150)//if leftmost sensor is on white line and rightmost sensor is on black region move left
		{
			set_pwm(100,220); //slower left motors and faster right motors
			move_forward();
		}
		if (sensor3 < 150 && sensor1 > 150)//if rightmost sensor is on white line and leftmost sensor is on black region move right
		{
			set_pwm(220,100); //slower right motors and faster left motors
			move_forward();
		}
		if((sensor1 > 150) && (sensor2 > 150) && (sensor3 > 150))
		{ //if all three sensors are on black region turn right
			set_pwm(150,150);
			move_right();
		}
		if (( sensor3 > 150 ) && (sensor2 > 150)) //if middle sensor and right sensor are on black regions turn left
		{
			set_pwm(100,220);
			move_forward();
		}
		if ((sensor1 > 150 ) && (sensor2 > 150)) //if middle sensor and left sensor are on black regions turn right
		{
			set_pwm(220,100);
			move_forward();
		}
		
	}
	
	else if(data == 'R') //with similar conditions move backwards when character sent is 'B'
	{
		set_pwm(190,190);
		move_right();
	}
	
		
	if(data == 'H') //if character sent is 'H', initiate hitting mechanism
	{
		//stop();
		
		set_pwm(220,220);		
		move_left(); //move left to allign with the coin
		_delay_ms(170);	
		move_backward();
		_delay_ms(170);
		
		stop();
		
		//beep buzzer twice
		PORTD |= (1<< BUZZER);
		_delay_ms(1000);
		PORTD &= ~(1<<BUZZER);
		_delay_ms(1000);
		PORTD |= (1<< BUZZER);
		_delay_ms(1000);
		PORTD &= ~(1<<BUZZER);
		_delay_ms(1000);
		
		//move servos to hit
		
		OCR1B = 1600; //blue up
		_delay_ms(2500);
		//_delay_ms(000);
		
		//beep buzzer once after hitting
		
		PORTD |= (1<< BUZZER);
		_delay_ms(1000);
		PORTD &= ~(1<<BUZZER);
		_delay_ms(1000);
		
		//move servos to pull back the spring
		
		OCR1A = 900;  //pull back spring
		_delay_ms(1000);
		//_delay_ms(10000);
		
		OCR1B = 1050;  //hold the pulled back spring
		_delay_ms(1500);
		//_delay_ms(10000);
		
		
		OCR1A = 2000;  //move servo used to pull back the spring to its initial position
		_delay_ms(1000);
		//_delay_ms(10000);
		
		send_data(acknowledgment); //send data to python script indicating dispatch
		
	}
	if (data == 'S') //if character recieved is 'S', stop
	{
		stop();
		
		//beep buzzer for 5 seconds
		
		PORTD |= (1<< BUZZER);
		_delay_ms(5000);
		PORTD &= ~(1<<BUZZER);
		//_delay_ms(1000);
	}
			
	}
}		
