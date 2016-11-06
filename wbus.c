
////////////////////////////////////////////////////////////////
// w-bus commander
// version 1.06, 27.10.2014
////////////////////////////////////////////////////////////////

#define _LEGACY_HEADERS
#include "htc.h"
#include "wbus.h"

__CONFIG(HS & MCLRDIS & WDTEN & PWRTEN & BOREN & LVPDIS & PROTECT);

#define		MINS_TO_RUN		15
#define		MAX_TEMP		75

#define 	XTAL				18432000
#define 	TICKS_PER_SECOND    1000
#define 	DIVIDE  			(XTAL/4/32/TICKS_PER_SECOND)

//								-------
//	              RA2/AN2/VREF |1    18| RA1/AN1			
//	              RA3/AN3/CMP1 |2    17| RA0/AN0			alive
//				RA4/TOCKI/CMP2 |3    16| RA7/OSC1/CLKIN		
//  	 		  RA5/MCLR/THV |4    15| RA6/OSC2/CLKOUT	
//	GND			           VSS |5    14| VDD				VCC
//	low beam	       RB0/INT |6    13| RB7/T1OSI			
//	RX    		     RB1/RX/DT |7    12| RB6/T1OSO/T1CKI	
//	TX    		     RB2/TX/CK |8    11| RB5				
//	ignition          RB3/CCP1 |9    10| RB4/PGM			
//								-------

#define ON		1
#define OFF		0

#define DIRBITS_A       0B11111110
#define INITBITS_A      0B11111111
#define DIRBITS_B       0B11111011
#define INITBITS_B      0B11111111

#define PORTBIT(adr, bit)   ((unsigned)(&adr)*8+(bit))

static bit      alive				@PORTBIT(PORTA, 0);
static bit      low_beam			@PORTBIT(PORTB, 0);
static bit      tx_line				@PORTBIT(PORTB, 2);
static bit      ignition			@PORTBIT(PORTB, 3);

static bit      led_circ_pump;	//	@PORTBIT(PORTB, 3);


#define RESPONSE_TIMEOUT	(TICKS_PER_SECOND/2)
#define ERROR_DELAY			(1*TICKS_PER_SECOND)
#define NACK_DELAY			(5*TICKS_PER_SECOND)
#define ERROR_TMO_CHK_ADR	 250
#define ERROR_NACK			 240

BYTE	rs232_dummy, rs232_byte;
BYTE 	bytes_count, cnt;

BYTE	error, tries1, tries2;

BYTE 	buffer[15], message_data[10], message_length;
BYTE 	checksum_byte;

WORD	rx_ticks, delay_ticks, pump_check_ticks, low_beam_ticks, sleep_ticks;

DWORD	active_time, post_active_time;
BYTE	start_pressed, stop_pressed;


//--------------------------------------------------------------------------------

#define int_on()		{ INTCON=0xA0; }
#define int_off()		{ do { INTCON=0x00; } while (GIE!=0); }
#define reset_sleep()	{ sleep_ticks=10*TICKS_PER_SECOND; }
#define wait_ms(x)		{ delay_ticks=x; do { CLRWDT(); } while (delay_ticks); }

//--------------------------------------------------------------------------------
//
unsigned char checksum(unsigned char *buf, unsigned char len, unsigned char chk)
{
	for ( ; len!=0; len--) { chk ^= *buf++; }
	return chk;
}

//--------------------------------------------------------------------------------

void interrupt timer(void)	// 1 ms tick
{
	TMR0 = -DIVIDE;
	T0IF = 0;
	//
	alive = 0;
	//
	if (delay_ticks) delay_ticks--;
	if (pump_check_ticks) pump_check_ticks--;
	if (sleep_ticks) sleep_ticks--;
	//
	if (low_beam_ticks) { low_beam_ticks++; reset_sleep(); }
	if (ignition==1) low_beam_ticks = 0; // forced stop
	if (ignition==0 && low_beam==1 && low_beam_ticks==0) low_beam_ticks = 1; // start ticking
	if (ignition==0 && low_beam==0 && low_beam_ticks!=0)
	{
		if (low_beam_ticks>1*TICKS_PER_SECOND && low_beam_ticks<5*TICKS_PER_SECOND) start_pressed = 1;
		if (low_beam_ticks>5*TICKS_PER_SECOND && low_beam_ticks<15*TICKS_PER_SECOND) stop_pressed = 1;
		low_beam_ticks = 0;
	}
	//
	if (active_time) 
	{
		// at end of active_time signal to stop the heater
		if (active_time == 1) stop_pressed = 1;
		active_time--;
	}
	if (post_active_time) post_active_time--;
	//
	rx_ticks++;
	//
	alive = 1;
}

//--------------------------------------------------------------------------------

void send_serial_byte(byte value)
{
	CLRWDT();
	// compute even parity
    rs232_dummy = 0;
    if (value&1) rs232_dummy++;
    if (value&2) rs232_dummy++; 
    if (value&4) rs232_dummy++; 
    if (value&8) rs232_dummy++; 
    if (value&16) rs232_dummy++;
    if (value&32) rs232_dummy++;
    if (value&64) rs232_dummy++;
    if (value&128) rs232_dummy++;
    // wait if any other transmission
    while (TRMT==0) CLRWDT();
    // set parity first
	TX9D = rs232_dummy&0x01;
	// send the byte
    TXREG = value;
}

//--------------------------------------------------------------------------------

BYTE read_serial_byte(void)
{
	CLRWDT();
    if (RCIF)
    {
		if (OERR==1)
		{
		    CREN = 0;
		    rs232_dummy = RCREG;
		    rs232_dummy = RCREG;
		    rs232_dummy = RCREG;
		    CREN = 1;
		    return 0;
		}
		if (FERR==1)
		{
		    rs232_dummy = RCREG;
		    rs232_dummy = RCREG;
		    rs232_dummy = RCREG;
		    return 0;
		}
		rs232_byte = RCREG;
	    return 1;
	}
	else
	    return 0;
}

//--------------------------------------------------------------------------------

void msg_send(BYTE cmd)
{
	buffer[0] = ((WBUS_CLIENT_ADDR<<4)|WBUS_HOST_ADDR);
	buffer[1] = message_length + 2;
	buffer[2] = cmd;
	for (cnt=0, bytes_count=3; cnt<message_length; cnt++, bytes_count++) buffer[bytes_count] = message_data[cnt];
	// make checksum
	buffer[bytes_count++] = checksum(buffer, bytes_count, 0);
	// Send message
	for (cnt=0; cnt<bytes_count; cnt++) send_serial_byte(buffer[cnt]);
    // wait if any other transmission
    while (TRMT==0) CLRWDT();
	// and purge all receptions
    CREN = 0;
    rs232_dummy = RCREG;
    rs232_dummy = RCREG;
    rs232_dummy = RCREG;
    CREN = 1;
}

//--------------------------------------------------------------------------------

BYTE msg_recv(BYTE cmd, BYTE skip)
{
	message_data[0] = 0; 
	message_data[1] = 0; 
	message_data[2] = 0; 
	message_data[3] = 0; 
	message_data[4] = 0;
	message_data[5] = 0;
	message_data[6] = 0;
	message_data[7] = 0;
	message_data[8] = 0;
	message_data[9] = 0;
	//
	bytes_count = 0;
	rx_ticks = 0;
	do
	{
		if (read_serial_byte() != 0)
		{
			buffer[bytes_count] = rs232_byte;
			bytes_count++;
			rx_ticks = 0;
		}
	}
	while (rx_ticks<RESPONSE_TIMEOUT && bytes_count<sizeof(buffer));
	//
	// check timeout
	if (bytes_count==0)
	{
		// Timeout
		return ERROR_TMO_CHK_ADR;
	}
	// check checksum
	if (checksum(buffer, bytes_count, 0) != 0)
	{
		// Checksum error
		return ERROR_TMO_CHK_ADR;
	}
	// check address
	if (buffer[0] != ((WBUS_HOST_ADDR<<4)|WBUS_CLIENT_ADDR))
	{
		// Incorrect address
		return ERROR_TMO_CHK_ADR;
	}
	// check validity
	if (buffer[2] != (cmd|0x80)) 
	{
		// Request was rejected...
		message_length = 0;
		return ERROR_NACK;
	}
	// copy info to data
	message_length = buffer[1] - 2 - skip;
	bytes_count = 3 + skip;
	for (cnt=0; cnt<message_length; cnt++, bytes_count++) message_data[cnt] = buffer[bytes_count];
	return 0;
}

//--------------------------------------------------------------------------------
//
// Send a client W-Bus request and read answer from Heater.
//
void wb_io(BYTE cmd, BYTE skip)
{
	tries2 = 0;
	do 
	{
		if (tries2!=0) wait_ms(NACK_DELAY);
		tries1 = 0;
		do 
		{
			if (tries1!=0) wait_ms(ERROR_DELAY);
			// wake-up / init sequence
			reset_sleep();
			SPEN = 0;		// disable serial transmission
			tx_line = 0;	// set low the tx pin
			wait_ms(25);	// wait 25 ms
			tx_line = 1;	// set high the tx pin
			SPEN = 1;		// re-enable serial transmission
			wait_ms(25);	// wait another 25 ms
			// send message
			msg_send(cmd);
			// receive reply
			error = msg_recv(cmd, skip);
			tries1++;
		}
		while (tries1<10 && error==ERROR_TMO_CHK_ADR); // timeout, bad checksum or invalid address
		tries2++;
	}
	while (tries2<10 && error==ERROR_NACK); // request denied
}

//--------------------------------------------------------------------------------
//
// Turn heater on for time minutes
//
void turnOn(BYTE time)
{
	message_data[0] = time; 
	message_length = 1; 
	wb_io(WBUS_CMD_ON_PH, 0);
	//
	if (error == 0) 
	{
		int_off();
		active_time = (DWORD)time * (DWORD)60*TICKS_PER_SECOND + 2; // +2 to make sure it will send stop at +1
		post_active_time = 0;
	    pump_check_ticks = 20*TICKS_PER_SECOND;
		int_on();
	}		
}

//--------------------------------------------------------------------------------
//
// Turn heater off
//
void turnOff(void)
{
	message_length = 0; 
	wb_io(WBUS_CMD_OFF, 0);
}

//--------------------------------------------------------------------------------
//	
// get status
//
void get_status(void)
{
	message_data[0] = WBUS_CMD_ON_PH; // polling (keep-alive)
	message_data[1] = 0; 
	message_length = 2; 
	wb_io(WBUS_CMD_CHK, 0); if (error != 0) return;
	//
	message_data[0] = 15; // status 2 - level of devices
	message_length = 1;
	wb_io(WBUS_CMD_QUERY, 1); if (error != 0) return;
	led_circ_pump = OFF; if (message_data[4] != 0) led_circ_pump = ON;
	//
	message_data[0] = 5; // sensors - temperature, volts, flame detection, glow plug resistance
	message_length = 1;
	wb_io(WBUS_CMD_QUERY, 1); if (error != 0) return;
	if (active_time!=0 && message_data[0]==MAX_TEMP+50) active_time = 2; // stop if desired temperature is reached
}





// programul principal  --------------------------------------------------------------

void main(void)
{
	// initializare microcontroler
	PORTA = INITBITS_A;			// port A output latches 0
	TRISA = DIRBITS_A;			// port A direction
	PORTB = INITBITS_B;			// port B output latches 0
	TRISB = DIRBITS_B;			// port B direction
	PIR1 = 0;					// clear peripheral flags
	PIE1 = 0; 					// disable peripheral interrupts
    CMCON = 7;		            // digital i/o
	OPTION = 0B10000100;	 	// no pullups, 32 prescaler
	//
	// serial init
    SPBRG = 119;				// low baud rate (2400), pt 20MHz, 129, (18.432 = 119)
    TXSTA = 0B01100000;         // transmit enable, asynchronous, low baud rate, 9 bits transmission (parity)
    RCSTA = 0B11010000;         // reception enable, continuous receive, 9 bits reception (parity)
    rs232_dummy = RCREG;		// clear uart receiver
    rs232_dummy = RCREG;		// including fifo
    rs232_dummy = RCREG;		// which is three deep
	//
    T0IF = 0;
    TMR0 = -DIVIDE;
    int_on();
	//
	led_circ_pump = OFF; 
	//
	delay_ticks = 0;
	pump_check_ticks = 0;
	low_beam_ticks = 0;
	reset_sleep();
	//
	active_time = 0;
	post_active_time = 0;
	start_pressed = 0;
	stop_pressed = 0;
	//
    while(1)
    {
	    CLRWDT();
	    if (sleep_ticks == 0)
		{
			SLEEP(); 
			NOP();
		    if (ignition==0 && low_beam==1) reset_sleep();
		}
		else
		{
			if (start_pressed == 1)		// manually start
			{
				// set up variables
				int_off();
				start_pressed = 0;
				int_on();
				// and turn it on
				turnOn(MINS_TO_RUN);
			}
		    //
		    if (active_time!=0 || post_active_time!=0) get_status();
		    //
		    if (pump_check_ticks == 0 && led_circ_pump == OFF)
		    {
			    if (active_time > 2) { int_off(); active_time = 1; int_on(); } // va genera stop
			    if (post_active_time > TICKS_PER_SECOND+1) { int_off(); post_active_time = TICKS_PER_SECOND; int_on(); }
			}
			//
			if (stop_pressed == 1)
			{
				// set up variables
				int_off();
				stop_pressed = 0;
				active_time = 0;
			    post_active_time = 5*60*TICKS_PER_SECOND; // monitor for max. another 5 minutes;
			    pump_check_ticks = 0;
				int_on();
				// and turn it off
			    turnOff();
			}
		}
    }
}

// ends here..... /////////////////////////////////////////////////////////////////////
