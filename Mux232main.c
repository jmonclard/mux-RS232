/*--- MUX232MAIN.C ----*/
#include <xc.h>
#include <pic.h>
#include <ctype.h>
#include "memory.h"


/*__IDLOC(1);*/


#pragma config FOSC = HS    // => Quartz externe 20MHz
#pragma config WDTE = OFF   // => Watchdog timer disabled
#pragma config MCLRE = OFF  // => Master clear disabled, GPIO3=Input
#pragma config BOREN = OFF  // => Brown out detection disabled
#pragma config CPD = OFF    // => No protection of program code and data block
#pragma config PWRTE = ON   // => Power up timer enabled


#define UART0_IN	RA0
#define UART1_IN	RA1
#define UART2_IN	RA2
#define VALID		RA3
#define LED0		RC2
#define VBATT		RC3
/*#define TXD		RC4*/
/*#define RXD		RC5*/

/* Freq quartz = 20 MHz => Tquartz=50ns => Tosc= 200ns */
/* on veut 4800 bauds avec un suréchantillonage de 4 => IT à 19200 Hz */
/* 19200 Hz <=> 52.083333 us soit 260.416667 fois Tosc. */
/* Le temps de latence de l'IT est de 21 fois Tosc => il faut compter 239.416667 périodes de Tosc */
/* on initialise donc T0 à la valeur 256-239 = 17 */
/* La duree reelle sera de : */
/* 256-17=239; 239+21=260; 260*200ns=52us => 19231 Hz; 19231/4=4808 bauds (+2%) */

#define TIMER_INI_DEFAULT 29	
#define WAIT_DEFAULT 200 /* attente après le dernier caractère 200/19231=0.01s soit la durée de 5 caracteres */
#define LED_OFF 1024

typedef unsigned char byte;
typedef unsigned int word;
/*
eeprom byte mem_timer;
eeprom byte mem_wait;
*/
#define LF 0x0A

#define FIFO_SIZE 128
#define FIFO_LAST 0x7F
#define FIFO_MASK 0x7F

byte fifo0[FIFO_SIZE];
bank1 byte fifo1[FIFO_SIZE];
bank1 byte fifo2[FIFO_SIZE];
byte fifo0wr;
byte fifo0rd;
byte fifo1wr;
byte fifo1rd;
byte fifo2wr;
byte fifo2rd;

byte send_channel;
byte send_data;
byte received_data;
bit sending;
byte send_wait_next_byte;

byte receive0_cpt_ticks;
byte receive0_data;
byte receive1_cpt_ticks;
byte receive1_data;
byte receive2_cpt_ticks;
byte receive2_data;

word cpt_led;
byte etat;
byte i;
word somme;
word wval;

byte par_hi;
byte timer_ini;
byte wait;
byte state;

/**
 * 
 */
void receive0(void)
{
	if (receive0_cpt_ticks==0xff) /* waiting for start bit */
	{
		if (UART0_IN==0)	/* start bit */
		{
			receive0_cpt_ticks = 0;
		}
	}
	else
	{
		receive0_cpt_ticks++;
	
		switch (receive0_cpt_ticks)
		{
			case  6 :
			case 10 :
			case 14 :
			case 18 :
			case 22 :
			case 26 :
			case 30 :
			case 34 :	
						receive0_data >>= 1;
						if (UART0_IN==1) receive0_data |= 0x80;
						break;

			case 38 :	
						if (UART0_IN==1)			/* stop bit => save data if you can*/
						{
							if (((fifo0wr+1) & FIFO_LAST)!=fifo0rd)
							{
								fifo0[fifo0wr]=receive0_data;
								fifo0wr++;
								fifo0wr &= FIFO_MASK;
							}						
							/* else fifo full => discard data */
						}							
						/* else no stop bit => discard data */
						receive0_cpt_ticks = 0xff;	/* anyway, wait for start bit */
						break;
		}
	}
}

/**
 * 
 */
void receive1(void)
{
	if (receive1_cpt_ticks==0xff) /* waiting for start bit */
	{
		if (UART1_IN==0)	/* start bit */
		{
			receive1_cpt_ticks = 0;
		}
	}
	else
	{
		receive1_cpt_ticks++;
	
		switch (receive1_cpt_ticks)
		{
			case  6 :
			case 10 :
			case 14 :
			case 18 :
			case 22 :
			case 26 :
			case 30 :
			case 34 :	
						receive1_data >>= 1;
						if (UART1_IN==1) receive1_data |= 0x80;
						break;

			case 38 :	
						if (UART1_IN==1)			/* stop bit => save data if you can*/
						{
							if (((fifo1wr+1) & FIFO_LAST)!=fifo1rd)
							{
								fifo1[fifo1wr]=receive1_data;
								fifo1wr++;
								fifo1wr &= FIFO_MASK;
							}						
							/* else fifo full => discard data */
						}							
						/* else no stop bit => discard data */
						receive1_cpt_ticks = 0xff;	/* anyway, wait for start bit */
						break;
		}
	}
}


void receive2(void)
{
	if (receive2_cpt_ticks==0xff) /* waiting for start bit */
	{
		if (UART2_IN==0)	/* start bit */
		{
			receive2_cpt_ticks = 0;
		}
	}
	else
	{
		receive2_cpt_ticks++;
	
		switch (receive2_cpt_ticks)
		{
			case  6 :
			case 10 :
			case 14 :
			case 18 :
			case 22 :
			case 26 :
			case 30 :
			case 34 :	
						receive2_data >>= 1;
						if (UART2_IN==1) receive2_data |= 0x80;
						break;

			case 38 :	
						if (UART2_IN==1)			/* stop bit => save data if you can*/
						{
							if (((fifo2wr+1) & FIFO_LAST)!=fifo2rd)
							{
								fifo2[fifo2wr]=receive2_data;
								fifo2wr++;
								fifo2wr &= FIFO_MASK;
							}						
							/* else fifo full => discard data */
						}							
						/* else no stop bit => discard data */
						receive2_cpt_ticks = 0xff;	/* anyway, wait for start bit */
						break;
		}
	}
}



void CheckIfSomethingToBeSent(void)
{
	/* check current channel if empty then next time check the other one */
	if (send_channel==0)
	{
		if (fifo0rd==fifo0wr) {		/* fifo 0 empty */
			sending=0;
			send_wait_next_byte--;
			if (send_wait_next_byte==0)
			{
				send_wait_next_byte = wait;
				send_channel = 1;		/* next time check channel 1 */
			}
		}
		else	/* get data */
		{
			send_data = fifo0[fifo0rd];
			fifo0rd++;
			fifo0rd &= FIFO_MASK;
			sending=1;
			TXREG = send_data;
			send_wait_next_byte = wait;
//			if (send_data==LF) send_channel=1;				/* terminator => next time check channel 1 */
		}
	}
	else if (send_channel==1)
	{
		if (fifo1rd==fifo1wr) {		/* fifo 1 empty */
			sending=0;
			send_wait_next_byte--;
			if (send_wait_next_byte==0)
			{
				send_wait_next_byte = wait;
			 	send_channel = 2;		/* next time check channel 2 */
			}
		}
		else	/* get data */
		{
			send_data = fifo1[fifo1rd++];
			fifo1rd &= FIFO_MASK;
			sending=1;
			TXREG = send_data;
			send_wait_next_byte = wait;
//			if (send_data==LF) send_channel=2;				/* terminator => next time check channel 2 */
		}
	}
	else
	{
		if (fifo2rd==fifo2wr) {		/* fifo 2 empty */
			sending=0;
			send_wait_next_byte--;
			if (send_wait_next_byte==0)
			{
				send_wait_next_byte = wait;
			 	send_channel = 0;		/* next time check channel 0 */
			}
		}
		else	/* get data */
		{
			send_data = fifo2[fifo2rd++];
			fifo2rd &= FIFO_MASK;
			sending=1;
			TXREG = send_data;
			send_wait_next_byte = wait;
//			if (send_data==LF) send_channel=0;				/* terminator => next time check channel 0 */
		}
	}
}


/**
 * 
 */
void Init(void)
{

	TRISA = 0xFF;	/* Tout le port A en entree */
	TRISC = 0x2A;	/* RC0, RC2 et RC4 en sorties, les autre en entrees */

	PORTC = 0xFF;
	//VRCON = 0x00;	/* disable VREF */
        FVREN=0;
	//CMCON0 = 0x07;	/* disable comparator */
        C1ON = 0;
        C2ON = 0;

	TMR0 = 0x00;
	//OPTION = 0xDF;	/* disable pull up */
        WPUA = 0x00;
        WPUC = 0x00;
TODO
	//JM ANSEL = 0x00;
	//JM ANSEL = 0x80;	/* RC3/AN7 <= analog */
	ADCON1 = 0x20;
	ADCON0 = 0x9C;
	//JM IOCA = 0x08;   /* interrupt on change sur RA3 */
	
	/* initialisation UART */
	SPBRGH = 0x00;
	SPBRG = 64;
	//JM BAUDCTL = 0x00;
	RCSTA = 0x90;
	TXSTA = 0x20;
	
	TMR0 = timer_ini;
	T0CS = 0; /*enable T0 */
	INTCON = 0xA8; /* IOCA3 et Timer0 */
	T0IF = 0;

	return;
}


/**
 * 
 */
void interrupt timer0(void)
{
	RC0 = 1;

	if (T0IF==1)
	{
		TMR0 = timer_ini;
		receive0();
		receive2();
		receive1();
		if (sending==0) CheckIfSomethingToBeSent();
		cpt_led++;
		T0IF = 0;
	}
	if (TXIF==1)
	{
		TXIF=0;
		CheckIfSomethingToBeSent();
	}
        TODO
	//JM RAIF=0;
	EEIF=0;

	RC0 = 0;
}

void send(byte b)
{
	while (fifo2wr != fifo2rd) ; // wait for an empty fifo (=> low priority)
	GIE=0;
	fifo2[fifo2wr]=b;
	fifo2wr++;
	fifo2wr &= FIFO_MASK;
	GIE=1;
}

void sendline(const char *pc)
{
	while (*pc!=0)
	{
		send(*pc);
		pc++;
	}
	send(0x0d);
	send(0x0a);
}


byte ValueToAsciiHex(byte c)
{
	byte tmp;

	tmp = c & 0x0f;
	if (tmp<10) 
	{
		tmp = tmp | 0x30;
	}
	else
	{
		tmp = tmp -10 + 0x41;
	}
	return tmp;
}

byte AsciiHexToValue(byte c)
{
	byte tmp_c;

	if ((c>=0x30) && (c<=0x39))
	{
		tmp_c = c & 0x0f;
	}
	else
	{
		tmp_c = c & 0xdf;	// convert to uppercase
		tmp_c = tmp_c - 0x41 + 10;
	}
	return tmp_c;
}

byte TwoAsciiHexToValue(byte c1, byte c2)
{
	byte tmp;
	byte tmp2;

	tmp = AsciiHexToValue(c1);
	tmp = tmp <<4;
	tmp2 = AsciiHexToValue(c2);
	tmp = tmp | tmp2;
	return tmp;
}


/**
 * 
 */
void main(void)
{
	receive0_cpt_ticks = 0xFF;
	receive1_cpt_ticks = 0xFF;
	receive2_cpt_ticks = 0xFF;
	fifo0wr=0;
	fifo0rd=0;
	fifo1wr=0;
	fifo1rd=0;
	fifo2wr=0;
	fifo2rd=0;
	send_channel = 0;
	cpt_led = 0;
	sending = 0;
	state = 0;
	timer_ini = DATAEE_ReadByte(0x00);
	if (timer_ini==0xff) {
		timer_ini=TIMER_INI_DEFAULT;
		DATAEE_WriteByte(0x00,timer_ini);
	}

	wait = DATAEE_ReadByte(0x01);
	if (wait==0xff) {
		wait=WAIT_DEFAULT;
		DATAEE_WriteByte(0x01,wait);
	}
	send_wait_next_byte = wait;

	Init();

	LED0 = 0;

	while(1)
	{
		/* flash de 600us toutes les 3.4 secondes */

		if (VALID==0)
		{
			LED0 = 0;
			asm("sleep");
			asm("nop");
		}

		if (cpt_led<LED_OFF)
		{
			LED0 = 1;
		}
		else
		{
			LED0 = 0;
		}
		if (RCIF==1)
		{
			received_data = RCREG;
			RCIF=0;
			switch(state)
			{
				case 1 :
					send(received_data);
					switch(received_data)
					{
						case 'v' : // display current timer value
							send(ValueToAsciiHex(timer_ini/16));
							send(ValueToAsciiHex(timer_ini%16));
							send(0x0d);
							send(0x0a);
							state=0;
						break;
						case 'w' : // write timer value to EEPROM
							DATAEE_WriteByte(0x00,timer_ini);
							sendline("Ok.");
							state=0;
						break;
						case 'r' : // read timer value from EEPROM
							timer_ini = DATAEE_ReadByte(0x00);
							send(ValueToAsciiHex(timer_ini/16));
							send(ValueToAsciiHex(timer_ini%16));
							send(0x0d);
							send(0x0a);
							state=0;
						break;
						case 'd' : // define timer value, need 2 extra bytes
							send('=');
							state=2;
						break;
						default :
							sendline("??? command aborted.");
							state=0;
						break;
					}
					break;

				case 2 :
					send(received_data);
					if (isxdigit(received_data))
					{
						par_hi = received_data;
						state = 3;
					}
					else
					{
						sendline("??? command aborted.");
						state = 0;
					}
					received_data=' ';
					break;
				
				case 3 :
					send(received_data);
					if (isxdigit(received_data))
					{
						timer_ini = TwoAsciiHexToValue(par_hi,received_data);
						sendline(" Ok.");
						state = 0;
					}
					else
					{
						sendline("??? command aborted.");
						state = 0;
					}
					received_data=' ';
					break;

				case 11 :
					send(received_data);
					switch(received_data)
					{
						case 'v' : // display current wait value
							send(ValueToAsciiHex(wait/16));
							send(ValueToAsciiHex(wait%16));
							send(0x0d);
							send(0x0a);
							state=0;
						break;
						case 'w' : // write wait value to EEPROM
							DATAEE_WriteByte(0x01,wait);
							sendline("Ok.");
							state=0;
						break;
						case 'r' : // read wait value from EEPROM
							wait = DATAEE_ReadByte(0x01);
							send(ValueToAsciiHex(wait/16));
							send(ValueToAsciiHex(wait%16));
							send(0x0d);
							send(0x0a);
							state=0;
						break;
						case 'd' : // define wait value, need 2 extra bytes
							send('=');
							state=12;
						break;
						default :
							sendline("??? command aborted.");
							state=0;
						break;
					}
					break;

				case 12 :
					send(received_data);
					if (isxdigit(received_data))
					{
						par_hi = received_data;
						state = 13;
					}
					else
					{
						sendline("??? command aborted.");
						state = 0;
					}
					received_data=' ';
					break;
				
				case 13 :
					send(received_data);
					if (isxdigit(received_data))
					{
						wait = TwoAsciiHexToValue(par_hi,received_data);
						sendline(" Ok.");
						state = 0;
					}
					else
					{
						sendline("??? command aborted.");
						state = 0;
					}
					received_data=' ';
					break;

				default :
					switch (received_data)
					{
						case 'V' :
						case 'v' :
							sendline("1.1");
						break;

						case 'P' :
						case 'p' :
							sendline("00168A");
						break;

						case 'S' :
						case 's' :
							sendline("0005");
						break;

						case 'B' :
						case 'b' :
							ADCON0 = 0x9D;	// power on
							somme = 0;
							for (i=0; i<100; i++)
							{
								ADCON0 = 0x9F;	// go
								do
								{
									etat = (ADCON0 & 0x02);
								}
								while (etat==0x02);
								wval= ADRESH;
								wval = wval << 8;
								wval = wval + ADRESL;
								somme += wval;
							}
							wval = somme / 39;
							if (wval<=325)
							{
								wval=0;
							}
							else
							if (wval>=416)
							{
								wval=100;
							}
							else
							{
								wval=wval -316;
							}
							send((wval/100)+0x30);
							send(((wval/10)%10)+0x30);
							send(((wval % 10)+0x30));
							sendline ("%");
							ADCON0 = 0x9C;	// power off
						break;

						case 'T' :
						case 't' :
							send(received_data);
							state = 1;
						break;

						case 'W' :
						case 'w' :
							send(received_data);
							state = 11;
						break;

						default :
							sendline ("");
							sendline ("------ RS232 Mux (c)Metraware 2007 ------");
							sendline ("List of supported commands :");
							sendline ("H    : This help");
							sendline ("V    : Software version");
							sendline ("P    : Part number");
							sendline ("S    : Serial number");
							sendline ("B    : Battery (0-100%)");
							sendline ("Tdhh : Define value for baud rate generator. From 00 to FF. Default 1D (decimal 29)");
							sendline ("Tv   : Current value of T");
							sendline ("Tw   : Save T value in EEPROM");
							sendline ("Tr   : Read T value from EEPROM");
							sendline ("Wdhh : Define value for timeout at end of line. From 00 to FF. Default C8 (decimal 200)");
							sendline ("Wv   : Current value of timeout");
							sendline ("Ww   : Save timeout value in EEPROM");
							sendline ("Wr   : Read timeout value from EEPROM");
							sendline ("?    : This help");
							sendline ("");
							sendline ("Configuration : 3 channels, FIFO_SIZE bytes fifo.");
							sendline ("-------------------------------------------");
							sendline ("");
						break;
					}
				break;
			}
		}

	}
}


