#include <msp430.h>
#include "uart.h"

#define SERVO_1 (BIT4)
#define SERVO_2 (BIT5)
#define SERVO_COUNT (2)
#define START_CHAR ('#')

#define ixServo (0)
#define ixPulseWidth (1)

unsigned char buffer[2];
unsigned char step;

unsigned int counter = 0;
unsigned int servoPosition[SERVO_COUNT] = {1500, 1500};
const unsigned int servoOn[SERVO_COUNT] = {SERVO_1, SERVO_2};

void uart_rx_isr(unsigned char c);
void setPulseWidth(void);

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ; // Set DCO
    DCOCTL = CALDCO_1MHZ;

    //setup UART to receive signals
    step = 0;
    uart_init();

    P1DIR |= BIT0 + BIT6 + SERVO_1 + SERVO_2; //LED 1 + P1.4 + P1.5
    P1OUT &= ~(BIT0 + BIT6 + SERVO_1 + SERVO_2);
    P1SEL &= ~(BIT0 + BIT6 + SERVO_1 + SERVO_2); //set servo pins as I/O

    // register ISR called when data was received
    uart_set_rx_isr_ptr(uart_rx_isr);

    //set timer
    TA0CCR0 = 10000;

    TACTL = TASSEL_2 + MC_1; //SMCLK, Up mode
    TACCTL0 = CCIE; //enable interrupt

    //enter low power mode with interrupts enabled
    _bis_SR_register(LPM0_bits + GIE);
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
	if (counter > SERVO_COUNT) {
		counter = 0;
	}

	//stop output to ports
	P1OUT &= ~(BIT0 + SERVO_1 + SERVO_2);

	if (counter < SERVO_COUNT) {
		P1OUT |= BIT0 + servoOn[counter];
		TA0CCR0 = servoPosition[counter];
	} else {
		//add padding to complete the 20ms period
		TA0CCR0 = 20000 - (servoPosition[0] + servoPosition[1]);
	}

	counter++;
}

void uart_rx_isr(unsigned char c)
{
	switch (step) {

	case 0:
		//only proceed when we receive start char
		if (c == START_CHAR) {
			step++;
		}
		break;

	case 1: //servo id
		buffer[ixServo] = c;
		step++;
		break;

	case 2: //pulse width
		buffer[ixPulseWidth] = c;
		setPulseWidth();
		step = 0;
		break;
	}

	P1OUT ^= BIT6;		// toggle P1.6 (green led)
}

void setPulseWidth()
{
	int w = buffer[ixPulseWidth];
	//usual range is 10ms to 20ms
	//but we allow 5ms to 25ms to allow some overflow
	if (50 <= w && w <= 250) {
		w *= 10;
		char servo = buffer[ixServo];

		if (servo == 'A') {
			servoPosition[0] = w;

		} else if (servo == 'B') {
			servoPosition[1] = w;

		} else {
			//invalid instruction, reset to default position
			servoPosition[0] = 1500;
			servoPosition[1] = 1500;
		}
	}
}
