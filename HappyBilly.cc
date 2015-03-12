#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/siginfo.h>
#include <sys/neutrino.h>
#include <sys/trace.h>		// to support TraceEvent calls
#include <sys/netmgr.h>
#include <sys/syspage.h>
#include <stdint.h>       /* for uintptr_t */
#include <hw/inout.h>     /* for in*() and out*() functions */
#include <sys/mman.h>     /* for mmap_device_io() */

#define A_D_PORT_LENGTH (1)
#define A_D_BASE_ADDRESS (0x280)
#define A_D_COMMAND_REGISTER (A_D_BASE_ADDRESS)
#define A_D_MSB_REGISTER (A_D_BASE_ADDRESS + 1)
#define A_D_CHANNEL_REGISTER (A_D_BASE_ADDRESS + 2)
#define A_D_INPUT_GAIN_REGISTER (A_D_BASE_ADDRESS + 3)
#define A_D_INPUT_STATUS_REGISTER (A_D_INPUT_GAIN_REGISTER)
#define I_O_CONTROL_REGISTER (A_D_BASE_ADDRESS + 4)
#define D_A_LSB_REGISTER (A_D_BASE_ADDRESS + 6)
#define D_A_MSB_CHANNELNO_REGISTER (A_D_BASE_ADDRESS + 7)

#define D_I_O_PORT_LENGTH (1)
#define D_I_O_CONTROL_REGISTER (A_D_BASE_ADDRESS + 0x0b)
#define D_I_O_PORT_A (A_D_BASE_ADDRESS + 0x08)
#define D_I_O_PORT_B (A_D_BASE_ADDRESS + 0x09)

// make static as good programming practice (not in global symbol table)
static uintptr_t a_d_command_handle ;
static uintptr_t a_d_LSB_handle ;
static uintptr_t a_d_MSB_handle ;
static uintptr_t a_d_channel_handle ;
static uintptr_t a_d_input_status_handle ;	// also used once for analog input gain setting

//static uintptr_t d_i_o_control_handle ;		// control register for ports A, B, and C
//static uintptr_t d_i_o_port_a_handle ;
//static uintptr_t d_i_o_port_b_handle ;

static void SetSingleAtoDchannel( int channelNumber )
{
	if ( channelNumber <= 15 && channelNumber >= 0 )
	{
		// configure to only use just one input channel
		out8( a_d_channel_handle, channelNumber | ( channelNumber << 4 ) ) ;
		while ( in8( a_d_input_status_handle ) & 0x20 ) ;	// wait for WAIT bit to go low
															//-- means channel mux is ready to use (takes 9 microseconds)
	}
}

static short MeasureVoltageOnChannel( int channelNumber )
{
	unsigned short value ;
	unsigned short lsb_value ;

	SetSingleAtoDchannel( channelNumber ) ;
	// start the capture of this channel
	out8( a_d_command_handle, 0x90 ) ;	// reset the FIFO and start the measurement
	while ( in8( a_d_input_status_handle ) & 0x80 ) ;		// wait for STS bit to go false meaning data is ready

	lsb_value = in8( a_d_LSB_handle ) ;
	value = in8( a_d_MSB_handle ) << 8 ;
	value |= lsb_value ;
	return (short)value ;
}

static void SendVoltageOnChannel( int channelNumber, int value )
{
	 // Range is 0-4095
	 uint8_t  lsb_value = 0;
	 uint8_t  msb_value = 0;

	 // Put the channel into the MSB.
	 // Bits 6 & 7 hold the DA channels 0-3
	 msb_value = channelNumber;
	 msb_value <<= 6;

	 printf( "\n Output voltage channel number %04x\n", msb_value) ;

	 // compute the value output value first.
	 // Put the lower end of the value into the LSB.
	 lsb_value = value & 255;

     // concatinate bits 11-8 to the channel.
	 msb_value |= (value / 256);

	 printf( "\n Output msb_value %04x\n", msb_value);
	 out8(D_A_LSB_REGISTER, lsb_value);
	 out8(D_A_MSB_CHANNELNO_REGISTER, msb_value);

	 while ( in8( A_D_INPUT_GAIN_REGISTER) & 0x08 ) ;

	 printf( "\n Voltage set\n");
}
/*
static void TestPorts()
{
	unsigned int testValue = 1 ;
	int count ;
	unsigned int portAvalue ;
	unsigned int portBvalue ;

	out8( d_i_o_control_handle, 0x02) ;		// make port B input

	// test output port A one bit at a time from low bit to high bit
	for ( count = 0 ; count < 8 ; count++, testValue <<= 1 )
	{
		out8( d_i_o_port_a_handle, testValue ) ;
		portBvalue = in8( d_i_o_port_b_handle ) ;
		if ( testValue != portBvalue )
			printf( "\nERROR on Port A Out to Port B In at bit %d -- got %u expected %u", count, portBvalue, testValue ) ;
	}

	out8( d_i_o_control_handle, 0x10) ;		// make port A input

	// test output port B one bit at a time from low bit to high bit
	testValue = 1 ;
	for ( count = 0 ; count < 8 ; count++, testValue <<= 1 )
	{
		out8( d_i_o_port_b_handle, testValue ) ;
		portAvalue = in8( d_i_o_port_a_handle ) ;
		if ( testValue != portAvalue )
			printf( "\nERROR on Port B Out to Port A In at bit %d -- got %u expected %u", count, portAvalue, testValue ) ;
	}
	printf( "\nDigital I O ports A and B testing completed\n" ) ;
}
*/

// This board is configured for -10 to +10 volts.
static void SetupAtoD()
{
	uintptr_t i_o_control_handle ;

	/* Get handles to the A to D registers */
	a_d_command_handle = mmap_device_io( A_D_PORT_LENGTH, A_D_COMMAND_REGISTER );
	a_d_LSB_handle = a_d_command_handle ;	// read on command port to get the A/D LSB
	a_d_MSB_handle = mmap_device_io( A_D_PORT_LENGTH, A_D_MSB_REGISTER );
	a_d_channel_handle = mmap_device_io( A_D_PORT_LENGTH, A_D_CHANNEL_REGISTER );
	a_d_input_status_handle = mmap_device_io( A_D_PORT_LENGTH, A_D_INPUT_GAIN_REGISTER );	// set to gain of 1
	i_o_control_handle = mmap_device_io( A_D_PORT_LENGTH, I_O_CONTROL_REGISTER ) ;			// only need for init

	/* Initialize the A/D converter */
	out8( a_d_command_handle, 0x7f );		// clear everything but do not start a new A/D conversion
	out8( a_d_input_status_handle, 0 );		// set to 10 volt range and clear scan mode enable
	out8( i_o_control_handle, 0 ) ;			// set  AINTE to off for polling mode for trigger via A/D command register
}

int GetRootAccess()
{
	int status = 0 ;
	int privity_err ;

	/* Give this thread root permissions to access the hardware */
	privity_err = ThreadCtl( _NTO_TCTL_IO, NULL );
	if ( privity_err == -1 )
	{
		fprintf( stderr, "can't get root permissions\n" );
		status = -1;
	}

	return status ;
}

int main(int argc, char *argv[])
{
	int loop ;

	//TraceEvent( _NTO_TRACESTART ) ;
	//TraceEvent( _NTO_TRACE_INSERTUSRSTREVENT, _NTO_TRACE_USERFIRST, "start test" ) ;
	if ( ! GetRootAccess() )
	{
		SetupAtoD() ;

		printf( "\nStarting measurement on analog input line 0\n" ) ;
		for ( loop = 0 ; loop < 41 ; loop++ )
		{
			printf( "\n%6d", MeasureVoltageOnChannel( 0 ) ) ;

			// Multiply by 100 so that you see a real voltage change.
			SendVoltageOnChannel(0, loop * 100);
			sleep (1);
		}
		printf( "\n\nStarting Port A and Port B tests\n" ) ;

		// set the voltage back to 0.
		SendVoltageOnChannel(0, 0);
	}
	else
		printf( "\nFailure getting root access for I/O register mapping\n") ;

	//TraceEvent( _NTO_TRACE_INSERTUSRSTREVENT, _NTO_TRACE_USERFIRST, "stop test" ) ;
	//TraceEvent( _NTO_TRACESTOP ) ;
	return EXIT_SUCCESS;
}
