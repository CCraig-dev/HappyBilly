#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>

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

#include <pthread.h>

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

typedef union {
	struct _pulse pulse;
} my_message_t;

// make pulse code for timer
#define RTC_COMPUTE   _PULSE_CODE_MINAVAIL
#define RTC_TERMINATE   _PULSE_CODE_MINAVAIL + 1
#define RTC_INTERNAL_EXIT   _PULSE_CODE_MINAVAIL + 2

// make static as good programming practice (not in global symbol table)
static uintptr_t a_d_command_handle;
static uintptr_t a_d_LSB_handle;
static uintptr_t a_d_MSB_handle;
static uintptr_t a_d_channel_handle;
static uintptr_t a_d_input_status_handle; // also used once for analog input gain setting

using namespace std;

#define NUMBEROFITERATIONS 1000
const double voltageRange = 20;

int GetRootAccess() {
	int status = 0;
	int privity_err;

	/* Give this thread root permissions to access the hardware */
	privity_err = ThreadCtl(_NTO_TCTL_IO, NULL );
	if (privity_err == -1) {
		fprintf(stderr, "can't get root permissions\n");
		status = -1;
	}

	return status;
}

// This board is configured for -10 to +10 volts.
static void SetupAtoD() {
	uintptr_t i_o_control_handle;

	/* Get handles to the A to D registers */
	a_d_command_handle = mmap_device_io(A_D_PORT_LENGTH, A_D_COMMAND_REGISTER );
	a_d_LSB_handle = a_d_command_handle; // read on command port to get the A/D LSB
	a_d_MSB_handle = mmap_device_io(A_D_PORT_LENGTH, A_D_MSB_REGISTER );
	a_d_channel_handle = mmap_device_io(A_D_PORT_LENGTH, A_D_CHANNEL_REGISTER );
	a_d_input_status_handle = mmap_device_io(A_D_PORT_LENGTH,
			A_D_INPUT_GAIN_REGISTER ); // set to gain of 1
	i_o_control_handle = mmap_device_io(A_D_PORT_LENGTH, I_O_CONTROL_REGISTER ); // only need for init

	/* Initialize the A/D converter */
	out8(a_d_command_handle, 0x7f); // clear everything but do not start a new A/D conversion
	out8(a_d_input_status_handle, 0); // set to 10 volt range and clear scan mode enable
	out8(i_o_control_handle, 0); // set  AINTE to off for polling mode for trigger via A/D command register
}

static void SetSingleAtoDchannel(int channelNumber) {
	if (channelNumber <= 15 && channelNumber >= 0) {
		// configure to only use just one input channel
		out8(a_d_channel_handle, channelNumber | (channelNumber << 4));
		while (in8(a_d_input_status_handle) & 0x20)
			; // wait for WAIT bit to go low
		//-- means channel mux is ready to use (takes 9 microseconds)
	}
}

static double MeasureVoltageOnChannel(int channelNumber) {
	const double voltsPerLSB = voltageRange / 65536; // Page 59 of the Athena manual.

	double volts = 0;

	short value;
	unsigned short lsb_value;

	SetSingleAtoDchannel(channelNumber);
	// start the capture of this channel
	out8(a_d_command_handle, 0x90); // reset the FIFO and start the measurement
	while (in8(a_d_input_status_handle) & 0x80)
		; // wait for STS bit to go false meaning data is ready

	lsb_value = in8(a_d_LSB_handle);
	value = in8(a_d_MSB_handle) << 8;
	value |= lsb_value;

	printf("input value %d\n", value);

	// convert the voltage from bits to actual volts.
	volts = value * voltsPerLSB;

	printf("input volts %f\n", volts);

	return volts;
}

// going to need a wrapper function here.


static void SendVoltageOnChannel(int channelNumber, double volts) {
	const double voltsPerLSB = voltageRange / 4096; // Page 59 of the Athena manual.

	unsigned short voltsInBits = 0;

	// used to push the voltsInBits value into the registers.
	uint8_t lsb_value = 0;
	uint8_t msb_value = 0;

	// convert the voltage value to the range of 0-4095.
	// 0 = -10V and 4095 = 9.9951      // Page 59 of the Athena manual.

	// What we're doing here is shifting our negative values up by so
	// so that they map to the 0 - 4095 range.  The chip doesn't care
	// what the value means to us.
	voltsInBits = ((volts + voltageRange / 2.0) / voltsPerLSB);

	// corner case the range is 0 - 4095 if we get 4096
	// we wrap around.
	if (voltsInBits > 4095) {
		voltsInBits = 4095;
	}

	//	 printf("volts %f\n", volts);
	//	 printf("voltsInBits %d\n", voltsInBits);

	// Put the channel into the MSB.
	// Bits 6 & 7 hold the DA channels 0-3
	msb_value = channelNumber;
	msb_value <<= 6;

	printf("\n Output voltage channel number %04x\n", msb_value);

	// compute the value output value first.
	// Put the lower end of the value into the LSB.
	lsb_value = voltsInBits & 255;

	// concatenate bits 11-8 to the channel.
	msb_value |= (voltsInBits / 256);

	printf("\n Output lsb_value %04x\n", lsb_value);
	printf("\n Output msb_value %04x\n", msb_value);

	out8(D_A_LSB_REGISTER, lsb_value);
	out8(D_A_MSB_CHANNELNO_REGISTER, msb_value);

	// Poll the DACBSY bit until the D to A conversion is done.
	while (in8(A_D_INPUT_GAIN_REGISTER) & 0x08)
		;

	printf("\n Voltage set\n");
}

int main(int argc, char *argv[]) {

	double results[NUMBEROFITERATIONS];
	double stepInputValueVolts = 0;
	double prevStepInputValueVolts = 0;

	int channelID = 0;
	struct sigevent event; //Event to fire every 10ms
	timer_t timer_id;
	struct itimerspec itime; //Set the timer for polling (10ms)
	my_message_t msg;
	int rcvid;

	//Worker thread for performing the parameterization of the voltage-in

	//Message Channel for thread messaging
	channelID = ChannelCreate(0);

	//Event to fire every 10ms
	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, channelID,
			_NTO_SIDE_CHANNEL, 0);
	event.sigev_priority = getprio(0);
	event.sigev_code = RTC_COMPUTE;
	timer_create(CLOCK_REALTIME, &event, &timer_id);

	//Set the timer for polling (10ms)
	itime.it_value.tv_sec = 0;
	// 100 million nsecs = .1 secs
	itime.it_value.tv_nsec = 100000000;
	itime.it_interval.tv_sec = 0;
	// 100 million nsecs = .1 secs
	itime.it_interval.tv_nsec = 100000000;
	timer_settime(timer_id, 0, &itime, NULL);

	//A timer will exist to drive the register polling (10ms)
	//The timer replaces the sleep command in this instance
	pthread_mutex_t timimgMutex;
	pthread_mutex_init(&timimgMutex, NULL);
	pthread_mutex_lock(&timimgMutex);

	if (!GetRootAccess()) {

		SetupAtoD();
		int k = 0;

		while (true) {

			if (k == NUMBEROFITERATIONS) {
				MsgSendPulse(channelID, 10, _PULSE_CODE_MAXAVAIL, NULL);
				continue;
			}

			rcvid = MsgReceive(channelID, &msg, sizeof(msg), NULL);
			if (rcvid == 0) {
				if (msg.pulse.code == RTC_COMPUTE) {
					//Recompute the transfer function and modify local state

					// Step 1 Determine the time-domain difference equation for the transfer function.
					//  y(k+1) = y(k) - 0.632y(k-1) + 0.368 u(k) + 0.264 u(k - 1)

					stepInputValueVolts = MeasureVoltageOnChannel(0);

					// The first two iterations are special cases because we don't have
					// any history
					if (k == 0) {
						// y(k+1) = 0.368 u(k);
						results[k] = 0.368 * stepInputValueVolts;
					} else if (k == 1) {
						// y(k+1) = y(k) + 0.368 u(k) + 0.264 u(k - 1);
						results[k] = results[k - 1] + 0.368
								* stepInputValueVolts + 0.264
								* prevStepInputValueVolts;
					} else {
						//  y(k+1) = y(k) - 0.632y(k-1) + 0.368 u(k) + 0.264 u(k - 1)
						results[k] = results[k - 1] - 0.632 * results[k - 2]
								+ 0.368 * stepInputValueVolts + 0.264
								* prevStepInputValueVolts;
					}
					prevStepInputValueVolts = stepInputValueVolts;

					// 3a. Run this calculation as a periodic task executing at a 10Hz period

					SendVoltageOnChannel(0, results[k]);
				}
				if (msg.pulse.code == RTC_TERMINATE) {
					//End the program
					cout << "End the program.\n";

					// 2b. Open a file in /tmp and write the output values to that file. Give the
					//     file a .csv extension
					ofstream outputFile;
					outputFile.open("/tmp/example.csv");

					for (int k = 0; k < NUMBEROFITERATIONS; ++k) {
						outputFile << results[k] << ", ";
					}

					outputFile << endl;

					outputFile.close();
					exit(0);
				}
				if (msg.pulse.code == RTC_INTERNAL_EXIT) {
					//Unknown, needs clarification
					cout << "Death.\n";
				}
			}
			//The number of iterations passed has increased by one
			k++;
		}
		// set the voltage back to 0.
		SendVoltageOnChannel(0, 0);
	}
	return EXIT_SUCCESS;
}
