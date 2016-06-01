/* Capstone - Team Ellypsis */
/* University of Washington, Mechanical Engineering */
/* Winter-Spring 2015-16 */

/* example of change... */

/* includes */
#include "stdio.h"
#include "MyRio.h"
#include "me477.h"
#include <string.h>
#include <pthread.h>
#include "TimerIRQ.h"
#include "IRQConfigure.h"
#include "ctable.h"
#include "Encoder.h"
#include "time.h"
#include "AIO.h"
#include "matlabfiles.h"
#include <math.h>

struct biquad {
	double b0; double b1; double b2; 	// numerator
	double a0; double a1; double a2; 	// denominator
	double x0; double x1; double x2;	// input
	double y1; double y2; };			// output

#include "Controller4000x.h"
#include "Impulse.h"


/* prototypes */
void *Timer_Irq_Thread(void* resource);
void *Table_Update_Thread(void* resource);
double cascade(int ns, double xin, struct biquad *fa);
double position(MyRio_Encoder encC);
double angle(void);
double double_in(char *prompt);
NiFpga_Status	EncoderC0_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encC0p);
NiFpga_Status	EncoderC1_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encC0p);

/* definitions */
MyRio_Aio CI0;					// Input channel
MyRio_Aio CO0;					// Output channel
NiFpga_Session myrio_session;
MyRio_Encoder encC0;			// encoder
MyRio_Encoder encC1;			// encoder
// uint32_t timeoutValue = 1000;	// T - us

#define IMAX	40000				// max points
#define KMAX	125
static double bufferV[IMAX];	// speed buffer
static double *bpV = bufferV;	// speed buffer pointer
static double bufferV1[IMAX];	// speed buffer
static double *bpV1 = bufferV1;	// speed buffer pointer
static double bufferX[IMAX];	// torque buffer
static double *bpX = bufferX;	// torque buffer pointer
static double bufferT[IMAX];	// speed buffer
static double *bpT = bufferT;	// speed buffer pointer
//static double bufferVDA[KMAX];	// speed buffer
//static double *bpVDA = bufferVDA;	// speed buffer pointer
static int mode, confirm = 0; // 0 = Regular, 1 = Impulse
static double angularVelocity;

// Thread Resource structure for Timer IRQ thread
typedef struct {
	NiFpga_IrqContext timerContext; 	// IRQ context reserved
	struct table *a_table;				// table
	NiFpga_Bool TimerThreadRdy;			// IRQ thread ready flag
} ThreadResource;

// Thread Resource structure for Table Update thread
typedef struct {
	NiFpga_Bool UpdateThreadRdy;		// Table IRQ thread ready flag
	double period_ns;					// Period (nanoseconds)
} ThreadResource2;


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  MAIN SUBPROGRAM  WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW
// Purpose: Initialize table editor variables, analog I/O, and encoder counter.
// Set up timer IRQ and create two threads, one to catch the timer interrupt and one to update the table.
// Call the table editor, then signal both other threads to terminate once table editor exits and
// unregister interrupt.
int main(int argc, char **argv) {
	NiFpga_Status status;

    status = MyRio_Open();		    /*Open the myRIO NiFpga Session.*/
    if (MyRio_IsNotSuccess(status)) return status;

    printf("Capstone Hello World!\n");			// Print to Console
    while (confirm == 0) {
        mode = double_in("\f0 - Regular\n1 - Impulse : \n");
        confirm = double_in("\fare you sure? \nyes - 1, no - 0 : \n");
    }
//    mode = 1;
    MyRio_IrqTimer irqTimer0;
    ThreadResource irqThread0;
    pthread_t thread;

    ThreadResource2 updateThread;
    pthread_t thread2;

    // 1. Initialize table editor variables
    char *Table_Title = "Payload Stabilizer";
    static struct table motor_table[] = {
    		{"Theta:",  0, 0},
    		{"X_ball:", 0, 0},
    		{"X_cart:", 0, 0},
    		{"V_cart:", 0, 0},
    		{"VDAout:", 0, 0},
    };

    irqThread0.a_table = motor_table;


    // 2. Initialize analog interfaces before allowing IRQ
    AIO_initialize(&CI0, &CO0); 		// initialize analog I/O
    Aio_Write(&CO0, 0.0); 				// zero analog output

    // 3. Specify IRQ channel settings
    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;


    // Configure Timer IRQ. Terminate if not successful
    status = Irq_RegisterTimerIrq(
    		&irqTimer0,
    		&irqThread0.timerContext,
    		timeoutValue);

    // Set the indicator to allow the new threads
    irqThread0.TimerThreadRdy = NiFpga_True;
    updateThread.UpdateThreadRdy = NiFpga_True;

    // 4. Initialize encoder counter
    EncoderC0_initialize(myrio_session, &encC0);
    EncoderC1_initialize(myrio_session, &encC1);

    // 5. Create new thread to catch the Timer IRQ.
    status = pthread_create(
    		&thread,
    		NULL,
    		Timer_Irq_Thread,
    		&irqThread0);

    // 6. Create new thread to catch the Table Update IRQ.
    updateThread.period_ns = 500000000L;
    status = pthread_create(
    		&thread2,
    		NULL,
    		Table_Update_Thread,
    		&updateThread);

    // 7. Call table editor
    ctable(Table_Title, motor_table, 5);
    Aio_Write(&CO0, 0);

    // 8. Signal interrupt thread to stop
    irqThread0.TimerThreadRdy = NiFpga_False;
    updateThread.UpdateThreadRdy = NiFpga_False;
    pthread_join(thread, NULL);
    pthread_join(thread2, NULL);

    // Unregister DIO IRQ
    status = Irq_UnregisterTimerIrq(
    		&irqTimer0,
    		irqThread0.timerContext);
    printf("Angular Velocity: %g \n",angularVelocity);

	status = MyRio_Close();	 /*Close the myRIO NiFpga Session. */
	return status;
}


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  TIMER IRQ THREAD  WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWM
// Purpose: Define array of biquad structures corresponding to desired system
// Wait for next interrupt to occur (every 0.5 millisecond), then schedule next interrupt
// Service interrupt by reading analog input value, passing it through cascade function to
// calculate output, send output value back to analog, and acknowledge the interrupt
// Inputs: Generic 'resource' argument; cast to ThreadResource structure
// Outputs: None
void *Timer_Irq_Thread(void* resource) {
	ThreadResource* threadResource = (ThreadResource*) resource;
	MATFILE *mf;
	int err;
	double Xc;
	double Xb;
	double Vc;
	double Th;
	double Vout;
	double Vtemp;
	double Xcomp = 0;
	double Xcartprev = 0;
//	double Vreq = 0;
//	double Vavg = 0;	// Average voltage
//	double Vdrop = 0;	// Oldest value of voltage array, being replaced
	#define L	0.7		// string length (m)

	double *Theta = &((threadResource->a_table+0)->value);
	double *Xball = &((threadResource->a_table+1)->value);
	double *Xcart = &((threadResource->a_table+2)->value);
	double *Vcart = &((threadResource->a_table+3)->value);
	double *VDA   = &((threadResource->a_table+4)->value);
	int 	Ns = 2;					// No. of sections
	double targetD;
	double t = 0, t_inc;
	t_inc = (double) timeoutValue;
	double currentP = 0;
	static double angP1, angP2;
//	static double filteredCartPosition, filteredAngle;

// For system identification
//	int timer = 0;
//	double Vconst = -1;

	while(threadResource->TimerThreadRdy == NiFpga_True) {
		uint32_t irqAssert = 0;
		double V1, V2;
//		double Xballprev = 0;
//		double Vball = 0;
//		double VDAprev;
		int Vmax;
		Irq_Wait( threadResource->timerContext,	// Wait for next interrupt
				  TIMERIRQNO,
				  &irqAssert,
				  (NiFpga_Bool*) &(threadResource->TimerThreadRdy));

		// Schedule next interrupt
		NiFpga_WriteU32( myrio_session,
						 IRQTIMERWRITE,
						 timeoutValue);
		NiFpga_WriteBool( myrio_session,
						  IRQTIMERSETTIME,
						  NiFpga_True);

		if(irqAssert) {

			// ********************** TABLE CALCULATIONS & ROUNDING ************************
			Xc = position(encC0) - currentP;
			*Xcart = round(100000*Xc)/1000.0;
			angP2 = Th;
			Th = 1*angle();
			*Theta = round(1000*Th)/1000.0;
			angP1 = Th;
			Xb = Xc + L*sin(Th*3.1416/180);
			*Xball = round(100000*Xb)/1000.0;
			Vc = (Xc - Xcartprev)/(timeoutValue / 1000000.0);
			*Vcart = round(1000*Vc)/1000.0;
//			Vb = (Xb - Xballprev)/timeoutValue;
			Xcartprev = Xc;


			// ************************ CHECK MODE, RUN CONTROLLER ************************
			if (mode == 1) { //impulse
				Vout = -2;
			} else if (mode == 2) { //wait until angle = 0
//				Vout = -cascade(1, 2*(targetD - Xc), ImpulseController);
//				if (Vout > 0) {
//					Vout = Vout + 3;
//				} else if (Vout < 0){
//					Vout = Vout - 3;
//				}
				Vout = 0;

				if (Th > 0) {
					currentP = Xc;
					mode = 0;
					angularVelocity = M_PI*(angP1 - angP2)/(180.0*timeoutValue/1000000.0); //saves in r/s
//					encC0.cnfg = ENCC_0CNFG;
//				    encC0.stat = ENCC_0STAT;
//				    encC0.cntr = 1;
//				    encC0.cntr = ENCC_0CNTR;
//					EncoderC0_initialize(myrio_session, &encC0);
				}
			} else {
//				filteredAngle = cascade(Butter_ns, Th, Butter);
//				filteredCartPosition = cascade(Butter_ns, Xb, Butter);
//				filteredAngle = Th;
//				filteredCartPosition = Xb;

				Xcomp = cascade(1, Xb, controller3);
				V1 = -cascade(Ns, Xb + Xcomp, controller1);		// Implement control law
				V2 = -cascade(Ns, Th*3.1416/180, controller2);
				// if (bpV < bufferV + IMAX) *bpV++ = V1;			// Store voltage data in array
				// if (bpV1 < bufferV1 + IMAX) *bpV1++ = V2;		// Store voltage data in array

				Vtemp = (V1 + V2)/2.5;
//				Vout = (V1 + V2)/2.5;
				Vout = cascade(1, Vtemp, Butter);
			}
			*VDA = round(1000*Vout)/1000.0;

			if (bpV1 < bufferV1 + IMAX) *bpV1++ = Vtemp;	// Store voltage data in array

			if (bpV < bufferV + IMAX) *bpV++ = Vout;	// Store voltage data in array


//			VDAprev = Vout;

//			// ************************* FRICTION COMPENSATION ****************************
//			if (Vout > 0) {
//				Vout = Vout + 0.78;
//			} else if (Vout < 0){
//				Vout = Vout - 0.78;
//			}


			// *************************** VOLTAGE LIMITING *******************************
			Vmax = 7;
			if (Vout > Vmax) {						// Set saturation voltages
				Vout = Vmax;
			} else if (Vout < -Vmax) {
				Vout = -Vmax;
			}


			// ************************ UPDATE BUFFERS FOR MATLAB *************************
			// if (bpV < bufferV + IMAX) *bpV++ = Vc;	// Store speed data in array
			if (bpX < bufferX + IMAX) *bpX++ = Xc;	// Store position data in array
			if (bpT < bufferT + IMAX) *bpT++ = Th;	// Store angle data in array



//			// **************************** VOLTAGE AVERAGING *****************************
//			Vdrop = *bpVDA;
//			*bpVDA = Vout;
//			if (bpVDA < bufferVDA + KMAX - 1) {
//				bpVDA++;
//			} else {
//				bpVDA = bufferVDA;
//			}
//
//			Vavg = (Vavg*KMAX - Vdrop + Vout)/KMAX;		// Average of recently requested voltages
//			if (bpV1 < bufferV1 + IMAX) *bpV1++ = Vavg;	// Store voltage data in array


//			// Set zero voltage if settled
//			if(Xb > -0.015 && Xb < 0.015) {
//				Vout = 0;
//			}


//			// ************************** SYSTEM IDENTIFICATION ***************************
//			static double Vconst = -2.5;
//			if(timer < 750) {
//				Vout = Vconst;
//			} else {
//				Vout= 0;
//			}
//			timer++;
//
//			static double Vconst = -1;
//
//			timer++;
//			if(timer > 1000) {
//				Vconst = 0;
//			}



			Aio_Write(&CO0, Vout);			// Write output voltage to motor
//			Aio_Write(&CO0, Vconst);			// Write output voltage to motor

			if (mode == 1 && t > 0.5) {
				mode = 2;
				targetD = Xc;
//				printf("/f%f", targetD);
			}
			t = t + t_inc/1000000;

			Irq_Acknowledge(irqAssert);		// Acknowledge interrupt
		}
	}

	// ***************************** SAVE DATA TO MATLAB FILE *****************************
	mf = openmatfile("Lab.mat", &err);
	matfile_addstring(mf, "myName", "ellipsys");
	if (!mf) printf("Can't open mat file %d\n", err);
    matfile_addmatrix(mf, "Vreq", bufferV, IMAX, 1, 0);
    matfile_addmatrix(mf, "Vact", bufferV1, IMAX, 1, 0);
    matfile_addmatrix(mf, "Xcart", bufferX, IMAX, 1, 0);
    matfile_addmatrix(mf, "Theta", bufferT, IMAX, 1, 0);
	matfile_addmatrix(mf, "ThetaDot", &angularVelocity, 1, 1, 0); //r/s

    matfile_close(mf);

	pthread_exit(NULL);
	return NULL;
}


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  TABLE UPDATE THREAD WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWM
// Purpose: Periodically update table of motor and controller values
// Inputs: Generic 'resource' argument; cast to ThreadResource2 structure
// Outputs: None
void *Table_Update_Thread(void* resource) {
	ThreadResource2* threadResource2 = (ThreadResource2*) resource;
	struct timespec time;
	time.tv_sec = 0;
	time.tv_nsec = threadResource2->period_ns;

	while(threadResource2->UpdateThreadRdy == NiFpga_True) {
		nanosleep(&time, NULL);
		update();
	}

	pthread_exit(NULL);
	return NULL;
}


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  CASCADE SUBPROGRAM  WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWM
// Purpose: implement biquad cascade by sending input value through series of biquads
// corresponding to desired transfer function
// Used by *Timer_Irq_Thread to implement control law
// Inputs: number of biquad sections ns (int), input value to transfer function (error here),
// and array of biquad structures containing coefficients and history variables for all sections
// Output: calculated current value of system output y(n) (double)
double cascade(int ns, double xin, struct biquad *fa) {
	struct biquad *f = fa;
	double y0;
	int i;
	y0 = xin;

	for(i = 0; i < ns; i++) {
		f->x0 = y0;		// Current section input is output of last section

		// Calculate current output of each section
		y0 = (f->b0*f->x0 + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2) / f->a0;

		// Update previous x and y values
		f->x2 = f->x1;
		f->x1 = f->x0;
		f->y2 = f->y1;
		f->y1 = y0;
		f++;
	}
	return y0;
}


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  POSITION SUBPROGRAM  WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW
// Purpose: Return cart position in meters by reading encoder counter
// Used by *Timer_Irq_Thread
// Inputs: None
// Outputs: (double) cart position (m)
double position(MyRio_Encoder encC) {
	static int Cn;			// current count
	static int Czero;		// count at zero position
	double x;				// cart position (m)
	double r = 0.022;		// pulley radius (m);
	static int first = 1;

	Cn = Encoder_Counter(&encC);	// read current encoder count
	if(first == 1) {				// set previous count to current count
		Czero = Cn;					// only for first run of function
		first = 0;
	}

	x = (2*3.1416*r/2000)*(Cn - Czero);

	return x;
}


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  ANGLE SUBPROGRAM  WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWM
// Purpose: Return pendulum angle in degrees by reading encoder counter
// Used by *Timer_Irq_Thread
// Inputs: None
// Outputs: (double) angle (degrees)
double angle(void) {
	static int Cn;			// current count
	static int Czero;		// count at zero position
	double theta;			// cart position (m)
	static int first = 1;

	Cn = Encoder_Counter(&encC1);	// read current encoder count
	if(first == 1) {				// set previous count to current count
		Czero = Cn;					// only for first run of function
		first = 0;
	}

	theta = -(Cn - Czero)*360.0/4096;

	return theta;
}


// ***************************************************************************************************
// WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW  DOUBLE_IN SUBPROGRAM  WMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWM
double double_in(char *prompt) {
	int err;
	char String[40]; //not sure why 40
	err = 1; //used as a check to determine if program returns an error
	double val;
	printf_lcd("\f");
	while (err == 1) {
		printf_lcd("\v%s\n", prompt);
		if (fgets_keypad(String, 40) == NULL) {
			printf_lcd("\f\n\nShort. Try Again.");
		} else if ((strpbrk(String, "[]") != NULL) ||
				(strpbrk(String+1, "-") != NULL) ||
				(strstr(String, "..") != NULL)) {
			printf_lcd("\f\n\nBad Key. Try Again.");
		} else {
			err = 0; sscanf(String, "%lf", &val);
		}
	}
	return val;
}
