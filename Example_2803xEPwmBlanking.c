 #include "DSP28x_Project.h"
#include <stdint.h>
#include <math.h>
#ifdef SUB_DIVISION
#include <IQmathLib.h>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SINGLE_FOUR_STEPS
//#define SUB_DIVISION

#define PUL_DUTY 1024
#define PUL_INV	850
#define PUL_VAL	PUL_DUTY-PUL_INV
#define TOL_TM		5000
#define POWER_TM_28V	300

#define MOTOR_STOPPED 0
#define MOTOR_RUNNING	1

#define CLOCKWISE		0
#define COUNTERCW	1

#define DUTY_MIN	7000
#define DUTY_MAX	18000
#define DUTY_STOP	4000
#define ACC_STEPS		24
#define true	1
#define false	0
struct HsmState_BITS{
	Uint16 state:1;		// Stepper motor state, running or stopped
	Uint16 dir:1;		//Stepper motor running direction, clockwise or counterclockwise
	Uint16 first:1;	//Whether the first step or not
	Uint16 rsv0:13;	//reserved
};

union HsmState{
	Uint16 all;
	struct HsmState_BITS bit;
};

struct HSM {
	union HsmState hst;
	int16 steps;
	float32 usSetSpeed;
#ifdef SUB_DIVISION
	int16 subDivN;
	_iq stepW;
#endif
};

volatile struct HSM Motor;
Uint16 Voltage[3];
Uint16 uLoops=0;
volatile Uint32 uSpeed=0;
Uint16 uDir=0;

#ifdef SUB_DIVISION
long GlobalQ = GLOBAL_Q;
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma CODE_SECTION(isr_Adc, "ramfuncs");
#pragma CODE_SECTION(isr_Ecap1, "ramfuncs");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
__interrupt void  isr_Adc(void);
__interrupt void isr_Ecap1(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Enable_Soc(Uint16 index,Uint16 channel);
void Config_Adc(void);
void Config_2104(Uint16 bShutdown);
void Config_Led(Uint16 bLightUp);
void Init_Epwm(Uint16 index);
void Init_Eqep(void);
void InitECapture(void);
void InitGpio5(void);
void InitHSM(void);
void Adjust_Duty(Uint16 index,Uint16 channel,Uint16 chopPRD,Uint16 chopCLR);
void Motor_Run(float32 usStep);
void StopMotor(void);
void StartMotor(Uint32 uSpd);
void CalcStepTm(float32 Tmin,float32* StepTm);
void SlowDecay(void);
void ECap1Int(Uint16 bEnable);

#ifdef SUB_DIVISION
Uint16  CalcSPWM(Uint16 index);
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void main(void) {
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
   	InitSysCtrl();

  	// Copy time critical code and Flash setup code to RAM
  	//memcpy((uint16_t *)&RamfuncsRunStart,(uint16_t *)&RamfuncsLoadStart, (unsigned long)&RamfuncsLoadSize);
  	// Call Flash Initialization to setup flash waitstates
  	// This function must reside in RAM
  	InitFlash();

	// Step 2. Initialize GPIO:
	// This example function is found in the DSP2803x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	InitSciaGpio();
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEQep1Gpio();
	InitECap1Gpio();
	InitGpio5();

	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
	InitPieVectTable();

	//ACCORDING TO PROPERINTERRUPT INITIALIZATION PROCEDURE
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.ADCINT1 = &isr_Adc;
	PieVectTable.ECAP1_INT = &isr_Ecap1;
	EDIS;	// This is needed to disable write to EALLOW protected registers

	// Step 4. Initialize all the Device Peripherals:
	// Initialize the ADC:
	InitAdc();  // For this example, init the ADC
	AdcOffsetSelfCal();

	//ACCORDING TO PROPERINTERRUPT INITIALIZATION PROCEDURE
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	Config_Led(1);	//light the higher LED
	Config_2104(0);	//disable the IR2104

	// Enable CPU INT3 which is connected to EPWM1-3 INT: EPWM
	//IER |= M_INT3;
	// Enable EPWM INTn in the PIE: Group 3 interrupt 1-2
	//PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
	//PieCtrlRegs.PIEIER3.bit.INTx2 = 1;

	IER |= M_INT1; 						// Enable CPU Interrupt 1
	IER |= M_INT4;
	// Enable ADCINT1 in PIE
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// Enable INT 1.1 in the PIE
	// Enable eCAP INTn in the PIE: Group 3 interrupt 1-6
	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;

	Config_Adc();

	// Enable global Interrupts and higher priority real-time debug events:
	EINT;	// Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

	Init_Epwm(0);
	Enable_Soc(0,1);
	Init_Epwm(1);
	Enable_Soc(1,1);
	Init_Eqep();
	InitECapture();
	InitHSM();

	// Step 6. IDLE loop. Just sit and loop forever (optional):
	while (1) {
#ifdef SINGLE_FOUR_STEPS
	if (Motor.hst.bit.state == MOTOR_RUNNING &&DUTY_STOP<uSpeed
			&& uSpeed <=DUTY_MIN) {
		StopMotor();
	}
	if (Motor.hst.bit.state == MOTOR_STOPPED && DUTY_MIN < uSpeed
			&& uSpeed < DUTY_MAX) {
		StartMotor(uSpeed);
	}
//	if (Motor.hst.bit.state == MOTOR_RUNNING && uSpeed < STOP_SPEED
//			&& uDir != Motor.hst.bit.dir) {
//		StopMotor();
//		StartMotor(uDir, uSpeed);
//	}
	if (Motor.hst.bit.state == MOTOR_RUNNING) {
		Motor_Run(Motor.usSetSpeed);
	}

#endif

#ifdef SUB_DIVISION
		Motor_Run(TOL_TM / Motor.subDivN);
#endif
	}
}

void InitHSM(void)
{
	Motor.hst.all=0;
	Motor.hst.bit.state=MOTOR_STOPPED;
	Motor.hst.bit.dir=CLOCKWISE;
	//uDir=GpioDataRegs.GPADAT.bit.GPIO6;
	Motor.hst.bit.first=1;
	Motor.steps=0;
	Motor.usSetSpeed=0.0;

	SlowDecay();
	Config_2104(1);	//enable the IR2104

#ifdef SUB_DIVISION
	Motor.subDivN = 8;
	Motor.stepW = _IQdiv(_IQ(1.5707963), _IQ(Motor.subDivN));
#endif
}

void Motor_Run(float32 usStep) {
	Uint16 tmLeft=0;
#ifdef SINGLE_FOUR_STEPS
	switch (Motor.hst.bit.dir) {
	case CLOCKWISE:
		Motor.steps  = (Motor.steps  + 1) % 4;
		break;
	case COUNTERCW:
		Motor.steps  = (Motor.steps  + 3) % 4;
		break;
	default:
		break;
	}

	//Power up and fast decay phase
	switch (Motor.steps) {
	case 0:
		//Phase A+
		if (Motor.hst.bit.first) {
			Motor.hst.bit.first = 0;
			Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);	// B+ slow decay
		} else
			Adjust_Duty(1, 0, PUL_DUTY, 0);	//B+ fast decay

		Adjust_Duty(0, 0, PUL_DUTY, 0);	//A+ power up
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
		break;
	case 1:
		//Phase B+
		Adjust_Duty(0, 1, PUL_DUTY, 0);	//A- fast decay
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, 0);	//B+ power up
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
		break;
	case 2:
		//Phase A-
		Adjust_Duty(1, 1, PUL_DUTY, 0);	//B- fast decay
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(0, 1, PUL_DUTY, 0);	//A- power up
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		break;
	case 3:
		//Phase B-
		Adjust_Duty(0, 0, PUL_DUTY, 0);	//A+ fast decay
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, 0);	//B- power up
		break;
	default:
		break;
	}
	DELAY_US(POWER_TM_28V);
	tmLeft = usStep - POWER_TM_28V;

	//Slow decay phase
	switch (Motor.steps) {
	case 0:
		//Phase A+
		Adjust_Duty(0, 0, PUL_DUTY, PUL_INV);
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
		break;
	case 1:
		//Phase B+
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, PUL_INV);
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
		break;
	case 2:
		//Phase A-
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(0, 1, PUL_DUTY, PUL_INV);
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
		break;
	case 3:
		//Phase B-
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, PUL_INV);
		break;
	default:
		break;
	}
	DELAY_US(tmLeft);
#endif

#ifdef SUB_DIVISION
	int16 subDivN=Motor.subDivN;
	switch (Motor.hst.bit.dir) {
	case CLOCKWISE:
		Motor.steps = (Motor.steps + 1) % (4 * subDivN);
		break;
	case COUNTERCW:
		Motor.steps  = (Motor.steps  + 4 * subDivN - 1) % (4 * subDivN);
		break;
	default:
		break;
	}

	if (0 <= Motor.steps && Motor.steps  < subDivN) {
		//Phase A+B+
		Adjust_Duty(0, 0, PUL_DUTY, CalcSPWM(0));
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, CalcSPWM(1 ));
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
	} else if (subDivN <= Motor.steps  && Motor.steps  < 2 * subDivN) {
		//Phase B+A-
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(0, 1, PUL_DUTY, CalcSPWM(0));
		Adjust_Duty(1, 0, PUL_DUTY, CalcSPWM(1));
		Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
	} else if (2 * subDivN <= Motor.steps  && Motor.steps  < 3 * subDivN) {
		//Phase A-B-
		Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(0, 1, PUL_DUTY, CalcSPWM(0));
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, CalcSPWM(1 ));
	} else {
		//Phase B-A+
		Adjust_Duty(0, 0, PUL_DUTY, CalcSPWM(0));
		Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
		Adjust_Duty(1, 1, PUL_DUTY, CalcSPWM(1 ));
	}
	DELAY_US(usStep);
#endif
}

#ifdef SUB_DIVISION
Uint16  CalcSPWM(Uint16 index)
{
	_iq iqResult;
	int16 steps=Motor.steps;
	int16 subDivN=Motor.subDivN;
	_iq stepW=Motor.stepW;

	if (0 <= steps && steps < subDivN) {
		if (index == 0) {
			iqResult = _IQcos(_IQmpy(stepW,_IQ(steps)));
		} else {
			iqResult = _IQsin(_IQmpy(stepW,_IQ(steps)));
		}

	} else if (subDivN <= steps && steps < 2 * subDivN) {
		if (index == 0) {
			iqResult = _IQcos(_IQmpy(stepW,_IQ(steps)));
			iqResult = _IQabs(iqResult);
		} else {
			iqResult = _IQsin(_IQmpy(stepW,_IQ(steps)));
		}

	} else if (2 * subDivN <= steps && steps < 3 * subDivN) {
		if (index == 0) {
			iqResult = _IQcos(_IQmpy(stepW,_IQ(steps)));
			iqResult = _IQabs(iqResult);
		} else {
			iqResult = _IQsin(_IQmpy(stepW,_IQ(steps)));
			iqResult = _IQabs(iqResult);
		}

	} else {
		if (index == 0) {
			iqResult = _IQcos(_IQmpy(stepW,_IQ(steps)));
		} else {
			iqResult = _IQsin(_IQmpy(stepW,_IQ(steps)));
			iqResult = _IQabs(iqResult);
		}
	}

	return PUL_DUTY - _IQint(_IQmpy(iqResult,_IQ(PUL_VAL)));
}
#endif

void Adjust_Duty(Uint16 index,Uint16 channel,Uint16 chopPRD,Uint16 chopCLR)
{
	volatile struct EPWM_REGS* pEPWM = 0;

	if (index == 0)
		pEPWM = &EPwm1Regs;
	else
		pEPWM = &EPwm2Regs;

	pEPWM->TBPRD = chopPRD;
	if(channel==0)
		pEPWM->CMPA.half.CMPA = chopCLR;
	else
		pEPWM->CMPB = chopCLR;;
}

void Init_Epwm(Uint16 index)
{
	volatile struct EPWM_REGS* pEPWM = 0;
	if (index == 0)
		pEPWM = &EPwm1Regs;
	else
		pEPWM = &EPwm2Regs;

	//pEPWM->TBPRD = DEFAULT_FRQ;            // Set timer period
	pEPWM->TBPHS.all = 0;                         // Phase is 0
	pEPWM->TBCTR = 0x0000;                      // Clear counter

	// Setup TBCLK
	pEPWM->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	pEPWM->TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	pEPWM->TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	pEPWM->TBCTL.bit.CLKDIV = TB_DIV1;      // Slow just to observe on the scope
	pEPWM->TBCTL.bit.PRDLD = TB_SHADOW;
	pEPWM->TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	pEPWM->CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
	pEPWM->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	pEPWM->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	pEPWM->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	pEPWM->AQCTLA.bit.CAU = AQ_SET;             // Set PWM2A on CAU
	pEPWM->AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM2A on CAD

	pEPWM->AQCTLB.bit.CBU = AQ_SET;           // Clear PWM2B on CAU
	pEPWM->AQCTLB.bit.CBD = AQ_CLEAR;             // Set PWM2B on CAD

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
}

void Enable_Soc(Uint16 index,Uint16 channel){
	volatile struct EPWM_REGS* pEPWM = 0;

	if (index == 0) {
		pEPWM = &EPwm1Regs;
	} else {
		pEPWM = &EPwm2Regs;
	}

	if (channel == 0) {
		pEPWM->ETSEL.bit.SOCAEN = 1;		// Enable SOC on A group
		pEPWM->ETSEL.bit.SOCASEL = 2;		// Select SOC from TBCNT = TBPRD
		pEPWM->ETPS.bit.SOCAPRD = 3;		// Generate pulse on 3st event
	} else {
		pEPWM->ETSEL.bit.SOCBEN = 1;		// Enable SOC on A group
		pEPWM->ETSEL.bit.SOCBSEL = 2;	// Select SOC from TBCNT = TBPRD
		pEPWM->ETPS.bit.SOCBPRD = 3;		// Generate pulse on 3st event
	}
}

/*
 *
 */
void Init_Eqep(void)
{
    EQep1Regs.QUPRD=600000;         // Unit Timer for 100Hz at 60 MHz SYSCLKOUT

    EQep1Regs.QDECCTL.bit.QSRC=00;      // QEP quadrature count mode

    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
    EQep1Regs.QEPCTL.bit.PCRM=01;       // PCRM=00 mode - QPOSCNT reset on maxium position
    EQep1Regs.QEPCTL.bit.UTE=1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM=1;        // Latch on unit time out
    EQep1Regs.QPOSMAX=0xffffffff;
    EQep1Regs.QEPCTL.bit.QPEN=1;        // QEP enable

    EQep1Regs.QCAPCTL.bit.UPPS=5;       // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS=6;       // 1/64 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN=1;        // QEP Capture Enable
}

/*
 *
 */
void InitECapture(void) {
	ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
	ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped

	// Configure peripheral registers
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;      // One-shot
	ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
	ECap1Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
	ECap1Regs.ECCTL1.bit.CAP2POL = 0;          //
	ECap1Regs.ECCTL1.bit.CAP3POL = 0;          //
	ECap1Regs.ECCTL1.bit.CAP4POL = 0;          //
	ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Absolute time stamp
	ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          //
	ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          //
	ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          //
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;         // Enable sync in
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units

	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
	ECap1Regs.ECCTL2.bit.REARM = 1;            // arm one-shot
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
	ECap1Regs.ECEINT.bit.CEVT4 = 1;            // 4 events = interrupt
}

/*
 *
 */
void InitGpio5(void) {
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;      // Enable pull-up on GPIO5
	GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0;    // Synch to SYSCLKOUT GPIO5
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     // Configure GPIO5 as GPIO5
	GpioCtrlRegs.GPADIR.bit.GPIO5=0;	// Configure GPIO5 as Input
	EDIS;
}

/*
 *
 */
void Config_Adc(){
	// Configure ADC
	// Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata

	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;//ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1E = 1;	//Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT = 0;	//Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL = 1;	//setup EOC1 to trigger ADCINT1 to fire
	AdcRegs.INTSEL1N2.bit.INT2E = 1;	//Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT2CONT = 0;	//Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT2SEL = 2;	//setup EOC2 to trigger ADCINT2 to fire
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 8;//set SOC0 channel select to ADCINB1(dummy sample for rev0 errata workaround)
	AdcRegs.ADCSOC1CTL.bit.CHSEL = 8;	//set SOC1 channel select to ADCINB1
	AdcRegs.ADCSOC2CTL.bit.CHSEL = 9;	//set SOC2 channel select to ADCINB2
	AdcRegs.ADCSOC3CTL.bit.CHSEL = 10;	//set SOC3 channel select to ADCINB3
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 6;	//set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 6;	//set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 8;	//set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
	AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 6;	//set SOC3 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;//set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;//set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;//set SOC3 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	EDIS;
}

/*
 *
 */
__interrupt void  isr_Adc(void){
	if(AdcRegs.ADCINTFLG.bit.ADCINT1)
	{
		Voltage[0]=AdcResult.ADCRESULT0;
		Voltage[0]=AdcResult.ADCRESULT1;
		Voltage[2]=AdcResult.ADCRESULT3;
		AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;//Clear ADCINT1 flag reinitialize for next SOC
	}
	else if(AdcRegs.ADCINTFLG.bit.ADCINT2)
	{
		Voltage[1]=AdcResult.ADCRESULT2;
		AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;//Clear ADCINT1 flag reinitialize for next SOC
	}
	else
	{
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

/*
 *
 */
__interrupt void isr_Ecap1(void)
{
		uSpeed = (ECap1Regs.CAP2 + ECap1Regs.CAP3 + ECap1Regs.CAP4)/ 3;

		ECap1Regs.ECCLR.bit.CEVT4 = 1;
		ECap1Regs.ECCLR.bit.INT = 1;
		ECap1Regs.ECCTL2.bit.REARM = 1;

		// Acknowledge this interrupt to receive more interrupts from group 4
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

/*
 *  LIGHT THE LED
 *  1:	light the upper led
 *  0:	light the lower led
 */
void Config_Led(Uint16 bLightUp){
	EALLOW;
	// Config GPIO26  as general input/output
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0; // GPIO26 = GPIO26
	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0; //Enable Pullup
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;  // GPIO26 = OUTPUT

	if (bLightUp == 0)
		GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;	//CLEAR
	else
		GpioDataRegs.GPASET.bit.GPIO26 = 1;		//SET

	EDIS;
}

/*
 * SHUT DOWN 2104
 * 0:	Disable 2104
 * 1: Enable 2104
 */
void Config_2104(Uint16 bShutdown)
{
	EALLOW;
	// Config GPIO12  as general input/output
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0; // GPIO12 = GPIO12
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0; //Enable Pullup
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;  // GPIO12 = OUTPUT

	if (bShutdown == 0)
		GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;	//CLEAR
	else
		GpioDataRegs.GPASET.bit.GPIO12 = 1;		//SET

	EDIS;
}

/*
 * Stop the running motor using the trapezoid decrease speed method
 */
void StopMotor(void) {
Uint16 i = 0;
float32 StepTm[ACC_STEPS];

ECap1Int(false);
CalcStepTm(Motor.usSetSpeed, StepTm);
for (i = 0; i < ACC_STEPS; i++)
	Motor_Run(StepTm[ACC_STEPS - 1 - i]);
SlowDecay();

Config_Led(1);
Motor.hst.bit.state = MOTOR_STOPPED;
ECap1Int(true);
}

/*
 * Dive motor to run at given speed from standstill
 * Input 'uSpd' The maximum speed motor should running at
 * CAUTION Motor must at 'Stopped' status before calling this function
 */
void StartMotor(Uint32 uSpd) {
Uint16 i = 0;
float32 StepTm[ACC_STEPS];

ECap1Int(false);
Motor.hst.bit.state = MOTOR_RUNNING;
Motor.usSetSpeed = uSpd / 10.0;
CalcStepTm(Motor.usSetSpeed, StepTm);

Config_Led(0);
for (i = 0; i < ACC_STEPS; i++)
	Motor_Run(StepTm[i]);

ECap1Int(true);
}

/*
 * Calculating Motor Step Time using Trapezoid Acceleration or Deceleration Methods.
 * 	Description: Motor run to the maximum speed from stop or verse at given steps, So we need to calculate every step's period.
 * 	Input 'Tmin' The maximum speed's period
 * 	Input 'StepTm' The array used to store the results
 */
void CalcStepTm(float32 Tmin, float32* StepTm) {
Uint16 i = 0;
float32 Ta = 0;
Ta = Tmin / (sqrt(1.0 / ACC_STEPS) * (sqrt(ACC_STEPS) - sqrt(ACC_STEPS - 1)));
for (i = 0; i < ACC_STEPS - 1; i++) {
	StepTm[i] = Ta * sqrt(1.0 / ACC_STEPS) * (sqrt(i + 1) - sqrt(i));
	StepTm[i] = StepTm[i];
}
StepTm[ACC_STEPS - 1] = Tmin;
}

/*
 * Set H Bridge to Slow Decay Mode
 */
void SlowDecay(void) {
Adjust_Duty(0, 0, PUL_DUTY, PUL_DUTY);
Adjust_Duty(0, 1, PUL_DUTY, PUL_DUTY);
Adjust_Duty(1, 0, PUL_DUTY, PUL_DUTY);
Adjust_Duty(1, 1, PUL_DUTY, PUL_DUTY);
}

/*
 * Enable or Disable the interrupt of the ECapture Module
 * Input: bEnable '0' Disable interrupt; '1' Enable interrupt
 */
void ECap1Int(Uint16 bEnable) {
if (bEnable) {
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
	ECap1Regs.ECCTL2.bit.REARM = 1;            // arm one-shot
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;         // Enable CAP1-CAP4 register loads
	ECap1Regs.ECEINT.bit.CEVT4 = 1;            // 4 events = interrupt
} else {
	ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
	ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;        // Disable CAP1-CAP4 register loads
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;      // Make sure the counter is stopped
}
}
