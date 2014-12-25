

#include "DSP281x_Device.h"	// DSP281x Headerfile Include File
#include "DSP281x_Examples.h"	// DSP281x Examples Include File

#include "Example_DPS2812M_AD.H"
//#include "Example_DPS2812M_DA.H"

#include "IQmathLib.h"
#include "math.h"
#include "Algorithm_ByLee.h"




#define pi 3.1415926  //Բ����

//����������
#define Ku 151.11		//LV25_P��ѹ�������Ŵ���������������Ϊ110k������������270��
//#define Kcur 6.67		//��LA55-P/SP50��������������������Ϊ150��
//#define Kcur 0.16667		//��LA55-P/SP50��������������������Ϊ1200����������Ϊ5
#define Kcur 13.33333	//��LA100-P/SP50��������������������Ϊ150��
#define Res 0.000305	//ADоƬ�ķֱ���
#define U_max 20        //�̱�������ѹ


//����Ƶ�ʺ��ӳٵ���
#define fk 5.0               //����Ƶ��,��λΪkHz
//#define V_Dly 90.0              //�ӳ�90��
#define V_T1PR 37.5*1000.0/2.0/fk   //������/��ģʽ37.5*1000/2.0/fk��������ģʽ37.5*1000/fk-1
//#define Dot fk*20.0          //һ�����ڵĵ���,0.02/Ts
//#define Dly V_Dly*fk*20.0/360.0    //��Ҫ�ӳٵĵ���
#define V_ACTRA 0x0666    //����Ч0x0999.����Ч��Ϊ0x0666
#define V_T1CON 0x0942    //��ʱ��T1ʱ�����壺37.5MHz,������/��ģʽ0942;������ģʽ1142
#define V_DBTCONA 0x0AF8  //������������0x0AECʱ������ʱ��Ϊ4.27us
#define Tk 1.0/fk/1000

//����
 #define f 50  //��������Ƶ��
 #define T 1.0/50//����
 #define phi 0  //����������λ��
 #define w 2*pi*f 

// #define PI_Kp 4
// #define PI_Ki 150
// #define PI_OutMax 300
// #define PI_OutMin -300

 #define Ud_Ref 2.8867
 #define Uq_Ref 0

// #define Udc_ref 20
// #define Iq_ref 0

//#define Id_Ref 0.5
//#define Iq_Ref 0

//Uint16 EVAInterruptCount;
Uint16 x=0;//temp[128],
unsigned int ADflag=1;
unsigned int AD_corrention_flag=1;//ADУ����־λ
//unsigned int channal=5;


/****������****/
//float Temp;
//float a[200],b[200];//,cc[200];
//float temp1[128];//,temp2[128];
/****End****/

unsigned int cnt=0;//
//unsigned int xint=0,tint=0;
float sample_time=Tk;

float U1,U2;
float I1,I2,I3;
float U1_offset=0,U2_offset=0;
float I1_offset=0,I2_offset=0;//,Udc_offset=0;
float U1_offset_temp=0,U2_offset_temp=0;
float I1_offset_temp=0,I2_offset_temp=0;//,Udc_offset_temp=0;
float Ua_pwm,Ub_pwm,Uc_pwm;

interrupt void ADC_T1TOADC_isr(void);
interrupt void ADC_SampleINT(void);	

void inverter_pll_calc(void);
void pi_calc(PI_Ctrl *p,float Ref,float Feedback);

// ADC start parameters
//#define ADC_MODCLK 0x3	// HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3) = 25MHz
// Global variable for this example
//DAC_DRV DAC=DAC_DRV_DEFAULTS;
ADC_DRV AD=ADC_DRV_DEFAULTS;

inverter_pll ip;
PLL pll_I,pll_U,pll2;
line2phase l2p_U;
CLARKE c_U,c_I,c2;
PARK p_I,p_U;
ANTICLARKE ac;
ANTIPARK ap;

PI_Ctrl PI_Ud={
				4.0,			// Parameter: Proportional gain  
				150.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				300,		// Parameter: Maximum output 
				-300,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};
				
PI_Ctrl PI_Uq={
				4.0,			// Parameter: Proportional gain  
				150.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				300,		// Parameter: Maximum output 
				-300,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};

PI_Ctrl PI_Id={
				30.0,			// Parameter: Proportional gain  
				0.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				500.0,		// Parameter: Maximum output 
				-500.0,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};
PI_Ctrl PI_Iq={
				30.0,			// Parameter: Proportional gain  
				0.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				500.0,		// Parameter: Maximum output 
				-500.0,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};

/*
PI_Ctrl PI_Ia={
				20.0,			// Parameter: Proportional gain  
				0.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				300.0,		// Parameter: Maximum output 
				-300.0,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};
PI_Ctrl PI_Ib={
				20.0,			// Parameter: Proportional gain  
				0.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				300.0,		// Parameter: Maximum output 
				-300.0,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};
PI_Ctrl PI_Ic={
				20.0,			// Parameter: Proportional gain  
				0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		    // Variable: Pre-saturated output 
				300,		// Parameter: Maximum output 
				-300,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};  */




void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP281x_SysCtrl.c file.
	InitSysCtrl();
	//MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	
// Step 2. Initialize GPIO: 
// This example function is found in the DSP281x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();	

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
	DINT;
	IER=0x0000;
	IFR=0x0000;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP281x_PieCtrl.c file.
	InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
	//IER = 0x0000;
	//IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP281x_DefaultIsr.c.
// This function is found in DSP281x_PieVect.c.
	InitPieVectTable();
	Init_Gpio();
	//MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	
// ��ʼ��EVA��ʱ��1	
	ADREG=0;
	EvaRegs.GPTCONA.all=0;
	
//����ͨ��Ŀ�Ķ�ʱ��1������;
	EvaRegs.T1PR=V_T1PR;     //(0x0823,9kHz)(0x1D4B,5kHz,)(0x0200, ����512)
	//EvaRegs.T1CMPR=0x0000;		
	
//ʹ��ͨ��Ŀ�Ķ�ʱ��1�������ж�
//���ϼ�����Ԥ����x128���ڲ�ʱ�ӡ�ʹ�ܱȽϡ�ʹ���Լ�������ֵ
	EvaRegs.EVAIMRA.bit.T1PINT=1;	
	EvaRegs.EVAIFRA.bit.T1PINT=1; 
	
//���ͨ��Ŀ�Ķ�ʱ��1�ļ�����ֵ
	EvaRegs.T1CNT=0x0000;
	EvaRegs.T1CON.all=V_T1CON;  //0x0942,
	EvaRegs.ACTRA.all = V_ACTRA;    //����Ч0x0999.����Ч��Ϊ0x0666
	EvaRegs.DBTCONA.all = V_DBTCONA;   //��������
	EvaRegs.COMCONA.all = 0xA600;     //0xA600

	
//��ͨ��Ŀ�Ķ�ʱ��1�����ж�ʱ����ADC�任
	EvaRegs.GPTCONA.bit.T1TOADC=2;//0x10
		
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.T1PINT = &ADC_T1TOADC_isr;
	PieVectTable.XINT1  = &ADC_SampleINT;  //�ⲿ�ж�
	EDIS;	// This is needed to disable write to EALLOW protected registers
	
	XIntruptRegs.XINT1CR.all=0x0001;
	XIntruptRegs.XINT2CR.all=0x0001;
	PieCtrlRegs.PIEIER1.bit.INTx3=1;
	PieCtrlRegs.PIEIER1.bit.INTx4=1;
	PieCtrlRegs.PIEIER2.all=M_INT4;
// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP281x_InitPeripherals.c
// InitPeripherals(); 
//	DAC.DACChannelSel=0x00;
//	DAC.DACDataCycle=128;
//	DAC.DACDataOffset=0;
//	DAC.DataSel=0;
// Step 5. User specific code, enable interrupts:
	// Step 5. User specific code, enable interrupts:
	
	IER |=(M_INT2|0x0001);
	
// Enable global Interrupts and higher priority real-time debug events:
	EINT;	// Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

	for(;;)
	{
		if(AD.ADCFlag.bit.ADCSampleFlag==1) {
			//if(AD.LoopVar>=128) 
				//AD.LoopVar=0;
			//AD.ADChannelSel=0;
			if(ADflag==0) {

				ADCSmplePro(&AD);
				//AD.LoopVar++;
				
				
				/**������ɺ�ʼ���ݴ���**/
				
				
				clarke_calc(&c_U,l2p_U.a,l2p_U.b);//��ѹ3/2�任
				
				pll_calc(&pll_U,c_U.Alpha,c_U.Beta);//�õ�ѹֵ�����ж���
					
				park_calc(&p_U,c_U.Alpha,c_U.Beta,pll_U.sin,pll_U.cos);//��ѹ2s/2r�任
				
				pi_calc(&PI_Ud,Ud_Ref,p_U.Ds);
				pi_calc(&PI_Uq,Uq_Ref,p_U.Qs);//��ѹdq��PI
				
/************************************����dq����***************************************************************/
				clarke_calc(&c_I,I1,I2);
				pll_calc(&pll_I,c_I.Alpha,c_I.Beta);//�õ���ֵ�����ж���

				park_calc(&p_I,c_I.Alpha,c_I.Beta,pll_I.sin,pll_I.cos);

				//pi_calc(&PI_Id,Id_Ref,p_I.Ds);
				//pi_calc(&PI_Iq,Iq_Ref,p_I.Qs);//��������
			
				pi_calc(&PI_Id,PI_Ud.Out,p_I.Ds);
				pi_calc(&PI_Iq,PI_Uq.Out,p_I.Qs);//˫�ջ�

				antipark_calc(&ap,PI_Id.Out,PI_Iq.Out,pll2.sin,pll2.cos);
				anticlarke_calc(&ac,ap.Alpha,ap.Beta);

				/******��һ��********/
				Ua_pwm=0.5+ac.As/2000;
				Ub_pwm=0.5+ac.Bs/2000;
				Uc_pwm=0.5+ac.Cs/2000;
				/*******************/

/***********************************************************************************************************/	

/************************************����abc����***************************************************************/
//				antipark_calc(&ap,PI_Ud.Out,PI_Uq.Out,pll2.sin,pll2.cos);
//				anticlarke_calc(&ac,ap.Alpha,ap.Beta);
//				
//				pi_calc(&PI_Ia,I1,ac.As);
//				pi_calc(&PI_Ib,I2,ac.Bs);
//				pi_calc(&PI_Ic,I3,ac.Cs);
//
//				/******��һ��*********/
//				Ua_pwm=0.5+PI_Ia.Out/1000;
//				Ub_pwm=0.5+PI_Ib.Out/1000;
//				Uc_pwm=0.5+PI_Ic.Out/1000;
//				/********************/
/***********************************************************************************************************/

				/**�趨ռ�ձȣ��Ƚ��жϣ�**/
				EvaRegs.CMPR1=Ua_pwm*EvaRegs.T1PR;
				EvaRegs.CMPR2=Ub_pwm*EvaRegs.T1PR;
				EvaRegs.CMPR3=Uc_pwm*EvaRegs.T1PR;



			}
			AD.ADCFlag.bit.ADCSampleFlag=0;
		}
	}
}

interrupt void ADC_T1TOADC_isr(void)  
{
	//tint++;
	inverter_pll_calc();
	if(ADflag==0) {
		*AD_CONVST=0;
		EvaRegs.EVAIMRA.bit.T1PINT=1;
		EvaRegs.EVAIFRA .all =BIT7;
	}//����δ�������ʱ�����жϣ��������б�־λ��λ�Ա�֮������ж�
    if(ADflag==1) {
//		DAC.DACPro(&DAC);
		ADflag=0;
		
		*AD_CONVST=0;
		EvaRegs.EVAIMRA.bit.T1PINT=1;  //
		EvaRegs.EVAIFRA .all =BIT7;    //
		//EvaRegs.EVAIFRA.bit.T1PINT=1;
		//PieCtrlRegs.PIEACK .all =PIEACK_GROUP2;
	} 
	
	PieCtrlRegs.PIEACK .all =PIEACK_GROUP2; 
}

interrupt void ADC_SampleINT(void)
{

	XIntruptRegs.XINT1CR .all =0x0000;
	if(AD.ADCFlag.bit.ADCSampleFlag==0) {
		AD.ADCFlag.bit.ADCSampleFlag=1;
	}

	XIntruptRegs.XINT1CR .all =0x0001;
	PieCtrlRegs.PIEACK .all =PIEACK_GROUP1;	
}

void ADCSmplePro(ADC_DRV *v)
{
	if(v->ADChannelSel==6) 
		v->ADChannelSel=0;
	if(v->ADCFlag.bit.ADCCS0==1) {
		v->ADSampleResult0[x]=*AD_CHIPSEL0;
		v->ADSampleResult1[x]=*AD_CHIPSEL0;
		v->ADSampleResult2[x]=*AD_CHIPSEL0;
		v->ADSampleResult3[x]=*AD_CHIPSEL0;
		v->ADSampleResult4[x]=*AD_CHIPSEL0;
		v->ADSampleResult5[x]=*AD_CHIPSEL0;
	}
	if(v->ADCFlag.bit.ADCCS1==1) {
		v->ADSampleResult0[x]=*AD_CHIPSEL1;
		v->ADSampleResult1[x]=*AD_CHIPSEL1;
		v->ADSampleResult2[x]=*AD_CHIPSEL1;
		v->ADSampleResult3[x]=*AD_CHIPSEL1;
		v->ADSampleResult4[x]=*AD_CHIPSEL1;
		v->ADSampleResult5[x]=*AD_CHIPSEL1;
	}	


	U1=(signed int)v->ADSampleResult0[x]*Ku*Res-U1_offset;	
	U2=(signed int)v->ADSampleResult1[x]*Ku*Res-U2_offset;
	
	I1=(signed int)v->ADSampleResult2[x]*Kcur*Res-I1_offset;	
	I2=(signed int)v->ADSampleResult3[x]*Kcur*Res-I2_offset;

	//Udc=(signed int)v->ADSampleResult4[x]*Ku*Res-Udc_offset;

	if(AD_corrention_flag==1) {
		
		U1_offset=U1+U1_offset;//���㵱ǰʵ��ƫ��ֵ
		U1_offset_temp+=U1_offset;//�ۼ�ƫ��ֵ
		U1_offset=U1_offset_temp/(x+1);//�����´ι���ƫ��ֵ

		//if(U1_offset>20) while(1);

		U2_offset=U2+U2_offset;//���㵱ǰʵ��ƫ��ֵ
		U2_offset_temp+=U2_offset;//�ۼ�ƫ��ֵ
		U2_offset=U2_offset_temp/(x+1);//�����´ι���ƫ��ֵ

		//if(U2_offset>20) while(1);
		
		I1_offset=I1+I1_offset;//���㵱ǰʵ��ƫ��ֵ
		I1_offset_temp+=I1_offset;//�ۼ�ƫ��ֵ
		I1_offset=I1_offset_temp/(x+1);//�����´ι���ƫ��ֵ
		
		I2_offset=I2+I2_offset;//���㵱ǰʵ��ƫ��ֵ
		I2_offset_temp+=I2_offset;//�ۼ�ƫ��ֵ
		I2_offset=I2_offset_temp/(x+1);//�����´ι���ƫ��ֵ

//		Udc_offset=Udc+Udc_offset;//���㵱ǰʵ��ƫ��ֵ
//		Udc_offset_temp+=Udc_offset;//�ۼ�ƫ��ֵ
//		Udc_offset=Udc_offset_temp/(x+1);//�����´ι���ƫ��ֵ

		if(x==50) //����50�κ�ֹͣУ��
			AD_corrention_flag=0;
	}

/******************Fo*******************/
//	if(U1>U_max) {
//		GpioDataRegs.GPFCLEAR.bit.GPIOF4=1;
//		GpioDataRegs.GPBSET.bit.GPIOB8=1;
//		for(;;);
//	} 
//
//	if(U2>U_max) {
//		GpioDataRegs.GPFCLEAR.bit.GPIOF4=1;
//		GpioDataRegs.GPBSET.bit.GPIOB8=1;
//		for(;;);
//	}


/***int line_to_phase(line2phase *l,float Uab,float Ubc)***********/
	line_to_phase(&l2p_U,U1,U2);//
	I3=-I1-I2;

/******Graphic*******/
	//Temp=U1;
	//temp1[x]=U1;
	//temp2[x]=l2p.Ubc;
/******Graphic End*******/

	ADflag=1;
	x++;
	if(x>=128)
		x=0;
	
}

void inverter_pll_calc(void)
{
	//_iq sin_out;
	_iq angle_IQ,delta_phi_IQ;
	_iq sina_IQ,sinb_IQ,sinc_IQ;
	
	
	if(cnt > (fk*1000/f)) //
		cnt = 0;
	else 
		cnt++;
	
	//ip.sina=(float)sin(w*Tk*cnt+phi);
	//ip.sinb=(float)sin(w*Tk*cnt+phi-0.66*pi);
	//ip.sinc=(float)sin(w*Tk*cnt+phi+0.66*pi);

	angle_IQ=_IQ(cnt * Tk * w);
	delta_phi_IQ=_IQ(0.66666 * pi);//����120��
	
	sina_IQ=_IQsin(angle_IQ);
	sinb_IQ=_IQsin(angle_IQ - delta_phi_IQ);
	sinc_IQ=_IQsin(angle_IQ + delta_phi_IQ);
	
	ip.sina=_IQtoF( sina_IQ );
	ip.sinb=_IQtoF( sinb_IQ );
	ip.sinc=_IQtoF( sinc_IQ );

	/* Graphic */
	//a[cnt]=ip.sinb;
	//b[cnt]=_IQ24toF(sin_out);
	/* Graphic end */

	if(ip.sina > 0.9999)
		ip.sina=0.9999;
	if(ip.sinb > 0.9999)
		ip.sinb=0.9999;
	if(ip.sinc > 0.9999)
		ip.sinc=0.9999;
//int clarke_calc(CLARKE *c,float As,float Bs,float Cs)
	clarke_calc(&c2,ip.sina,ip.sinb);
//int pll_calc(pll *p,float Alpha,float Beta)
	pll_calc(&pll2,c2.Alpha,c2.Beta);


	/* Graphic */
	//a[cnt]=pll.sin;
	//cc[cnt]=pll.cos;
	/* Graphic end */

}

void pi_calc(PI_Ctrl *p,float Ref,float Feedback) 
{	 
 // Compute the error
    p->Errp = Ref - Feedback;
    
    // Compute the proportional output
    p->Up = p->Kp * p->Errp;

    // Compute the integral output
    p->Ui = (p->OutPreSat == p->Out)?(p->Ui + p->Ki * p->Errp * sample_time):p->Ui;

    // Compute the pre-saturated output
    p->OutPreSat = p->Up + p->Ui ;     
    

    if (p->OutPreSat > p->OutMax)                   
	{
	 p->Out =  p->OutMax;
	}
    else if (p->OutPreSat < p->OutMin)
	{ 
	 p->Out =  p->OutMin; 
	}
	else
	 p->Out = p->OutPreSat;   	  
}



//===========================================================================
// No more.
//===========================================================================

