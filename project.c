//*****************************Introduction***************************

//Code made for AML mini project by Rochan Nehete and Parth Mehta//
//This code
//Connection Details: 
//Motor: 	                L-1---->P1.21;		L-2---->P0.22;
//   						R-1---->P0.10;		R-2---->P0.11;
//   						P0.7 (PWM2) ----> Logic 1; 	P0.21 (PWM5) ----> Logic 1;
//
//LCD Connections:
// 			  LCD	  Microcontroller Pins
// 			  RS  --> P1.19
//			  RW  --> P1.18
//			  EN  --> P1.17
//			  DB7 --> P1.25
//			  DB6 --> P1.24
//			  DB5 --> P1.23
//			  DB4 --> P1.22
//ADC Connection:
//              Sharp IR range sensor 2 --> AD0.6(P0.4)
//			  	Sharp IR range sensor 3 --> AD1.0(P0.6)
//			  	Sharp IR range sensor 4 --> AD0.7(P0.5)
//

//Frequency=12MHz 
//---------------------------------------------------------------------**//
#include <lpc214x.h>
#include "LCD.h"	
#include <math.h>
//-------------------------------Macros----------------------------------//
#define Fosc            12000000                    //10MHz~25MHz
#define Fcclk           (Fosc * 5)                  //Fosc(1~32)<=60MHZ
#define Fcco            (Fcclk * 4)                 //CCO Fcclk 2¡4¡8¡16±156MHz~320MHz
#define Fpclk           (Fcclk / 4) * 1             //VPB(Fcclk / 4) 1¡2¡4
#define  UART_BPS	9600 		//Change Baud Rate Setting here
//------------------------------------------------------------------------//

//-----------------------Initializations Predefining--------------------//
void DelaymSec(unsigned int j)

void Init_Peripherals(void);
void Init_Ports(void);
void Init_ADC_Pin(void);
void Init_ADC0(void);
void Init_ADC1(void);
unsigned int AD0_Conversion(unsigned char channel);
unsigned int AD1_Conversion(unsigned char channel);
unsigned int Sharp_GP2D12_Estimation(unsigned int Val);

void Forward(void);
void Back(void);
void Left(void);
void Right(void);
void Stop(void);
void Soft_Left(void);
void Soft_Right(void);
void Soft_Left2(void);
void Soft_Right2(void);
void L_Forward(void);
void L_Back(void);
void R_Forward(void);
void R_Back(void);
void L_Stop(void);
void R_Stop(void);
void Init_PWM(void);
void UpdateLeftPWM(unsigned int vel);
void UpdateRightPWM(unsigned int vel);
void Init_Motion_Pin(void);

void  __irq IRQ_UART0(void);
void Init_UART0(void);
void UART0_SendByte(unsigned char data);
void UART0_SendStr(const unsigned char *str);

//-----------------------------------------------------------//


//------------------------Global variables-------------------//
extern unsigned char String1[16];	//This variable is defined in LCD.c
extern unsigned char String2[16];	//This variable is defined in LCD.c
unsigned int ADC_Data[8];
unsigned char temp1;
//-----------------------------------------------------------//



//-----------------------Initializations--------------------//

void DelaymSec(unsigned int j)		 //Generates app. 1mSec delay
{  
 unsigned int  i;
 for(;j>0;j--)
 {
  for(i=0; i<10000; i++);
 } 
}


void Init_ADC_Pin(void)
{
 PINSEL0&= 0xF0FFC0FF;
 PINSEL0|= 0x0F003F00;		//Set pins P0.4, P0.5, P0.6, P0.12, P0.13 as ADC pins
 PINSEL1&= 0xF0FFFFFF;		
 PINSEL1|= 0x05000000;	    //Set pins P0.28, P0.29 as ADC pins
}


void Init_ADC0(void)
{
 AD0CR=0x00200E00;	//first seven bits are for selecting channel bits 15-8 are for clock division. we set bits 8-11 as 1110. CLK=PCLK/(CLKDIV+1)
 //10 BITS CLOCK bits 17-19 are set to 000. bit from 21 to reserve is 1 to power up the ADC
                    // SEL = 1 	ADC0 channel 1	Channel 1
					// CLKDIV = Fpclk / 1000000 - 1 ;1MHz
					// BURST = 0 
					// CLKS = 0 
 					// PDN = 1 
 					// START = 1
  					// EDGE = 0 (CAP/MAT)
} 

void Init_ADC1(void)
{
 AD1CR=0x00200E00;	//first seven bits are for selecting channel bits 15-8 are for clock division, set as 0. we set bits 8-11 as 1110. CLK=PCLK/(CLKDIV+1)
 //10 BITS CLOCK bits 17-19 are set to 000. bit from 21 to reserve is 1 to power up the ADC
                    // SEL = 1 	ADC0 channel 1	Channel 1
					// CLKDIV = Fpclk / 1000000 - 1 ;1MHz
					// BURST = 0 
					// CLKS = 0 
 					// PDN = 1 
 					// START = 1
  					// EDGE = 0 (CAP/MAT)
} 
                           

//This function converts ADC0 channels. Channel number is passed to this function as integer.
unsigned int AD0_Conversion(unsigned char channel)
{
 unsigned int Temp;
 if(channel!=0)
 {
  AD0CR = (AD0CR & 0xFFFFFF00) | (1<<channel);  //select channel
 }
 else
 {
  AD0CR = (AD0CR & 0xFFFFFF00) | 0x01;
 }
 AD0CR|=(1 << 24);      //start conversion
 while((AD0GDR&0x80000000)==0);     //check if last bit EOC is completed or not
 Temp = AD0GDR;						
 Temp = (Temp>>8) & 0xFF;           //bits 6-15 are data bits. next bits are from which channel it is obtained
 return Temp;
}

//This function converts ADC1 channels. Channel number is passed to this function as integer.
unsigned int AD1_Conversion(unsigned char channel)
{
 unsigned int Temp;
 if(channel!=0)
 {
  AD1CR = (AD1CR & 0xFFFFFF00) | (1<<channel);      //select channel
 }
 else
 {
  AD1CR = (AD1CR & 0xFFFFFF00) | 0x01;
 }
 AD1CR|=(1 << 24);      //start conversion
 while((AD1GDR&0x80000000)==0);         //check if last bit EOC is completed or not
 Temp = AD1GDR;						
 Temp = (Temp>>8) & 0xFF;           //bits 6-15 are data bits. next bits are from which channel it is obtained
 return Temp;
}

//This Function estimates the raw digital data of Sharp sensor in mm
unsigned int Sharp_GP2D12_Estimation(unsigned int Val)
{
 float Distance;
 unsigned int DistanceInt;
 Distance = (int)(10.00*(2799.6*(1.00/(pow(Val,1.1546)))));
 DistanceInt = (int)Distance;
 if(DistanceInt>800)
 {
  DistanceInt=800;
 }
 return DistanceInt;
}
//---------------------------adc end---------------------------//
//----------------------motion control------------------------//

void Init_Motion_Pin(void)
{
 PINSEL0&=0xFF0F3FFF;		
 PINSEL0|=0x00000000;		//Set Port pins P0.7, P0.10, P0.11 as GPIO
 PINSEL1&=0xFFFFF0FF;
 PINSEL1|=0x00000000;		//Set Port pins P0.21 and 0.22 as GPIO
 IO0DIR&=0xFF9FF37F;
 IO0DIR|= (1<<10) | (1<<11) | (1<<21) | (1<<22) | (1<<7); 	//Set Port pins P0.10, P0.11, P0.21, P0.22, P0.7 as Output pins
 IO1DIR&=0xFFDFFFFF;
 IO1DIR|= (1<<21);		// Set P1.21 as output pin
 Stop();				// Stop both the motors on start up
 //IO0SET = 0x00200080;	// Set PWM pins P0.7/PWM2 and P0.21/PWM5 to logic 1. This pins are set as PWM pins in Init_PWM fuction
}

//Function to move Left motor forward
void L_Forward(void)
{
 IO1SET = 0x00200000;		//Set P1.21 to logic '1'
}

//Function to move Left motor backward
void L_Back(void)
{
 IO0SET = 0x00400000;		//Set P0.22 to logic '1'
}

//Function to move Right motor forward
void R_Forward(void)
{
 IO0SET = 0x00000400;		//Set P0.10 to logic '1'
}

//Function to move Right motor backward
void R_Back(void)
{
 IO0SET = 0x00000800;		//Set P0.11 to logic '1'
}

//Function to stop left motor
void L_Stop(void)
{
 IO1CLR = 0x00200000;		//Set P1.21 to logic '0'
 IO0CLR = 0x00400000;		//Set P0.22 to logic '0'
}

//Function to stop Right motor
void R_Stop(void)
{
 IO0CLR = 0x00000400;		//Set P0.10 to logic '0'
 IO0CLR = 0x00000800;		//Set P0.11 to logic '0'
}

//Function to move robot in forward direction
void Forward(void)
{
 Stop();
 L_Forward();
 R_Forward();
}

//Function to move robot in backward direction
void Back(void)
{
 Stop();
 L_Back();
 R_Back();
}

//Function to turn robot in Left direction
void Left(void)
{ 
 Stop();
 L_Back();
 R_Forward();
}

//Function to turn robot in right direction
void Right(void)
{ 
 Stop();
 L_Forward();
 R_Back();
}

//Function to turn robot in Left direction by moving right wheel forward
void Soft_Left(void)
{
 
 R_Forward();
}

//Function to turn robot in right direction by moving left wheel forward
void Soft_Right(void)
{
 
 L_Forward();
}

//Function to turn robot in left direction by moving left wheel backward
void Soft_Left2(void)
{
 Stop();
 L_Back();
}

//Function to turn robot in right direction by moving right wheel backward 
void Soft_Right2(void)
{
 Stop();
 R_Back();
}

//Function to stop the robot at its current location
void Stop(void)
{
 L_Stop();
 R_Stop();
}




void Init_PWM(void)
{
 PINSEL0&=0xFFFF3FFF;
 PINSEL0|=0x00008000;	//Enabling P0.7 as PWM2
 PINSEL1&=0xFFFFF3FF;		
 PINSEL1|=0x00000400;	//Enabling P0.22 as PWM5

 PWMPR	= 30;	//PWM Prescaler PCLK/30 = 500KHz
 PWMPC	= 0;	//PWMPC increments on every PCLK
 PWMTC	= 0;	//PWMTC increments on every PWMPC=PWMPR
 PWMMR0 = 500;	//PWM base frequency 500KHz/500=1KHz	 
 PWMMR1 = 0;
 PWMMR2 = 0;
 PWMMR3 = 0;
 PWMMR4 = 0;
 PWMMR5 = 0;
 PWMMR6 = 0;
 PWMMCR = 0x00000002;
 PWMPCR	= 0x2600;
 PWMLER	= 0x7F;
 PWMTCR = 0x01;
}  



void UpdateLeftPWM(unsigned int vel)            //vel is between 0-500//
{
 PWMMR2 = vel;
 PWMLER = 0x04;
}

void UpdateRightPWM(unsigned int vel)           //vel is between 0-500//
{
 PWMMR5 = vel;
 PWMLER = 0x20;
}

//----------------motion end-------------------///




//-----------------serial----------------------//

//This function is UART0 Receive ISR. This functions is called whenever UART0 receives any data
void  __irq IRQ_UART0(void)
{  
 Temp = U0RBR;			
  
 if(Temp == 0x38) //ASCII value of 8
 {
  Forward();  //forward
 }
 
 if(Temp == 0x32) //ASCII value of 2
 {
  Back(); //back
 }

 if(Temp == 0x34) //ASCII value of 4
 {
  Left();  //left
 }
  
 if(Temp == 0x36) //ASCII value of 6
 {
  Right(); //right
 }

 if(Temp == 0x35) //ASCII value of 5
 {
  Stop(); //stop
 }

 if(Temp == 0x37) //ASCII value of 7
 {
  Soft_Left();
 }

 if(Temp == 0x39) //ASCII value of 9
 {
  Soft_Right();
 } 

 VICVectAddr = 0x00;
 UART0_SendByte(Temp);	//Echo Back received character
}		


/************************************************************

	Function 		: Init_UART0
	Return type		: None
	Parameters		: None
	Description 	: Initialises UART0 module. 
************************************************************/
void Init_UART0(void)
{  
   unsigned int Baud16;
   PINSEL0&=0xFFFFFFF0;
   PINSEL0|=0x00000005;

   U0LCR = 0x83;		            // DLAB = 1
   Baud16 = (Fpclk / 16) / UART_BPS;  
   U0DLM = Baud16 / 256;							
   U0DLL = Baud16 % 256;						
   U0LCR = 0x03;
   U0IER = 0x00000001;		//Enable Rx interrupts

   VICIntSelect = 0x00000000;		// IRQ
   VICVectCntl0 = 0x20|6;			// UART0
   VICVectAddr0 = (int)IRQ_UART0; 	//UART0 Vector Address
   VICIntEnable = (1<<6);	// Enable UART0 Rx interrupt

}
				

//This function sends a single character on the serial port
void UART0_SendByte(unsigned char data)
{  
   U0THR = data;				    
   while( (U0LSR&0x40)==0 );	    
}

//This function sends a string of characters on the serial port
void UART0_SendStr(const unsigned char *str)
{  
   while(1)
   {  
      if( *str == '\0' ) break;
      UART0_SendByte(*str++);	    
   }
}

//---------------------------------------------//
void Init_Ports(void)
{
 Init_Motion_Pin();
 Init_LCD_Pin();
 Init_ADC_Pin();
}

void Init_Peripherals(void)
{
 Init_Ports();
 Init_ADC0();
 Init_ADC1();
 Init_PWM();
}
//------------------------------------------------------//

int main(void)
{ 
 unsigned int Temp=0;
 unsigned int left_distance=0;
 unsigned int centre_distance=0;
 unsigned int right_distance=0;
 float ratio;
 PINSEL0 = 0x00000000;		// Enable GPIO on all pins
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 DelaymSec(40);
 Init_Peripherals();
 LCD_4Bit_Mode();
 LCD_Init();
 LCD_Command(0x01);
 DelaymSec(20);
   
 while(1)	
 { 
  ADC_Data[0] = AD0_Conversion(6);	  //Left IR
  left_distance=Sharp_GP2D12_Estimation(ADC_Data[0]);
  LCD_Print(1,1,left_distance,3);

  ADC_Data[1] = AD1_Conversion(0);	  //Front IR
  centre_distance = Sharp_GP2D12_Estimation(ADC_Data[1]);
  LCD_Print(1,5,centre_distance,3);

  ADC_Data[2] = AD0_Conversion(7);	  //Right IR
  right_distance=Sharp_GP2D12_Estimation(ADC_Data[2]);
  LCD_Print(1,9,right_distance,3);

  ratio=(left_distance/right_distance);

  UpdateLeftPWM(300);
  UpdateRightPWM(300);
  if(centre_distance>40){

  if(ratio<1){
      UpdateLeftPWM(500);
      Soft_Right();
  }
  if(ratio>1){
      UpdateRightPWM(500);
      Soft_Left();
  }

  if (ratio==1){
      Forward();
      UpdateLeftPWM(300);
      UpdateRightPWM(300);
  }

  else{
      Forward();
        UpdateLeftPWM(300);
        UpdateRightPWM(300);
    }
  
  }
  else {
      Stop();
      Right();
      DelaymSec(1500);
  }
  DelaymSec(100);
 }
 
}