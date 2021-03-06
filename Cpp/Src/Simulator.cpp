/**
  ******************************************************************************
  * @file    lcd.cpp
  * @author  Fotona d.d.
  * @version V1
  * @date    30-Sept-2013
  * @brief	 DA & DMA converters initialization
  *
  */ 
/** @addtogroup
* @{
*/
#include	"simulator.h"
#include	"ioc.h"

	#define Uc1		pComp
	#define Uc2		pBott
	#define Uc3		pNozz
	
	#define Rin		30
	#define Rout	50
	
	#define R2		100
	#define Rw		300
	#define Ra		300
	#define Rsp		100
	#define C1		0.01f
	#define C3		0.0001f
	#define C2		0.05f
	#define dt		0.001f
	
/*******************************************************************************
* Function Name	:
* Description		: 
* Output				:
* Return				: 
********************************************************************************

                         ///------R2----Uc2-----///-----Rout----Pext
                          |              |
                          |             ///
                          |              |
         Pin-----Rin-----Uc1              \____Rw_____
                          |                            \
                          |                            Uc3-----Rsp----Pext
                         XXX___________________Ra______/

*******************************************************************************/
_SIMULATOR::_SIMULATOR() {
	lcd=new _LCD;
	
	plot.Clear();
	plot.Add(&_ADC::fval.compressor,_BAR(1.0f),_BAR(0.02f), LCD_COLOR_YELLOW);
	plot.Add(&_ADC::fval.bottle,_BAR(1.0f),_BAR(0.02f), LCD_COLOR_GREY);
	plot.Add(&_ADC::fval.air,_BAR(1.0f),_BAR(0.02f), LCD_COLOR_MAGENTA);
	
	rate=0;
	tau1=tau2=0;
	srand(__time__);
	
	Pin=3.8f;	
	pComp= pBott= pNozz= Pext= 1.0f;
	offset.air += rand() % 1000 - 500;
	offset.bottle += rand() % 1000 - 500;
	offset.compressor += rand() % 1000 - 500;
}
/*******************************************************************************
* Function Name	:
* Description		: 
* Output				:
* Return				: 
********************************************************************************/
_SIMULATOR::~_SIMULATOR() {
	if(lcd) {
		LCD_Clear(LCD_COLOR_BLACK);
		delete lcd;
		lcd=NULL;
	}
	plot.Clear();
}
/*******************************************************************************
* Function Name	:
* Description		: 
* Output				:
* Return				: 
********************************************************************************/
void	_SIMULATOR::Poll(void *v) {
_IOC		*ioc = static_cast<_IOC *>(v);

	float	Iin=(Pin-Uc1)/Rin;
	float	I12=(Uc1-Uc2)/R2;
	float	I13=(Uc1-Uc3)/Ra;
	float	I23=(Uc2-Uc3)/Rw;
	float	I3=(Uc3-Pext)/Rsp;
	float	Iout=(Uc2 - Pext)/Rout;

	I13 = I13*valve_drive[2]/__PWMRATE;

	if(ioc->spray.BottleIn->Closed()) {
		I12=0;
		if(I23 < 0)
			plot.Colour(&_ADC::fval.bottle,LCD_COLOR_GREEN);
		else
			plot.Colour(&_ADC::fval.bottle,LCD_COLOR_GREY);
			
	} else
		plot.Colour(&_ADC::fval.bottle,LCD_COLOR_RED);

	if(ioc->spray.BottleOut->Closed())
		Iout=0;
	else
		plot.Colour(&_ADC::fval.bottle,LCD_COLOR_BLUE);

	if(ioc->spray.Water->Closed())
		I23=0;

	Uc1 += (Iin-I12-I13)/C1*dt;
	Uc2 += (I12-I23-Iout)/C2*dt;
	Uc3 += (I23+I13-I3)/C3*dt;	

	fval.compressor	=_BAR(pComp);
	fval.bottle			=_BAR(pBott + 0.05f * I12*R2 + 0.03f);
	fval.air				=_BAR(pNozz + I13*Ra - 0.01f);

	fval.V5		= _V5to16X;
	fval.V12	= _V12to16X;
	fval.V24	= _V24to16X;

//	fval.T2=(unsigned short)0xafff;
	
	if(__time__ > rate) {
		rate = __time__ + 10;
		if(lcd && plot.Refresh())
			lcd->Grid();
	}
	
	fval.Ipump=pump_drive<<5;
	flowTacho=(22000/30/100)*pump_drive/(1<<12)/2;
	fanTacho=(22000/30/100)*fan_drive/(1<<12)/2;
}
/**
* @}
*/ 
