#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "cli.h"
#include "ws2812.h"

#define		nBits	9
#define		nStop	1
#define		nBaud	51

typedef enum { ON, NA, BIT, FIRE } opmode;
typedef enum { NORMAL, VERSION, COMM, COUNT } submode;
typedef enum { SECOND, FIRST } echo;

__packed struct {
		uint8_t addr:3, spare1:5;
		uint8_t opmode:2, echo:1, spare2:3, submode:2;
		uint8_t factmode:4, minrange:4;
		uint8_t	chk;
} callW = {1,0,ON,1,0,NORMAL,8,0,0};


__packed struct  a482 {
	uint8_t addr:3, spare1:1, commfail:1, id:1, update:1, spare2:1;  
	uint8_t r100m:4, r1000m:4;
	uint8_t r10m:4, r10000m:1, r1m:1, submode:2;
	uint8_t	opmode:2, echo:1, spare3:5;
	uint8_t	multi:1, minrange:1, laser_fail:1, general_fail:1, spare4:4;
	uint8_t	chk;	
} ;

__packed struct a483 {
	uint8_t addr:3, spare1:1, commfail:1, id:1, update:1, spare2:1;  
	uint8_t SwRev:5,SwUpdt:2,spare3:1;
	uint8_t r10m:4, r10000m:1, r1m:1, submode:2;
	uint8_t	opmode:2, echo:1, spare4:5;
	uint8_t	multi:1, minrange:1, laser_fail:1, general_fail:1, spare5:4;
	uint8_t	chk;	
} ;
	
__packed struct a485 {
	uint8_t addr:3, count:1, CountLow:4;
	uint8_t CountMed;
	uint8_t CountHi;
	uint8_t	opmode:2, serialLo:6;
	uint8_t	serialHi;
	uint8_t	chk;	
} ;

__packed struct  a486 {
	uint8_t addr:3, spare1:1, commfail:1, id:1, update:1, spare2:1;  
	uint8_t SwChk:1,RAM:1,EPROM:1,FIFO:1,Sim:1,To:1,Qsw:1,HV:1;
	uint8_t BITinproc:1,spare3:6;
	uint8_t	opmode:2, echo:1, spare4:5;
	uint8_t	multi:1, minrange:1, laser_fail:1, general_fail:1, spare5:4;
	uint8_t	chk;	
} ;

__packed union  ackW {
		__packed struct a482;
		__packed struct a483;
		__packed struct a485;
		__packed struct a486;
	} ackW ;

	
extern		TIM_HandleTypeDef htim4;
extern "C" {
void tim9Callb1(uint16_t) ;
uint32_t timeout;
_buffer		*b1,*b2;
}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
static uint16_t	*sendBytes(uint8_t *s, uint16_t *d, uint16_t len) {
	uint16_t dat= 0x200;
	while(len--) {
		dat |= (*s++ | (1 << nBits)) << 1;
		for (uint32_t i=1; i < (1 << (nBits + 2)); i<<=1) {
			dat & i ? d[0] = nBaud+1 : d[0] = 0;
			dat & i ? d[1] = 0 : d[1] = nBaud+1;
			++d; ++d;
		}
		dat=0;
	}
	return d;
}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
static void			sendCallW(void) {
	callW.chk=0;
	for(int n=0; n<sizeof(callW)-1; ++n)
		callW.chk += ((uint8_t *)&callW)[n];
	__rearmDMA(sendBytes((uint8_t *)&callW, led_drive, sizeof(callW))-led_drive);
}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
void 							test(_CLI *cli) {
		int32_t		ch;
		_print("ESLRF test\r\n");
		b1=_buffer_init(1024);
//	b2=_buffer_init(1024);
	htim4.Instance->ARR=nBaud;
	
	do {
		ch = cli->Escape();
		switch (ch) {
			case __Esc:
				break;
			case __F1:
				callW.opmode=ON;
				callW.submode=NORMAL;
				sendCallW();
				break;
			case __F2:
				callW.opmode=ON;
				callW.submode=VERSION;
				sendCallW();
				break;
			case __F3:
				callW.opmode=ON;
				callW.submode=COMM;
				sendCallW();
				break;
			case __F4:
				callW.opmode=ON;
				callW.submode=COUNT;
				sendCallW();
				break;
			case __F5:
				callW.opmode=BIT;
				callW.submode=NORMAL;
				sendCallW();
				break;
			case __F12:
				callW.opmode=FIRE;
				callW.submode=NORMAL;
				sendCallW();
				break;
			case EOF:
			break;
			default:
				__rearmDMA(sendBytes((uint8_t *)&ch, led_drive, 1)-led_drive);

			break;
		}
		
		if(b1) {
			uint16_t i;
			while(_buffer_pull(b1,&i,sizeof(uint16_t))) 
				if(i & 0x100)
					_print("\r\n%02X ",i & 0xff);
				else
					_print("%02X ",i & 0xff);
		}
		
		if(timeout && HAL_GetTick() > timeout) {
			tim9Callb1(0);
			timeout=0;
		} 
		_wait(2);
		
	} while(ch != __Esc);
	_print("exit"); 		
}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
extern "C" {
	
void tim9Callb1(uint16_t ic) {

	static uint16_t to, bit, dat, cnt;
	
	if(!b1)
		return;
	if (to) {
		uint32_t n = ((ic - to) % 0x10000 + nBaud/2/2) / nBaud/2;
		while (n-- && cnt <= nBits + nStop) {
			dat = (dat | bit) >> 1;
			++cnt;
		}
		timeout=HAL_GetTick()+5;
//		_buffer_push(b1,&dat,sizeof(uint16_t));
		bit ^= 1 << (nBits + nStop);
		if (cnt > nBits + nStop) {
			_buffer_push(b1,&dat,sizeof(uint16_t));
//			printf("%04X ", dat);
			dat = cnt = 0;
		}
	}
	else 
		bit = dat = cnt = 0;
	to = ic;
	}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
	void tim9Callb2(uint16_t ic) {
		if(b2)
			_buffer_push(b2,&ic,sizeof(uint16_t));
	}
}
