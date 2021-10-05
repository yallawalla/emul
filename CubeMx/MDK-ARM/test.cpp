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

__packed struct  {
	uint8_t addr:3, spare1:1, commfail:1, id:1, update:1, spare2:1;  
	uint8_t r100m:4, r1000m:4;
	uint8_t r10m:4, r10000m:1, r1m:1, submode:2;
	uint8_t	opmode:2, echo:1, spare3:5;
	uint8_t	multi:1, minrange:1, laser_fail:1, general_fail:1, spare4:4;
	uint8_t	chk;	
} ackW = {

1,0,0,1,0,0,
0,0,
0,0,NORMAL,0,
ON,FIRST,0,
0,0,0,0,0,
0	
};

extern		TIM_HandleTypeDef htim4;

_buffer		*b1,*b2;
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
static uint16_t *sendBytes(uint8_t *s, uint16_t *d, uint16_t len) {
	while(len--) {
		int dat = (*s++ | (1 << nBits)) << 1, i = 1;
		while (i < (1 << (nBits + 2))) {
			dat & i ? d[0] = nBaud+1 : d[0] = 0;
			dat & i ? d[1] = 0 : d[1] = nBaud+1;
			++d; ++d;
			i<<=1;
		}
	}
	return d;
}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
static void sendCallW(void) {
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
static void sendAckW(void) {
	ackW.chk=0;
	for(int n=0; n<sizeof(ackW)-1; ++n)
		ackW.chk += ((uint8_t *)&ackW)[n];
	__rearmDMA(sendBytes((uint8_t *)&ackW, led_drive, sizeof(ackW))-led_drive);
}
/*******************************************************************************
* Function Name	: 
* Description		: 
* Output				:
* Return				:
*******************************************************************************/
void test(_CLI *cli) {
		int32_t		ch;
		b1=_buffer_init(1024);
//	b2=_buffer_init(1024);
	htim4.Instance->ARR=nBaud;
	
	do {
		ch = cli->Escape();
		switch (ch) {
			case __Esc:
				break;
			case __F1:
				sendCallW();
				break;
			case EOF:
			break;
			default:
			break;
		}
		
		if(b1) {
			uint16_t i;
			while(_buffer_pull(b1,&i,sizeof(uint16_t)))
				_print("%02X ",i);
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
	
static int16_t to,bit,dat,cnt;
static int32_t rxtout;
		
		if(b1) {
			if(rxtout && HAL_GetTick() < rxtout) {
				uint32_t n=((ic-to) % 0x10000 + nBaud)/2/nBaud;
				cnt += n;
				while(n--)
					dat = (dat | bit)>>1;
				bit ^= 1<<nBits;
				if(cnt==nBits+1) {
					_buffer_push(b1,&dat,sizeof(uint16_t));
					rxtout=0;
				} else
					rxtout = HAL_GetTick()+5;
			}	else {
				bit=dat=cnt=0;
				rxtout = HAL_GetTick()+5;
			}
			to=ic;
			}
		}	
	
	void tim9Callb2(uint16_t ic) {
		if(b2)
			_buffer_push(b2,&ic,sizeof(uint16_t));
	}

}
