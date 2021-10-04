#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "cli.h"
#include "ws2812.h"

#define		nBits	9
#define		nStop	1
#define		nBaud	51

extern		TIM_HandleTypeDef htim4;

uint16_t	*p=led_drive;
int32_t		ch, txtout,rxtout;
_buffer		*b1,*b2;


int16_t to,bit,dat,cnt;
	
void test(_CLI *cli) {
	b1=_buffer_init(1024);
//	b2=_buffer_init(1024);
	htim4.Instance->ARR=nBaud;
	
	do {
		ch = cli->Escape();
		switch (ch) {
			case __Esc:
				break;
			case EOF:
				if (txtout && HAL_GetTick() > txtout) {
					txtout=0;
					__rearmDMA(p-led_drive);
					p = led_drive;
				}
				break;
			default:
				int dat = (ch | (1 << nBits)) << 1, i = 1;
				while (i < (1 << (nBits + 2))) {
					dat & i ? p[0] = nBaud+1 : p[0] = 0;
					dat & i ? p[1] = 0 : p[1] = nBaud+1;
					++p; ++p;
					i<<=1;
				}
				txtout=HAL_GetTick()+10;
				break;
		}
		
		if(b1) {
			uint16_t ic;
			while(_buffer_pull(b1,&ic,sizeof(uint16_t))) {
				if(rxtout) {
					uint32_t n=((ic-to) % 0x10000 + nBaud)/2/nBaud;
					cnt += n;
					while(n--)
						dat = (dat | bit)>>1;
					bit ^= 1<<nBits;
					if(cnt==nBits+1) {
						_print("%c",dat); 
						rxtout=0;
					} else
						rxtout = HAL_GetTick()+5;
				}	else {
					bit=dat=cnt=0;
					rxtout = HAL_GetTick()+5;
				}
				to=ic;
			}
			if(rxtout && HAL_GetTick() > rxtout) {
				rxtout=0;
				_print("frame \r\n"); 
			}
		}
		_wait(5);
	} while(ch != __Esc);
	_print("exit"); 		
}

extern "C" {
	void tim9Callb1(uint16_t ic) {
		if(b1)
			_buffer_push(b1,&ic,sizeof(uint16_t));
	}
	
	void tim9Callb2(uint16_t ic) {
		if(b2)
			_buffer_push(b2,&ic,sizeof(uint16_t));
	}
}
