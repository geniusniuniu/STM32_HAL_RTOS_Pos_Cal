#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//??????OS,??????????????????.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS???	 
#include "task.h"
#endif


static u32 fac_us=0;							//us?????????

#if SYSTEM_SUPPORT_OS		
    static u16 fac_ms=0;				        //ms?????????,??os??,????????????ms??
#endif

 
extern void xPortSysTickHandler(void);
//void SysTick_Handler(void)
//{  
//    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//?????????
//    {
//        xPortSysTickHandler();	
//    }
//    HAL_IncTick();
//}
			   

void delay_init(u8 SYSCLK)
{
	u32 reload;
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTick????HCLK
	fac_us=SYSCLK;						    //??????????OS,fac_us????????
	reload=SYSCLK;					        //????????????? ??��?K	   
	reload*=1000000/configTICK_RATE_HZ;		//????configTICK_RATE_HZ?څ??????
											//reload?24��?????,????:16777216,??180M??,???0.745s????	
	fac_ms=1000/configTICK_RATE_HZ;			//????OS??????????????��		
    SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//????SYSTICK?��?
	SysTick->LOAD=reload; 					//?1/configTICK_RATE_HZ?????	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //????SYSTICK
}								    

    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD???	    	 
	ticks=nus*fac_us; 						//?????????? 
	told=SysTick->VAL;        				//??????????????
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//??????????SYSTICK??????????????????????.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//?????/????????????,?????.
		}  
	};									    
}  
	

void delay_ms(u32 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//?????????
	{		
		if(nms>=fac_ms)						//???????????OS????????????? 
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS???
		}
		nms%=fac_ms;						//OS???????????��???????,?????????????    
	}
	delay_us((u32)(nms*1000));				//?????????
}


void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}


			 



































