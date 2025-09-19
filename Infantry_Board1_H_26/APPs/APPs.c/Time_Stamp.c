/**
 * @file Time_Stamp.c
 * @author Why
 * @brief 
 * @version 0.1
 * @date 2023-08-23
 * 
 * @copyright 
 * 
 */
 
#include "Time_Stamp.h"

static uint32_t GetTimeStamp(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date);

/**
  * @brief  通过RTC计算当前的时间戳
  * @param  
  * @retval 时间戳的计算值
  */
uint32_t Append_Time_Stamp()
{
	static RTC_TimeTypeDef Time;		
	static RTC_DateTypeDef Date;
	static uint32_t TimeStamp;
	
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BCD);
	TimeStamp = GetTimeStamp(&Time, &Date);
	
	return TimeStamp;
}
	

/**
  * @brief  通过RTC计算当前的时间戳
  * @param  RTC结构体
  * @retval ms的计数值
  */
uint32_t GetTimeStamp(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date)
{
	uint32_t TimeCount;
	
	TimeCount = (uint32_t)Time->Seconds + (uint32_t)Time->Minutes * 60\
				+ (uint32_t)Time->Hours * 3600 + (uint32_t)Date->Year * 31536000\
				+ ((uint32_t)Date->Date - 1) * 86400;
	switch(Date->Month)
	{
		case 0x01: break;
		case 0x02: TimeCount += 2678400; break;
		case 0x03: TimeCount += 5097600; break;
		case 0x04: TimeCount += 7776000; break;
		case 0x05: TimeCount += 1036800; break;
		case 0x06: TimeCount += 13046400; break;
		case 0x07: TimeCount += 15638400; break;
		case 0x08: TimeCount += 18316800; break;
		case 0x09: TimeCount += 20995200; break;
		case 0x0A: TimeCount += 23587200; break;
		case 0x0B: TimeCount += 26265600; break;
		case 0x0C: TimeCount += 28857600; break;
		default: break;
	}
	
	return TimeCount;
}
