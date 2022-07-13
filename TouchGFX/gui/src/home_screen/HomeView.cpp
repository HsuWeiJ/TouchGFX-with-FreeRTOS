#include <gui/home_screen/HomeView.hpp>
#include "stm32f4xx_hal.h"
/********RTOS************/
#include "FreeRTOS.h"
#include "event_groups.h"
#include <cstring>

extern EventGroupHandle_t Log_Out_Handler;
//extern float temperature;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
HomeView::HomeView()
{

}

void HomeView::setupScreen()
{
    HomeViewBase::setupScreen();
    timeDisplay();
}

void HomeView::tearDownScreen()
{
    HomeViewBase::tearDownScreen();
}
void HomeView::timeDisplay()
{
	Unicode::snprintf(textArea_HourBuffer, TEXTAREA_HOUR_SIZE, "%02d", sTime.Hours);
	Unicode::snprintf(textArea_MinuteBuffer, TEXTAREA_MINUTE_SIZE, "%02d", sTime.Minutes);
	Unicode::snprintf(textArea_SecBuffer, TEXTAREA_SEC_SIZE, "%02d", sTime.Seconds);
	Unicode::snprintf(textArea_YearBuffer, TEXTAREA_YEAR_SIZE, "%02d", sDate.Year);
	Unicode::snprintf(textArea_MonthBuffer, TEXTAREA_MONTH_SIZE, "%02d", sDate.Month);
	Unicode::snprintf(textArea_DateBuffer, TEXTAREA_DATE_SIZE, "%02d", sDate.Date);


	switch(sDate.WeekDay){
	case 0x01:
		strcpy(week, "Mon");
		break;
	case 0x02:
		strcpy(week, "Tue");
		break;
	case 0x03:
		strcpy(week, "Wed");
		break;
	case 0x04:
		strcpy(week, "Thu");
		break;
	case 0x05:
		strcpy(week, "Fri");
		break;
	case 0x06:
		strcpy(week, "Sat");
		break;
	case 0x07:
		strcpy(week, "Sun");
		break;
	}
	Unicode::strncpy(textArea_WeekBuffer, week, strlen(week));

	textArea_Hour.invalidate();
	textArea_Minute.invalidate();
	textArea_Sec.invalidate();
	textArea_Month.invalidate();
	textArea_Week.invalidate();
	textArea_Date.invalidate();
	textArea_Year.invalidate();
}
void HomeView::HomeToLog_Handle()
{
	xEventGroupSetBits(Log_Out_Handler,0x01); // 觸發HomeToLog事件
}
void HomeView::Cpu_Usage_updateview(uint16_t Cpu_Usage)
{
	Unicode::snprintf(textArea_CpuBuffer, TEXTAREA_CPU_SIZE, "%2d",Cpu_Usage);
	textArea_Cpu.invalidate();
}
