#include <gui/voltage_screen/VoltageView.hpp>
#include "stm32f4xx_hal.h"
#include <touchgfx/Color.hpp>
#include <touchgfx/Unicode.hpp>
/********RTOS************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"



#include <cstring>
//extern uint16_t ADC_VAL;
extern xQueueHandle DutyQueue; ;
extern EventGroupHandle_t Log_Out_Handler;
float ADC_Buf;
extern uint8_t duty;
VoltageView::VoltageView()
{

}

void VoltageView::setupScreen()
{
    VoltageViewBase::setupScreen();
    Unicode::snprintf(textArea_DutyBuffer, TEXTAREA_DUTY_SIZE, "%d", duty);
	textArea_Duty.invalidate();
	circleProgress_Duty.setValue(duty);
	circleProgress_Duty.invalidate();
	duty_view=duty;
}

void VoltageView::tearDownScreen()
{
    VoltageViewBase::tearDownScreen();
}

void VoltageView::adcupdateview(uint16_t ADC_data)
{
	ADC_Buf=(float)ADC_data/1000;
	Unicode::snprintfFloat(textArea_VolBuffer, TEXTAREA_VOL_SIZE, "%1.1f", ADC_Buf );
	textArea_Vol.invalidate();
}
void VoltageView::ButtonUpClicked()
{
	duty_view+=10;
	if(duty_view > 60) duty_view=10;
	Unicode::snprintf(textArea_DutyBuffer, TEXTAREA_DUTY_SIZE, "%d", duty_view);
	textArea_Duty.invalidate();
	SetCircleProgress();
	presenter->saveDuty(duty_view);
	xQueueOverwrite(DutyQueue,&duty_view);
}
void VoltageView::ButtonDownClicked()
{
	duty_view-=10;
	if(duty_view < 10) duty_view=60;
	Unicode::snprintf(textArea_DutyBuffer, TEXTAREA_DUTY_SIZE, "%d", duty_view);
	textArea_Duty.invalidate();
	SetCircleProgress();
	presenter->saveDuty(duty_view);
	xQueueOverwrite(DutyQueue,&duty_view);
}
void VoltageView::SetCircleProgress()
{
	circleProgress_Duty.setValue(duty_view);
	circleProgress_Duty.invalidate();
}
void VoltageView::VolToLog_Handle()
{
	xEventGroupSetBits(Log_Out_Handler,0x04); // 觸發VolToLog事件
}
