#include <gui/temperature_screen/TemperatureView.hpp>
#include "stm32f4xx_hal.h"
#include <touchgfx/Color.hpp>

#include <cstring>
/********RTOS************/
#include "FreeRTOS.h"
#include "event_groups.h"
#include <cstring>

extern EventGroupHandle_t Log_Out_Handler;
//extern float temperature,humidity;
TemperatureView::TemperatureView()
{

}

void TemperatureView::setupScreen()
{
    TemperatureViewBase::setupScreen();

}

void TemperatureView::tearDownScreen()
{
    TemperatureViewBase::tearDownScreen();
}
void TemperatureView::temupdateview(float temperature,float humidity)
{
	Unicode::snprintf(textArea_TemBuffer, TEXTAREA_TEM_SIZE, "%d", (int) temperature);
	Unicode::snprintf(textArea_HumBuffer, TEXTAREA_HUM_SIZE, "%d", (int) humidity);
	textArea_Tem.invalidate();
	textArea_Hum.invalidate();
	boxProgress_Tem.setValue((int)temperature);
	boxProgress_Hum.setValue((int)humidity);
	setcolor( temperature, humidity);
}
void TemperatureView::setcolor(float temperature,float humidity)
{
	//Temperature box progress set color
	if((int)temperature < 25) boxProgress_Tem.setColor(touchgfx::Color::getColorFromRGB(0, 0, 255));
	else if((int)temperature >= 30) boxProgress_Tem.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
	else boxProgress_Tem.setColor(touchgfx::Color::getColorFromRGB(0, 255, 0));
	boxProgress_Tem.invalidate();
	//Humidity box progress set color
	if((int)humidity < 33) boxProgress_Hum.setColor(touchgfx::Color::getColorFromRGB(0, 0, 255));
	else if((int)humidity >= 66) boxProgress_Hum.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
	else boxProgress_Hum.setColor(touchgfx::Color::getColorFromRGB(0, 255, 0));
	boxProgress_Hum.invalidate();
}
void TemperatureView::TemToLog_Handle()
{
	xEventGroupSetBits(Log_Out_Handler,0x02); // 觸發TemToLog事件
}
