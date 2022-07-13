#include <gui/log_screen/LogView.hpp>
#include <touchgfx/Color.hpp>
#include "stm32f4xx_hal.h"
#include <cstring>
LogView::LogView()
{

}

void LogView::setupScreen()
{
    LogViewBase::setupScreen();
}

void LogView::tearDownScreen()
{
    LogViewBase::tearDownScreen();
}
void LogView::RFIDupdateview(char confirm,uint8_t *ID)
{
	//textbufrefresh();
    if(confirm)
    {
    	textArea_Access.setColor(touchgfx::Color::getColorFromRGB(93, 194, 16));
    	Unicode::strncpy(textArea_AccessBuffer, "Access", strlen("Access"));
    	//textArea_Access.resizeToCurrentText();
    	textArea_Access.setVisible(true);
    	textArea_Access.invalidate();
    	Unicode::strncpy(textArea_IDBuffer, (char*)ID, strlen((char*)ID));
    	textArea_ID.invalidate();

    }
    else
    {
    	textArea_Access.setColor(touchgfx::Color::getColorFromRGB(194, 16, 51));
		Unicode::strncpy(textArea_AccessBuffer, "Denied", strlen("Denied"));
		//textArea_Access.resizeToCurrentText();
		textArea_Access.setVisible(true);
		textArea_Access.invalidate();
		Unicode::strncpy(textArea_IDBuffer, (char*)ID, strlen((char*)ID));
		textArea_ID.invalidate();
    }
}
void LogView::textbufrefresh()
{
	textArea_Access.setVisible(false);
	//textArea_Access.resizeToCurrentText();
	textArea_Access.invalidate();
}
void LogView::RFID_Resetupdateview( char switch_data)
{
	switch (switch_data)
	{
	case 0:
		textbufrefresh();
		break;
	case 1:
		textbufrefresh();
		application().gotoHomeScreenNoTransition();
		break;
	case 2:
		textbufrefresh();
		application().gotoTemperatureScreenNoTransition();
		break;
	case 4:
		textbufrefresh();
		application().gotoVoltageScreenNoTransition();
		break;
	}
}
