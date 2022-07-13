#include <gui/home_setting_screen/Home_settingView.hpp>
#include "stm32f4xx_hal.h"

extern RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime_;
RTC_DateTypeDef sDate_;

Home_settingView::Home_settingView()
{

}

void Home_settingView::setupScreen()
{
    Home_settingViewBase::setupScreen();
}

void Home_settingView::tearDownScreen()
{
    Home_settingViewBase::tearDownScreen();
	sTime_.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime_.StoreOperation = RTC_STOREOPERATION_RESET;
	HAL_RTC_SetTime(&hrtc, &sTime_, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &sDate_, RTC_FORMAT_BIN);
}
void Home_settingView::scrollWheel_YearUpdateItem(Scroll_Wheel_year& item, int16_t itemIndex)
{

	item.setNumber(itemIndex);
	scrollWheel_Year.invalidate();
	sDate_.Year = itemIndex-1;
}
void Home_settingView::scrollWheel_YearUpdateCenterItem(Scroll_Wheel_year& item, int16_t itemIndex)
{

	item.setNumber(itemIndex);
	scrollWheel_Year.invalidate();
}

void Home_settingView::scrollWheel_HourUpdateItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Hour.invalidate();
	sTime_.Hours = itemIndex-1;
}
void Home_settingView::scrollWheel_HourUpdateCenterItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Hour.invalidate();

}
void Home_settingView::scrollWheel_MinuteUpdateItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Minute.invalidate();
	sTime_.Minutes = itemIndex-1;
}
void Home_settingView::scrollWheel_MinuteUpdateCenterItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Minute.invalidate();

}
void Home_settingView::scrollWheel_SecUpdateItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Sec.invalidate();
	sTime_.Seconds = itemIndex-1;
}
void Home_settingView::scrollWheel_SecUpdateCenterItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Sec.invalidate();

}
void Home_settingView::scrollWheel_DateUpdateItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Minute.invalidate();
	sDate_.Date = itemIndex-1;
}
void Home_settingView::scrollWheel_DateUpdateCenterItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Minute.invalidate();

}
void Home_settingView::scrollWheel_MonthUpdateItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Minute.invalidate();
	sDate_.Month = itemIndex-1;
}
void Home_settingView::scrollWheel_MonthUpdateCenterItem(Scroll_Wheel_year& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Minute.invalidate();

}
void Home_settingView::scrollWheel_WeekUpdateItem(Scroll_Wheel_week & item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Week.invalidate();
	sDate_.WeekDay = itemIndex-1;
}
void Home_settingView::scrollWheel_WeekUpdateCenterItem(Scroll_Wheel_week& item, int16_t itemIndex)
{
	item.setNumber(itemIndex);
	scrollWheel_Week.invalidate();

}


