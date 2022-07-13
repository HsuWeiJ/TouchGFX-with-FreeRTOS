#ifndef HOME_SETTINGVIEW_HPP
#define HOME_SETTINGVIEW_HPP

#include <gui_generated/home_setting_screen/Home_settingViewBase.hpp>
#include <gui/home_setting_screen/Home_settingPresenter.hpp>

class Home_settingView : public Home_settingViewBase
{
public:
    Home_settingView();
    virtual ~Home_settingView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void scrollWheel_YearUpdateItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_YearUpdateCenterItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_HourUpdateItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_HourUpdateCenterItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_MinuteUpdateItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_MinuteUpdateCenterItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_SecUpdateItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_SecUpdateCenterItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_DateUpdateItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_DateUpdateCenterItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_MonthUpdateItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_MonthUpdateCenterItem(Scroll_Wheel_year & item, int16_t itemIndex);
	virtual void scrollWheel_WeekUpdateItem(Scroll_Wheel_week & item, int16_t itemIndex);
	virtual void scrollWheel_WeekUpdateCenterItem(Scroll_Wheel_week& item, int16_t itemIndex);



protected:
};

#endif // HOME_SETTINGVIEW_HPP
