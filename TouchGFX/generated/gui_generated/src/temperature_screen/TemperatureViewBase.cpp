/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/temperature_screen/TemperatureViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

TemperatureViewBase::TemperatureViewBase() :
    flexButtonCallback(this, &TemperatureViewBase::flexButtonCallbackHandler)
{

    __background.setPosition(0, 0, 320, 240);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));

    image1.setXY(0, 0);
    image1.setBitmap(touchgfx::Bitmap(BITMAP_BACKGROUND_ID));

    slideMenuLeft.setup(touchgfx::SlideMenu::EAST,
        touchgfx::Bitmap(BITMAP_LEFT_SLIDE_MENU_BACKGROUND_ID),
        touchgfx::Bitmap(BITMAP_LEFT_SLIDE_MENU_BUTTON_ID),
        touchgfx::Bitmap(BITMAP_LEFT_SLIDE_MENU_BUTTON_ID),
        0, 0, 50, 90);
    slideMenuLeft.setState(touchgfx::SlideMenu::COLLAPSED);
    slideMenuLeft.setVisiblePixelsWhenCollapsed(25);
    slideMenuLeft.setHiddenPixelsWhenExpanded(0);
    slideMenuLeft.setAnimationEasingEquation(touchgfx::EasingEquations::cubicEaseInOut);
    slideMenuLeft.setAnimationDuration(18);
    slideMenuLeft.setExpandedStateTimeout(180);
    slideMenuLeft.setXY(0, 0);

    Button_Tem.setIconBitmaps(Bitmap(BITMAP_HUMIDITY_ID), Bitmap(BITMAP_HUMIDITY_ID));
    Button_Tem.setIconXY(0, 0);
    Button_Tem.setPosition(6, 74, 39, 46);
    slideMenuLeft.add(Button_Tem);

    Button_Logout.setIconBitmaps(Bitmap(BITMAP_LOG_OUT_ID), Bitmap(BITMAP_LOG_OUT_ID));
    Button_Logout.setIconXY(0, 0);
    Button_Logout.setPosition(7, 183, 38, 41);
    Button_Logout.setAction(flexButtonCallback);
    slideMenuLeft.add(Button_Logout);

    Button_Home.setIconBitmaps(Bitmap(BITMAP_HOME_ID), Bitmap(BITMAP_HOME_ID));
    Button_Home.setIconXY(0, 0);
    Button_Home.setPosition(7, 17, 39, 45);
    Button_Home.setAction(flexButtonCallback);
    slideMenuLeft.add(Button_Home);

    Button_Vol.setIconBitmaps(Bitmap(BITMAP_THUNDER_ID), Bitmap(BITMAP_THUNDER_ID));
    Button_Vol.setIconXY(0, 0);
    Button_Vol.setPosition(5, 132, 41, 38);
    Button_Vol.setAction(flexButtonCallback);
    slideMenuLeft.add(Button_Vol);

    textArea_Tem.setXY(87, 39);
    textArea_Tem.setColor(touchgfx::Color::getColorFromRGB(240, 233, 233));
    textArea_Tem.setLinespacing(0);
    Unicode::snprintf(textArea_TemBuffer, TEXTAREA_TEM_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_0VUW).getText());
    textArea_Tem.setWildcard(textArea_TemBuffer);
    textArea_Tem.resizeToCurrentText();
    textArea_Tem.setTypedText(touchgfx::TypedText(T___SINGLEUSE_9A5M));

    textArea_Hum.setXY(87, 132);
    textArea_Hum.setColor(touchgfx::Color::getColorFromRGB(240, 233, 233));
    textArea_Hum.setLinespacing(0);
    Unicode::snprintf(textArea_HumBuffer, TEXTAREA_HUM_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_MKJG).getText());
    textArea_Hum.setWildcard(textArea_HumBuffer);
    textArea_Hum.resizeToCurrentText();
    textArea_Hum.setTypedText(touchgfx::TypedText(T___SINGLEUSE_LSMA));

    boxProgress_Tem.setXY(86, 87);
    boxProgress_Tem.setProgressIndicatorPosition(2, 2, 180, 16);
    boxProgress_Tem.setRange(20, 35);
    boxProgress_Tem.setDirection(touchgfx::AbstractDirectionProgress::RIGHT);
    boxProgress_Tem.setBackground(touchgfx::Bitmap(BITMAP_BLUE_PROGRESSINDICATORS_BG_MEDIUM_PROGRESS_INDICATOR_BG_ROUND_0_DEGREES_ID));
    boxProgress_Tem.setColor(touchgfx::Color::getColorFromRGB(0, 255, 47));
    boxProgress_Tem.setValue(25);

    boxProgress_Hum.setXY(86, 193);
    boxProgress_Hum.setProgressIndicatorPosition(2, 2, 180, 16);
    boxProgress_Hum.setRange(0, 100);
    boxProgress_Hum.setDirection(touchgfx::AbstractDirectionProgress::RIGHT);
    boxProgress_Hum.setBackground(touchgfx::Bitmap(BITMAP_BLUE_PROGRESSINDICATORS_BG_MEDIUM_PROGRESS_INDICATOR_BG_ROUND_0_DEGREES_ID));
    boxProgress_Hum.setColor(touchgfx::Color::getColorFromRGB(0, 255, 47));
    boxProgress_Hum.setValue(50);

    add(__background);
    add(image1);
    add(slideMenuLeft);
    add(textArea_Tem);
    add(textArea_Hum);
    add(boxProgress_Tem);
    add(boxProgress_Hum);
}

void TemperatureViewBase::setupScreen()
{

}

void TemperatureViewBase::flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src)
{
    if (&src == &Button_Logout)
    {
        //ChangeToLog
        //When Button_Logout clicked change screen to Log
        //Go to Log with no screen transition
        application().gotoLogScreenNoTransition();

        //ChangeToLog_Handle
        //When Button_Logout clicked call virtual function
        //Call TemToLog_Handle
        TemToLog_Handle();
    }
    else if (&src == &Button_Home)
    {
        //ChangeToHome
        //When Button_Home clicked change screen to Home
        //Go to Home with no screen transition
        application().gotoHomeScreenNoTransition();
    }
    else if (&src == &Button_Vol)
    {
        //ChangeToVol
        //When Button_Vol clicked change screen to Voltage
        //Go to Voltage with no screen transition
        application().gotoVoltageScreenNoTransition();
    }
}
