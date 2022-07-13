/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef FRONTENDAPPLICATIONBASE_HPP
#define FRONTENDAPPLICATIONBASE_HPP

#include <mvp/MVPApplication.hpp>
#include <gui/model/Model.hpp>

class FrontendHeap;

class FrontendApplicationBase : public touchgfx::MVPApplication
{
public:
    FrontendApplicationBase(Model& m, FrontendHeap& heap);
    virtual ~FrontendApplicationBase() { }

    virtual void changeToStartScreen()
    {
        gotoLogScreenNoTransition();
    }

    // Home
    void gotoHomeScreenNoTransition();

    // Home_setting
    void gotoHome_settingScreenNoTransition();

    // Log
    void gotoLogScreenNoTransition();

    // Temperature
    void gotoTemperatureScreenNoTransition();

    // Voltage
    void gotoVoltageScreenNoTransition();

protected:
    touchgfx::Callback<FrontendApplicationBase> transitionCallback;
    FrontendHeap& frontendHeap;
    Model& model;

    // Home
    void gotoHomeScreenNoTransitionImpl();

    // Home_setting
    void gotoHome_settingScreenNoTransitionImpl();

    // Log
    void gotoLogScreenNoTransitionImpl();

    // Temperature
    void gotoTemperatureScreenNoTransitionImpl();

    // Voltage
    void gotoVoltageScreenNoTransitionImpl();
};

#endif // FRONTENDAPPLICATIONBASE_HPP
