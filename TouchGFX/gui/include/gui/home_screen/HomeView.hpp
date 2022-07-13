#ifndef HOMEVIEW_HPP
#define HOMEVIEW_HPP

#include <gui_generated/home_screen/HomeViewBase.hpp>
#include <gui/home_screen/HomePresenter.hpp>

class HomeView : public HomeViewBase
{
public:
    HomeView();
    virtual ~HomeView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void timeDisplay();
    virtual void HomeToLog_Handle();
    virtual void Cpu_Usage_updateview(uint16_t Cpu_Usage);
protected:
    char week[10];
};

#endif // HOMEVIEW_HPP
