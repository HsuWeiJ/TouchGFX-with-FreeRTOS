#ifndef HOME_SETTINGPRESENTER_HPP
#define HOME_SETTINGPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Home_settingView;

class Home_settingPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Home_settingPresenter(Home_settingView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~Home_settingPresenter() {};

private:
    Home_settingPresenter();

    Home_settingView& view;
};

#endif // HOME_SETTINGPRESENTER_HPP
