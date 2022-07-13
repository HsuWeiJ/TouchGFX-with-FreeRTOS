#ifndef TEMPERATUREPRESENTER_HPP
#define TEMPERATUREPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class TemperatureView;

class TemperaturePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    TemperaturePresenter(TemperatureView& v);

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
    virtual void temrefresh(float temperature,float humidity);

    virtual ~TemperaturePresenter() {};

private:
    TemperaturePresenter();

    TemperatureView& view;
};

#endif // TEMPERATUREPRESENTER_HPP
