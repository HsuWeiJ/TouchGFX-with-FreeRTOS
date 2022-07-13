#ifndef LOGPRESENTER_HPP
#define LOGPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class LogView;

class LogPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    LogPresenter(LogView& v);

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
    virtual void Rfid_reset_refresh(char switch_data);
    virtual void RFID_refresh(char confirm,uint8_t *ID);

    virtual ~LogPresenter() {};

private:
    LogPresenter();

    LogView& view;
};

#endif // LOGPRESENTER_HPP
