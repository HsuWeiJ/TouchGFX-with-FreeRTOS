#ifndef VOLTAGEPRESENTER_HPP
#define VOLTAGEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class VoltageView;

class VoltagePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    VoltagePresenter(VoltageView& v);

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
    virtual void adcrefresh(uint16_t ADC_data);
    void saveDuty(int Duty)
	{
		model->saveDuty(Duty);
	}

    virtual ~VoltagePresenter() {};

private:
    VoltagePresenter();

    VoltageView& view;
};

#endif // VOLTAGEPRESENTER_HPP
