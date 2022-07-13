#ifndef VOLTAGEVIEW_HPP
#define VOLTAGEVIEW_HPP

#include <gui_generated/voltage_screen/VoltageViewBase.hpp>
#include <gui/voltage_screen/VoltagePresenter.hpp>

class VoltageView : public VoltageViewBase
{
public:
	uint8_t duty_view = 10;
    VoltageView();
    virtual ~VoltageView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void adcupdateview(uint16_t ADC_data);
    virtual void ButtonUpClicked();
	virtual void ButtonDownClicked();
	virtual void SetCircleProgress();
	virtual void VolToLog_Handle();

protected:

};

#endif // VOLTAGEVIEW_HPP
