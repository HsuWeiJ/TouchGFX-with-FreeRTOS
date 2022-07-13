#include <gui/voltage_screen/VoltageView.hpp>
#include <gui/voltage_screen/VoltagePresenter.hpp>

VoltagePresenter::VoltagePresenter(VoltageView& v)
    : view(v)
{

}

void VoltagePresenter::activate()
{

}

void VoltagePresenter::deactivate()
{

}
void VoltagePresenter::adcrefresh(uint16_t ADC_data)
{
	view.adcupdateview(ADC_data);
}
