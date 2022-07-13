#include <gui/temperature_screen/TemperatureView.hpp>
#include <gui/temperature_screen/TemperaturePresenter.hpp>

TemperaturePresenter::TemperaturePresenter(TemperatureView& v)
    : view(v)
{

}

void TemperaturePresenter::activate()
{

}

void TemperaturePresenter::deactivate()
{

}
void TemperaturePresenter::temrefresh(float temperature,float humidity)
{
	view.temupdateview( temperature, humidity);
}
