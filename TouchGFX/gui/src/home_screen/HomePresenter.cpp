#include <gui/home_screen/HomeView.hpp>
#include <gui/home_screen/HomePresenter.hpp>

HomePresenter::HomePresenter(HomeView& v)
    : view(v)
{

}

void HomePresenter::activate()
{

}

void HomePresenter::deactivate()
{

}

void HomePresenter::timeRefresh()
{
	view.timeDisplay();
}
void HomePresenter::Cpu_Usage_refresh(uint16_t Cpu_Usage)
{
	view.Cpu_Usage_updateview(Cpu_Usage);
}
