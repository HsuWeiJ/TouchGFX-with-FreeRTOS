#include <gui/log_screen/LogView.hpp>
#include <gui/log_screen/LogPresenter.hpp>

LogPresenter::LogPresenter(LogView& v)
    : view(v)
{

}

void LogPresenter::activate()
{

}

void LogPresenter::deactivate()
{

}
void LogPresenter::RFID_refresh(char confirm,uint8_t *ID)
{
	view.RFIDupdateview( confirm,ID);
}
void LogPresenter::Rfid_reset_refresh(char switch_data)
{
	view.RFID_Resetupdateview( switch_data);
}
