#ifndef LOGVIEW_HPP
#define LOGVIEW_HPP

#include <gui_generated/log_screen/LogViewBase.hpp>
#include <gui/log_screen/LogPresenter.hpp>

class LogView : public LogViewBase
{
public:
    LogView();
    virtual ~LogView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void RFIDupdateview(char confirm,uint8_t *ID);
    virtual void RFID_Resetupdateview( char switch_data);
    virtual void textbufrefresh();
protected:
};

#endif // LOGVIEW_HPP
