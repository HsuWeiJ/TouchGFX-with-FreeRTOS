#ifndef SCROLL_WHEEL_YEAR_HPP
#define SCROLL_WHEEL_YEAR_HPP

#include <gui_generated/containers/Scroll_Wheel_yearBase.hpp>

class Scroll_Wheel_year : public Scroll_Wheel_yearBase
{
public:
    Scroll_Wheel_year();
    virtual ~Scroll_Wheel_year() {}

    virtual void initialize();
    void setNumber(int no)
   	{
   		Unicode::snprintf(textArea_yearBuffer, TEXTAREA_YEAR_SIZE, "%02d", no);
   	}
protected:
};

#endif // SCROLL_WHEEL_YEAR_HPP
