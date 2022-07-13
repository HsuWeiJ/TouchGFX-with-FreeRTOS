#ifndef SCROLL_WHEEL_WEEK_HPP
#define SCROLL_WHEEL_WEEK_HPP

#include <gui_generated/containers/Scroll_Wheel_weekBase.hpp>
#include <cstring>

class Scroll_Wheel_week : public Scroll_Wheel_weekBase
{
public:
    Scroll_Wheel_week();
    virtual ~Scroll_Wheel_week() {}

    virtual void initialize();
    char week[10];
	void setNumber(int no)
	{
		switch(no){
			case 0:
				strcpy(week, "Mon");
				break;
			case 1:
				strcpy(week, "Tue");
				break;
			case 2:
				strcpy(week, "Wed");
				break;
			case 3:
				strcpy(week, "Thu");
				break;
			case 4:
				strcpy(week, "Fri");
				break;
			case 5:
				strcpy(week, "Sat");
				break;
			case 6:
				strcpy(week, "Sun");
				break;
			}
		Unicode::strncpy(textArea_weekBuffer, week, strlen(week));
	}
protected:
};

#endif // SCROLL_WHEEL_WEEK_HPP
