#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
extern int SHT_Flag;
extern int ADC_Flag;
extern int RTC_Flag;
extern xQueueHandle RFID_ResetQueue;
extern xQueueHandle RFID_TO_GUIQueue;
extern xQueueHandle CPU_Usage_Queue;
extern xQueueHandle TEM_TO_GUI_Queue;
extern xQueueHandle ADC_TO_GUI_Queue;
extern RFID_Data RFID_Data_;
//extern int RFID_Flag;
//extern float temperature;
//int duty =10;

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	if(RTC_Flag == 1){
		modelListener->timeRefresh();
		RTC_Flag = 0;
	}
//	if(SHT_Flag == 1){
//		modelListener->temrefresh();
//		SHT_Flag = 0;
//	}
//	if(ADC_Flag == 1){
//		modelListener->adcrefresh();
//		ADC_Flag = 0;
//	}
	if(xQueueReceive(TEM_TO_GUI_Queue, &modelListener->SHT_data, 0)==pdTRUE)
		{
			float temperature = modelListener->SHT_data.temperature;
			float humidity = modelListener->SHT_data.humidity;
			modelListener->temrefresh(temperature,humidity);
		}
	if(xQueueReceive(ADC_TO_GUI_Queue, &modelListener->ADC_data, 0)==pdTRUE)
	{
		uint16_t ADC_data = modelListener->ADC_data;
		modelListener->adcrefresh(ADC_data);
	}
	if(xQueueReceive(RFID_ResetQueue, &modelListener->switch_data, 0)==pdTRUE)
	{
		char switch_data = modelListener->switch_data;
		modelListener->Rfid_reset_refresh(switch_data);
	}
	if(xQueueReceive(RFID_TO_GUIQueue, &modelListener->RFID_data, 0)==pdTRUE)
	{
		char confirm = modelListener->RFID_data.confirm;
		uint8_t * ID = modelListener->RFID_data.ID;
		modelListener->RFID_refresh(confirm,ID);
	}
	if(xQueueReceive(CPU_Usage_Queue, &modelListener->Cpu_Usage, 0)==pdTRUE)
	{
		uint16_t Cpu_Usage = modelListener->Cpu_Usage;
		modelListener->Cpu_Usage_refresh(Cpu_Usage);
	}
}
void Model::saveDuty(int Duty)
{
	//duty = Duty;
}
