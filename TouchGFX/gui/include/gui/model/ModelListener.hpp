#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>
#include <stdint.h>
typedef struct{
	char confirm;
	uint8_t ID[13];
} RFID_Data;
typedef struct{
	float temperature;
	float humidity;
} SHT_Data;
class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}
    virtual void timeRefresh() {}
    virtual void temrefresh(float temperature,float humidity) {}
    virtual void adcrefresh(uint16_t ADC_data) {}
    virtual void Rfid_reset_refresh(char switch_data) {}

    virtual void RFID_refresh(char confirm,uint8_t *ID) {}
    virtual void Cpu_Usage_refresh(uint16_t Cpu_Usage) {}
    void bind(Model* m)
    {
        model = m;
    }
    RFID_Data RFID_data;
    uint16_t Cpu_Usage;
    char switch_data;
    SHT_Data SHT_data;
    uint16_t ADC_data;
protected:
    Model* model;

};

#endif // MODELLISTENER_HPP
