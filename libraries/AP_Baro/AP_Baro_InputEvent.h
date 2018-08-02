#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_InputEvent : public AP_Baro_Backend {
public:
    AP_Baro_InputEvent(AP_Baro &);
    static AP_Baro_Backend *probe(AP_Baro &baro);
    void update() override;
    void tick();
private:
    bool _init();
    //static void* _timer(void*);
    uint8_t _instance;
    float _pressure;
    int _fd;
    //pthread_t _thread;
    //volatile struct input_event* _ev;
    bool _data_avail;
    uint32_t _last_time;
};
