#include "AP_Baro_InputEvent.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/input.h>

extern const AP_HAL::HAL& hal;

AP_Baro_InputEvent::AP_Baro_InputEvent(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
}

AP_Baro_Backend *AP_Baro_InputEvent::probe(AP_Baro &baro)
{
    AP_Baro_InputEvent *sensor = new AP_Baro_InputEvent(baro);
    if (sensor->_init()) return sensor; else return 0;
}

bool AP_Baro_InputEvent::_init() 
{
    _data_avail = false;
    _last_time = 0;
    _fd = open("/dev/input/baro_event", O_RDONLY);
    if (_fd == -1) {
        printf("Unable to open baro event to read pressure\n");
        return false;
    }
    //_ev = (volatile struct input_event*)mmap(0, sizeof(struct input_event), PROT_READ, MAP_SHARED, fd, 0);
    //close(fd);
    _instance = _frontend.register_sensor();
    //pthread_create(&_thread, 0, _timer, this);
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Baro_InputEvent::tick, void));
    return true;
}

void AP_Baro_InputEvent::update()
{
    /*struct input_event ev;
    ssize_t n;
    float pressure;
    n = read(_fd, &ev, sizeof(ev));
    if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
        pressure = 100.f * ((float)ev.value) / 4096.f;
        _copy_to_frontend(_instance, pressure, 25.0f);
    }*/
    if (_data_avail && _sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _data_avail = false;
        _copy_to_frontend(_instance, _pressure, 27.0f);
        _sem->give();
    }
}

void AP_Baro_InputEvent::tick()
{
    struct input_event ev;
    ssize_t n;
    uint32_t now = AP_HAL::millis();
    if (now - _last_time > 50) {
        _last_time = now;
       n = read(_fd, &ev, sizeof(ev));
        if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
            _pressure = 100.f * ((float)ev.value) / 4096.f;
            _data_avail = true;
        }
    }
}

/*void* AP_Baro_InputEvent::_timer(void* arg)
{
    AP_Baro_InputEvent* baro = (AP_Baro_InputEvent*)arg;
    struct input_event ev;
    ssize_t n;

    while (1) {
    n = read(baro->_fd, &ev, sizeof(ev));
    if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
        baro->_pressure = 100.f * ((float)ev.value) / 4096.f;
    }
    }
}*/

