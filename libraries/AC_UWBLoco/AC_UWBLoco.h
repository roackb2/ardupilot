#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#define UWB_TAG_FRAME_NOT_FOUND 0
#define UWB_TAG_FRAME_HEADER 1
#define UWB_TAG_FRAME_FOUND 2
#define UWB_TAG_FRAME_OK 3
#define UWB_TAG_FRAME_BAD 4

class UWBLoco {
    public:
        UWBLoco(AP_SerialManager &_serial_manager);

        /* Do not allow copies */
        UWBLoco(const UWBLoco &other) = delete;
        UWBLoco &operator=(const UWBLoco&) = delete;

        void init();
        bool update(float& x, float& y, float& yaw);
    private:
        void read_tag_frame(AP_HAL::UARTDriver* uart, int& tag_x, int& tag_y, int& state, int& offset, unsigned char& chk_sum);
        AP_SerialManager &serial_manager;
        AP_HAL::UARTDriver *uart_l = nullptr;
        AP_HAL::UARTDriver *uart_r = nullptr;
        int tag_x_l;
        int tag_y_l;
        int state_l = UWB_TAG_FRAME_NOT_FOUND;
        int offset_l;
        unsigned char chk_sum_l;
        int tag_x_r;
        int tag_y_r;
        int state_r = UWB_TAG_FRAME_NOT_FOUND;
        int offset_r;
        unsigned char chk_sum_r;
        bool rdy_l = false;
        bool rdy_r = false;
        int rdy_tag_n_l;
        int rdy_tag_e_l;
        int rdy_tag_n_r;
        int rdy_tag_e_r;
};
