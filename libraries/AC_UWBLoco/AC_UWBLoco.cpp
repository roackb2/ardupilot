#include "AC_UWBLoco.h"

static bool uwb_l_not_found = true;
static bool uwb_r_not_found = true;

UWBLoco::UWBLoco(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager)
{
}

void UWBLoco::init() {
    bool uart_ok = true;
    uart_l = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UWBLocoL, 0);
    if (uart_l != nullptr) {
        uart_l->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_UWBLocoL, 0));
    } else uart_ok = false;
    uart_r = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UWBLocoR, 0);
    if (uart_r != nullptr) {
        uart_r->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_UWBLocoR, 0));
    } else uart_ok = false;
    if (uart_ok) gcs().send_text(MAV_SEVERITY_INFO, "uwbloco uart ok");
}

bool UWBLoco::update(float& n, float& e, float& yaw) {
    read_tag_frame(uart_l, tag_x_l, tag_y_l, state_l, offset_l, chk_sum_l);
    if (state_l == UWB_TAG_FRAME_OK) {
        if (uwb_l_not_found) {
            gcs().send_text(MAV_SEVERITY_INFO, "uwbloco left found");
            uwb_l_not_found = false;   
        }
        state_l = UWB_TAG_FRAME_NOT_FOUND;
        rdy_l = true;
        rdy_tag_n_l = tag_x_l;
        rdy_tag_e_l = -tag_y_l;
    } 
    read_tag_frame(uart_r, tag_x_r, tag_y_r, state_r, offset_r, chk_sum_r);
    if (state_r == UWB_TAG_FRAME_OK) {
        if (uwb_r_not_found) {
            gcs().send_text(MAV_SEVERITY_INFO, "uwbloco right found");
            uwb_r_not_found = false;   
        }
        state_r = UWB_TAG_FRAME_NOT_FOUND;
        rdy_r = true;
        rdy_tag_n_r = tag_x_r;
        rdy_tag_e_r = -tag_y_r;
    }
    if (rdy_l && rdy_r) {
        rdy_l = false;
        rdy_r = false;
        n = (rdy_tag_n_l + rdy_tag_n_r) * 0.0005f;
        e = (rdy_tag_e_l + rdy_tag_e_r) * 0.0005f;
        int lr_n = rdy_tag_n_r - rdy_tag_n_l;
        int lr_e = rdy_tag_e_r - rdy_tag_e_l;
        yaw = atan2f(lr_e, lr_n);
        return true;
    } else return false;
}

void UWBLoco::read_tag_frame(AP_HAL::UARTDriver* uart, int& tag_x, int& tag_y, int& state, int& offset, unsigned char& chk_sum) {
    if (uart == nullptr) {
        return;
    }
    int16_t nbytes = uart->available();
    while (nbytes > 0) {
        nbytes--;        
        unsigned char c = uart->read();
        if (state == UWB_TAG_FRAME_NOT_FOUND) {
            if (c == 0x55) {
                state = UWB_TAG_FRAME_HEADER;
                chk_sum = 0x55;
            }
        } else if (state == UWB_TAG_FRAME_HEADER) {
            if (c == 0x1) {
                state = UWB_TAG_FRAME_FOUND; 
                offset = 1;
                chk_sum += c;
            } else state = UWB_TAG_FRAME_NOT_FOUND;
        } else if (state == UWB_TAG_FRAME_FOUND) {
            offset++;
            if (offset == 4) {
                tag_x = c;
            } else if (offset == 5) {
                tag_x = tag_x | (c << 8);
            } else if (offset == 6) {
                tag_x = tag_x | (c << 16);           
            } else if (offset == 7) {
                tag_y = c;
            } else if (offset == 8) {
                tag_y = tag_y | (c << 8);
            } else if (offset == 9) {
                tag_y = tag_y | (c << 16);           
            } else if (offset == 127) {
                if (c == chk_sum) state = UWB_TAG_FRAME_OK; else state = UWB_TAG_FRAME_NOT_FOUND;
                return;
            }
            chk_sum += c;
        }
    }
}
