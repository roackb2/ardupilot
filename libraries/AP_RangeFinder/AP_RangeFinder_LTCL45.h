/****************************************************************************
 *      __     __ _____ _____ ____ _   __                                   *
 *     / /    / //_  _// ___//   // \ / /                                   *
 *    / /___ / /  / / / ___// U // A V /                                    *
 *   /_____//_/  /_/ /____//___//_/ \_/ Tech. Corp.                         *
 *                                                                          *
 * Multi-Rotor Far Field Motion Sensor for PX4                              *
 *                                                                          *
 * Written by by Michael Tang for Liteon Tech. Co.                          *
 * michael.tang@liteon.com                                                  *
 *                                                                          *
 ****************************************************************************/
/**
 * @file AP_RangeFinder_LTC45.h
 * @reference: AP_RangeFinder_PulsedLightLRF.h
 *
 * Driver for the LTC LIDAR 45m module connected via I2C.
 */
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

/* Connection diagram
 *
 *        ------------------------------------------------------------------------------------
 *        |           J2-1(LED) J2-2(5V) J2-3(Enable) J2-4(Ref Clk) J2-5(GND) J2-6(GND)      |
 *        |                                                                                  |
 *        |                                                                                  |
 *        |                                      J1-3(I2C Clk) J1-2(I2C Data) J1-1(GND)      |
 *        ------------------------------------------------------------------------------------
 */

// i2c address
#define AP_RANGEFINDER_LTCLMINI45_ADDR					0x28

// min and max distances
#define AP_RANGEFINDER_LTCLMINI45_MIN_DISTANCE			20
#define AP_RANGEFINDER_LTCLMINI45_MAX_DISTANCE			4450

// registers
#define AP_RANGEFINDER_LTCLMINI45_MEASURE_REG			0x00
#define AP_RANGEFINDER_LTCLMINI45_DISTHIGH_REG			0x00    // high byte of distance measurement
#define AP_RANGEFINDER_LTCLMINI45_DISTLOW_REG			0x04    // high byte of distance measurement
#define AP_RANGEFINDER_LTCLMINI45_DISTHIGH_BITINV_REG	0x00    // high byte of bitwise invert value of distance measurement
#define AP_RANGEFINDER_LTCLMINI45_DISTLOW_BITINV_REG	0x04    // high byte of bitwise invert value of distance measurement

// registers 
#define AP_RANGEFINDER_LTCLMINI45_WHOAMI_REG			0x13    // who am I register
#define AP_RANGEFINDER_LTCLMINI45_FWVER_MAIN_REG		0x16    // 
#define AP_RANGEFINDER_LTCLMINI45_FWVER_SUB_REG			0x17    // 
#define AP_RANGEFINDER_LTCLMINI45_SLAVEADDR_REG			0x20    // set the slave address

// command register values
#define AP_RANGEFINDER_LTCLMINI45_MSRREG_ACQUIRE        0x04    // Varies based on sensor revision, 0x04 is newest, 0x61 is older
enum LTCL45SWVER {
	LTCL45SWVER_V1 = 1,
};
class AP_RangeFinder_LTC45 : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);


private:
    // constructor
    AP_RangeFinder_LTC45(RangeFinder &ranger, uint8_t instance,
                                  RangeFinder::RangeFinder_State &_state);
    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
	uint8_t get_FWVER(uint8_t &fwver);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
	uint8_t _fw_version;
	uint16_t last_distance_cm;
};
