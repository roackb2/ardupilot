/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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
 * @file AP_RangeFinder_LTC45.cpp
 * @reference: AP_RangeFinder_PulsedLightLRF.cpp
 *
 * Driver for the LTC LIDAR 45m module connected via I2C.
 */
#include "AP_RangeFinder_LTCL45.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LTC45::AP_RangeFinder_LTC45(RangeFinder &_ranger, uint8_t instance,
                                                             RangeFinder::RangeFinder_State &_state)
    : AP_RangeFinder_Backend(_ranger, instance, _state)
    , _dev(hal.i2c_mgr->get_device(1, AP_RANGEFINDER_LTCLMINI45_ADDR))
{
}

/*
   detect if a PulsedLight rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_LTC45::detect(RangeFinder &_ranger, uint8_t instance,
                                                              RangeFinder::RangeFinder_State &_state)
{
    AP_RangeFinder_LTC45 *sensor
        = new AP_RangeFinder_LTC45(_ranger, instance, _state);
	sensor->last_distance_cm = 0;
	sensor->get_FWVER(sensor->_fw_version);
	if (sensor->_fw_version != LTCL45SWVER_V1) {
		// error handling;
	}
    if (!sensor || !sensor->start_reading()) {
        delete sensor;
        return nullptr;
    }

    uint16_t reading_cm;

    if (!sensor->get_reading(reading_cm)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_LTC45::start_reading()
{
    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LTC45::get_reading(uint16_t &reading_cm)
{
    //be16_t val;
	be16_t val[2];

    if (!_dev->get_semaphore()->take(1)) {
        return false;
    }

    // read the high and low byte distance registers
    bool ret = _dev->read_registers(AP_RANGEFINDER_LTCLMINI45_DISTHIGH_REG,
                                    (uint8_t *) &val, sizeof(val));
    _dev->get_semaphore()->give();

    if (!ret) {
        return false;
    }

	// combine results into distance
	uint16_t dist_cm = be16toh(val[0]);		// 2-bytes data from AP_RANGEFINDER_LTCLMINI45_DISTHIGH_REG
	uint16_t inv_dist_cm = be16toh(val[1]);	// 2-bytes data from AP_RANGEFINDER_LTCLMINI45_DISTHIGH_BITINV_REG

	// the I2C signal might interference by noise,
	// so we need to check the value by the DIST_BITINV register
	// the value of DIST_BITINV is complement of DIST register
	if ((dist_cm ^ inv_dist_cm) == 0xFFFF) {
		reading_cm = dist_cm;
		last_distance_cm = dist_cm;
	} else {
		//state.voltage_mv = dist_cm * 10;// for inspecting the measurement spike
		last_distance_cm = dist_cm;
	}

	//// remove momentary spikes
	//if (abs(dist_cm - last_distance_cm) < 100) {
	//	reading_cm = dist_cm;
	//	last_distance_cm = dist_cm;
	//} else {
	//	if (abs(dist_cm - last_distance_cm) > 50) {
	//		if (dist_cm > last_distance_cm) {
	//			state.voltage_mv = dist_cm *10;
	//		}
	//	}
	//	last_distance_cm = dist_cm;
	//}
	
	
    return true;
}

uint8_t AP_RangeFinder_LTC45::get_FWVER(uint8_t &fwver)
{
	uint8_t val = 0;

	if (!_dev || !_dev->get_semaphore()->take(1)) {
		return 0xFF;
	}

	bool ret = _dev->read_registers(AP_RANGEFINDER_LTCLMINI45_FWVER_MAIN_REG,
                                    (uint8_t *) &val, sizeof(val));
	fwver = val;

	_dev->get_semaphore()->give();

	return ret;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LTC45::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
