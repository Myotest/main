/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SENSOR_DATA_FORMAT_H__
#define __SENSOR_DATA_FORMAT_H__

#include <stdint.h>
#include "util/compiler.h"

/**
 * @defgroup sensor_data_formats  Sensor data format
 * @ingroup sensor_service
 * @{
 */

/**
 * Sensor type definition,it mustn't be more than 32
 */
typedef enum {
	ON_BOARD_SENSOR_TYPE_START = 0,
//start of on_board
	SENSOR_PHY_TYPE_BASE = ON_BOARD_SENSOR_TYPE_START, //start of phy sensor
	SENSOR_ACCELEROMETER,
	SENSOR_GYROSCOPE,
	SENSOR_MAGNETOMETER,
	SENSOR_BAROMETER,
	SENSOR_TEMPERATURE,
	SENSOR_HUMIDITY,
	SENSOR_OHRM,
	SENSOR_RESERVED0,
	SENSOR_ABS_TYPE_BASE,  //end of phy sensor,start of abstract sensor
	SENSOR_PATTERN_MATCHING_GESTURE,
	SENSOR_ABS_TAPPING,
	SENSOR_ABS_SIMPLEGES,
	SENSOR_ABS_STEPCOUNTER,
	SENSOR_ABS_ACTIVITY,
	SENSOR_ABS_CADENCE,
	SENSOR_ABS_OHRM,
	SENSOR_ALGO_KB,
	SENSOR_ALGO_DEMO,
	SENSOR_ANY_MOTION,
	SENSOR_NO_MOTION,
	SENSOR_TAPPING,
	SENSOR_ABS_ALTITUDE,
	SENSOR_ABS_TYPE_TOP,    //end of abstract sensor
//end of on_board
	ON_BOARD_SENSOR_TYPE_END = SENSOR_ABS_TYPE_TOP, //reduce the num of judgement position

	ANT_SENSOR_TYPE_START = ON_BOARD_SENSOR_TYPE_END, //reduce the num of judgement position
	ANT_SENSOR_TYPE_END,

	BLE_SENSOR_TYPE_START = ANT_SENSOR_TYPE_END,
	BLE_SENSOR_TYPE_END
} ss_sensor_type_t;

/**
 * Sensor type MASK definition, as the sensor_type_bit_map.
 */
#define ACCEL_TYPE_MASK     (1 << SENSOR_ACCELEROMETER)
#define GYRO_TYPE_MASK      (1 << SENSOR_GYROSCOPE)
#define MAG_TYPE_MASK       (1 << SENSOR_MAGNETOMETER)
#define BARO_TYPE_MASK      (1 << SENSOR_BAROMETER)
#define TEMP_TYPE_MASK      (1 << SENSOR_TEMPERATURE)
#define HUMI_TYPE_MASK      (1 << SENSOR_HUMIDITY)
#define OHRM_PHY_TYPE_MASK      (1 << SENSOR_OHRM)
#define PATTERN_MATCHING_TYPE_MASK       (1 << SENSOR_PATTERN_MATCHING_GESTURE)
#define TAPPING_TYPE_MASK   (1 << SENSOR_ABS_TAPPING)
#define SIMPLEGES_TYPE_MASK (1 << SENSOR_ABS_SIMPLEGES)
#define STEPCOUNTER_TYPE_MASK   (1 << SENSOR_ABS_STEPCOUNTER)
#define ACTIVITY_TYPE_MASK      (1 << SENSOR_ABS_ACTIVITY)
#define CADENCE_TYPE_MASK       (1 << SENSOR_ABS_CADENCE)
#define OHRM_ABS_TYPE_MASK      (1 << SENSOR_ABS_OHRM)
#define ALTITUDE_ABS_TYPE_MASK      (1 << SENSOR_ABS_ALTITUDE)
#define ALGO_KB_MASK        (1 << SENSOR_ALGO_KB)
#define ALGO_DEMO_MASK (1 << SENSOR_ALGO_DEMO)
#define BOARD_SENSOR_MASK   (ACCEL_TYPE_MASK |	    \
			     BARO_TYPE_MASK |	   \
			     TEMP_TYPE_MASK |	   \
			     HUMI_TYPE_MASK |	   \
			     MAG_TYPE_MASK |	  \
			     GYRO_TYPE_MASK |	   \
			     OHRM_PHY_TYPE_MASK |      \
			     PATTERN_MATCHING_TYPE_MASK |      \
			     TAPPING_TYPE_MASK |     \
			     SIMPLEGES_TYPE_MASK |   \
			     STEPCOUNTER_TYPE_MASK | \
			     ACTIVITY_TYPE_MASK |    \
			     CADENCE_TYPE_MASK |	\
			     ALTITUDE_ABS_TYPE_MASK |	 \
			     OHRM_ABS_TYPE_MASK |      \
			     ALGO_KB_MASK |	 \
			     ALGO_DEMO_MASK)
#define ANT_TYPE_MASK       (1 << ANT_SENSOR_TYPE_START)

#define BLE_TYPE_MASK       (1 << BLE_SENSOR_TYPE_START)

/**
 * Different kinds of sensor data
 */
typedef enum {
	ACCEL_DATA = 0,
	GYRO_DATA = 0,
	MAG_DATA = 0,
	BLE_HR_DATA = 0,
} ss_data_type_t;


/**
 * activity type definition
 */
typedef enum {
	NONACTIVITY = 0,
	WALKING,
	RUNNING,
	BIKING,
	SLEEPING,
	UPSTAIRING,
	DOWNSTAIRING,
	LEGSHAKING,
} ss_activity_type_t;
/**
 * Unique data struct provide all kinds of sensor data
 */
typedef struct {
	uint8_t sensor_type;           /*!< Sensor type as in \ref ss_sensor_type_t       */
	uint8_t subscription_type;     /*!< Defined for a specific sensor_type            */
	uint32_t timestamp;            /*!< Time when this data is generated              */
	uint8_t data_length;           /*!< Data size                                     */
	uint8_t data[0];               /*!< Start of data; Content depends on the sensor. */
} sensor_service_sensor_data_header_t;


/*
 * Phy and abs sensor priv data struct
 */

/** Sensor data for SENSOR_PATTERN_MATCHING_GESTURE sensor
 * @param nClassLabel recognition result
 *        - 1: up down
 *        - 2: left right
 *        - 3: down up
 *        - 4: right left
 *        - 5: counter clockwise
 *        - 6: clockwise
 *        - 7: alpha
 *        - 8: tick
 */
struct gs_personalize {
	uint16_t size;      /*!< reserved */
	int16_t nClassLabel; /*!< Recognition result of user defined gestures, see table above */
	int16_t reserved;
} __packed;

/** Sensor data for SENSOR_ABS_TAPPING sensor
 * @param  tapping_cnt tapping type
 *         - 0  no tap
 *         - 1  not used
 *         - 2  double tap
 *         - 3  triple tap
 */
struct tapping_result {
	int16_t tapping_cnt; /*!< Number of consecutive tapping detected, see table above */
} __packed;

/** Sensor data for SENSOR_ABS_SIMPLEGES sensor
 * @param type gesture type
 *        - 1:shake
 *        - 2:flick in negative x direction
 *        - 3:flick in positive x direction
 *        - 4:flick in negative y direction
 *        - 5:flick in positive y direction
 *        - 6:flick in negative z direction
 *        - 7:flick in positive z direction
 */
struct simpleges_result {
	int16_t type; /*!< gesture type, see table above */
} __packed;

/** Sensor data for SENSOR_ABS_STEPCOUNTER sensor
 * @param steps - related to activity type
 *        - for running/walking: steps
 *        - for biking: pedal counts
 *        - for climbing: stair counts
 */
struct stepcounter_result {
	uint32_t steps;  /*!< Cumulative step/stair/pedal count */
	uint32_t activity; /*!< Activity type as in \ref ss_activity_type_t*/
} __packed;

/**
 * Sensor data for SENSOR_ABS_ACTIVITY sensor
 */
struct activity_result {
	uint32_t type;   /*!< Activity type as in \ref ss_activity_type_t*/
} __packed;


/** Sensor data for SENSOR_ABS_STEPCADENCE sensor
 * @param cadence - related to activity type
 *        - for running/walking/climbing: step cadence
 *        - for biking: pedal cadence
 */
struct cadence_result {
	uint16_t cadence; /*!< Cadence currently detected */
	uint16_t activity; /*!< Activity type as in \ref ss_activity_type_t*/
} __packed;

/**************************
 * the struct below is definition of the report event data struct of basis algo
 * , which must be defined by user accordding to the basis algo result;
 ***************************/
struct demo_algo_result {
	int type;
	short ax; /*!< Acceleration for X-axis, unit:mg */
	short ay; /*!< Acceleration for Y-axis, unit:mg */
	short az; /*!< Acceleration for Z-axis, unit:mg */
	int gx; /*!< Angular velocity in rotation along X-axis, unit:m_degree/s */
	int gy; /*!< Angular velocity in rotation along Y-axis, unit:m_degree/s */
	int gz; /*!< Angular velocity in rotation along Z-axis, uint:m_degree/s */
} __packed;

/** Sensor data for SENSOR_ACCELEROMETER sensor
 */
struct accel_phy_data {
	short ax; /*!< Acceleration for X-axis, unit:mg */
	short ay; /*!< Acceleration for Y-axis, unit:mg */
	short az; /*!< Acceleration for Z-axis, unit:mg */
} __packed;

/** Sensor data for SENSOR_GYROSCOPE sensor
 */
struct gyro_phy_data {
	int gx; /*!< Angular velocity in rotation along X-axis, unit:m_degree/s */
	int gy; /*!< Angular velocity in rotation along Y-axis, unit:m_degree/s */
	int gz; /*!< Angular velocity in rotation along Z-axis, uint:m_degree/s */
} __packed;

/** Sensor data for SENSOR_MAGNETOMETER sensor
 */
struct mag_phy_data {
	int mx; /*!< X-axis magnetic data, unit:16LSB/uT */
	int my; /*!< Y-axis magnetic data, unit:16LSB/uT */
	int mz; /*!< Z-axis magnetic data, unit:16LSB/uT */
} __packed;

/** Sensor data for SENSOR_TEMPERATURE sensor
 */
typedef struct temp_phy_data {
	int32_t value; /*!< Temperature data, unit:DegC/100 */
}__packed phy_temp_data_t;

/** Sensor data for SENSOR_HUMIDITY sensor
 */
typedef struct humi_phy_data {
	uint32_t value; /*!< Humidity data, unit:1/1024 %RH */
}__packed phy_humi_data_t;

/** Sensor data for SENSOR_BAROMETER sensor
 */
typedef struct baro_phy_data {
	uint32_t value; /*!< Barometer data, unit:Pa/256 */
}__packed phy_baro_data_t;

/** Sensor data for SENSOR_ABS_ALTITUDE sensor
 */
struct altitude_result {
	int32_t abso_altitude; /*!< Absolute altitude (cm) */
} __packed;

/** Property data for SENSOR_ABS_ALTITUDE sensor
 */
struct altitude_property_data {
	int32_t alti_value; /*!< Standard height / cm or pressure / Pa */
	int32_t alti_flag; /*!< Property flag: 1-sea level pressure; 2-current altitude*/
} __packed;

/** Sensor data for SENSOR_ABS_OHRM sensor
 */
struct ohrm_result {
	uint8_t hr; /*!< Heartrate value in BPM */
	uint8_t conf; /*!< Confidence of hr value, from 0 to 100, the higher the better */
} __packed;

/** Sensor data for SENSOR_OHRM sensor (OHRM raw data)
 */
struct ohrm_phy_data {
	uint16_t adc_value; /*!< Raw value from adc conversion, resolution is 12bit to 3.3V*/
	uint16_t dac_code; /*!< Value set to DAC, from 0 to 4095 */
	uint8_t dcp_code; /*!< Value set to DIGPOT, from 128 to 255 */
	uint8_t mux_code; /*!< Value set to DIGPOT after conversion, from 0 to 7*/
	int16_t x;      /*!< ADXL362 acceleration value for X-axis, unit:mg */
	int16_t y;      /*!< ADXL362 acceleration value for Y-axis, unit:mg */
	int16_t z;      /*!< ADXL362 acceleration value for Z-axis, unit:mg */
	uint16_t adc_blind_value;
} __packed;
/** @} */

/*
 * Knowledge Builder Algorithm result structure
 */
typedef struct kb_result {
	int16_t nClassLabel; /*!< recognition result of user defined gestures */
}__packed kb_result_t;

#endif /*__SENSOR_DATA_FORMAT_H__*/
