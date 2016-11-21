/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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

#include "bmi160_support.h"
#include "bmi160_bus.h"
#include "bmi160_gpio.h"
#include "infra/tcmd/handler.h"
#include <stdio.h>

#define BUFFER_LENGTH               ((uint8_t)15)
#define SELF_TEST_DELAY             ((uint8_t)50)
#define ACCEL_POS_DIRECTION         ((uint8_t)1)
#define ACCEL_NEG_DIRECTION         ((uint8_t)0)
#define ACCEL_DIFF_2G               ((uint16_t)8192)
#define ERROR_CODE                  "Error code "
#define ERROR_SAVE_CONFIG           ((uint8_t)0x01)
#define ERROR_G_RANGE_SETTING       ((uint8_t)0x02)
#define ERROR_ACCEL_CONFIG          ((uint8_t)0x03)
#define ERROR_FIRST_COMMAND         ((uint8_t)0x04)
#define ERROR_SECOND_COMMAND        ((uint8_t)0x05)
#define ERROR_FIRST_ACCEL_RD        ((uint8_t)0x06)
#define ERROR_SECOND_ACCEL_RD       ((uint8_t)0x07)
#define ERROR_RESTORE_CONFIG        ((uint8_t)0x08)
#define ERROR_ACCEL_VALUES          ((uint8_t)0x09)

#define CALC_VAL_ABS(val)           (val >= 0 ? (val) : -(val))

/*
 * Starts the gyroscope self test
 *
 * @return results of bus communication function
 *      - 0 on success
 *      - positive value otherwise
 */
uint8_t bmi160_start_gyro_self_test(void)
{
	uint8_t rd_val = 0;
	uint8_t retval;

	/* Enable gyroscope normal mode */
	bmi160_change_sensor_powermode(BMI160_SENSOR_GYRO, BMI160_POWER_NORMAL);

	retval = bmi160_read_reg(BMI160_USER_SELF_TEST_ADDR, &rd_val);
	rd_val =
		BMI160_SET_BITSLICE(rd_val, BMI160_USER_GYRO_SELFTEST_START, 1);
	retval += bmi160_write_reg(BMI160_USER_SELF_TEST_ADDR, &rd_val);

	return retval;
}

/*
 * Gets the gyroscope self test result
 *
 * @param[out]   value : 1 if self test succeeded, 0 otherwise
 *
 * @return results of bus communication function
 *      - 0 on success
 *      - positive value otherwise
 */
uint8_t bmi160_get_gyro_self_test_result(uint8_t *value)
{
	uint8_t rd_val = 0;
	uint8_t retval;

	retval = bmi160_read_reg(BMI160_USER_STAT_ADDR, &rd_val);
	*value = BMI160_GET_BITSLICE(rd_val, BMI160_USER_STAT_GYRO_SELFTEST_OK);
	return retval;
}

/*
 * Starts the accelerometer self test
 *
 * @param[in]   direction : ACCEL_POS_DIRECTION for positive direction
 *                          ACCEL_NEG_DIRECTION for negative direction
 *
 * @return results of bus communication function
 *      - 0 on success
 *      - positive value otherwise
 */
uint8_t bmi160_start_accel_self_test(uint8_t direction)
{
	uint8_t rd_val = 0;
	uint8_t retval;

	/* Enable accelerometer normal mode */
	bmi160_change_sensor_powermode(BMI160_SENSOR_ACCEL,
				       BMI160_POWER_NORMAL);

	retval = bmi160_read_reg(BMI160_USER_SELF_TEST_ADDR, &rd_val);
	/* Enable accelerometer self test */
	rd_val =
		BMI160_SET_BITSLICE(rd_val, BMI160_USER_ACCEL_SELFTEST_AXIS, 1);
	/* Set direction */
	rd_val = BMI160_SET_BITSLICE(rd_val, BMI160_USER_ACCEL_SELFTEST_SIGN,
				     direction);
	/* Amplitude set high */
	rd_val = BMI160_SET_BITSLICE(rd_val, BMI160_USER_SELFTEST_AMP, 1);
	/* Update register */
	retval += bmi160_write_reg(BMI160_USER_SELF_TEST_ADDR, &rd_val);
	return retval;
}

/*
 * Starts the accelerometer self test
 *
 * @param[out]  old_accel_conf : ACC_CONF register content
 * @param[out]  old_selftest_val : SELF_TEST register content
 *
 * @return results of bus communication function
 *      - 0 on success
 *      - positive value otherwise
 */
uint8_t bmi160_save_accel_prev_conf(uint8_t *	old_accel_conf,
				    uint8_t *	old_selftest_val)
{
	uint8_t retval;

	retval = bmi160_read_reg(BMI160_USER_SELF_TEST_ADDR, old_selftest_val);
	retval +=
		bmi160_read_reg(BMI160_USER_ACCEL_CONFIG_ADDR, old_accel_conf);
	return retval;
}

/*
 * Starts the accelerometer self test
 *
 * @param[in]   old_accel_conf : ACC_CONF register content
 * @param[in]   old_selftest_val : SELF_TEST register content
 *
 * @return results of bus communication function
 *      - 0 on success
 *      - positive value otherwise
 */
uint8_t bmi160_restore_accel_prev_conf(uint8_t *old_accel_conf,
				       uint8_t *old_selftest_val)
{
	uint8_t retval;

	retval =
		bmi160_write_reg(BMI160_USER_ACCEL_CONFIG_ADDR, old_accel_conf);
	retval +=
		bmi160_write_reg(BMI160_USER_SELF_TEST_ADDR, old_selftest_val);
	return retval;
}

/*
 * Checks the difference between the two acceleration measures
 *
 * @param[in]   accel_1 : first x, y and z acceleration values
 *              accel_2 : second x, y and z acceleration values
 *
 * @return
 *      - 0 on success
 *      - 1 otherwise
 */
uint8_t check_accel_values(struct bmi160_s16_xyz_t	accel_1,
			   struct bmi160_s16_xyz_t	accel_2)
{
	if (CALC_VAL_ABS(accel_1.x - accel_2.x) < ACCEL_DIFF_2G)
		return 1;
	if (CALC_VAL_ABS(accel_1.y - accel_2.y) < ACCEL_DIFF_2G)
		return 1;
	if (CALC_VAL_ABS(accel_1.z - accel_2.z) < ACCEL_DIFF_2G)
		return 1;

	return 0;
}

/*
 * Triggers the gyroscope BIST : self_test gyro
 *
 * @param[in]   argc        Number of arguments in the Test Command (including group and name)
 * @param[in]   argv        Table of null-terminated buffers containing the arguments
 * @param[in]   ctx         The opaque context to pass to responses
 */
void self_test_gyro(int argc, char **argv, struct tcmd_handler_ctx *ctx)
{
	uint8_t result;

	if (bmi160_start_gyro_self_test() == DRV_RC_OK) {
		/* 50ms delay */
		local_task_sleep_ms(SELF_TEST_DELAY);

		if (!bmi160_get_gyro_self_test_result(&result)) {
			if (result) {
				TCMD_RSP_FINAL(ctx, NULL);
			} else {
				TCMD_RSP_ERROR(ctx, "Test failed");
			}
		}
	} else {
		TCMD_RSP_ERROR(ctx, "Write error");
	}
}

DECLARE_TEST_COMMAND_ENG(self_test, gyro, self_test_gyro);

/*
 * Triggers the accelerometer self test : self_test acc
 *
 * @param[in]   argc        Number of arguments in the Test Command (including group and name)
 * @param[in]   argv        Table of null-terminated buffers containing the arguments
 * @param[in]   ctx         The opaque context to pass to responses
 */
void self_test_accel(int argc, char **argv, struct tcmd_handler_ctx *ctx)
{
	uint8_t accel_conf = 0x2C;
	struct bmi160_s16_xyz_t accel_1;
	struct bmi160_s16_xyz_t accel_2;
	char buffer[BUFFER_LENGTH];
	uint8_t cx;
	uint8_t old_accel_conf;
	uint8_t old_selftest_val;

	do {
		/* Save current configuration */
		if (bmi160_save_accel_prev_conf
			    (&old_accel_conf, &old_selftest_val)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_SAVE_CONFIG);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}

		if (bmi160_set_accel_range(BMI160_ACCEL_RANGE_8G)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_G_RANGE_SETTING);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}
		if (bmi160_write_reg
			    (BMI160_USER_ACCEL_CONFIG_ADDR, &accel_conf)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_ACCEL_CONFIG);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}

		local_task_sleep_ms(SELF_TEST_DELAY * 2);

		if (bmi160_start_accel_self_test(ACCEL_NEG_DIRECTION)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_FIRST_COMMAND);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}
		/* 50ms delay */
		local_task_sleep_ms(SELF_TEST_DELAY);
		if (bmi160_read_accel_xyz(&accel_1, BMI160_SENSOR_ACCEL)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_FIRST_ACCEL_RD);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}
		if (bmi160_start_accel_self_test(ACCEL_POS_DIRECTION)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_SECOND_COMMAND);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}
		/* 50ms delay */
		local_task_sleep_ms(SELF_TEST_DELAY);
		if (bmi160_read_accel_xyz(&accel_2, BMI160_SENSOR_ACCEL)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_SECOND_ACCEL_RD);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}

		/* Restore old configuration */
		if (bmi160_restore_accel_prev_conf
			    (&old_accel_conf, &old_selftest_val)) {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_RESTORE_CONFIG);
			TCMD_RSP_ERROR(ctx, buffer);
			break;
		}
		/* Check acceleration values */
		if (!check_accel_values(accel_1, accel_2)) {
			TCMD_RSP_FINAL(ctx, NULL);
		} else {
			cx = snprintf(buffer, BUFFER_LENGTH, ERROR_CODE);
			snprintf(buffer + cx, BUFFER_LENGTH - cx, "%d",
				 ERROR_ACCEL_VALUES);
			TCMD_RSP_ERROR(ctx, buffer);
		}
	} while (0);
}

DECLARE_TEST_COMMAND_ENG(self_test, accel, self_test_accel);
