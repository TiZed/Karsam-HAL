/* karsam.c
 * 
 * Copyright (C) 2016 TiZed
 * 
 * Portions of this code are based on stepgen.c by John Kasunich and
 * PICnc V2 by GP Orcullo.
 * 
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ---------------------------------------------------------------------
 *
 * GPIO:
 * ----
 * BCM 23 (16) - KReal - Reset
 *
 * SPI:
 * ---
 * BCM  9 (21) - KReal - MISO
 * BCM 10 (19) - KReal - MOSI
 * BCM 11 (23) - KReal - SCLK
 * BCM  8 (24) - KReal - SS
 */


#include "hal.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"

#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <stdio.h>
#include <sched.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "karsam.h"

MODULE_AUTHOR("TiZed") ;
MODULE_DESCRIPTION("PIC32 based controller for Karsam") ;
MODULE_LICENSE("GPL") ;

static unsigned int num_axes = 0 ;
RTAPI_MP_INT(num_axes, "Number of axes.") ;

char * axis_name[MAX_AXES] ;
RTAPI_MP_ARRAY_STRING(axis_name, MAX_AXES, "axis name for up to 5 axes.");

static unsigned int num_pwm = 0 ;
RTAPI_MP_INT(num_pwm, "Number of PWM channels.") ;

static int comp_id ;
static const char *module_name = "karsam" ;
static const char *prefix = "karsam" ;

static platform_t platform ;

static stepgen_t * stepgen_array = NULL ;
static pwmgen_t * pwmgen_array = NULL ;

static update_data_t * update_data = NULL ;

static long base_freq ;
static long period_ns ;
static double period_fp ;
static double velocity_scale ;
static double accel_scale ;

static double dt ;
static long dt_ns ;
static double recip_dt ;

static int spi_fd ;
static int spi_speed ;
volatile unsigned int tx_buf[SPIBUFSIZE], rx_buf[SPIBUFSIZE] ;

static pthread_t spi_thread ;
static spi_wr_info_t spi_info ;

volatile unsigned * gpio ;

static int debug ;

platform_t check_platform(void)
{
	FILE *fp ;
	char buf[2048] ;
	size_t fsize ;

	debug = 0 ;

	fp = fopen("/proc/cpuinfo", "r") ;
	fsize = fread(buf, 1, sizeof(buf), fp) ;
	fclose(fp) ;

	if (fsize == 0 || fsize == sizeof(buf))
		return 0;

	/* NUL terminate the buffer */
	buf[fsize] = '\0' ;

	if (NULL != strstr(buf, "BCM2708"))
		return RPI ;
	else if (NULL != strstr(buf, "BCM2709"))
		return RPI_2 ;
	// WA for latest kernel CPU report bug
	else if (NULL != strstr(buf, "BCM2835"))
		return RPI_2 ;
	else
		return UNSUPPORTED ;
}

int rtapi_app_main(void)
{
	char name[HAL_NAME_LEN + 1] ;
	int retval, n, ret ;
	axis_name_t axis ;
	long int t, end_t ;

	rtapi_set_msg_level(RTAPI_MSG_ALL) ;
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Karsam starting.\n", module_name) ;

	if (num_axes == 0) {
		error_msg("%s: ERROR: No axes configured.\n") ;
		return -1;
    }

	platform = check_platform() ;

	// Check for Raspberry Pi and initialize SPI
	switch (platform) {
	case RPI:
	case RPI_2:
		rtapi_print_msg(RTAPI_MSG_INFO, "%s: Raspberry Pi detected.\n", module_name) ;

		// Setup SPI
		rtapi_print_msg(RTAPI_MSG_INFO, "%s: Entering SPI setup.\n", module_name) ;
        ret = bcm_spi_setup(16000000, 0x00) ;
        if (ret != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: bcm_spi_setup() returned with error code: %d\n", module_name, ret) ;
			return -1 ;
		}

        // Attach SPI buffers
		spi_info.tx_data = (unsigned char*) tx_buf ;
		spi_info.rx_data = (unsigned char*) rx_buf ;
		spi_info.run = 1 ;

		// Activate SPI loop in a different thread
		rtapi_print_msg(RTAPI_MSG_INFO, "%s: Starting SPI thread.\n", module_name) ;
		ret = pthread_create(&spi_thread, NULL, bcm_spi_thread, (void *) &spi_info) ;
		if (ret) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: Failed to spawn SPI thread, error code: %d\n", module_name, ret) ;
			return -1 ;
		}

        // GPIO Setup
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Entering GPIO setup.\n", module_name) ;
        ret = rpi_gpio_setup() ;
        if (ret != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: bcm_gpio_setup() returned with error code: %d\n", module_name, ret) ;
			return -1 ;
        }

        OUT_GPIO(8) ;		// Pin 8 used for chip select
        OUT_GPIO(22) ;      // Controller reset
        OUT_GPIO(27) ;		// Controller program

        GPIO_SET = (1 << 8) ;
        GPIO_CLR = (1 << 22) ;
        GPIO_SET = (1 << 27) ;

		break;
	default:
		error_msg("%s: ERROR: This HAL requires a Raspberry Pi.\n") ;
		return -1 ;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Initiating HAL module.\n", module_name) ;
	comp_id = hal_init(module_name) ;
	if (comp_id < 0) {
		error_msg("%s: ERROR: hal_init() failed\n") ;
		return -1 ;
	}

	/* Allocate axes shared memory */
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Allocating shared memory.\n", module_name) ;
	stepgen_array = hal_malloc(num_axes * sizeof(stepgen_t)) ;
	if (stepgen_array == NULL) {
		error_msg("%s: ERROR: hal_malloc() failed for axes\n") ;
		hal_exit(comp_id) ;
		return -1 ;
	}

	/* Allocate PWM shared memory */
	if (num_pwm > 0) {
		pwmgen_array = hal_malloc(num_pwm * sizeof(pwmgen_t)) ;
		if (pwmgen_array == NULL) {
			error_msg("%s: ERROR: hal_malloc() failed for pwm\n") ;
			hal_exit(comp_id) ;
			return -1 ;
		}
	}

	update_data = hal_malloc(sizeof(update_data_t)) ;
	if (update_data == NULL) {
		error_msg("%s: ERROR: hal_malloc() failed for update_data\n") ;
		hal_exit(comp_id) ;
		return -1 ;
	}

	update_data->stepgen = stepgen_array ;
	update_data->pwmgen = pwmgen_array ;

	update_data->num_axes = num_axes ;
	update_data->num_pwm = num_pwm ;

	/* export axes parameters */
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting axes pins and parameters.\n", module_name) ;
	for (n = 0 ; n < num_axes ; n++) {
		axis = parse_axis(axis_name[n]) ;
		if (axis == INVALID) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: Invalid axis name '%s'\n", module_name, axis_name[n]);
			hal_exit(comp_id) ;
			return -1 ;
		}
		retval = export_stepgen(n, &(stepgen_array[n]), axis) ;
		if(retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: stepgen %d export failed\n", module_name, n);
			hal_exit(comp_id) ;
			return -1 ;
		}
	}

	/* export PWM parameters */
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Exporting PWM pins and parameters.\n", module_name) ;
	for (n = 0 ; n < num_pwm ; n++) {
		retval = export_pwmgen(n, &(pwmgen_array[n])) ;
		if(retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"%s: ERROR: pwmgen %d export failed\n", module_name, n);
			hal_exit(comp_id) ;
			return -1 ;
		}
	}

	retval = hal_pin_bit_newf(HAL_OUT, &(update_data->emo),
			comp_id, "%s.emo", prefix) ;
	if (retval < 0) goto error ;
	*(update_data->emo) = 0 ;

	retval = hal_pin_bit_newf(HAL_OUT, &(update_data->z_level),
			comp_id, "%s.z_level", prefix) ;
	if (retval < 0) goto error ;
	*(update_data->z_level) = 0 ;

	retval = hal_pin_bit_newf(HAL_OUT, &(update_data->ready),
			comp_id, "%s.ready", prefix) ;
	if (retval < 0) goto error ;
	*(update_data->ready) = 0 ;

	retval = hal_pin_bit_newf(HAL_OUT, &(update_data->spi_error),
			comp_id, "%s.spi_error", prefix) ;
	if (retval < 0) goto error ;
	*(update_data->spi_error) = 0 ;

	retval = hal_pin_bit_newf(HAL_IN, &(update_data->enable),
			comp_id, "%s.enable", prefix) ;
	if (retval < 0) goto error ;
	*(update_data->enable) = 0 ;

	retval = hal_param_u32_newf(HAL_RW, &(update_data->basefreq),
			comp_id, "%s.base_frequency", prefix) ;
	if (retval < 0) goto error ;
	update_data->basefreq = 0 ;

error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
				module_name, retval) ;
		hal_exit(comp_id) ;
		return -1 ;
	}

	rtapi_snprintf(name, sizeof(name), "%s.update", prefix) ;
	retval = hal_export_funct(name, update, update_data, 1, 0, comp_id);
	if (retval < 0) {
		error_msg("%s: ERROR: update function export failed\n") ;
		hal_exit(comp_id) ;
		return -1 ;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: stepgen = %x, %x.\n", module_name, stepgen_array, update_data->stepgen) ;
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: pwmgen = %x, %x.\n", module_name, pwmgen_array, update_data->pwmgen) ;
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: update_data = %x.\n", module_name, update_data) ;


    // Wait for ~200msec.
	for(n = 0 ; n < 200 ; n++) {
		rtapi_delay(1000000) ;
	}

    // Clear Controller reset
    GPIO_SET = 1 << 22 ;

    // Wait for ~1sec.
    for(n = 0 ; n < 1000 ; n++) {
    	rtapi_delay(1000000) ;
    }

    // Clear step controllers reset
    GPIO_CLR = 1 << 22 ;

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", module_name) ;
	hal_ready(comp_id) ;
	return 0 ;
}

void rtapi_app_exit(void)
{
	int n ;

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: removing driver\n", module_name) ;
	spi_info.run = 0 ;
	stop_machine() ;
	pthread_join(spi_thread, NULL) ;
	// Clear Controller reset
	GPIO_SET = 1 << 22 ;

	// Wait for ~0.5sec.
	for(n = 0 ; n < 500 ; n++) {
		rtapi_delay(1000000) ;
	}

	// Clear step controllers reset
	GPIO_CLR = 1 << 22 ;

	bcm_close() ;

	hal_exit(comp_id) ;
}

void update(void * arg, long period)
{
	int n, i = 0, new_timing = 0, send_cfg = 0, send_pwm = 0 ;
	update_data_t * ud = arg ;
	stepgen_t * stepgen = ud->stepgen ;
	pwmgen_t * pwmgen = ud->pwmgen ;

	unsigned int checksum = 0 ;

	long min_period ;
	double max_freq, max_accl, pos_cmd, vel_cmd ;
	double curr_pos, match_time, new_vel, dp, dv ;
	double est_out, est_cmd, est_err ;
	int dir ;

	// Change in 'update' loop period
	if (period != dt_ns) {
		dt_ns = period ;
		dt = dt_ns * 1e-9 ;
		recip_dt = 1.0 / dt ;
	}

	// Change in u-controller stepgen loop frequency
	if(base_freq != ud->basefreq) {
		base_freq = ud->basefreq ;
		period_fp = 1.0 / (double)base_freq ;
		period_ns = (long)(period_fp * 1e9) ;

		velocity_scale = (1L << PICKOFF) * period_fp ;
		accel_scale = velocity_scale * period_fp ;

		new_timing = send_cfg = 1 ;
	}

	// Configuration changes pass
	for (n = 0 ; n < ud->num_axes ; n++) {
		/* check for scale change */
		if (stepgen->position_scale != stepgen->old_scale) {
			/* get ready to detect future scale changes */
			stepgen->old_scale = stepgen->position_scale;
			/* validate the new scale value */
			if ((stepgen->position_scale < 1e-20) && (stepgen->position_scale > -1e-20)) {
				/* value too small, divide by zero is a bad thing */
				stepgen->position_scale = 1.0;
			}
			/* we will need the reciprocal, and the accum is fixed point with
			   fractional bits, so we precalc some stuff */
			stepgen->scale_inv = (1.0 / (1L << PICKOFF)) / stepgen->position_scale;
		}

		if (new_timing || stepgen->step_len != stepgen->old_step_len) {
			stepgen->step_len = (stepgen->step_len == 0) ? period_ns : ulceil(stepgen->step_len, period_ns) ;
			stepgen->old_step_len = stepgen->step_len ;

			stepgen->step_len_ticks = stepgen->step_len / period_ns ;
			send_cfg = 1 ;
		}

		if (new_timing || stepgen->step_space != stepgen->old_step_space) {
			stepgen->step_space = (stepgen->step_space == 0) ? period_ns : ulceil(stepgen->step_space, period_ns) ;
			stepgen->old_step_space = stepgen->step_space ;

			stepgen->step_space_ticks = stepgen->step_space / period_ns ;
			send_cfg = 1 ;
		}

		if (new_timing || stepgen->dir_setup != stepgen->old_dir_setup) {
			stepgen->dir_setup = ulceil(stepgen->dir_setup, period_ns) ;
			stepgen->old_dir_setup = stepgen->dir_setup ;

			stepgen->dir_setup_ticks = stepgen->dir_setup / period_ns ;
			send_cfg = 1 ;
		}

		if (new_timing || stepgen->dir_hold != stepgen->old_dir_hold) {
			stepgen->dir_hold = (stepgen->dir_hold + stepgen->dir_setup == 0) ? period_ns : ulceil(stepgen->dir_hold, period_ns) ;
			stepgen->old_dir_hold = stepgen->dir_hold ;

			stepgen->dir_hold_ticks = stepgen->dir_hold / period_ns ;
			send_cfg = 1 ;
		}

		if (send_cfg) {
			// Maximum step frequency
			min_period = stepgen->step_len + stepgen->step_space ;
			max_freq = 1.0 / (4 * min_period * 1e-9) ;

			stepgen->act_maxvel = stepgen->maxvel ;

			if (stepgen->act_maxvel <= 0.0) stepgen->act_maxvel = 0.0 ;
			else {
				// If requested velocity is higher than the maximal step frequency, limit velocity
				if((stepgen->act_maxvel * rtapi_fabs(stepgen->position_scale)) < max_freq) {
					max_freq = stepgen->act_maxvel * rtapi_fabs(stepgen->position_scale) ;
				}
			}

			stepgen->act_maxvel = max_freq ;

			/* set internal accel limit to its absolute max, which is
					   zero to full speed in one thread period */
			max_accl = max_freq * recip_dt;

			stepgen->act_maxaccel = stepgen->maxaccel ;

			/* check for user specified accel limit parameter */
			if (stepgen->act_maxaccel <= 0.0) stepgen->act_maxaccel = 0.0 ;
			else {
				/* parameter is non-zero, compare to max_accl */
				if ((stepgen->act_maxaccel * rtapi_fabs(stepgen->position_scale)) > max_accl) {
					/* parameter is too high, lower it */
					stepgen->act_maxaccel = max_accl / rtapi_fabs(stepgen->position_scale) ;
				} else {
					stepgen->act_maxaccel *= rtapi_fabs(stepgen->position_scale) ;
				}
			}

			rtapi_print_msg(RTAPI_MSG_INFO, "%s: Axis %d: maxvel=%f ,%f, maxaccl=%f, %f ", module_name, n, stepgen->maxvel, stepgen->act_maxvel, stepgen->maxaccel, stepgen->act_maxaccel) ;
		}
	
		stepgen++ ;
	}

	for (n = 0 ; n < ud->num_pwm ; n++) {
		// Check if PWM frequency changed
		if (pwmgen->old_frequency != pwmgen->pwm_frequency) {
			pwmgen->old_frequency = pwmgen->pwm_frequency ;
			send_cfg = 1 ;
		}

		pwmgen++ ;
	}

	// Transmit new configuration if changed
	if (send_cfg) {
		rtapi_print_msg(RTAPI_MSG_INFO, "%s: Updating configuration.\n", module_name) ;
		if(!configure_cmd(ud)) {
			*(ud->spi_error) = 1 ;
			return ;
		}
	}

	// Reset pointers for motion pass
	stepgen = ud->stepgen ;
	pwmgen = ud->pwmgen ;

	tx_buf[i++] = CMD_UPD ;

	// Place holder for bit flags
	tx_buf[i++] = 0 ;

	// Velocity/Position calculations pass
	for (n = 0 ; n < ud->num_axes ; n++) {
        if (*(stepgen->vel_cmds) == 1)
            stepgen->cmd_mode = VELOCITY ;
        else
            stepgen->cmd_mode = POSITION ;

		// Position mode
		if (stepgen->cmd_mode == POSITION) {
			/* calculate position command in counts */
			pos_cmd = *(stepgen->position_cmd) * stepgen->position_scale ;
			/* calculate velocity command in counts/sec */
			vel_cmd = (pos_cmd - stepgen->old_pos) * recip_dt ;
			stepgen->old_pos = pos_cmd ;

			if(vel_cmd > stepgen->act_maxvel)
				vel_cmd = stepgen->act_maxvel ;
			else if(vel_cmd < -stepgen->act_maxvel)
				vel_cmd = -stepgen->act_maxvel ;

			curr_pos = fixed2fp(stepgen->accum) ;
			match_time = rtapi_fabs(vel_cmd - stepgen->velocity) / stepgen->act_maxaccel ;

//			rtapi_print_msg(RTAPI_MSG_INFO, "%s: Axis %d: Position=%e, cmd=%e, vel=%e", module_name, n, curr_pos, pos_cmd, vel_cmd) ;

			est_out = curr_pos + (vel_cmd + stepgen->velocity) * 0.5 * match_time ;
			est_cmd = pos_cmd + vel_cmd * (match_time - 1.5 * dt) ;
			est_err = est_out - est_cmd ;

			if (match_time < dt) {
				if (rtapi_fabs(est_err) < 1e-4) new_vel = vel_cmd ;
				else {
					new_vel = vel_cmd - 0.5 * est_err * recip_dt ;

					if (new_vel > (stepgen->velocity + stepgen->act_maxaccel * dt))
						new_vel = stepgen->velocity + stepgen->act_maxaccel * dt ;
					else if (new_vel < (stepgen->velocity - stepgen->act_maxaccel * dt))
						new_vel = stepgen->velocity - stepgen->act_maxaccel * dt ;
				}
			} else {
				dir = sign(vel_cmd - stepgen->velocity) ;
				dp = -2.0 * dir * stepgen->act_maxaccel * dt * match_time ;

				if (rtapi_fabs(est_err + dp * 2.0) < rtapi_fabs(est_err))
					new_vel = stepgen->velocity - dir * stepgen->act_maxaccel * dt ;
				else
					new_vel = stepgen->velocity + dir * stepgen->act_maxaccel * dt ;
			}

		// Velocity mode
		} else {
			vel_cmd = *(stepgen->velocity_cmd) * stepgen->position_scale ;

			dv = stepgen->act_maxaccel * dt ;
			if (vel_cmd > stepgen->velocity + dv)
				new_vel = stepgen->velocity + dv ;
			else if (vel_cmd < stepgen->velocity - dv)
				new_vel = stepgen->velocity - dv ;
			else
				new_vel = vel_cmd ;
		}

		if (new_vel > stepgen->act_maxvel) new_vel = stepgen->act_maxvel ;
		else if (new_vel < -stepgen->act_maxvel) new_vel = -stepgen->act_maxvel ;

		stepgen->velocity = new_vel ;
		stepgen->target_inc = stepgen->velocity * velocity_scale ;

//		rtapi_print_msg(RTAPI_MSG_INFO, "%s: Velocity of %d is %llu %f, scale=%f", module_name, n, stepgen->target_inc, stepgen->velocity, velocity_scale) ;

		tx_buf[i++] = (unsigned int) (stepgen->target_inc >> 32) ;
		tx_buf[i++] = (unsigned int) stepgen->target_inc ;

		if (*(stepgen->enable)) tx_buf[1] |= stepgen->axis ;

		stepgen++ ;
	} // for (n = 0 ; n < num_axes ...

	// PWM output pass
	for (n = 0 ; n < ud->num_pwm ; n++) {
		// Flag PWM transmit if duty cycle changed on one of the channels
		if ((float) *(pwmgen->value) != pwmgen->old_value) {
			pwmgen->old_value = *(pwmgen->value) ;

			if (pwmgen->type == PWM_ESC)
				pwmgen->duty = pwmgen->enable ? 3.5 + 6.5 * (*(pwmgen->value) / pwmgen->pwm_scale) : 0.0 ;
			else
				pwmgen->duty = pwmgen->enable ? 100.0 * (*(pwmgen->value) / pwmgen->pwm_scale) : 0.0 ;
		}

		tx_buf[i++] = *(unsigned int *) &pwmgen->duty ;
        if(*(pwmgen->enable)) tx_buf[1] |= 1 << (16 + 2 * n) ;
        if(*(pwmgen->reverse)) tx_buf[1] |= 1 << (16 + 2 * n + 1) ;

		pwmgen++ ;
	} // for (n = 0 ; n < num_pwm ...

	// Add checksum
	checksum = 0 ;
	for (n = 0 ; n < i ; n++) checksum ^= tx_buf[n] ;
	tx_buf[i++] = checksum ;

	// If expected return data is bigger than data to send, pad the buffer
	for(; i < (ud->num_axes * 2 + 3) ; tx_buf[i++] = 0) ;

	if (!spi_xmit(i)) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: Failed to send new velocity data.\n", module_name) ;
                *(ud->spi_error) = 1 ;
	}

	// Get position update from the machine
	if(!update_pos(ud)) {
		*(ud->spi_error) = 1 ;
	}
}

// Read position accumulators from u-controllers
static int update_pos(void * arg)
{
	unsigned int n = 0, fp = 1, flags_raw = 0 ;
	unsigned int ret = 1 ;
	static unsigned int old_flags = 0 ;
	update_data_t * ud = arg ;
	stepgen_t * stepgen = ud->stepgen ;
    cnc_flags_t flags ;
    int64_t pos ;

    flags_raw = SWAP_BYTES(rx_buf[fp]) ;
    fp++ ;
    memcpy((void *)&flags, (void *)&flags_raw, FLAGS_LEN * sizeof(unsigned int)) ;
    if(flags_raw != old_flags) {
    	old_flags = flags_raw ;
    	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Controller flags %x.\n", module_name, flags_raw) ;
    }

    if (flags.ucont_fault) {
    	rtapi_print_msg(RTAPI_MSG_ERR, "%s: Microcontroller error reported.", module_name) ;
    	ret = 0 ;
    }

    if (flags.xsum_error) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: Command checksum error reported.", module_name) ;
        ret = 0 ;
    }

    *(ud->emo) = flags.switch_emo ;
    *(ud->z_level) = flags.z_level ;

	for (n = 0 ; n < ud->num_axes ; n++) {

		stepgen[n].ucont_old_pos = stepgen[n].ucont_pos ;
		pos = SWAP_BYTES(rx_buf[fp]) ;
		fp++ ;
		pos <<= 32 ;
		pos += SWAP_BYTES(rx_buf[fp]) ;
		fp++ ;
		stepgen[n].ucont_pos = pos ;
		stepgen[n].accum += (stepgen[n].ucont_pos - stepgen[n].ucont_old_pos) ;
		
		*(stepgen[n].count) = stepgen[n].accum >> PICKOFF ;
		*(stepgen[n].position_fb) = (double)(stepgen[n].accum - (1 << (PICKOFF - 1))) * stepgen->scale_inv ;
//		rtapi_print_msg(RTAPI_MSG_INFO, "%s: Position of %d is %llu, %e, %llu, %e", module_name, n, stepgen[n].accum, *(stepgen[n].position_fb), pos, stepgen->scale_inv) ;

		// Update switches
		switch (stepgen[n].axis) {
		case AXIS_X:
			*(stepgen[n].limit) = flags.switch_x_limit ;
			*(stepgen[n].home)  = flags.switch_x_home ;
			*(stepgen[n].fault) = flags.x_drv_fault ;
			break ;
		case AXIS_Y:
			*(stepgen[n].limit) = flags.switch_y_limit ;
			*(stepgen[n].home)  = flags.switch_y_home ;
			*(stepgen[n].fault) = flags.y_drv_fault ;
			break ;
		case AXIS_Z:
			*(stepgen[n].limit) = flags.switch_z_limit ;
			*(stepgen[n].home)  = flags.switch_z_home ;
			*(stepgen[n].fault) = flags.z_drv_fault ;
			break ;
		case AXIS_A:
			*(stepgen[n].limit) = flags.switch_a_limit ;
			*(stepgen[n].home)  = flags.switch_a_home ;
			*(stepgen[n].fault) = flags.a_drv_fault ;
			break ;
		case AXIS_B:
			*(stepgen[n].limit) = flags.switch_b_limit ;
			*(stepgen[n].home)  = flags.switch_b_home ;
			*(stepgen[n].fault) = flags.b_drv_fault ;
			break ;
		case AXIS_C:
			*(stepgen[n].limit) = flags.switch_c_limit ;
			*(stepgen[n].home)  = flags.switch_c_home ;
			*(stepgen[n].fault) = flags.c_drv_fault ;
			break ;
		case AXIS_E:
			*(stepgen[n].limit) = flags.switch_e_limit ;
			*(stepgen[n].home)  = flags.switch_e_home ;
			*(stepgen[n].fault) = flags.e_drv_fault ;
			break ;
		default:
			ret = 0 ;
			break ;
		}
	}

	return ret ;
}

// Send timing configuration to u-controller
static int configure_cmd(void * arg) {
    int n, i = 0 ;
    unsigned int checksum = 0 ;
    update_data_t * ud = arg ;
    stepgen_t * stepgen = ud->stepgen ;

    tx_buf[i++] = CMD_CFG ;
    tx_buf[i++] = base_freq ;
    tx_buf[i++] = ud->num_axes ;
    tx_buf[i++] = ud->num_pwm ;

    for (n = 0 ; n < ud->num_axes ; n++) {
    	tx_buf[i++] = (unsigned int)stepgen_array[n].axis ;
        tx_buf[i++] = stepgen_array[n].step_len_ticks ;
        tx_buf[i++] = stepgen_array[n].step_space_ticks ;
        tx_buf[i++] = stepgen_array[n].dir_setup_ticks ;
        tx_buf[i++] = stepgen_array[n].dir_hold_ticks ;
    }

    for (n = 0 ; n < ud->num_pwm ; n++) {
    	tx_buf[i++] = pwmgen_array[n].pwm_frequency ;
    }

    for(n = 0 ; n < i ; n++) checksum ^= tx_buf[n] ;
    tx_buf[i++] = checksum ;

    spi_xmit(++i) ;

/*  Ignore SPI error on config, for now. This will show up as an SPI error
 *  on the controller side, should it fail to receive.

    if (!spi_xmit(++i)) {
    	rtapi_print_msg(RTAPI_MSG_ERR, "%s: Failed to send configuration update to controller.\n", module_name) ;
    	return 0 ;
    }
*/

    return 1 ;
}

static int stop_machine() {
	unsigned int checksum = 0 ;

	tx_buf[0] = CMD_STP ;
	tx_buf[1] = 0 ;
	tx_buf[2] = 0 ;
	checksum ^= tx_buf[0] ;
	tx_buf[3] = checksum ;

	spi_xmit(4) ;

	return 1 ;
}

// Export axis parameters and flags
static int export_stepgen(int num, stepgen_t * addr, axis_name_t axis) {
	int n, ret ;

	addr->axis = axis ;

	ret = hal_pin_float_newf(HAL_IN, &(addr->position_cmd),
			comp_id, "%s.axis.%d.position-cmd", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->position_cmd) = 0.0 ;

	ret = hal_pin_float_newf(HAL_IN, &(addr->velocity_cmd),
		comp_id, "%s.axis.%d.velocity-cmd", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->velocity_cmd) = 0.0 ;

	ret = hal_pin_float_newf(HAL_OUT, &(addr->position_fb),
		comp_id, "%s.axis.%d.position-fb", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->position_fb) = 0.0 ;

	ret = hal_param_float_newf(HAL_RW, &(addr->position_scale),
		comp_id, "%s.axis.%d.position-scale", prefix, num) ;
	if (ret < 0) return ret ;

	addr->position_scale = 1.0 ;

	ret = hal_param_float_newf(HAL_RW, &(addr->maxvel),
		comp_id, "%s.axis.%d.maxvel", prefix, num) ;
	if (ret < 0) return ret ;

	addr->maxvel = 1.0 ;

	ret = hal_param_float_newf(HAL_RW, &(addr->maxaccel),
		comp_id, "%s.axis.%d.maxaccel", prefix, num) ;
	if (ret < 0) return ret ;

	addr->maxaccel = 1.0 ;

	ret = hal_pin_s32_newf(HAL_OUT, &(addr->count),
		comp_id, "%s.axis.%d.counts", prefix, num) ;
	if (ret != 0) return ret ;

	*(addr->count) = 0 ;

	// Step timing parameters
	ret = hal_param_u32_newf(HAL_RW, &(addr->step_len),
			comp_id, "%s.axis.%d.step_len", prefix, num) ;
	if (ret < 0) return ret ;

	addr->step_len = 1 ;

	ret = hal_param_u32_newf(HAL_RW, &(addr->step_space),
				comp_id, "%s.axis.%d.step_space", prefix, num) ;
	if (ret < 0) return ret ;

	addr->step_space = 1 ;

	ret = hal_param_u32_newf(HAL_RW, &(addr->dir_setup),
				comp_id, "%s.axis.%d.dir_setup", prefix, num) ;
	if (ret < 0) return ret ;

	addr->dir_setup = 1 ;

	ret = hal_param_u32_newf(HAL_RW, &(addr->dir_hold),
					comp_id, "%s.axis.%d.dir_hold", prefix, num) ;
	if (ret < 0) return ret ;

	addr->dir_hold = 1 ;

	// Pins
	ret = hal_pin_bit_newf(HAL_OUT, &(addr->home),
		comp_id, "%s.axis.%d.home", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->home) = 0 ;

	ret = hal_pin_bit_newf(HAL_OUT, &(addr->limit),
		comp_id, "%s.axis.%d.limit", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->limit) = 0 ;

	ret = hal_pin_bit_newf(HAL_OUT, &(addr->fault),
			comp_id, "%s.axis.%d.fault", prefix, num) ;
		if (ret < 0) return ret ;

	*(addr->fault) = 0 ;

	ret = hal_pin_bit_newf(HAL_IN, &(addr->enable),
		comp_id, "%s.axis.%d.enable", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->enable) = 0 ;
    
    ret = hal_pin_bit_newf(HAL_IN, &(addr->vel_cmds),
		comp_id, "%s.axis.%d.vel_cmds", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->vel_cmds) = 0 ;

	addr->old_scale = 0.0 ;
	addr->ucont_old_pos = 0 ;
	addr->ucont_pos = 0 ;
	addr->velocity = 0.0 ;
	addr->old_pos = 0.0 ;

	addr->act_maxaccel = 0.0 ;
	addr->act_maxvel = 0.0 ;

	addr->cmd_mode = POSITION ;

	return 0 ;
}

// Export PWM parameters and flags
static int export_pwmgen(int num, pwmgen_t * addr) {
	int ret ;

	ret = hal_pin_float_newf(HAL_IN, &(addr->value),
		comp_id, "%s.pwm.%d.value", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->value) = 0.0 ;

	ret = hal_param_float_newf(HAL_RW, &(addr->pwm_scale),
		comp_id, "%s.pwm.%d.scale", prefix, num) ;
	if (ret < 0) return ret ;

	addr->pwm_scale = 1.0 ;

	ret = hal_param_u32_newf(HAL_RW, &(addr->pwm_frequency),
		comp_id, "%s.pwm.%d.frequency", prefix, num) ;
	if (ret < 0) return ret ;

	addr->pwm_frequency = 0 ;

	ret = hal_pin_bit_newf(HAL_IN, &(addr->enable),
		comp_id, "%s.pwm.%d.enable", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->enable) = 0 ;
    
    ret = hal_pin_bit_newf(HAL_IN, &(addr->forward),
		comp_id, "%s.pwm.%d.forward", prefix, num) ;
	if (ret < 0) return ret ;

	*(addr->forward) = 0 ;

	ret = hal_pin_bit_newf(HAL_IN, &(addr->reverse),
		comp_id, "%s.pwm.%d.reverse", prefix, num) ;
	if (ret < 0) return ret ;

	ret = hal_param_u32_newf(HAL_RW, &(addr->type),
		comp_id, "%s.pwm.%d.type", prefix, num) ;
	if (ret < 0) return ret ;

	addr->type = 0 ;

	*(addr->reverse) = 0 ;

    addr->old_value = 0.0 ;
    addr->duty = 0.0 ;
    
	return 0 ;
}

// SPI transactions thread
void * bcm_spi_thread(void * wr_info) {
	spi_wr_info_t  * info ;
	struct sched_param s_param ;

	info = (spi_wr_info_t *) wr_info ;

	// A semaphore to signal the start of SPI transfer
	if (sem_init(&info->start, 0, 0) != 0) {
		printf("Failed sem_init() of start\n") ;
		pthread_exit(NULL) ;
	}

	// A semaphore to signal when SPI transfer is done
	if (sem_init(&info->done, 0, 0) != 0) {
		printf("Failed sem_init() of done\n") ;
		pthread_exit(NULL) ;
	}

	// Set thread priority in the scheduler
	s_param.__sched_priority = SPI_PRIORITY ;
	if(sched_setscheduler(0, SCHED_FIFO, &s_param) == -1) {
		printf("sched_setscheduler failed!") ;
		pthread_exit(NULL) ;
	}

	info->frame_wait = 0 ;

	// Loop for as long as the program is running and wait for transactions
	while (info->run) {
		// Wait for 'start' signal from main thread
		sem_wait(&info->start) ;

		// Drop CS
		GPIO_CLR = (1 << 8) ;

		// Start SPI transaction
		info->ret = bcm_spi_RW(info->len, info->tx_data, info->rx_data, info->frame_wait) ;

		// Rise CS
		GPIO_SET = (1 << 8) ;
		info->frame_wait = 0 ;

		// Signal 'done' to main thread
		sem_post(&info->done) ;
	}

	// Destroy semaphores and exit thread
	rtapi_print_msg(RTAPI_MSG_INFO, "Exiting SPI thread") ;
	sem_destroy(&info->start) ;
	sem_destroy(&info->done) ;
	pthread_exit(NULL) ;
}

// SPI Read/Write
int bcm_spi_RW(int len, unsigned char *tx_data, unsigned char *rx_data, int wait) {
	struct spi_ioc_transfer spi = {0} ;

	spi.tx_buf = (unsigned long)(tx_data) ;
	spi.rx_buf = (unsigned long)(rx_data) ;
	spi.len = len ;
//	spi.delay_usecs = 0 ;
	spi.bits_per_word = 8 ;
	spi.speed_hz = spi_speed ;
//	spi.cs_change = 0 ;

	return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi) ;
}

// SPI Transmit and Receive
static int spi_xmit(unsigned int len) {
	int i ;
	unsigned int r_chk = 0 ;

	spi_info.len = sizeof(unsigned int) * (len) ;

	sem_post(&spi_info.start) ;
	sem_wait(&spi_info.done) ;

	if (spi_info.len != spi_info.ret) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: SPI receive size error expected %x, received %x.\n", module_name, spi_info.len, spi_info.ret) ;
		return 0 ;
	}

	if (debug)
		for (i = 0 ; i < len ; i++)
			rtapi_print_msg(RTAPI_MSG_INFO, "%s: Word 0x%02x : -> 0x%08x | <- 0x%08x\n", module_name, i, tx_buf[i], rx_buf[i]) ;

	for(i = 1 ; i < len ; i++) r_chk ^= rx_buf[i] ;

	if (r_chk) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: SPI receive checksum error, got %x.\n", module_name, r_chk) ;

		for (i = 0 ; i < len ; i++)
			rtapi_print_msg(RTAPI_MSG_INFO, "%s: Word 0x%02x : -> 0x%08x | <- 0x%08x\n", module_name, i, tx_buf[i], rx_buf[i]) ;

		return 0 ;
	}

	return 1 ;
}

// Raspberry Pi SPI setup
int bcm_spi_setup(unsigned int clk, unsigned int flags) {
	int bpw = 8 ;
	FILE *fp ;
	int mem_fd = -1 ;

	mem_fd = open(SPIDEV_0, O_RDWR) ;
	if (mem_fd < 0) return -5 ;

	spi_fd = mem_fd ;

	spi_speed = clk ;

	if(ioctl(spi_fd, SPI_IOC_WR_MODE, &flags) < 0) return errno ;
	if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) return errno ;
	if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &clk) < 0) return errno ;

	return 0 ;
}

// Raspberry Pi SPI close
void bcm_close(void) {
	close(spi_fd) ;
}

// Raspberry Pi GPIO setup
int rpi_gpio_setup() {
	int mem_fd ;
	void *gpio_map ;

	mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ;
	if (mem_fd < 0) return -1 ;

	gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, BCM2708_PERI_BASE + GPIO_BASE) ;

	close(mem_fd) ;

	if (gpio_map == MAP_FAILED) return -2 ;

	gpio = (volatile unsigned *) gpio_map ;
	return 0 ;
}

// Parse axis lable to enumerated type
static axis_name_t parse_axis(const char * axis) {
	if (!axis) return INVALID ;
	if (*axis == 'x' || *axis == 'X') return AXIS_X ;
	if (*axis == 'y' || *axis == 'Y') return AXIS_Y ;
	if (*axis == 'z' || *axis == 'Z') return AXIS_Z ;
	if (*axis == 'a' || *axis == 'A') return AXIS_A ;
	if (*axis == 'b' || *axis == 'B') return AXIS_B ;
	if (*axis == 'c' || *axis == 'C') return AXIS_C ;
	if (*axis == 'e' || *axis == 'E') return AXIS_E ;
	return INVALID ;

}
