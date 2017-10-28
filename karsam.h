/* karsam.h
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
 **/

#ifndef KARSAM
#define KARSAM

#include "hal.h"
#include <stdint.h>

// ucontroller flags

typedef struct {
    unsigned int switch_x_home :1 ;     // bit 1
    unsigned int switch_x_limit :1 ;    // bit 2
    unsigned int switch_y_home :1 ;     // bit 3
    unsigned int switch_y_limit :1 ;    // bit 4
    unsigned int switch_z_home :1 ;     // bit 5
    unsigned int switch_z_limit :1 ;    // bit 6
    unsigned int switch_a_home :1 ;     // bit 7
    unsigned int switch_a_limit :1 ;    // bit 8
    unsigned int switch_b_home :1 ;     // bit 9
    unsigned int switch_b_limit :1 ;    // bit 10
    unsigned int switch_c_home :1 ;     // bit 11
    unsigned int switch_c_limit :1 ;    // bit 12
    unsigned int switch_e_home :1 ;     // bit 13
    unsigned int switch_e_limit :1 ;    // bit 14
    unsigned int z_level :1 ;           // bit 15
    unsigned int reserved_sw :6 ;       // bits 16-21
    unsigned int xsum_error:1 ;         // bit 22
    unsigned int switch_emo :1 ;	    // bit 23
    unsigned int ucont_fault :1 ;       // bit 24
    unsigned int x_drv_fault :1 ;	    // bit 25
    unsigned int y_drv_fault :1 ;	    // bit 26
    unsigned int z_drv_fault :1 ;	    // bit 27
    unsigned int a_drv_fault :1 ;	    // bit 28
    unsigned int b_drv_fault :1 ;	    // bit 29
    unsigned int c_drv_fault :1 ;	    // bit 30
    unsigned int e_drv_fault :1 ;	    // bit 31
    unsigned int reserved :1 ;          // bit 32
} cnc_flags_t ;

#define FLAGS_LEN		1

typedef enum  {
	AXIS_X =  1,
	AXIS_Y =  2,
	AXIS_Z =  4,
	AXIS_A =  8,
	AXIS_B = 16,
	AXIS_C = 32,
	AXIS_E = 64,
	INVALID = 128,
} axis_name_t ;

enum cmd_mode_names {
	POSITION = 0,
	VELOCITY
} ;

enum pin_output_names {
	SPINDLE_ENABLE,
	LASER_ENBALE
} ;

enum pin_pwm_names {
	SPINDLE_SPEED,
	LASER_POWER
} ;

typedef enum {
	UNSUPPORTED,
	RPI,
	RPI_2
} platform_t ;

typedef struct {
	axis_name_t axis ;
	hal_bit_t *enable ;
	hal_bit_t *home ;
	hal_bit_t *limit ;
	hal_bit_t *fault ;
    hal_float_t *position_cmd ;
	hal_float_t *velocity_cmd ;
	hal_float_t *position_fb ;
	hal_float_t position_scale ;
	hal_float_t velocity ;
    int cmd_mode ;

    hal_s32_t *count ;
	
	hal_u32_t step_len ;
	hal_u32_t step_space ;
	hal_u32_t dir_setup ;
	hal_u32_t dir_hold ;
	
	uint32_t step_len_ticks ;
	uint32_t step_space_ticks ;
	uint32_t dir_setup_ticks ;
	uint32_t dir_hold_ticks ;

	double maxvel ;
	double maxaccel ;
	
	volatile long long int accum ;
	int32_t target_inc ;

	double scale_inv ;

	double old_vel ;
	double old_pos ;
	double old_scale ;

	unsigned int old_step_len ;
	unsigned int old_step_space ;
	unsigned int old_dir_setup ;
	unsigned int old_dir_hold ;

	int64_t ucont_pos ;
	int64_t ucont_old_pos ;
} stepgen_t ;

typedef struct {
	hal_float_t *pwm_duty ;
	hal_float_t pwm_scale ;
	hal_u32_t pwm_frequency ;
	hal_bit_t *enable ;
	hal_bit_t *reverse ;

	float old_duty ;
	unsigned int old_frequency ;
} pwmgen_t ;

typedef struct {
	hal_bit_t *z_level ;
	hal_bit_t *emo ;

	hal_bit_t *ready ;
	hal_bit_t *spi_error ;

	hal_u32_t basefreq ;

	stepgen_t * stepgen ;
	pwmgen_t * pwmgen ;

	unsigned int num_axes ;
	unsigned int num_pwm ;
} update_data_t ;

#define SPIDEV_0           "/dev/spidev0.0"
#define SPIDEV_1           "/dev/spidev0.1"

typedef struct {
	int len ;
	unsigned char *tx_data ;
	unsigned char *rx_data ;
	int ret ;

	int run ;
	int frame_wait ;
	sem_t start ;
	sem_t done  ;
} spi_wr_info_t ;

#define error_msg(msg) rtapi_print_msg(RTAPI_MSG_ERR, msg, module_name)

#define MAX_AXES	5
#define MAX_PWM		2

#define PICKOFF		(28)

#define BASEFREQ	(80000ul)

#define REQ_TIMEOUT		(10000ul)
#define SPIBUFSIZE		(100)
#define SPI_RESP_DELAY  20

#define SPI_CLKDIV	32		// Set SPI to 8MHz (16 - 16MHz)
#define SPI_PRIORITY 49

#define PERIODFP 		((double)1.0 / (double)(BASEFREQ))
#define VELSCALE		((double)PICKOFF * PERIODFP)
#define ACCELSCALE		(VELSCALE * PERIODFP)

// Commands
#define CMD_CFG     0x47464323    // #CFG
#define CMD_UPD     0x44505523    // #UPD
#define CMD_STP     0x50545323    // #STP
#define CMD_CHK     0x4b484323    // #CHK

#define ulceil(val, inc)	(inc * (1 + (val - 1) / inc))
#define fixed2fp(val)		(double)((val - (1L << (PICKOFF - 1))) * (1.0 / (1L << PICKOFF)))
#define sign(val)			(int)(val >= 0) ? 1 : -1

#define SWAP_BYTES(IN)  IN << 24 | (IN & 0xff00) << 8 | (IN & 0xff0000) >> 8 | IN >> 24

static void update(void * arg, long period) ;
static int update_pos(void * arg) ;
static axis_name_t parse_axis(const char * axis) ;
static int export_stepgen(int num, stepgen_t * addr, axis_name_t axis) ;
static int export_pwmgen(int num, pwmgen_t * addr) ;
static int configure_cmd(void * arg) ;
static int stop_machine() ;

static int spi_xmit(unsigned int len) ;
static int spi_rcv_cmd(unsigned int cmd, unsigned int rcv_words) ;
static int spi_txm_cmd(unsigned int tx_words) ;

int bcm_spi_RW(int len, unsigned char *tx_data, unsigned char *rx_data, int wait) ;
int bcm_spi_setup(unsigned int clk, unsigned int flags) ;
void *bcm_spi_thread(void * wr_info) ;
void bcm_close() ;

int rpi_gpio_setup() ;

#define BCM2708_PERI_BASE        0x3f000000 	// For RPi 1: 0x20000000
#define GPIO_BASE				 0x00200000

#define PAGE_SIZE				(2 * 1024)
#define BLOCK_SIZE			    (2 * 1024)

// I/O access
volatile unsigned * gpio ;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3)) ; *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

#endif
