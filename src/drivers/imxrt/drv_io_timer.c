/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file drv_io_timer.c
 *
 * Servo driver supporting PWM servos connected to imxrt FLEXPWM blocks.
 */

#include <px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include "drv_io_timer.h"

#include <chip.h>
#include "chip/imxrt_flexpwm.h"

#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define FLEXPWM_SRC_CLOCK_FREQ 16000000

#define MAX_CHANNELS_PER_TIMER 2

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)


/* Timer register accessors */

#define rCNT(_tim)        REG(_tim,IMXRT_FLEXPWM_SM0CNT_OFFSET)        /* Counter Register */
#define rINIT(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* Initial Count Register */
#define rCTRL2(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* Control 2 Register */
#define rCTRL(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* Control Register */
#define rVAL0(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* Value Register 0 */
#define rFRACVAL1(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET)   /* Fractional Value Register 1 */
#define rVAL1(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* Value Register 1 */
#define rFRACVAL2(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET)   /* Fractional Value Register 2 */
#define rVAL2(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* Value Register 2 */
#define rFRACVAL3(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET)   /* Fractional Value Register 3 */
#define rVAL3(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* Value Register 3 */
#define rFRACVAL4(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET)   /* Fractional Value Register 4 */
#define rVAL4(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* Value Register 4 */
#define rFRACVAL5(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET)   /* Fractional Value Register 5 */
#define rVAL5(_tim)       REG(_tim,IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* Value Register 5 */
#define rFRCTRL(_tim)     REG(_tim,IMXRT_FLEXPWM_SM0FRCTRL_OFFSET)     /* Fractional Control Register */
#define rOCTRL(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0OCTRL_OFFSET)      /* Output Control Register */
#define rSTS(_tim)        REG(_tim,IMXRT_FLEXPWM_SM0STS_OFFSET)        /* Status Register */
#define rINTEN(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0INTEN_OFFSET)      /* Interrupt Enable Register */
#define rDMAEN(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0DMAEN_OFFSET)      /* DMA Enable Register */
#define rTCTRL(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0TCTRL_OFFSET)      /* Output Trigger Control Register */
#define rDISMAP0(_tim)    REG(_tim,IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* Fault Disable Mapping Register 0 */
#define rDISMAP1(_tim)    REG(_tim,IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* Fault Disable Mapping Register 1 */
#define rDTCNT0(_tim)     REG(_tim,IMXRT_FLEXPWM_SM0DTCNT0_OFFSET)     /* Deadtime Count Register 0 */
#define rDTCNT1(_tim)     REG(_tim,IMXRT_FLEXPWM_SM0DTCNT1_OFFSET)     /* Deadtime Count Register 1 */
#define rCAPTCTRLA(_tim)  REG(_tim,IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET)  /* Capture Control A Register */
#define rCAPTCOMPA(_tim)  REG(_tim,IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET)  /* Capture Compare A Register */
#define rCAPTCTRLB(_tim)  REG(_tim,IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET)  /* Capture Control B Register */
#define rCAPTCOMPB(_tim)  REG(_tim,IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET)  /* Capture Compare B Register */
#define rCAPTCTRLX(_tim)  REG(_tim,IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET)  /* Capture Control X Register */
#define rCAPTCOMPX(_tim)  REG(_tim,IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET)  /* Capture Compare X Register */
#define rCVAL0(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CVAL0_OFFSET)      /* Capture Value 0 Register */
#define rCVAL0CYC(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET)   /* Capture Value 0 Cycle Register */
#define rCVAL1(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CVAL1_OFFSET)      /* Capture Value 1 Register */
#define rCVAL1CYC(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET)   /* Capture Value 1 Cycle Register */
#define rCVAL2(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CVAL2_OFFSET)      /* Capture Value 2 Register */
#define rCVAL2CYC(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET)   /* Capture Value 2 Cycle Register */
#define rCVAL3(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CVAL3_OFFSET)      /* Capture Value 3 Register */
#define rCVAL3CYC(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET)   /* Capture Value 3 Cycle Register */
#define rCVAL4(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CVAL4_OFFSET)      /* Capture Value 4 Register */
#define rCVAL4CYC(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET)   /* Capture Value 4 Cycle Register */
#define rCVAL5(_tim)      REG(_tim,IMXRT_FLEXPWM_SM0CVAL5_OFFSET)      /* Capture Value 5 Register */
#define rCVAL5CYC(_tim)   REG(_tim,IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET)   /* Capture Value 5 Cycle Register */


#define IMXRT_FLEXPWM_SM0CVAL_OFFSET(n) (IMXRT_FLEXPWM_SM0CVAL0_OFFSET+ ((n) * 2*sizeof(uint32_t)))

#define CnSC_RESET          0
#define CnSC_CAPTURE_INIT   0 // Both

#if defined(BOARD_PWM_DRIVE_ACTIVE_LOW)
#define CnSC_PWMOUT_INIT     0
#else
#define CnSC_PWMOUT_INIT    0
#endif

//												 				  NotUsed   PWMOut  PWMIn Capture OneShot Trigger
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX,   0,  0,  0, 0, 0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_allocation_t once = 0;

typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];


static int io_timer_handler(uint16_t timer_index)
{
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending */

	uint32_t statusr = 0;// rSTATUS(timer_index);

	/* Acknowledge all that are pending */

	//rSTATUS(timer_index) = 0;

	/* Iterate over the timer_io_channels table */

	for (unsigned chan_index = tmr->first_channel_index; chan_index <= tmr->last_channel_index; chan_index++) {

		uint16_t chan = 1 << chan_index;

		if (statusr & chan) {

			io_timer_channel_stats[chan_index].isr_cout++;

			/* Call the client to read the rCnV etc and clear the CHnF */

			if (channel_handlers[chan_index].callback) {
				channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
								      chan_index, &timer_io_channels[chan_index],
								      now, count, _REG32(tmr->base, IMXRT_FLEXPWM_SM0CVAL_OFFSET(chan_index)));
			}
		}

		/* Did it set again during call out ?*/

		if (0/*rSTATUS(timer_index) & chan*/) {

			/* Error we has a second edge before we serviced the fist */

			io_timer_channel_stats[chan_index].overflows++;
		}
	}

	return 0;
}

int io_timer_handler0(int irq, void *context, void *arg)
{

	return io_timer_handler(0);
}

int io_timer_handler1(int irq, void *context, void *arg)
{
	return io_timer_handler(1);

}

int io_timer_handler2(int irq, void *context, void *arg)
{
	return io_timer_handler(2);

}

int io_timer_handler3(int irq, void *context, void *arg)
{
	return io_timer_handler(3);

}

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

static inline int is_timer_uninitalized(unsigned timer)
{
	int rv = 0;

	if (once & 1 << timer) {
		rv = -EBUSY;
	}

	return rv;
}

static inline void set_timer_initalized(unsigned timer)
{
	once |= 1 << timer;
}

static inline void set_timer_deinitalized(unsigned timer)
{
	once &= ~(1 << timer);
}

static inline int channels_timer(unsigned channel)
{
	return timer_io_channels[channel].timer_index;
}

static inline int get_timers_firstchannels(unsigned timer)
{
	int channel = -1;

	if (validate_timer_index(timer) == 0) {
		channel = timer_io_channels[io_timers[timer].first_channel_index].timer_channel;
	}

	return channel;
}

static uint32_t get_timer_channels(unsigned timer)
{
	uint32_t channels = 0;
	static uint32_t channels_cache[MAX_IO_TIMERS] = {0};

	if (validate_timer_index(timer) < 0) {
		return channels;

	} else {
		if (channels_cache[timer] == 0) {
			const io_timers_t *tmr = &io_timers[timer];

			/* Gather the channel bits that belong to the timer */

			for (unsigned chan_index = tmr->first_channel_index; chan_index <= tmr->last_channel_index; chan_index++) {
				channels |= 1 << chan_index;
			}

			/* cache them */

			channels_cache[timer] = channels;
		}
	}

	return channels_cache[timer];
}

static inline int is_channels_timer_uninitalized(unsigned channel)
{
	return is_timer_uninitalized(channels_timer(channel));
}

int io_timer_is_channel_free(unsigned channel)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		if (0 == (channel_allocations[IOTimerChanMode_NotUsed] & (1 << channel))) {
			rv = -EBUSY;
		}
	}

	return rv;
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	if (channel < MAX_TIMER_IO_CHANNELS && timer_io_channels[channel].timer_channel != 0) {

		unsigned timer = timer_io_channels[channel].timer_index;

		/* test timer for validity */

		if ((io_timers[timer].base != 0) &&
		    (timer_io_channels[channel].gpio_out != 0) &&
		    (timer_io_channels[channel].gpio_in != 0)) {
			rv = 0;
		}
	}

	return rv;
}

int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	if (mode < IOTimerChanModeSize) {
		return channel_allocations[mode];
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	io_timer_channel_allocation_t bit = 1 << channel;

	for (int mode = IOTimerChanMode_NotUsed; mode < IOTimerChanModeSize; mode++) {
		if (bit & channel_allocations[mode]) {
			return mode;
		}
	}

	return -1;
}

static int reallocate_channel_resources(uint32_t channels, io_timer_channel_mode_t mode,
					io_timer_channel_mode_t new_mode)
{
	/* If caller mode is not based on current setting adjust it */

	if ((channels & channel_allocations[IOTimerChanMode_NotUsed]) == channels) {
		mode = IOTimerChanMode_NotUsed;
	}

	/* Remove old set of channels from original */

	channel_allocations[mode] &= ~channels;

	/* Will this change ?*/

	uint32_t before = channel_allocations[new_mode] & channels;

	/* add in the new set */

	channel_allocations[new_mode] |= channels;

	/* Indicate a mode change */

	return before ^ channels;
}

static inline int allocate_channel_resource(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = io_timer_is_channel_free(channel);

	if (rv == 0) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
	}

	return rv;
}


static inline int free_channel_resource(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

int io_timer_free_channel(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_set_enable(false, mode, 1 << channel);
		free_channel_resource(channel);

	}

	return 0;
}


static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = allocate_channel_resource(channel, mode);
		}
	}

	return rv;
}

static int timer_set_rate(unsigned timer, unsigned rate)
{

	irqstate_t flags = px4_enter_critical_section();
#if 0
	uint32_t save = rSC(timer);
	rSC(timer) = save & ~(FTM_SC_CLKS_MASK);

	/* configure the timer to update at the desired rate */
	rMOD(timer) = (BOARD_PWM_FREQ / rate) - 1;
	rSC(timer) = save;

	/* generate an update event; reloads the counter and all registers */
	rSYNC(timer) = FTM_SYNC;
#endif
	px4_leave_critical_section(flags);

	return 0;
}

static inline uint32_t div2psc(int div)
{
	return 31 - __builtin_clz(div);
}

static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas The FTM anly support onshot capture)
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 4.68 Ms.
	 */

	irqstate_t flags = px4_enter_critical_section();
#if 0
	rSC(timer) &= ~(FTM_SC_CLKS_MASK | FTM_SC_PS_MASK);
	rMOD(timer) = 0xffff;
	rSC(timer) |= (FTM_SC_CLKS_EXTCLK | div2psc(FLEXPWM_SRC_CLOCK_FREQ / BOARD_ONESHOT_FREQ));
#endif
	px4_leave_critical_section(flags);
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	irqstate_t flags = px4_enter_critical_section();
#if 0
	rSC(timer) &= ~(FTM_SC_CLKS_MASK | FTM_SC_PS_MASK);
	rSC(timer) |= (FTM_SC_CLKS_EXTCLK | div2psc(FLEXPWM_SRC_CLOCK_FREQ / BOARD_PWM_FREQ));
#endif
	px4_leave_critical_section(flags);
}

void io_timer_trigger(void)
{
	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot);
	uint32_t action_cache[MAX_IO_TIMERS] = {0};
	int actions = 0;

	/* Pre-calculate the list of timers to Trigger */

	for (int timer = 0; timer < MAX_IO_TIMERS; timer++) {
		if (validate_timer_index(timer) == 0) {
			int channels = get_timer_channels(timer);

			if (oneshots & channels) {
				action_cache[actions++] = io_timers[timer].base;
			}
		}
	}

	/* Now do them all wit the shortest delay in between */

	irqstate_t flags = px4_enter_critical_section();

	for (actions = 0; action_cache[actions] != 0 &&  actions < MAX_IO_TIMERS; actions++) {
		//_REG32(action_cache[actions], KINETIS_FTM_SYNC_OFFSET) |= FTM_SYNC;
	}

	px4_leave_critical_section(flags);
}

int io_timer_init_timer(unsigned timer)
{
	/* Do this only once per timer */

	int rv = is_timer_uninitalized(timer);

	if (rv == 0) {

		irqstate_t flags = px4_enter_critical_section();

		set_timer_initalized(timer);

		/* enable the timer clock before we try to talk to it */
#if 0
		uint32_t regval = _REG(io_timers[timer].clock_register);
		regval |= io_timers[timer].clock_bit;
		_REG(io_timers[timer].clock_register) = regval;

		/* disable and configure the timer */

		rSC(timer)    = FTM_SC_CLKS_NONE;
		rCNT(timer)   = 0;

		rMODE(timer) = 0;
		rSYNCONF(timer)   = (FTM_SYNCONF_SYNCMODE | FTM_SYNCONF_SWWRBUF | FTM_SYNCONF_SWRSTCNT);

		/* Set to run in debug mode */

		rCONF(timer)   |= FTM_CONF_BDMMODE_MASK;

		/* enable the timer */
#endif
		io_timer_set_PWM_mode(timer);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		timer_set_rate(timer, 50);

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */

		irq_attach(io_timers[timer].vectorno, io_timers[timer].handler, NULL);

		up_enable_irq(io_timers[timer].vectorno);

		px4_leave_critical_section(flags);
	}

	return rv;
}


int io_timer_set_rate(unsigned timer, unsigned rate)
{
	int rv = EBUSY;

	/* Get the channel bits that belong to the timer */

	uint32_t channels = get_timer_channels(timer);

	/* Check that all channels are either in PWM or Oneshot */

	if ((channels & (channel_allocations[IOTimerChanMode_PWMOut] |
			 channel_allocations[IOTimerChanMode_OneShot] |
			 channel_allocations[IOTimerChanMode_NotUsed])) ==
	    channels) {

		/* Change only a timer that is owned by pwm or one shot */

		/* Request to use OneShot ?*/

		if (rate == 0) {

			/* Request to use OneShot
			 *
			 * We are here because ALL these channels were either PWM or Oneshot
			 * Now they need to be Oneshot
			 */

			/* Did the allocation change */
			if (reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot)) {
				io_timer_set_oneshot_mode(timer);
			}

		} else {

			/* Request to use PWM
			 *
			 * We are here because  ALL these channels were either PWM or Oneshot
			 * Now they need to be PWM
			 */

			if (reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut)) {
				io_timer_set_PWM_mode(timer);
			}

			timer_set_rate(timer, rate);
		}

		rv = OK;
	}

	return rv;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
	uint32_t gpio = timer_io_channels[channel].gpio_in;
	uint32_t clearbits = CnSC_RESET;
	uint32_t setbits = CnSC_CAPTURE_INIT;

	/* figure out the GPIO config first */

	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		setbits = CnSC_PWMOUT_INIT;
		gpio = 0;
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		break;

	case IOTimerChanMode_NotUsed:
		setbits = 0;
		break;

	default:
		return -EINVAL;
	}

	int rv = allocate_channel(channel, mode);

	/* Valid channel should now be reserved in new mode */

	if (rv >= 0) {

		/* Blindly try to initialize the timer - it will only do it once */

		io_timer_init_timer(channels_timer(channel));

		irqstate_t flags = px4_enter_critical_section();

		/* Set up IO */
		if (gpio) {
			px4_arch_configgpio(gpio);
		}


		unsigned timer = channels_timer(channel);

		/* configure the channel */

		uint32_t chan = timer_io_channels[channel].timer_channel - 1;

		uint16_t rvalue = 0;//;REG(timer, KINETIS_FTM_CSC_OFFSET(chan));
		rvalue &= ~clearbits;
		rvalue |=  setbits;

		UNUSED(timer);
		UNUSED(chan);
		//REG(timer, KINETIS_FTM_CSC_OFFSET(chan)) = rvalue;

		//if (gpio){ TODO: NEEDED?
		//REG(timer, KINETIS_FTM_CV_OFFSET(0)) = 0;

		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
		px4_leave_critical_section(flags);
	}

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	typedef struct action_cache_rp_t {
		uint32_t cnsc_offset;
		uint32_t cnsc_value;
		uint32_t gpio;
	} action_cache_rp_t;
	struct action_cache_t {
		uint32_t base;
		uint32_t index;
		action_cache_rp_t cnsc[MAX_CHANNELS_PER_TIMER];
	} action_cache[MAX_IO_TIMERS];

	memset(action_cache, 0, sizeof(action_cache));

	uint32_t bits =  state ? CnSC_PWMOUT_INIT : 0;

	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_Trigger:
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		if (state) {
			bits |= 0;//FTM_CSC_CHIE;
		}

		break;

	default:
		return -EINVAL;
	}

	/* Was the request for all channels in this mode ?*/

	if (masks == IO_TIMER_ALL_MODES_CHANNELS) {

		/* Yes - we provide them */

		masks = channel_allocations[mode];

	} else {

		/* No - caller provided mask */

		/* Only allow the channels in that mode to be affected */

		masks &= channel_allocations[mode];

	}

	/* Pre calculate all the changes */

	for (int chan_index = 0; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS; chan_index++) {
		if (masks & (1 << chan_index)) {
			masks &= ~(1 << chan_index);

			if (io_timer_validate_channel_index(chan_index) == 0) {
				uint32_t chan = timer_io_channels[chan_index].timer_channel - 1;
				uint32_t timer = channels_timer(chan_index);
				action_cache[timer].base  = io_timers[timer].base;
//				action_cache[timer].cnsc[action_cache[timer].index].cnsc_offset = io_timers[timer].base + KINETIS_FTM_CSC_OFFSET(chan);
				UNUSED(chan);
				action_cache[timer].cnsc[action_cache[timer].index].cnsc_value = bits;

				if ((state &&
				     (mode == IOTimerChanMode_PWMOut ||
				      mode == IOTimerChanMode_OneShot ||
				      mode == IOTimerChanMode_Trigger))) {
					action_cache[timer].cnsc[action_cache[timer].index].gpio = timer_io_channels[chan_index].gpio_out;
				}

				action_cache[timer].index++;
			}
		}
	}

	irqstate_t flags = px4_enter_critical_section();


	for (unsigned actions = 0; actions < arraySize(action_cache); actions++) {
		uint32_t any_on = 0;

		if (action_cache[actions].base != 0) {
			for (unsigned int index = 0; index < action_cache[actions].index; index++) {

				if (action_cache[actions].cnsc[index].gpio) {
					px4_arch_configgpio(action_cache[actions].cnsc[index].gpio);
				}

				_REG(action_cache[actions].cnsc[index].cnsc_offset) = action_cache[actions].cnsc[index].cnsc_value;
				any_on |= action_cache[actions].cnsc[index].cnsc_value;
			}

			/* Any On ?*/

			/* Assume not */
#if 0
			uint32_t regval = _REG32(action_cache[actions].base, KINETIS_FTM_SC_OFFSET);
			regval &= ~(FTM_SC_CLKS_MASK);

			if (any_on != 0) {

				/* force an update to preload all registers */
				_REG32(action_cache[actions].base, KINETIS_FTM_SYNC_OFFSET) |= FTM_SYNC;

				/* arm requires the timer be enabled */
				regval |= (FTM_SC_CLKS_EXTCLK);
			}

			_REG32(action_cache[actions].base, KINETIS_FTM_SC_OFFSET) = regval;
#endif
		}
	}

	px4_leave_critical_section(flags);

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);
	int mode = io_timer_get_channel_mode(channel);

	if (rv == 0) {
		if ((mode != IOTimerChanMode_PWMOut) &&
		    (mode != IOTimerChanMode_OneShot) &&
		    (mode != IOTimerChanMode_Trigger)) {

			rv = -EIO;

		} else {

			/* configure the channel */

			/* todo:This is a HACK!
			 * Was getting:
			 * servo 0 readback error, got 1001 expected 1000
			 * None of the SW syncs seamed update the CV
			 * So we stop the counter to get an immediate update.
			 */
			uint32_t timer = channels_timer(channel);
			irqstate_t flags = px4_enter_critical_section();

			UNUSED(timer);
#if 0
			uint32_t save = rSC(timer);
			rSC(timer) = save & ~(FTM_SC_CLKS_MASK);
			REG(timer, KINETIS_FTM_CV_OFFSET(timer_io_channels[channel].timer_channel - 1)) = value;
			rSC(timer) = save;
#endif
			px4_leave_critical_section(flags);
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0) {
		int mode = io_timer_get_channel_mode(channel);

		if ((mode == IOTimerChanMode_PWMOut) ||
		    (mode == IOTimerChanMode_OneShot) ||
		    (mode == IOTimerChanMode_Trigger)) {
			value = 0;//REG(channels_timer(channel), KINETIS_FTM_CV_OFFSET(timer_io_channels[channel].timer_channel - 1));
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
