/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#include <px4_config.h>
#include <px4_log.h>
#include <systemlib/px4_macros.h>
#include <debug.h>

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include "chip.h"
#include "chip/imxrt_gpt.h"
#include "imxrt_periphclks.h"

#include <systemlib/err.h>
#include <circuit_breaker/circuit_breaker.h>

#include <px4_workqueue.h>

#include <lib/tunes/tunes.h>
#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>

#define CAT3_(A, B, C)    A##B##C
#define CAT3(A, B, C)     CAT3_(A, B, C)

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_TIMER)  && defined(HRT_TIMER)
# if TONE_ALARM_TIMER == HRT_TIMER
#   error TONE_ALARM_TIMER and HRT_TIMER must use different timers.
# endif
#endif

/*
* Period of the free-running counter, in microseconds.
*/
#define TONE_ALARM_COUNTER_PERIOD  4294967296

/* Tone Alarm configuration */

#define TONE_ALARM_TIMER_CLOCK  BOARD_GPT_FREQUENCY                       /* The input clock frequency to the GPT block */
#define TONE_ALARM_TIMER_BASE   CAT3(IMXRT_GPT, TONE_ALARM_TIMER,_BASE)   /* The Base address of the GPT */
#define TONE_ALARM_TIMER_VECTOR CAT(IMXRT_IRQ_GPT, TONE_ALARM_TIMER)      /* The GPT Interrupt vector */

#if TONE_ALARM_TIMER == 1
#  define TONE_ALARM_CLOCK_ALL()  imxrt_clockall_gpt_bus()         /* The Clock Gating macro for this GPT */
#elif TONE_ALARM_TIMER == 2
#  define TONE_ALARM_CLOCK_ALL()  imxrt_clockall_gpt2_bus()        /* The Clock Gating macro for this GPT */
#endif

#if TONE_ALARM_TIMER == 1 && defined(CONFIG_IMXRT_GPT1)
#  error must not set CONFIG_IMXRT_GPT1=y and TONE_ALARM_TIMER=1
#elif   TONE_ALARM_TIMER == 2 && defined(CONFIG_IMXRT_GPT2)
#  error must not set CONFIG_IMXRT_GPT2=y and TONE_ALARM_TIMER=2
#endif


# define TONE_ALARM_TIMER_FREQ    1000000

/*
* Tone Alarm clock must be a multiple of 1MHz greater than 1MHz
*/
#if (TONE_ALARM_TIMER_CLOCK % TONE_ALARM_TIMER_FREQ) != 0
#  error TONE_ALARM_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if TONE_ALARM_TIMER_CLOCK <= TONE_ALARM_TIMER_FREQ
# error TONE_ALARM_TIMER_CLOCK must be greater than 1MHz
#endif

#if (TONE_ALARM_TIMER_CHANNEL > 1) || (TONE_ALARM_TIMER_CHANNEL > 3)
#  error TONE_ALARM_CHANNEL must be a value between 1 and 3
#endif


/* Register accessors */

#define _REG(_addr) (*(volatile uint32_t *)(_addr))

/* Timer register accessors */

#define REG(_reg) _REG(TONE_ALARM_TIMER_BASE + (_reg))

#define rCR         REG(IMXRT_GPT_CR_OFFSET)
#define rPR         REG(IMXRT_GPT_PR_OFFSET)
#define rSR         REG(IMXRT_GPT_SR_OFFSET)
#define rIR         REG(IMXRT_GPT_IR_OFFSET)
#define rOCR1       REG(IMXRT_GPT_OCR1_OFFSET)
#define rOCR2       REG(IMXRT_GPT_OCR2_OFFSET)
#define rOCR3       REG(IMXRT_GPT_OCR3_OFFSET)
#define rICR1       REG(IMXRT_GPT_ICR1_OFFSET)
#define rICR2       REG(IMXRT_GPT_ICR2_OFFSET)
#define rCNT        REG(IMXRT_GPT_CNT_OFFSET)

/*
* Specific registers and bits used by Tone Alarm sub-functions
*/

#define rOCR         CAT(rOCR, TONE_ALARM_CHANNEL)               /* GPT Output Compare Register used by HRT */
#define rSTATUS      CAT(GPT_SR_OF, TONE_ALARM_CHANNEL)          /* OF Output Compare Flag */
#define CR_OM        CAT3(GPT_CR_OM, TONE_ALARM_CHANNEL,_TOGGLE) /* Output Compare mode */


#define CBRK_BUZZER_KEY 782097

class ToneAlarm : public device::CDev
{
public:
	ToneAlarm();
	~ToneAlarm();

	virtual int init();
	void status();

	enum {
		CBRK_OFF = 0,
		CBRK_ON,
		CBRK_UNINIT
	};

private:
	volatile bool _running;
	volatile bool _should_run;
	bool _play_tone;

	Tunes _tunes;

	unsigned _silence_length; // if nonzero, silence before next note

	int _cbrk; ///< if true, no audio output
	int _tune_control_sub;

	tune_control_s _tune;

	static work_s _work;

	// Convert a frequency value into a divisor for the configured timer's clock.
	//
	unsigned frequency_to_divisor(unsigned frequency);

	// Start playing the note
	//
	void start_note(unsigned frequency);

	// Stop playing the current note and make the player 'safe'
	//
	void stop_note();

	// Parse the next note out of the string and play it
	//
	void next_note();

	// work queue trampoline for next_note
	//
	static void next_trampoline(void *arg);

};

struct work_s ToneAlarm::_work = {};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[]);


ToneAlarm::ToneAlarm() :
	CDev("tone_alarm", TONEALARM0_DEVICE_PATH),
	_running(false),
	_should_run(true),
	_play_tone(false),
	_tunes(),
	_silence_length(0),
	_cbrk(CBRK_UNINIT),
	_tune_control_sub(-1)
{
	// enable debug() calls
	//_debug_enabled = true;
}

ToneAlarm::~ToneAlarm()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		usleep(100000);
	}
}

int ToneAlarm::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

	/* Enable the Module clock */

	TONE_ALARM_CLOCK_ALL();


	/* disable and configure the timer */

	/* disable and configure the timer */

	rCR =  GPT_CR_CLKSRC_IPG | GPT_CR_DBGEN | GPT_CR_OM1_DIS | GPT_CR_OM2_DIS | GPT_CR_OM3_DIS | GPT_CR_FRR
	       | GPT_CR_CLKSRC_IPG | GPT_CR_ENMOD;

	/* CLKSRC field is divided by [PRESCALER + 1] */

	rPR = (TONE_ALARM_TIMER_CLOCK / TONE_ALARM_TIMER_FREQ) - 1;
	rCNT = 0;

	/* enable the timer and output toggle */

	rCR |= CR_OM | GPT_CR_EN;

	DEVICE_DEBUG("ready");
	_running = true;
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, 0);
	return OK;
}

void ToneAlarm::status()
{
	if (_running) {
		PX4_INFO("running");

	} else {
		PX4_INFO("stopped");
	}
}

unsigned ToneAlarm::frequency_to_divisor(unsigned frequency)
{
	float period = 0.5f / frequency;

	// and the divisor, rounded to the nearest integer
	unsigned divisor = (period * TONE_ALARM_TIMER_FREQ) + 0.5f;

	return divisor;
}

void ToneAlarm::start_note(unsigned frequency)
{
	// check if circuit breaker is enabled
	if (_cbrk == CBRK_UNINIT) {
		_cbrk = circuit_breaker_enabled("CBRK_BUZZER", CBRK_BUZZER_KEY);
	}

	if (_cbrk != CBRK_OFF) { return; }

	// compute the divisor
	unsigned divisor = frequency_to_divisor(frequency);

	rCNT = 0;
	rOCR = divisor;		// load new toggle period
	rCR |= GPT_CR_EN;

	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarm::stop_note()
{
	/* stop the current note */
	rCR &= ~GPT_CR_EN;

	/*
	 * Make sure the GPIO is not driving the speaker.
	 */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}

void ToneAlarm::next_note()
{
	if (!_should_run) {
		if (_tune_control_sub >= 0) {
			orb_unsubscribe(_tune_control_sub);
		}

		_running = false;
		return;
	}

	// subscribe to tune_control
	if (_tune_control_sub < 0) {
		_tune_control_sub = orb_subscribe(ORB_ID(tune_control));
	}

	// do we have an inter-note gap to wait for?
	if (_silence_length > 0) {
		stop_note();
		work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, USEC2TICK(_silence_length));
		_silence_length = 0;
		return;
	}

	// check for updates
	bool updated = false;
	orb_check(_tune_control_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tune_control), _tune_control_sub, &_tune);
		_play_tone = _tunes.set_control(_tune) == 0;
	}

	unsigned frequency = 0;
	unsigned duration = 0;

	if (_play_tone) {
		_play_tone = false;
		int parse_ret_val = _tunes.get_next_tune(frequency, duration, _silence_length);

		if (parse_ret_val >= 0) {
			// a frequency of 0 correspond to stop_note
			if (frequency > 0) {
				// start playing the note
				start_note(frequency);

			} else {
				stop_note();
			}


			if (parse_ret_val > 0) {
				// continue playing
				_play_tone = true;
			}
		}

	} else {
		// schedule a call with the tunes max interval
		duration = _tunes.get_maximum_update_interval();
		// stop playing the last note after the duration elapsed
		stop_note();
	}

	// and arrange a callback when the note should stop
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, USEC2TICK(duration));
}

void ToneAlarm::next_trampoline(void *arg)
{
	ToneAlarm *ta = (ToneAlarm *)arg;
	ta->next_note();
}

/**
 * Local functions in support of the shell command.
 */
namespace
{

ToneAlarm	*g_dev;

} // namespace

void tone_alarm_usage();

void tone_alarm_usage()
{
	PX4_INFO("missing command, try 'start', status, 'stop'");
}

int tone_alarm_main(int argc, char *argv[])
{

	if (argc > 1) {
		const char *argv1 = argv[1];

		if (!strcmp(argv1, "start")) {
			if (g_dev != nullptr) {
				PX4_ERR("already started");
				return 1;
			}

			if (g_dev == nullptr) {
				g_dev = new ToneAlarm();

				if (g_dev == nullptr) {
					PX4_ERR("couldn't allocate the ToneAlarm driver");
					return 1;
				}

				if (OK != g_dev->init()) {
					delete g_dev;
					g_dev = nullptr;
					PX4_ERR("ToneAlarm init failed");
					return 1;
				}
			}

			return 0;
		}

		if (!strcmp(argv1, "stop")) {
			delete g_dev;
			g_dev = nullptr;
			return 0;
		}

		if (!strcmp(argv1, "status")) {
			g_dev->status();
			return 0;
		}

	}

	tone_alarm_usage();
	return 0;
}
