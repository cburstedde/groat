
/*******************************************************\
* This I/O layout matches the hardware of the SOMA RoAT *
* ----------------------------------------------------- *
*                                                       *
* Switch ground pins:                                   *
*  - voice         D5                                   *
*  - modes         B4                                   *
*  - EREC          B5                                   *
*                                                       *
* LED end pins:                                         *
*  - voice         D7                                   *
*  - modes         D0                                   *
*                                                       *
* I/O pins: D1, ..., D4                                 *
* PAD pins: B0, B1, B3, B2                              *
* poti pins: ADC0, ..., ADC3                            *
*                                                       *
* sync pin: C4 / ADC4                                   *
* output pin: D6 / OSC0A                                *
*                                                       *
* ----------------------------------------------------- *
*                 WARNING: B3 is MOSI!                  *
*                 WARNING: B4 is MISO!                  *
*                 WARNING: B5 is SCK!                   *
\*******************************************************/

/* if nested interrupts shall be enabled */
//#define NESTED_ISR

/* low side of buttons */
#define SW_VOICE_D PD5
#define SW_MODE_B PB4
#define SW_EREC_B PB5

/* high side of LEDs */
#define LED_VOICE_D PD7
#define LED_MODE_D PD0

/* button/LED bit mask */
#define IO_MASK_D 0x1E

/* capacitive touch pads */
#define PAD_MASK_B 0x0F

/* sync input pin */
#define SYNC_INPUT_C PC4

/* audio output pin */
#define AUDIO_OUT_D PD6

/* define frequency data */
#define NCYCLES 512
#define NTONES 37
static const unsigned periods[NTONES] = {
  1097, 1163, 1232, 1305, 1383, 1465, 1552, 1644, 1742, 1845,
  1955, 2071, 2195, 2325, 2463, 2610, 2765, 2930, 3104, 3288,
  3484, 3691, 3910, 4143, 4389, 4650, 4927, 5220, 5530, 5859,
  6207, 6577, 6968, 7382, 7821, 8286, 8779
};

/* define waveforms */
static const unsigned char sine[256] = {
  /* sine wave */
  128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174,
  177, 179, 182, 185, 188, 191, 193, 196, 199, 201, 204, 206, 209, 211, 213, 216,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
  255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 246,
  245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220,
  218, 216, 213, 211, 209, 206, 204, 201, 199, 196, 193, 191, 188, 185, 182, 179,
  177, 174, 171, 168, 165, 162, 159, 156, 153, 150, 147, 144, 140, 137, 134, 131,
  128, 125, 122, 119, 116, 112, 109, 106, 103, 100,  97,  94,  91,  88,  85,  82,
   79,  77,  74,  71,  68,  65,  63,  60,  57,  55,  52,  50,  47,  45,  43,  40,
   38,  36,  34,  32,  30,  28,  26,  24,  22,  21,  19,  18,  16,  15,  13,  12,
   11,  10,   8,   7,   6,   6,   5,   4,   3,   3,   2,   2,   2,   1,   1,   1,
    1,   1,   1,   1,   2,   2,   2,   3,   3,   4,   5,   6,   6,   7,   8,  10,
   11,  12,  13,  15,  16,  18,  19,  21,  22,  24,  26,  28,  30,  32,  34,  36,
   38,  40,  43,  45,  47,  50,  52,  55,  57,  60,  63,  65,  68,  71,  74,  77,
   79,  82,  85,  88,  91,  94,  97, 100, 103, 106, 109, 112, 116, 119, 122, 125
};

/* variables modified by interrupts */
static volatile unsigned int millsec;
static volatile unsigned int voice_phase[4];
static volatile char pad_state;
static volatile unsigned char pad_bit;
static volatile unsigned int millsync;
static volatile unsigned char synced;
static volatile unsigned char poti_value;
static volatile unsigned char adc_complete;

/* variable copies from interrupt to main context */
static unsigned int millnow;
static unsigned char millbyte;
static unsigned char voice_pnow[4];

/* variables used by interrupt context */
static volatile unsigned char which_bound;
static volatile unsigned int count_prev;
static unsigned char voice_omul[4];
static unsigned int voice_delpha[4];

/* sequencer properties */
static unsigned char seq_millis;
static unsigned int last_sync;
static unsigned int sync_interval;
static unsigned char ttime[6], seq_target;
static unsigned char mtick, pulse, beat;
static unsigned char seq_pulses = 16;
static unsigned char sequence[192];
static unsigned char tone_invert;
static unsigned char tone_state;
#define SYNC_MINIMUM 63
#define SYNC_MAXIMUM 500

/* tap tempo state */
static unsigned int tempo_millnow;

/* voice properties */
static unsigned char voice_tone[4];
static unsigned char voice_wave[4];
static unsigned char voice_pads;

/* LFO properties */
static unsigned char lfo_speed[4];
static unsigned char lfo_phase[4];

/* potentiometer inputs */
static unsigned char rest_ADMUX;
static unsigned char prev_value[4];
static unsigned char which_mode;

/* pad check context */
static unsigned char pad_millis;
static unsigned char which_pad;
static unsigned char calib_base[4] = { 40, 40, 40, 40 };
static const unsigned char voice_from_pad[4] = { 0, 1, 3, 2 };
static char pad_on_count[4];

/* process all other I/O */
static unsigned char io_millis;
static unsigned char which_led;
static unsigned char voice_leds;
static unsigned char mode_leds;
static unsigned char which_button;
static unsigned char button_offset;
static unsigned char button_state[3];
static unsigned char button_mlast[10];
static const unsigned char button_bounce = 14;
static const unsigned char button_count[3] = { 4, 4, 2 };
#define BUTTON_RECORD 2
#define BUTTON_ERASE 1
static unsigned char button_erec;

/* LED blink context */
static const unsigned int led_interv = 1000;
static unsigned int led_millis;

/* this function must run with interrupts disabled */
static void interrupt_set_tone (unsigned char ivoice, unsigned char itone)
{
	unsigned char omul = 0;
	while (itone < 48) {
		++omul;
		itone += 12;
	}
	voice_omul[ivoice] = 1 << omul;
	voice_delpha[ivoice] = periods[itone - 48];
}

/* timer 1 capture interrupt handler */
ISR(TIMER1_CAPT_vect) {
	static unsigned char tirolls;

	/* allow to tune down by a number of octaves */
	static unsigned char ocount[4];

#ifdef NESTED_ISR
	/* always give priority to pin change */
	sei ();
#endif

	/* count milliseconds */
	if ((tirolls += 4) >= 125) {
		tirolls -= 125;
		++millsec;
	}

	/* for each voice, advance phase */
	unsigned char ivoice;
	for (ivoice = 0; ivoice < 4; ++ivoice) {
		/* only advance voice if the octave multiplier is reached */
		if (++ocount[ivoice] >= voice_omul[ivoice]) {
			voice_phase[ivoice] += voice_delpha[ivoice];
			ocount[ivoice] = 0;
		}
	}
}

/* safe to run regardless of interrupt status */
static void read_from_interrupt (void)
{
	unsigned char ivoice;

	/* read values from interrupt context */
	const unsigned char last_sreg = SREG;
	cli ();
	millnow = millsec;
	for (ivoice = 0; ivoice < 4; ++ivoice) {
		voice_pnow[ivoice] = voice_phase[ivoice] >> 8;
	}
	SREG = last_sreg;
	millbyte = millnow & 0xFF;
}

/* read the touch pads using this ISR */
ISR(PCINT0_vect) {
	/* the configured pad pin changed its value */
	const unsigned char loc_bit = pad_bit;
	if (PINB & loc_bit) {

	        /* roll at 256: this is ok since expected times are shorter */
	        const unsigned char counter_elapsed = TCNT1 - count_prev;

	        /* the pin has returned to positive, force it back to ground */
	        DDRB |= loc_bit;

	        /* remember present pad state */
	        if (counter_elapsed >= which_bound) {
	                pad_state = 1;
	        }
	        else {
	                pad_state = -1;
	        }

	        /* we are done with this function */
	        pad_bit = 0;
	}
}

/* react to sync signal using this ISR */
ISR(PCINT1_vect) {
	/* the configured pad pin changed its value */
	if (!synced && (PINC & _BV (SYNC_INPUT_C))) {
		/* sync input went high */
		millsync = millsec;
		synced = 1;
	}
}

/* ADC conversion interrupt handler */
ISR(ADC_vect) {

#ifdef NESTED_ISR
	/* always give priority to pin change */
	sei ();
#endif

	/* must read ADCL first and then ADCH */
	const unsigned char adc_low = ADCL & 0xC0;
	const unsigned char adc_value = ADCH;
	if (adc_low == 0x80 || adc_low == 0x40) {
		/* for stability, only accept the two inside 10-bit values */
		poti_value = adc_value;
	}

	/* let the main loop know we are done */
	adc_complete = 1;
}

static void voice_output (void)
{
	unsigned char ivoice;

	/* calculate momentary voice amplitude */
	const unsigned char active_tones = tone_state ^ tone_invert;
	unsigned char compensation = 128;
	unsigned int sum = 0;
	for (ivoice = 0; ivoice < 4; ++ivoice) {
		/* sum all active voice amplitudes */
		if (active_tones & (1 << ivoice)) {
			const unsigned char wave = voice_wave[ivoice];
			const unsigned char pnow = voice_pnow[ivoice];
			if (!(wave & 2)) {
				if (!(wave & 1)) {
					/* sine */
					sum += sine[pnow];
				}
				else {
					/* triangle */
					sum += (!(pnow & 0x80)) ?
						(unsigned char) (pnow << 1) :
						(unsigned char) (255 - (pnow << 1));
				}
			}
			else {
				if (!(wave & 1)) {
					/* square */
					sum += (unsigned char)
						 (80 + ((pnow & 0x80) >> 1) +
						       ((pnow & 0x80) >> 2));
				}
				else {
					/* saw */
					sum += (unsigned char) (64 + (pnow >> 1));
				}
			}
			compensation -= 32;
		}
	}

	/* output momentary voice amplitude */
	OCR0A = (sum >> 2) + compensation;
}

/* deactivate the common pins of all buttons */
static void groat_buttons_high (void) {

	/* deactivate all button sinks at HIGH */
	PORTB |= _BV (SW_MODE_B) | _BV (SW_EREC_B);
	DDRB |= _BV (SW_MODE_B) | _BV (SW_EREC_B);
	PORTD |= _BV (SW_VOICE_D);
	DDRD |= _BV (SW_VOICE_D);
}

/* deactivate the common pins of all LEDs */
static void groat_LEDs_low (void) {

	/* deactivate all LED sources at LOW */
	PORTD &= ~(_BV (LED_VOICE_D) | _BV (LED_MODE_D));
	DDRD |= _BV (LED_VOICE_D) | _BV (LED_MODE_D);
}

/* deactivate all I/O pins and ground the signal pins */
static void groat_IO_off (void) {

        /* deactivate all common pins */
        groat_buttons_high ();
        groat_LEDs_low ();

        /* set all I/O signals to active LOW */
        PORTD &= ~IO_MASK_D;
        DDRD |= IO_MASK_D;
}

/* divide one 1/16 time interval into 6 MIDI tick intervals */
static void groat_seq_divide (void) {
	static unsigned int last_interval = 1 << 15;

	/* optimize away an exact duplicate result */
	if (sync_interval != last_interval) {

		unsigned char SH = sync_interval >> 1;
		unsigned char second;
		ttime[0] = (SH * 0x55) >> 8;
		ttime[1] = (second = (SH * 0xAA) >> 8) - ttime[0];
		ttime[2] = SH - second;

		SH = sync_interval - SH;
		ttime[3] = (SH * 0x55) >> 8;
		ttime[4] = (second = (SH * 0xAA) >> 8) - ttime[3];
		ttime[5] = SH - second;

		/* remember latest configuration */
		last_interval = sync_interval;
	}

	/* immediately register the updated interval */
	seq_target = ttime[mtick];
}

/* implement a tap tempo functionality */
static void tap_tempo_down (void) {
	static unsigned int tap_last = 1 << 15;
	static unsigned int last_tap_interval;
	static unsigned char tap_count;
	const unsigned int tap_now = tempo_millnow;
	unsigned int tap_interval = tap_now - tap_last;

	/* any tap restarts the sequence */
	seq_millis = millbyte;
	mtick = pulse = beat = 0;
	mode_leds |= 8;

	/* only accept a legal time interval for quarter notes */
	if (4 * SYNC_MINIMUM <= tap_interval && tap_interval <= 4 * SYNC_MAXIMUM) {
		if (++tap_count != 1) {
			/* run a moving window filter */
			tap_interval = (last_tap_interval + tap_interval + 1) >> 1;
		}
		last_tap_interval = tap_interval;

		/* set new tap interval as tempo */
		sync_interval = (tap_interval + 2) >> 2;
		groat_seq_divide ();
	}
	else {
		/* reset for invalid tap timing */
		tap_count = 0;
	}

	/* always remember the last tap stamp */
	tap_last = tap_now;
}

static void groat_setup (void) {

	/*** setup I/O pins ***/

	/*
	 * PINx     Input pin values are read-only.
	 *          Writing a 1 toggles the value of PORTx.
	 * DDRx     Direction: 0 for input and 1 for output.
	 * PORTx    When configured as input, writing 1 activates pull-up.
	 *          When configured as output, writing 0/1 sets the value.
	 */

	/* set all pins to input (default DDR is 0) with pull-up */
	PORTB = 0xFF;
	PORTC = 0xFF;
	PORTD = 0xFF;

	/* disable pull-up and input buffer for ADC inputs */
	PORTC &= 0xF0;
	DIDR0 |= 0x0F;

	/* disable pull-up and enable voice output */
	PORTD &= ~_BV (AUDIO_OUT_D);
	DDRD |= _BV (AUDIO_OUT_D);

	/* prepare for tempo sync input handling */
	PCMSK1 |= _BV (PCINT12);
	PCICR |= _BV (PCIE1);

	/* enforce defined state for all I/O */
	groat_IO_off ();

	/* initialize touch pads */
	PORTB &= ~PAD_MASK_B;
	DDRB |= PAD_MASK_B;
	PCMSK0 |= PAD_MASK_B;
	PCICR |= _BV (PCIE0);

	/*** setup standard timers ***/

	/* set and start fast PWM mode for 8-bit timer 0 */
	OCR0A = 128;
	TCCR0A |= _BV (COM0A1) | _BV (WGM01) | _BV (WGM00);
	TCCR0B |= _BV (CS00);

	/* set CTC mode for 16-bit timer at NCYCLES clock cycles */
	TCCR1B |= _BV (WGM13) | _BV (WGM12);
	TIMSK1 |= _BV (ICIE1);
	ICR1 = NCYCLES - 1;

	/*** prepare ADC ***/

	/* left-adjust conversion result and remember setting */
	rest_ADMUX = (ADMUX |= _BV (ADLAR)) & 0xF0;

	/* run ADC at 125 kHz clock*/
	ADCSRA |= _BV (ADEN) | _BV (ADPS2) | _BV (ADPS1) | _BV (ADPS0);

	/* retrieve initial potentiometer values */
	unsigned char ipoti;
	for (ipoti = 0; ipoti < 4; ++ipoti) {
		ADMUX = rest_ADMUX | ipoti;
		ADCSRA |= _BV (ADSC);
		while (ADCSRA & _BV (ADSC));
		prev_value[ipoti] = ADCH;
	}
	ADMUX = rest_ADMUX;
	ADCSRA |= _BV (ADIF);

	/* enable ADC interrupt */
	ADCSRA |= _BV (ADIE);

	/*** initialize the sequencer ***/

	last_sync = 1 << 15;
	sync_interval = 125;
	groat_seq_divide ();

	/*** I/O initialization ***/

	mode_leds = 8;
	button_state[0] = button_state[1] = 0x0F;
	button_state[2] = 0x03;

	/*** setup voices ***/

	tone_invert = 0x0F;
	interrupt_set_tone (0, voice_tone[0] = 12);
	interrupt_set_tone (1, voice_tone[1] = 15);
	interrupt_set_tone (2, voice_tone[2] = 18);
	interrupt_set_tone (3, voice_tone[3] = 21);

	/*** start timing and interrupt processing ***/

	TCCR1B |= _BV (CS10);
	sei ();
}

static void groat_loop (void) {
	static unsigned char pad_pending;
	static unsigned char seq_todouble;
	static unsigned char tempo_wanted;

	/* output new voice value */
	voice_output ();

	/* read values from interrupt context */
	read_from_interrupt ();

	/* wait busy while pad measurement is in progress */
	if (pad_bit) {
		return;
	}

	/* begin I/O cycle by reading the next pad */
	if (!pad_pending) {
                /* access pad status every millisecond */
                const unsigned char paddiff = millbyte - pad_millis;
                if (paddiff > 0) {
                        ++pad_millis;

			/* mark an ongoing pad measurement */
                     	pad_pending = 1;
                        which_bound = calib_base[which_pad];

                        /* obtain time stamp and set pad pin to input */
                        cli ();
                        count_prev = TCNT1;
                        DDRB &= ~(pad_bit = (1 << which_pad));
                        sei ();

	                /* just started a touch pad measurement */
	                return;
		}

		/* this is one fall-through path */
	}
	else if (pad_pending == 1) {

		/* access new pad value and do something with it */
		const unsigned char ivoice = voice_from_pad[which_pad];
		const unsigned char voibit = 1 << ivoice;
		unsigned char pval;
		char on_count = pad_on_count[which_pad];

		/* count this up or down */
		on_count += pad_state;
		if (on_count == 3) {
			if (!(voice_pads & voibit)) {
				voice_pads |= voibit;

				/* this pad has just been touched */
				if (!button_erec) {
					/* neither record nor erase is active */
					tone_state |= voibit;
				}
				else if (button_erec == BUTTON_RECORD) {
					tone_state = sequence[beat] |= voibit;
				}
				else {
					/* this is BUTTON_ERASE */
					tone_state = sequence[beat] &= ~voibit;
				}
			}
		}
		else if (on_count == -3) {
			if (voice_pads & voibit) {
				voice_pads &= ~voibit;

				/* this pad has just been released */
				tone_state = (tone_state & ~voibit) | (sequence[beat] & voibit);
			}
		}
		else {
			/* no change on pad status */
			pad_on_count[which_pad] = on_count;
		}

		/* initialize with old potentiometer value */
		poti_value = prev_value[ivoice];

		/* set pad number as channel and sample */
		ADMUX = rest_ADMUX | ivoice;
		ADCSRA |= _BV (ADSC);
		pad_pending = 2;

	        /* we have done enough for one loop iteration */
		return;
	}
	else if (adc_complete) {

		/* process potentiometer reading */
		const unsigned char ivoice = voice_from_pad[which_pad];
		const unsigned char mval = poti_value;
		if (mval != prev_value[ivoice]) {
			prev_value[ivoice] = mval;

			if (!(which_mode & 1)) {
				/* update note to play as necessary */
				const unsigned char itone = (mval * 0x49) >> 8;
				if (itone != voice_tone[ivoice]) {
					voice_tone[ivoice] = itone;
					cli ();
					interrupt_set_tone (ivoice, itone);
					sei ();
				}
			}
			else {
				/* update voice waveform */
				voice_wave[ivoice] = mval >> 6;
			}
		}

		/* completed pad measurement */
		adc_complete = 0;
		pad_pending = 0;

            	/* cycle to next pad */
                if (++which_pad == 4) {
                        which_pad = 0;
                }

	        /* we have done enough for one loop iteration */
	        return;
	}
	else {
		/* this is the second fall-through path */
	}

	/* handle external sync signal */
	if (synced) {
		/* do we have a legal elapsed time */
		sync_interval = millsync - last_sync;
		if (SYNC_MINIMUM <= sync_interval && sync_interval <= SYNC_MAXIMUM) {

			/* calculate new tick intervals */
			groat_seq_divide ();
		}

		/* update state for the next time */
		last_sync = millsync;
		synced = 0;

		/* we have done enough for one loop iteration */
		return;
	}

	/* operate the sequencer */
	const unsigned char seqdiff = millbyte - seq_millis;
	if (seqdiff >= seq_target) {
		seq_millis += seq_target;

		/* advance MIDI ticks and sync pulse */
		++beat;
		if (++mtick == 6) {
			mtick = 0;
			if (++pulse == seq_pulses) {
				pulse = 0;
				beat = 0;
			}

			/* flash beat LED on every quarter */
			if (!(pulse & 3)) {
				mode_leds |= 8;
			}
			else {
				mode_leds &= ~8;
			}
		}

		/* register next target interval */
		seq_target = ttime[mtick];

		/* enter new sequencer step */
		if (!button_erec) {
			tone_state = sequence[beat] | voice_pads;
		}
		else if (button_erec == BUTTON_RECORD) {
			tone_state = sequence[beat] |= voice_pads;
		}
		else {
			/* this is BUTTON_ERASE */
			tone_state = sequence[beat] &= ~voice_pads;
		}

		/* we have done enough for one loop iteration */
		return;
	}

	/* step the LED and button cycle */
	const unsigned char iodiff = millbyte - io_millis;
	if (iodiff >= 3) {
		io_millis += 3;

		/* turn LEDS off and pull-up on */
		unsigned char next_portD = PORTD;
		next_portD &= ~(_BV (LED_VOICE_D) | _BV (LED_MODE_D));
		next_portD |= IO_MASK_D;
		PORTD = next_portD;

		/* prepare button measurement */
		DDRD &= ~IO_MASK_D;
		switch (which_button) {
		case 0:
			PORTD &= ~_BV (SW_VOICE_D);
			break;
		case 1:
			PORTB &= ~_BV (SW_MODE_B);
			break;
		case 2:
			PORTB &= ~_BV (SW_EREC_B);
			break;
		}

		/* examine new button values */
		const unsigned char binp = button_state[which_button] ^ (PIND >> 1);
		unsigned char ibut;
		unsigned char bust;
		for (ibut = 0; ibut < button_count[which_button]; ++ibut) {
			const unsigned char ilast = button_offset + ibut;
			const unsigned char bbit = 1 << ibut;
			if (binp & bbit) {
				/* button value changed.  Was it stable long enough? */
				if (millbyte - button_mlast[ilast] >= button_bounce) {
					/* yes, enough time has elapsed */
					button_mlast[ilast] = millbyte;

					/* voice and mode buttons only act on down events */
					const unsigned char bust = button_state[which_button] ^= bbit;
					if (!(bust & bbit)) {

						/* button down even registered */
						switch (which_button) {
						case 0:
							/* these are the voice buttons */
							tone_invert ^= bbit;
							break;
						case 1:
							/* these are the mode buttons */
							if (ibut == 0) {
								which_mode ^= 1;
								mode_leds ^= 1;
							}
							else if (ibut == 1) {
								which_mode ^= 2;
								mode_leds ^= 2;

								/* switch sequencer length */
								if (!(which_mode & 2)) {
									/* shorten to 96 ticks */
									if (pulse >= 16) {
										pulse -= 16;
										beat -= 96;
									}
									seq_pulses = 16;
								}
								else {
									/* defer copying to loop */
									seq_todouble = 1;
								}
							}
							else if (ibut == 3) {
								/* enable button tap tempo and restart */
								tempo_millnow = millnow;
								tempo_wanted = 1;
							}
							break;
						}
					}

					/* the record/erase buttons act on both up and down */
					if (which_button == 2) {
						if (bust == 1) {
							button_erec = BUTTON_RECORD;
							sequence[beat] |= voice_pads;
						}
						else if (bust == 2) {
							button_erec = BUTTON_ERASE;
							sequence[beat] &= ~voice_pads;
						}
						else {
							button_erec = 0;
						}
					}
				}
			}
			else {
				/* button stayed at or reverted to current value */
				button_mlast[ilast] = millbyte;
			}
		}

		/* neutralize button end */
		switch (which_button) {
		case 0:
			PORTD |= _BV (SW_VOICE_D);
			break;
		default:
			PORTB |= _BV (SW_MODE_B) | _BV (SW_EREC_B);
			break;
		}

		/* cycle to next button set */
		button_offset += 4;
		if (++which_button == 3) {
			which_button = 0;
			button_offset = 0;
		}

		/* cycle to next LED set */
		if (++which_led == 2) {
			which_led = 0;
		}

		/* turn on LEDs */
		next_portD &= ~IO_MASK_D;
		switch (which_led) {
		case 0:
			next_portD |= _BV (LED_VOICE_D) |
				(~((tone_state ^ tone_invert) << 1) & IO_MASK_D);
			break;
		case 1:
			next_portD |= _BV (LED_MODE_D) |
				(~(mode_leds << 1) & IO_MASK_D);
			break;
		}
		PORTD = next_portD;
		DDRD |= IO_MASK_D;

		/* we have done enough for one loop iteration */
		return;
	}

	/* handle tap tempo request asynchronously */
	if (tempo_wanted) {
		tap_tempo_down ();
		tempo_wanted = 0;

		/* we have done enough for one loop iteration */
		return;
	}

	/* copying four bars of the sequence takes some time */
	if (seq_todouble) {

		/* double to 192 ticks */
		unsigned char i;
		for (i = 0; i < 96; ++i) {
			sequence[i + 96] = sequence[i];
		}
		seq_pulses = 32;
		seq_todouble = 0;

		/* we have done enough for one loop iteration */
		return;
	}

	/* blink mode LED */
	const unsigned int milldiff = millnow - led_millis;
	if (milldiff >= led_interv) {
		led_millis += led_interv;
		mode_leds ^= 4;
	}
}

/*** variables for touch pad calibration algorithm ***/

/* how many retries we do for the same calibration value at any pin */
static unsigned char calib_tries[4];

/* how many of these tries have yielded an too-long interval */
static unsigned char calib_lives[4];

/* the bit position of the correction that we are examining */
static unsigned char calib_cubit[4];

/* bitwise OR of active calibration pins */
static unsigned char calib_status;

/* elapsed milliseconds for the steps */
static unsigned char calib_millis;

#define CALIB_LEVEL 6
#define CALIB_TRIES 3

/* initalize the self-calibration algorithm for the touch pads */
static void groat_calib_start (void) {

	/* read values from interrupt context */
	read_from_interrupt ();
	calib_millis = millbyte;

	/* set variables such that at least one step is possible */
	unsigned char i;
	for (i = 0; i < 4; ++i) {
		/* we run a binary search from MSB to LSB */
		calib_cubit[i] = 7;

		/* we test each pin value several times */
		calib_tries[i] = calib_lives[i] = CALIB_TRIES;

		/* we might alternatively use the defaults minus 32 */
		calib_base[i] = 40;
	}

	/* contains one bit per pad which is true if still calibrating */
	calib_status = 0x0F;
}

/* assumption on entry is that at least one measurement remains */
static int groat_calib_step (void) {
	static unsigned char calib_pending;

	/* read values from interrupt context */
	read_from_interrupt ();

	/* only enter if there is no pad measurement ongoing */
	if (pad_bit == 0) {
		/* are we ready for a new touch pad measurement */
		if (!calib_pending) {
			/* measure pad status every millisecond */
			const unsigned char startdiff = millbyte - calib_millis;
			if (startdiff > 0) {
				++calib_millis;

				/* we have an ongoing measurement */
				calib_pending = 1;
				which_bound = calib_base[which_pad]
					 + (1 << calib_cubit[which_pad]) - 1;

				/* obtain time stamp and set pad pin to input */
				cli ();
				count_prev = TCNT1;
				DDRB &= ~(pad_bit = (1 << which_pad));
				sei ();
			}
		}
		else {
			unsigned char i, j;

			/* we enter here if the measurement is complete */
			j = calib_cubit[i = which_pad];
			if (j > CALIB_LEVEL || pad_state == -1) {
				/* the bound value was too high */
				--calib_lives[i];
			}
			if (--calib_tries[i] == 0) {
				/* we have finished this pad's bit position */
				if (calib_lives[i] >= CALIB_TRIES - 1) {
					/* accept only if not gone too far */
					calib_base[i] += (1 << j);
				}
				if (j == 0) {
					/* we are done with this pad */
					calib_status &= ~(1 << i);
				}
				else {
					/* restart measurement one bit to the left */
					calib_tries[i] = calib_lives[i] = CALIB_TRIES;
					--calib_cubit[i];
				}
			}
			calib_pending = 0;

			/* cycle to next calibrating pad */
			if (++which_pad == 4) {
				which_pad = 0;
			}
		}
	}

	/* if no measurement remains to be performed we are done */
	return calib_status;
}

/* entry point for application */
int main () {
	/* initialize the chip */
	groat_setup ();

	/* calibrate touch pads */
	for (groat_calib_start (); groat_calib_step (););

	/* start main loop with proper timing */
	read_from_interrupt ();
	led_millis = millnow;
	pad_millis = seq_millis = io_millis = millbyte;
	for (unsigned char ilast = 0; ilast < 10; ++ilast) {
		button_mlast[ilast] = millbyte;
	}
	for (;;) {
		groat_loop ();
	}
}
