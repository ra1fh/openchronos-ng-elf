/**
    modules/flyback.c: flyback stopwatch module for openchronos-ng

	Copyright (C) 2018 Ralf Horstmann <ralf@ackstorm.de>

    http://github.com/ra1fh/openchronos-ng-elf

    This file is part of openchronos-ng-elf.

    openchronos-ng-elf is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    openchronos-ng-elf is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/

/* flyback.c: flyback stopwatch with logging
 *
 * The flyback mode (menu text "FLYBK") is similar to a flyback
 * stopwatch with the additional feature of storing timestamps and
 * counting the number of events (up to 19 events/timestamps). In
 * contrast to a regular stopwatch, the timer will automatically
 * restart on button press.
 *
 * Long press of the '*' button will clear all recorded timestamps and
 * reset the stopwatch to 0 and switch back to the start screen. Long
 * press of the '#' button deletes the last recorded timestamp and
 * adjusts the stopwatch to show time since latest remaining time
 * stamp. Short press of '#' will cycle through up to four different
 * screens. Each screen has it's own symbol in the middle symbol line:
 *
 * - chrono screen (stopwatch symbol)
 * - list screen (record symbol)
 *
 * On the chrono screen, both the up and the down button will record a
 * new timestamp and restart the stopwatch from zero. Which button was
 * used to restart the timer is stored along with the timestamp. This
 * can be used to differentiate events (start, stop, ...).
 *
 * On the list screen, the up and down buttons will scroll through the
 * recorded timestamps or the list of time intervals between recorded
 * timestamps. No new timestamp will be recorded when using the up and
 * down buttons on these screens.
 *
 * The list of timestamps is kept in main memory and is persistent
 * across mode switches. Also the stopwatch will continue to run when
 * using a different mode in between. But, timestamps will be lost
 * when removing the battery, reset or other power loss.
 *
 * The timestamps are stored by copying the current real time clock
 * (RTC) time into memory slots. When all memory slots are in use, the
 * last slot will be overwritten when recording a time stamp.
 *
 * When changing the time in the clock module, stored timestamps are
 * not adjusted. This means that the stopwatch view and interval times
 * to previously stored timestamps might become inaccurate.
 *
 * === 1. CHRONO SCREEN ===
 *
 * The chrono will show the current time in the first line. All the
 * digits of the second line are used to display the stopwatch
 * timer. Up to 20 hours it will show hours:minutes:seconds. After 20
 * hours up to 100 hours it will switch to four digits showing
 * hours:minutes. When exceeding 100 hours the display will show
 * '-----'.
 *
 * The up and down buttons will restart the stopwatch and record the
 * current time in a memory slot.
 *
 * === 2. LIST SCREEN ===
 *
 * The list screen shows the total time from first to last timestamp,
 * the recorded timestamps and intervals. It starts with the total
 * time, indicated with the 'TOTAL' symbol. The first line shows hours
 * and minutes, the right two digits of the lower display line show
 * the seconds. The number of records is shown in the left two digits
 * of the second line. In case there are less than 2 timestamps, the
 * total display will show all zeros.
 *
 * The up and down buttons can be used to scroll through the list of
 * timestamps and intervals.

 * Starting with the total time, scrolling down will show interval
 * time when there are 2 or more timestamps recorded. Otherwise the
 * down button will not allow to scroll down.
 *
 * Starting with the total time, scrolling up with show recorded time
 * stamps in case there are one or more timestamps recorded.
 *
 * In case the timestamp on display was recorded on a different day,
 * the record symbol will blink.
 *
 * To give a visual hint whether timestamps or intervals are shown,
 * there are two parallel bars ('||') in the second line, either in
 * down position (for intervals) or in the up position (for
 * timestamps). When total time is shown, there are no bars.
 *
 * The time delta view is limited to less than 100 hours. When
 * exceeding 100 hours between two timestamps, the interval will be
 * shown as '--:--'. The delta calculation is calendar correct across
 * changes of month or year. E.g. you can record a timestamp on 28th
 * of February in a leap year and the next one 1st of March, the time
 * delta will be correct.
 *
 * === Caveats ===
 *
 * - There is currently no way to display the day, month or year of a
 *   recorded timestamp. A blinking record symbol will indicate that
 *   the timestamp on display was recorded on a different day though.
 *
 * - When adjusting the clock, the recorded timestamps are not being
 *   adjusted. So the time delta recorded after the clock change might
 *   be inaccurate, as well as the stopwatch time.
 *
 * - If for some reason the time delta calculation returns an error,
 *   the chrono screen might show '-E-' where the
 *   stopwatch time is normally shown. This can happen if the
 *   conversion from RTC time to seconds returns a negative number or
 *   the difference between the values get negative.
 *
 * - The RTC to seconds conversion uses 32bit values, so it will
 *   overflow eventually. When exactly has yet to be determined.
 */

#include <string.h>
#include <time.h>

#include "messagebus.h"
#include "menu.h"

#include "drivers/rtca.h"
#include "drivers/display.h"
#include "drivers/timer.h"

enum {
	FLYBACK_CHRONO,
	FLYBACK_LIST,
	FLYBACK_END,
};

#define FLYBACK_FIRST_SCREEN 0

enum {
	FLYBACK_MARK_NONE,
	FLYBACK_MARK_UP,
	FLYBACK_MARK_DOWN,
	FLYBACK_MARK_BOTH,
};

enum {
	FLYBACK_LIST_TOTAL,
	FLYBACK_LIST_INTERVAL,
	FLYBACK_LIST_TIMESTAMP,
};

#define FLYBACK_MAX_TIMESTAMPS 19

#define TENMINUTES   (10L * 60)
#define TENHOURS     (10L * 60 * 60)
#define TWENTYHOURS  (20L * 60 * 60)
#define HUNDREDHOURS (100L * 60 * 60)

struct ts_s {
	uint16_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t mark;
};

static struct flyback_state {
	struct ts_s ts[FLYBACK_MAX_TIMESTAMPS + 1];
	/* records are stored from 1..MAX, ts[0] is used for temporary storage*/
	time_t seconds;       /* stopwatch time in seconds                    */
	uint8_t mode;         /* active screen                                */
	uint8_t count;        /* number of ts records in use                  */
	uint8_t display_mode; /* display mode for list and interval screen    */
	uint8_t display_count;/* displayed record on list and interval screen */
} flyback_state;

static struct flyback_screen {
	void (*init)();                      /* initialization once on mode activation     */
	void (*enter)();                     /* called on screen activation                */
	void (*statechange)();               /* flyback_state changed (new/deleted record) */
	void (*stopwatch)();                 /* stopwatch time changed                     */
	void (*event)(enum sys_message msg); /* standard events                            */
	void (*updown)(int mark);            /* up/down button pressed                     */
} flyback_screens[];

static void flyback_stopwatch();
static void flyback_statechange();
static void flyback_make_tm(struct tm* tm, struct ts_s *ts);
static void flyback_copy_rtc(struct ts_s *ts, int mark);
static int flyback_diff_ts(struct ts_s *ts1, struct ts_s *ts2, time_t *s);
static int flyback_diff_entries(int ires, int i1, int i2);
static void flyback_state_record(int mark);
static void flyback_num_pressed();
static void flyback_update_mark(int display, int mark);

/**********************************************************************/
/**************************** SCREENS *********************************/
/**********************************************************************/

/************************** chrono screen ***************************/

static void flyback_chrono_init()
{
	display_symbol(FLYBACK_CHRONO, LCD_ICON_STOPWATCH, SEG_ON);
}

static void flyback_chrono_statechange()
{
	uint8_t mark;
	if (flyback_state.count) {
		mark = flyback_state.ts[flyback_state.count].mark;
	} else {
		mark = FLYBACK_MARK_NONE;
	}
	flyback_update_mark(FLYBACK_CHRONO, mark);
	display_symbol(FLYBACK_CHRONO, LCD_SYMB_MAX,
				   flyback_state.count >= FLYBACK_MAX_TIMESTAMPS ? SEG_ON : SEG_OFF);
}

static void flyback_chrono_event(enum sys_message msg)
{
	if (msg & SYS_MSG_RTC_SECOND) {
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L1_COL, ((rtca_time.sec & 0x01) ? SEG_ON : SEG_OFF));
	}
	if (msg & SYS_MSG_RTC_HOUR) {
		_printf(FLYBACK_CHRONO, LCD_SEG_L1_3_2, "%02u", rtca_time.hour);
	}
	if (msg & SYS_MSG_RTC_MINUTE) {
		_printf(FLYBACK_CHRONO, LCD_SEG_L1_1_0, "%02u", rtca_time.min);
	}
}

static void flyback_chrono_stopwatch()
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	if (flyback_state.count == 0) {
		display_chars(FLYBACK_CHRONO, LCD_SEG_L2_5_0, "000000", SEG_SET);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL0, SEG_ON);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL1, SEG_ON);
		flyback_update_mark(FLYBACK_CHRONO, FLYBACK_MARK_NONE);
		return;
	}
	if (flyback_state.seconds < 0) {
		display_chars(FLYBACK_CHRONO, LCD_SEG_L2_5_0, " --E--", SEG_SET);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL1, SEG_OFF);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL0, SEG_OFF);
		return;
	}

	sec   = (flyback_state.seconds % 60);
	min   = (flyback_state.seconds / 60) % 60;
	hour  = (flyback_state.seconds / 60) / 60;

	if (flyback_state.seconds >= HUNDREDHOURS) {
		display_chars(FLYBACK_CHRONO, LCD_SEG_L2_5_0, " -----", SEG_SET);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL1, SEG_OFF);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL0, SEG_OFF);
	} else if (flyback_state.seconds >= TWENTYHOURS) {
		display_chars(FLYBACK_CHRONO, LCD_SEG_L2_5_4, "  ", SEG_SET);
		_printf(FLYBACK_CHRONO, LCD_SEG_L2_3_2, "%02u", hour);
		_printf(FLYBACK_CHRONO, LCD_SEG_L2_1_0, "%02u", min);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL1, SEG_OFF);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL0, SEG_ON);
	} else {
		_printf(FLYBACK_CHRONO, LCD_SEG_L2_5_4, "%02u", hour);
		_printf(FLYBACK_CHRONO, LCD_SEG_L2_3_2, "%02u", min);
		_printf(FLYBACK_CHRONO, LCD_SEG_L2_1_0, "%02u", sec);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL1, SEG_ON);
		display_symbol(FLYBACK_CHRONO, LCD_SEG_L2_COL0, SEG_ON);
	}
}

static void flyback_chrono_updown(int mark)
{
	flyback_state_record(mark);
}

/*************************** list screen **************************/

static void flyback_list_init()
{
	display_symbol(FLYBACK_LIST, LCD_ICON_RECORD, SEG_ON);
	display_symbol(FLYBACK_LIST, LCD_SEG_L1_COL, SEG_ON);
}

static void flyback_list_enter()
{
	flyback_state.display_count = flyback_state.count;
	flyback_state.display_mode = FLYBACK_LIST_TOTAL;
}

static void flyback_list_statechange()
{
	uint8_t hour, min, sec, mark;
	uint8_t blink;

	display_symbol(FLYBACK_LIST, LCD_SYMB_MAX,
				   flyback_state.count >= FLYBACK_MAX_TIMESTAMPS ? SEG_ON : SEG_OFF);

	switch (flyback_state.display_mode) {
	case FLYBACK_LIST_TOTAL:
		_printf(FLYBACK_LIST, LCD_SEG_L2_5_4, "%02u", flyback_state.display_count);
		display_chars(FLYBACK_LIST, LCD_SEG_L2_3_2, "  ", SEG_SET);
		display_symbol(FLYBACK_LIST, LCD_SEG_L2_COL0, SEG_ON);
		display_symbol(FLYBACK_LIST, LCD_SYMB_TOTAL, SEG_ON);
		display_symbol(FLYBACK_LIST, LCD_ICON_RECORD, SEG_SET | BLINK_OFF);
		flyback_update_mark(FLYBACK_LIST, FLYBACK_MARK_BOTH);
		if (flyback_diff_entries(0, 1, flyback_state.display_count) < 0) {
			display_chars(FLYBACK_LIST, LCD_SEG_L1_3_0, "----", SEG_SET);
			display_chars(FLYBACK_LIST, LCD_SEG_L2_1_0,   "--", SEG_SET);
			return;
		}
		_printf(FLYBACK_LIST, LCD_SEG_L1_3_2, "%02u", flyback_state.ts[0].hour);
		_printf(FLYBACK_LIST, LCD_SEG_L1_1_0, "%02u", flyback_state.ts[0].min);
		_printf(FLYBACK_LIST, LCD_SEG_L2_1_0, "%02u", flyback_state.ts[0].sec);
		break;

	case FLYBACK_LIST_INTERVAL:
		/* we enter list mode with minimum 2 records, so display count is 2.
		 * it makes most sense to start numbering interval at 1, so we subtract
		 * 1 from display_count
		 */
		_printf(FLYBACK_LIST, LCD_SEG_L2_5_4, "%02u", flyback_state.display_count - 1);
		display_bits(FLYBACK_LIST, LCD_SEG_L2_3, 0x40, SEG_SET);
		display_bits(FLYBACK_LIST, LCD_SEG_L2_2, 0x04, SEG_SET);
		display_symbol(FLYBACK_LIST, LCD_SEG_L2_COL0, SEG_ON);
		display_symbol(FLYBACK_LIST, LCD_SYMB_TOTAL, SEG_OFF);
		if (flyback_diff_entries(0, flyback_state.display_count - 1, flyback_state.display_count) < 0) {
			display_chars(FLYBACK_LIST, LCD_SEG_L1_3_0, "----", SEG_SET);
			display_chars(FLYBACK_LIST, LCD_SEG_L2_1_0,   "--", SEG_SET);
			flyback_update_mark(FLYBACK_LIST, FLYBACK_MARK_NONE);
			return;
		}
		flyback_update_mark(FLYBACK_LIST, flyback_state.ts[0].mark);
		_printf(FLYBACK_LIST, LCD_SEG_L1_3_2, "%02u", flyback_state.ts[0].hour);
		_printf(FLYBACK_LIST, LCD_SEG_L1_1_0, "%02u", flyback_state.ts[0].min);
		_printf(FLYBACK_LIST, LCD_SEG_L2_1_0, "%02u", flyback_state.ts[0].sec);
		break;

	case FLYBACK_LIST_TIMESTAMP:
		_printf(FLYBACK_LIST, LCD_SEG_L2_5_4, "%02u", flyback_state.display_count);
		display_bits(FLYBACK_LIST, LCD_SEG_L2_3, 0x20, SEG_SET);
		display_bits(FLYBACK_LIST, LCD_SEG_L2_2, 0x01, SEG_SET);
		display_symbol(FLYBACK_LIST, LCD_SEG_L2_COL0, SEG_ON);
		display_symbol(FLYBACK_LIST, LCD_SYMB_TOTAL, SEG_OFF);
		if (flyback_state.display_count == 0) {
			hour = min = sec = 0;
			mark = FLYBACK_MARK_NONE;
			blink = BLINK_OFF;
		} else {
			hour = flyback_state.ts[flyback_state.display_count].hour;
			min = flyback_state.ts[flyback_state.display_count].min;
			sec = flyback_state.ts[flyback_state.display_count].sec;
			mark = flyback_state.ts[flyback_state.display_count].mark;
			if (flyback_state.ts[flyback_state.display_count].day != rtca_time.day ||
				flyback_state.ts[flyback_state.display_count].mon != rtca_time.mon ||
				flyback_state.ts[flyback_state.display_count].year != rtca_time.year) {
				blink = BLINK_ON;
			} else {
				blink = BLINK_OFF;
			}
		}
		flyback_update_mark(FLYBACK_LIST, mark);
		display_symbol(FLYBACK_LIST, LCD_ICON_RECORD, SEG_SET | blink);
		_printf(FLYBACK_LIST, LCD_SEG_L1_3_2, "%02u", hour);
		_printf(FLYBACK_LIST, LCD_SEG_L1_1_0, "%02u", min);
		_printf(FLYBACK_LIST, LCD_SEG_L2_1_0, "%02u", sec);
		break;

	default:
		(void) 0;
		break;
	}
}

static void flyback_list_updown(int mark)
{
	switch (flyback_state.display_mode) {
	case FLYBACK_LIST_TOTAL:
		if (mark == FLYBACK_MARK_UP && flyback_state.count > 0) {
			flyback_state.display_count = 1;
			flyback_state.display_mode = FLYBACK_LIST_TIMESTAMP;
			flyback_statechange();
		}
		if (mark == FLYBACK_MARK_DOWN && flyback_state.count > 1) {
			flyback_state.display_count = flyback_state.count;
			flyback_state.display_mode = FLYBACK_LIST_INTERVAL;
			flyback_statechange();
		}
		break;

	case FLYBACK_LIST_INTERVAL:
		if (mark == FLYBACK_MARK_UP) {
			if (flyback_state.display_count < flyback_state.count && flyback_state.display_count < FLYBACK_MAX_TIMESTAMPS) {
				flyback_state.display_count++;
				flyback_statechange();
			} else {
				flyback_state.display_mode = FLYBACK_LIST_TOTAL;
				flyback_state.display_count = flyback_state.count;
				flyback_statechange();
			}
		} else {
			if (flyback_state.display_count > 2) {
				flyback_state.display_count--;
				flyback_statechange();
			}
		}
		break;

	case FLYBACK_LIST_TIMESTAMP:
		if (mark == FLYBACK_MARK_UP) {
			if (flyback_state.display_count < flyback_state.count && flyback_state.display_count < FLYBACK_MAX_TIMESTAMPS) {
				flyback_state.display_count++;
				flyback_statechange();
			}
		} else {
			if (flyback_state.display_count > 1) {
				flyback_state.display_count--;
				flyback_statechange();
			} else {
				flyback_state.display_mode = FLYBACK_LIST_TOTAL;
				flyback_state.display_count = flyback_state.count;
				flyback_statechange();
			}
		}
		break;

	default:
		(void) 0;
		/* should not happen */
	}
}

static struct flyback_screen flyback_screens[] = {
	{
		.init        = flyback_chrono_init,
		.enter       = NULL,
		.statechange = flyback_chrono_statechange,
		.stopwatch 	 = flyback_chrono_stopwatch,
		.event     	 = flyback_chrono_event,
		.updown    	 = flyback_chrono_updown,
	},
	{
		.init        = flyback_list_init,
		.enter       = flyback_list_enter,
		.statechange = flyback_list_statechange,
		.event       = NULL,
		.stopwatch   = NULL,
		.updown      = flyback_list_updown,
	},
};

/**********************************************************************/
/************************ HELPER FUNCTIONS ****************************/
/**********************************************************************/

static void flyback_make_tm(struct tm* tm, struct ts_s *ts)
{
	tm->tm_year = ts->year - 1900;
	tm->tm_mon = ts->mon - 1;
	tm->tm_mday = ts->day;
	tm->tm_hour = ts->hour;
	tm->tm_min = ts->min;
	tm->tm_sec = ts->sec;
	tm->tm_wday = 0;
	tm->tm_yday = 0;
	tm->tm_isdst = 0;
}

static void flyback_copy_rtc(struct ts_s *ts, int mark)
{
	ts->year = rtca_time.year;
	ts->mon = rtca_time.mon;
	ts->day = rtca_time.day;
	ts->hour = rtca_time.hour;
	ts->min = rtca_time.min;
	ts->sec = rtca_time.sec;
	ts->mark = mark;
}

static int flyback_diff_ts(struct ts_s *ts1, struct ts_s *ts2, time_t *s)
{
	struct tm tm1, tm2;
	time_t t1, t2;
	time_t seconds;

	flyback_make_tm(&tm1, ts1);
	flyback_make_tm(&tm2, ts2);
	t1 = mktime(&tm1);
	t2 = mktime(&tm2);

	if (t1 < 0 || t2 < 0) {
		return -1;
	}

	seconds = t2 - t1;
	if (seconds < 0) {
		return -1;
	}

	*s = seconds;
	return 0; /* success */
}

static int flyback_diff_entries(int ires, int i1, int i2) {
	time_t seconds;

	if (flyback_state.display_count < 2 || i1 == i2) {
		flyback_state.ts[ires].hour = 0;
		flyback_state.ts[ires].min = 0;
		flyback_state.ts[ires].sec = 0;
		flyback_state.ts[ires].mark = FLYBACK_MARK_NONE;
		return 0;
	}

	if (flyback_diff_ts(&flyback_state.ts[i1],
						&flyback_state.ts[i2],
						&seconds) < 0)
		return -1;

	if (seconds >= HUNDREDHOURS)
		return -1;

	flyback_state.ts[ires].mark = flyback_state.ts[i2].mark;
	flyback_state.ts[ires].sec  = (seconds % 60);
	flyback_state.ts[ires].min  = (seconds / 60) % 60;
	flyback_state.ts[ires].hour = (seconds / 60) / 60;
	return 0;
}

static void flyback_update_mark(int display, int mark)
{
	switch(mark) {
	case FLYBACK_MARK_UP:
		display_symbol(display, LCD_SYMB_ARROW_UP, SEG_ON);
		display_symbol(display, LCD_SYMB_ARROW_DOWN, SEG_OFF);
		break;
	case FLYBACK_MARK_DOWN:
		display_symbol(display, LCD_SYMB_ARROW_UP, SEG_OFF);
		display_symbol(display, LCD_SYMB_ARROW_DOWN, SEG_ON);
		break;
	case FLYBACK_MARK_BOTH:
		display_symbol(display, LCD_SYMB_ARROW_UP, SEG_ON);
		display_symbol(display, LCD_SYMB_ARROW_DOWN, SEG_ON);
		break;
	default:
		display_symbol(display, LCD_SYMB_ARROW_UP, SEG_OFF);
		display_symbol(display, LCD_SYMB_ARROW_DOWN, SEG_OFF);
		break;
	}
}

static void flyback_state_record(int mark)
{
	if (flyback_state.count < FLYBACK_MAX_TIMESTAMPS) {
		flyback_state.count++;
	}
	flyback_copy_rtc(&flyback_state.ts[flyback_state.count], mark);
	flyback_stopwatch();
	flyback_statechange();
}

static void flyback_statechange()
{
	if (flyback_screens[flyback_state.mode].statechange)
		flyback_screens[flyback_state.mode].statechange();
}

static void flyback_stopwatch()
{
	if (flyback_state.count > 0) {
		flyback_copy_rtc(&flyback_state.ts[0], 0);
		if (flyback_diff_ts(&flyback_state.ts[flyback_state.count],
							&flyback_state.ts[0], &flyback_state.seconds) < 0)
			flyback_state.seconds = -1;
	}
	if (flyback_screens[flyback_state.mode].stopwatch)
		flyback_screens[flyback_state.mode].stopwatch();
}

static void flyback_event(enum sys_message msg)
{
	if (flyback_screens[flyback_state.mode].event)
		flyback_screens[flyback_state.mode].event(msg);
	if (msg & SYS_MSG_RTC_SECOND)
		flyback_stopwatch();
}

/**********************************************************************/
/************************ MENU CALLBACKS ******************************/
/**********************************************************************/
static void flyback_activate()
{
	lcd_screens_create(FLYBACK_END);
	for (int i = 0; i < FLYBACK_END; i++) {
		if (flyback_screens[i].init) {
			flyback_screens[i].init();
		}
	}
	flyback_state.mode = FLYBACK_END - 1;
	flyback_num_pressed();

	sys_messagebus_register(&flyback_event,
	                        SYS_MSG_RTC_HOUR | SYS_MSG_RTC_MINUTE | SYS_MSG_RTC_SECOND
	);
}

static void flyback_deactivate()
{
	sys_messagebus_unregister_all(&flyback_event);

	/* destroy virtual screens */
	lcd_screens_destroy();

	/* clean up screen */
	display_clear(FLYBACK_FIRST_SCREEN, 0);
}

static void flyback_num_pressed()
{
	if (++flyback_state.mode == FLYBACK_END) {
		flyback_state.mode = FLYBACK_FIRST_SCREEN;
	}
	lcd_screen_activate(flyback_state.mode);

	if (flyback_screens[flyback_state.mode].enter) {
		flyback_screens[flyback_state.mode].enter();
	}

	flyback_statechange();
	flyback_event(SYS_MSG_RTC_HOUR | SYS_MSG_RTC_MINUTE | SYS_MSG_RTC_SECOND);
}

static void flyback_reset_all()
{
	flyback_state.count = 0;
	flyback_state.mode = FLYBACK_END - 1;
	flyback_num_pressed();
}

static void flyback_reset_one()
{
	static uint16_t flyback_last_press = 0;

	/* After CONFIG_BUTTONS_LONG_PRESS_TIME the longpress callbacks
	 * get called every 50ms, which will quickly delete all entries.
	 * To prevent that, store the 20Hz counter value and make sure
	 * there is a little time between invocations.
	 */
	if (timer0_20hz_counter - flyback_last_press < CONFIG_BUTTONS_LONG_PRESS_TIME) {
		flyback_last_press = timer0_20hz_counter;
		return;
	} else {
		flyback_last_press = timer0_20hz_counter;
	}

	if (flyback_state.count > 0) {
		flyback_state.count--;
		if (flyback_screens[flyback_state.mode].enter) {
			flyback_screens[flyback_state.mode].enter();
		}
		flyback_stopwatch();
		flyback_statechange();
	}
}

static void flyback_up_pressed()
{
	if (flyback_screens[flyback_state.mode].updown)
		flyback_screens[flyback_state.mode].updown(FLYBACK_MARK_UP);
}

static void flyback_down_pressed()
{
	if (flyback_screens[flyback_state.mode].updown)
		flyback_screens[flyback_state.mode].updown(FLYBACK_MARK_DOWN);
}

void mod_flyback_init()
{
	menu_add_entry("FLYBK",
	               &flyback_up_pressed,   /* up         */
	               &flyback_down_pressed, /* down       */
	               &flyback_num_pressed,  /* num        */
	               &flyback_reset_all,    /* star long  */
	               &flyback_reset_one,    /* num long   */
	               NULL,                  /* up + down  */
	               &flyback_activate,     /* activate   */
	               &flyback_deactivate);  /* deactivate */
}
