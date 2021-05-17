/*
    UF_IPMC/user-sensor.c

    Copyright (C) 2020 Aleksei Greshilov
    aleksei.greshilov@cern.ch

    UF_IPMC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    UF_IPMC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with UF_IPMC.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "string.h"
#include "ipmi.h"
#include "picmg.h"
#include "event.h"
#include "ipmc.h"
#include "i2c.h"
#include "i2c-sensor.h"
#include "event.h"
#include "debug.h"
#include "timer.h"
#include "sensor.h"
#include "logger.h"
#include "user-sensor.h"
#include "semaphore.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#define qbv_on_off              0x1220000

extern unsigned long long int lbolt;
extern unsigned long long int payload_timeout_init;
extern FRU_CACHE fru_inventory_cache[];
extern unsigned char current_sensor_count;
extern SDR_ENTRY sdr_entry_table[];
extern int i2c_fd_snsr[];
extern pthread_mutex_t* mutex[];
extern FULL_SENSOR_RECORD sdr[];
extern SENSOR_DATA sd[];

/*==============================================================*/
/* Local Function Prototypes					*/
/*==============================================================*/
int user_module_payload_status( void );
void pok_state_poll( unsigned char *arg );

/*==============================================================*/
/* USER SEMAPHORE INITIALIZATION				*/
/*==============================================================*/
void
semaphore_initialize( void )
{
	if (create_semaphore(1) < 0) 
	{
		logger("ERROR", "Semaphore initialization failed for sensor bus 1");
	}

	if (create_semaphore(2) < 0) 
	{
		logger("ERROR", "Semaphore initialization failed for sensor bus 2");
	}
}

/*==============================================================
 * USER SENSOR STATE POLL INITIALIZATION
 *==============================================================*/
void
user_sensor_state_poll( void )
{
	/*==============================================================*/
	/* 		State Poll Functions call			*/
	/*==============================================================*/
	pok_state_poll ( 0 );

} // end of user_module_init() function



/*==============================================================
 * USER MODULE SENSORS INITIALIZATION
 *==============================================================*/
void
user_module_sensor_init( void )
{
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+		Dummy Temperature Sensor			      +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[4].scan_function = read_sensor_temp_dummy;



	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+		Temperature Sensor	(Temp 1)		      +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[5].scan_function = read_sensor_temp_1;



	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+		FPGA modules POK Sensors			      +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[6].scan_function = read_sensor_top_pok;
	sd[7].scan_function = read_sensor_bottom_pok;


} // end of user_module_sensor_init() function

// this function reads payload power status
// returns 1 if power is on and POK flags should be stable
// returns 2 if power was just turned on and POK flags may be unstable
int 
user_module_payload_status( void )
{
	unsigned int payload_read;
//	static int ramp_up_cnt = 0; // ramp up timie counter

	payload_read = reg_read(devmem_ptr, qbv_on_off);
	payload_read &= 0x20;
	if (payload_read != 0)
	{
//		if (ramp_up_cnt < 100) 
		if (lbolt - payload_timeout_init < 1000)	// lbolt - payload_timeout_init < 0.1 sec		
		{
//			ramp_up_cnt++;
			return 2; // ramping up, POK flags are not stable yet
		}
		else
		{
			return 1;  // power on and ramp up counter expired, POK flags stable
		}
	}
//	else
//	{
//		ramp_up_cnt = 0;
//	}
	return 0;
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+	Dummy Temperature Sensor				      +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void 
read_sensor_temp_dummy( void )
{
	sd[4].last_sensor_reading = 0x19; //25 degrees C
	sd[4].sensor_scanning_enabled = 1;
	sd[4].event_messages_enabled = 1;
	sd[4].unavailable = 0;

	if (sd[4].last_sensor_reading < sdr[4].lower_critical_threshold
	|| sd[4].last_sensor_reading > sdr[4].upper_critical_threshold)
	{
		FRU_TEMPERATURE_EVENT_MSG_REQ msg;
	
		msg.command = 0x02;
		msg.evt_msg_rev = 0x04;
		msg.sensor_type = 0x01;
		msg.sensor_number = 0x04;
		msg.evt_direction = 0x01;
		msg.evt_data2_qual = 0x01;
		msg.evt_data3_qual = 0x01;
		msg.evt_reason = 0x09;
		msg.temp_reading = sd[4].last_sensor_reading;
		msg.threshold = 0x4b;

		ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
	}
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+	Temperature Sensor (Temp 1)				      +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void 
read_sensor_temp_1( void )
{
	unsigned char result;
	
	i2c_read(i2c_fd_snsr[0], 0x48, 0x00, &result);
	sd[5].last_sensor_reading = result; 
	sd[5].sensor_scanning_enabled = 1;
	sd[5].event_messages_enabled = 1;
	sd[5].unavailable = 0;

	if (sd[5].last_sensor_reading < sdr[5].lower_critical_threshold
	|| sd[5].last_sensor_reading > sdr[5].upper_critical_threshold)
	{
		FRU_TEMPERATURE_EVENT_MSG_REQ msg;
	
		msg.command = 0x02;
		msg.evt_msg_rev = 0x04;
		msg.sensor_type = 0x01;
		msg.sensor_number = 0x04;
		msg.evt_direction = 0x01;
		msg.evt_data2_qual = 0x01;
		msg.evt_data3_qual = 0x01;
		msg.evt_reason = 0x09;
		msg.temp_reading = sd[5].last_sensor_reading;
		msg.threshold = 0x4b;

		ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
	}
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+	FPGA top POK Sensors					      +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int 
read_sensor_pok(int first_time, int i2c_bus, int sensor_number, int topbot)
{
	char line[1000];
	int res = 0;
	int skip_sensor = 0;

	// program the multiplexor for sensor access
	res += i2c_write(i2c_fd_snsr[i2c_bus], 0x71, 0, 0); // disconnect all
	res += i2c_write(i2c_fd_snsr[i2c_bus], 0x71, 0, 1); // select expansion register

	if (first_time == 1)
	{
		// initialize expansion register on first pass
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x09, 0x55); // ports  7: 4 FF selection outputs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x0a, 0x55); // ports 11: 8 FF selection outputs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x0b, 0x55); // ports 15:12 FF selection outputs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x0c, 0x55); // ports 19:16 FF selection outputs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x0d, 0x55); // ports 23:20 FF selection outputs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x0e, 0xff); // ports 27:24 POK inputs, pullups
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x0f, 0xff); // ports 31:28 POK inputs, pullups
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x40, 0xff); // deselect FFs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x48, 0xff); // deselect FFs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x50, 0xff); // deselect FFs
		res += i2c_write(i2c_fd_snsr[i2c_bus], 0x45, 0x04, 0x01); // enable register

	}

	// read POK sensors
	u8 regval;
	res += i2c_read (i2c_fd_snsr[i2c_bus], 0x45, 0x58, &regval); // reads ports 24..31 from register

	if (res != 0)
	{
		// I2C devices are not accessible on this module
		// log a message and stop polling
		sprintf (line, "%s Module POK I2C failure: %d. Marking this sensor as invalid\n", topbot == 1 ? "Top" : "Bottom", res);
		logger ("WARNING", line);
		skip_sensor = 1; 
		sd[sensor_number].last_sensor_reading = 0;
		sd[sensor_number].sensor_scanning_enabled = 0;
		sd[sensor_number].event_messages_enabled = 0;
		sd[sensor_number].unavailable = 1;
		return skip_sensor;
	}

	// mask POK bits
	u8 pok = regval & 0x3f;
//	printf ("##################### POK value: %x ########################\n", pok);

	sd[sensor_number].last_sensor_reading = pok;
	sd[sensor_number].sensor_scanning_enabled = 1;
	sd[sensor_number].event_messages_enabled = 1;
	sd[sensor_number].unavailable = 0;

	// shut down payload if any of the POK bits is down and the power is on
	if (pok != 0x3f && user_module_payload_status() == 1 && fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M4_ACTIVE) 
	{
		sprintf (line, "Abnormal %s Module Power OK bits: %x Shutting down payload power\n", topbot == 1 ? "Top" : "Bottom", pok);
		logger ("POWER FAILURE", line);
		module_payload_off();
	}
	return skip_sensor;
}

void 
read_sensor_top_pok( void )
{
	lock(1);

	static int skip_sensor = 0;

	if (skip_sensor != 1)
	{
		static int first_time = 1;

		// params: first_time, i2c_bus, sensor_number, top/bottom (1/2)
		skip_sensor = read_sensor_pok (first_time, 1, 6, 1);

		first_time = 0;
	}

	unlock(1);
}

void 
read_sensor_bottom_pok( void )
{
	lock(2);

	static int skip_sensor = 0;
	if (skip_sensor != 1)
	{
		static int first_time = 1;

		// params: first_time, i2c_bus, sensor_number, top/bottom (1/2)
		skip_sensor = read_sensor_pok (first_time, 2, 7, 2); 

		first_time = 0;
	}

	unlock(2);
}

void
pok_state_poll( unsigned char *arg )
{
	unsigned char top_pok_timer_handle;

	read_sensor_top_pok();	
	read_sensor_bottom_pok();	
	
	// Re-start the timer
	timer_add_callout_queue( (void *)&top_pok_timer_handle,
			0.001*SEC, pok_state_poll, 0 ); /* 0 sec timeout */
}

