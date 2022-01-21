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
#include "user-sensor-octopus-qsfp.h"
#include "user-payload-octopus-qsfp.h"
#include "semaphore.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include "octopus.h"
#define qbv_on_off              0x1220000

extern unsigned long long int lbolt;
extern unsigned long long int payload_timeout_init;
extern FRU_CACHE fru_inventory_cache[];
extern unsigned char current_sensor_count;
extern SDR_ENTRY sdr_entry_table[];
extern int i2c_fd_snsr[];
extern FULL_SENSOR_RECORD sdr[];
extern SENSOR_DATA sd[];

/*==============================================================*/
/* Local Function Prototypes					*/
/*==============================================================*/
int user_module_payload_status( void );
void pgood_state_poll( unsigned char *arg );
void temp_vup_state_poll( unsigned char *arg );
void temp_k7_state_poll( unsigned char *arg );
void temp_rail_2v7_intermediate_state_poll( unsigned char *arg );
void temp_mgt_vup_state_poll( unsigned char *arg );

/*==============================================================*/
/* USER SEMAPHORE INITIALIZATION				*/
/*==============================================================*/
void semaphore_initialize(void) {
	if (create_semaphore(1) < 0) {
		logger("ERROR", "Semaphore initialization failed for sensor bus 1");
	} else {
		logger("SUCCESS", "Semaphore initialization complete for sensor bus 1");
    }

	if (create_semaphore(2) < 0) {
		logger("ERROR", "Semaphore initialization failed for sensor bus 2");
	} else {
		logger("SUCCESS", "Semaphore initialization complete for sensor bus 2");
	}
}

/*==============================================================
 * USER SENSOR STATE POLL INITIALIZATION
 *==============================================================*/
void user_sensor_state_poll(void) {
	/*==============================================================*/
	/* 		State Poll Functions call			*/
	/*==============================================================*/
  //  pgood_state_poll ( 0 );
  //temp_vup_state_poll( 0 );
  //temp_k7_state_poll( 0 );
  //temp_rail_2v7_intermediate_state_poll( 0 );
  //temp_mgt_vup_state_poll( 0 );
//    temp_qsfpdd_state_poll( 0 );
//    optics_state_poll( 0 );
    //temp_rail_0v85_vccint_vup_state_poll( 0 );

} // end of user_module_init() function



/*==============================================================
 *  * USER MODULE SENSORS INITIALIZATION
 *   *==============================================================*/
void user_module_sensor_init(void) {
	/*==============================================================*/
	/* 		All Sensors											*/
	/*==============================================================*/
  sd[4].scan_function = read_sensor_pgood_remote;
  sd[5].scan_function = read_sensor_temp_vup_remote;
  sd[6].scan_function = read_sensor_temp_k7_remote;
  sd[7].scan_function = read_sensor_temp_rail_2v7_intermediate_remote;
  sd[8].scan_function = read_sensor_temp_mgt_vup_remote;
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * +	OCTOPUS and X20 PGOOD Sensor					      +
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void read_sensor_pgood_remote(void) {
    lock(1);

    //  Debug info
    char line[1000];
    int res = 0;
    //  Wrapper parameters
    u8 reg_val = 0x00;    
    //  Sensor Data Record
    u8 sensor_N = 4;

    if (check_power_up()) {
        //  Get result
        res = i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x04, &reg_val);
	printf("Reading PGPOOD = %d\n",reg_val);
        if (res != 0) {
            // I2C devices are not accessible on this module
            // log a message and stop polling
            sprintf(line, "PGOOD I2C failure: %d. Marking this sensor as invalid\n", res);
            logger("WARNING", line);
            sd[sensor_N].last_sensor_reading = 0;
            sd[sensor_N].sensor_scanning_enabled = 0;
            sd[sensor_N].event_messages_enabled = 0;
            sd[sensor_N].unavailable = 1;
        } else {
            sd[sensor_N].last_sensor_reading = reg_val;
            sd[sensor_N].sensor_scanning_enabled = 1;
            sd[sensor_N].event_messages_enabled = 1;
            sd[sensor_N].unavailable = 0;

            // Check if PGOOD value is less than cutoff
            u8 sensor_cutoff = 0xff;
            if (sd[sensor_N].last_sensor_reading != sensor_cutoff) {
                logger("WARNING","PGOOD sensor reading too low");
		printf("PGOOD sensor reading: %d\n",sd[sensor_N].last_sensor_reading);
	        unlock(1);
                picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
                lock(1);
            }
        }
    } else {
        sd[sensor_N].last_sensor_reading = 0;
        sd[sensor_N].sensor_scanning_enabled = 0;
        sd[sensor_N].event_messages_enabled = 0;
        sd[sensor_N].unavailable = 1;
    }

    unlock(1);
}

/*==============================================================*/
/* 		VUP Temp Sensor											*/
/*==============================================================*/
void read_sensor_temp_vup_remote(void) {
    lock(1);
    //	Debug info
    char line[1000];
    int res = 0;
    //	Sensor Data Record
    u8 sensor_N = 5;
    u8 base_reading=7+44;
    u8 lsb=0;
    u8 msb=0;



    if (check_power_up()) {


      u8 ready=0;
      u8 reg_val=0;
      while(!ready) {
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x03, &reg_val);
	ready=((reg_val &0x40)==0);
      }
        //	Read the temp
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, base_reading, &lsb);
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, base_reading+1, &msb);
	int tempint = (lsb>>4)|(msb<<4);
	float temp_f = twos_complement(tempint,12)*0.0625;
        //	Convert float to byte and get precision
        u8 temp_b = (u8)(temp_f);
	sd[sensor_N].last_sensor_reading = temp_b;
	sd[sensor_N].sensor_scanning_enabled = 1;
	sd[sensor_N].event_messages_enabled = 1;
	sd[sensor_N].unavailable = 0;

        static int first_time = 1;
        static int up_noncrt_assert = 0;

        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for VUP temperature sensor");
            lock(1);
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for VUP temperature sensor");
            lock(1);
        } else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
            // Assertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x01;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
            // Deassertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x81;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 0;
        }
    } else {
        sd[sensor_N].last_sensor_reading = 0;
        sd[sensor_N].sensor_scanning_enabled = 0;
        sd[sensor_N].event_messages_enabled = 0;
        sd[sensor_N].unavailable = 1;
    }

    unlock(1);
}

/*==============================================================*/
/* 		K7 Temp Sensor											*/
/*==============================================================*/
void read_sensor_temp_k7_remote(void) {
	lock(1);

	//	Debug info
	char line[1000];
	int res = 0;

	//	Sensor Data Record
	u8 sensor_N = 6;

	u8 base_reading=7+20;
	u8 lsb=0;
	u8 msb=0;
    if (check_power_up()) {
      u8 ready=0;
      u8 reg_val=0;
      while(!ready) {
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x03, &reg_val);
	ready=((reg_val &0x40)==0);
      }

        //	Read the temp
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, base_reading, &lsb);
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, base_reading+1, &msb);
	int tempint = (lsb>>4)|(msb<<4);
	float temp_f = twos_complement(tempint,12)*0.0625;
        //	Convert float to byte and get precision
        u8 temp_b = (u8)(temp_f);
	sd[sensor_N].last_sensor_reading = temp_b;
	sd[sensor_N].sensor_scanning_enabled = 1;
	sd[sensor_N].event_messages_enabled = 1;
	sd[sensor_N].unavailable = 0;


        static int first_time = 1;
        static int up_noncrt_assert = 0;

        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for K7 temperature sensor");
            lock(1);
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for K7 temperature sensor");
            lock(1);
        } else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
            // Assertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x01;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
            // Deassertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x81;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 0;
        }
    } else {
        sd[sensor_N].last_sensor_reading = 0;
        sd[sensor_N].sensor_scanning_enabled = 0;
        sd[sensor_N].event_messages_enabled = 0;
        sd[sensor_N].unavailable = 1;
    }

    unlock(1);
}

/*==============================================================*/
/* 		RAIL_2V7_INTERMEDIATE Local=0 Temp Sensor											*/
/*==============================================================*/
void read_sensor_temp_rail_2v7_intermediate_remote(void) {
	lock(1);

	//	Debug info
	char line[1000];
	int res = 0;

	//	Sensor Data Record
	u8 sensor_N = 7;
	u8 base_reading=7+32;
	u8 lsb=0;
	u8 msb=0;
	u8 reg_val;
    if (check_power_up()) {

      u8 ready=0;
      u8 reg_val=0;
      while(!ready) {
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x03, &reg_val);
	ready=((reg_val &0x40)==0);
      }

      //	Read the temp
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, base_reading, &lsb);
        res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, base_reading+1, &msb);
	int tempint = (lsb>>4)|(msb<<4);
	float temp_f = twos_complement(tempint,12)*0.0625;
        //	Convert float to byte and get precision
        u8 temp_b = (u8)(temp_f);
	sd[sensor_N].last_sensor_reading = temp_b;
	sd[sensor_N].sensor_scanning_enabled = 1;
	sd[sensor_N].event_messages_enabled = 1;
	sd[sensor_N].unavailable = 0;


        static int first_time = 1;
        static int up_noncrt_assert = 0;
        static int up_crt_assert = 0;
        static int up_nonrec_assert = 0;

        //logger("EVENT", "Int Temp (reading)  = %d, Int Temp (temp_b) = %d, Int Temp (temp_f) = %f, Threshold = %d",sd[sensor_N].last_sensor_reading,temp_b,temp_f,sdr[sensor_N].upper_non_critical_threshold);

        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for Intermediate Rail temperature sensor");
            lock(1);
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for Intermediate Rail temperature sensor");
            lock(1);
        } else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
            // Assertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x01;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
            // Deassertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x81;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 0;
        }
    } else {
        sd[sensor_N].last_sensor_reading = 0;
        sd[sensor_N].sensor_scanning_enabled = 0;
        sd[sensor_N].event_messages_enabled = 0;
        sd[sensor_N].unavailable = 1;
    }

    unlock(1);
}

/*==============================================================*/
/* 		MGT_VUP Local=0 Temp Sensor								*/
/*==============================================================*/
void read_sensor_temp_mgt_vup_remote(void) {
	lock(1);
      u8 ready=0;
      u8 reg_val=0;
      while(!ready) {
        i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x03, &reg_val);
	ready=((reg_val &0x40)==0);
      }

    //	Debug info
	char line[1000];
	int res = 0;
	u8 sensor_N = 8;
	u8 index=0;
	int i=0;
	u8 lsb=0;
	u8 msb=0;


	if (check_power_up()) {	  
	  float temp_f=30;
	  for (i=12;i<56;i=i+4) {
	    index=(i-12)/4;
	    if (index==2||index==5||index==8)
	      continue;
		//	Read the temp
	    res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 7+i, &lsb);
	    res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 7+i+1, &msb);
	    int tempint = (lsb>>4)|(msb<<4);
	    float t = twos_complement(tempint,12)*0.0625;
	    if (t>temp_f)
	      temp_f=t;
	  }
	  //This is the last octopus sensor so let's trigger a monitoring cycle
	  res += i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 142, &lsb);
	  
	  u8 temp_b = (u8)(temp_f);
	  printf("Measured temperatures=%f %d\n",temp_f,temp_b);
	  sd[sensor_N].last_sensor_reading = temp_b;
	  sd[sensor_N].sensor_scanning_enabled = 1;
	  sd[sensor_N].event_messages_enabled = 1;
	  sd[sensor_N].unavailable = 0;

	  static int first_time = 1;
	  static int up_noncrt_assert = 0;

        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for MGT VUP Rails temperature sensor");
            lock(1);
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
            unlock(1);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for MGT VUP Rails temperature sensor");
            lock(1);
        } else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
            // Assertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x01;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
            // Deassertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;

            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x81;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 0;
        }
	} else {
        sd[sensor_N].last_sensor_reading = 0;
        sd[sensor_N].sensor_scanning_enabled = 0;
        sd[sensor_N].event_messages_enabled = 0;
        sd[sensor_N].unavailable = 1;
    }

    unlock(1);
}


void pgood_state_poll(unsigned char *arg) {
    unsigned char pgood_timer_handle;

	read_sensor_pgood_remote();

	// Re-start the timer
	timer_add_callout_queue( (void *)&pgood_timer_handle,
			5*SEC, pgood_state_poll, 0 ); /* 0 sec timeout */
}

void temp_vup_state_poll( unsigned char *arg ) {
    unsigned char temp_vup_timer_handle;

	read_sensor_temp_vup_remote();

	// Re-start the timer
	timer_add_callout_queue( (void *)&temp_vup_timer_handle,
			5*SEC, temp_vup_state_poll, 0 ); /* 0 sec timeout */
}

void temp_k7_state_poll( unsigned char *arg ) {
    unsigned char temp_k7_timer_handle;

	read_sensor_temp_k7_remote();

	// Re-start the timer
	timer_add_callout_queue( (void *)&temp_k7_timer_handle,
			5*SEC, temp_k7_state_poll, 0 ); /* 0 sec timeout */
}

void temp_rail_2v7_intermediate_state_poll( unsigned char *arg ) {
    unsigned char temp_rail_2v7_intermediate_timer_handle;

	read_sensor_temp_rail_2v7_intermediate_remote();

	// Re-start the timer
	timer_add_callout_queue( (void *)&temp_rail_2v7_intermediate_timer_handle,
			5*SEC, temp_rail_2v7_intermediate_state_poll, 0 ); /* 0 sec timeout */
}

void temp_mgt_vup_state_poll( unsigned char *arg ) {
    unsigned char temp_mgt_vup_timer_handle;

	read_sensor_temp_mgt_vup_remote();

	// Re-start the timer
	timer_add_callout_queue( (void *)&temp_mgt_vup_timer_handle,
			5*SEC, temp_mgt_vup_state_poll, 0 ); /* 0 sec timeout */
}


