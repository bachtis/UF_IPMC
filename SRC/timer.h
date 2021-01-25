/*
    UF_IPMC/timer.h

    Original work Copyright (C) 2007-2008 Gokhan Sozmen
    http://www.coreipm.com

    Modified work Copyright 2020 Aleksei Greshilov
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
#define SEC 10000 	
//#define SEC 1 	
#define CQE_FREE	0
#define CQE_ACTIVE	1
#define CQE_PENDING	3

extern void timer_initialize(void);
extern void timer_process_callout_queue( void );
extern int timer_add_callout_queue( 
	void *handle,
	unsigned long ticks, 
	void(*func)( unsigned char *), 
	unsigned char *arg );
extern void timer_remove_callout_queue( void *handle );
unsigned long timer_get_expiration_time( void *handle );
void timer_reset_callout_queue( void *handle, unsigned long ticks );



