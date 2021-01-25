/*
    UF_IPMC/debug.h

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

/* subsystem */
#define DBG_IPMI	0x0100
#define DBG_SERIAL	0x0200
//#pragma once

#define DBG_I2C		0x0400
#define DBG_WD		0x0800
#define DBG_TIMER	0x1000
#define DBG_LAN		0x2000
#define DBG_GPIO	0x4000
#define DBG_WS		0x8000

/* qualifier */
#define DBG_INOUT	0x01	/* function ingress & egress */
#define DBG_ERR		0x02	/* error messages */
#define DBG_WARN	0x04	/* warning messages */
#define DBG_LVL1	0x08	/* operation information messages, mainly used for debugging */
#define DBG_LVL2	0x10

#define USE_DPRINTF

#ifdef USE_DPRINTF
void dbprintf(unsigned flags, char *fmt, ...);
#endif
void putstr( char *str );
void dputstr( unsigned flags, char *str);

#ifndef USE_DPRINTF
#define dprintf
#endif

void puthex( unsigned char ch );
