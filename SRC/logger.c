/*
    UF_IPMC/logger.c

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
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "logger.h"

void logger(const char* tag, const char* fmt, ...) {
	time_t now;
	time(&now);
	FILE* fp;

	fp = fopen("/var/log/uf_ipmc.log", "a+");
	if (fp != NULL) {
		va_list argp;
		
		va_start(argp, fmt);
		fprintf(fp, "\n\n%s [%s]:\n ", ctime(&now), tag);
		fprintf(fp, fmt, argp);
		va_end(argp);
		fflush(fp);
		fclose(fp);
	}
}
