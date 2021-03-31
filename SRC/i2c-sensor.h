/*
    UF_IPMC/i2c-sensor.h

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
typedef unsigned char u8;

int i2c_write(int i2c_fd_snsr, u8 slave_addr, u8 reg, u8 data); 
int i2c_read(int i2c_fd_snsr, u8 slave_addr, u8 reg, u8 *result);
void i2c_sensor_initialize(void);
void i2c_sensor_deinitialize(void);
