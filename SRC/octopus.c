#include "octopus.h"
#include <unistd.h>
#include "logger.h"



int twos_complement(int val,u8 bits) {
  if ((val & (1<< (bits-1)))!=0)
    return val-(1<<bits);
  return val;
}


int i2c_read_octopus_bus(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 reg, u8 *result,u8 reg16b) {
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,2,slave_addr<<1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,3,reg&0xff);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,4,(reg>>8)&0xff);
  u8 d=0;
  if (reg16b)
    d=0xc;
  else
    d=0xe;

  d=d|(octopus_bus<<4);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,d);
  u8 resu=0;
  while (resu==0) {
    res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,0,&resu);
  }
  res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,1,result);
  return (res!=0) ||(resu==0x02);
}

int i2c_read_word_octopus_bus(int i2c_fd_snsr, u8 octopus_bus,u8 slave_addr, u8 reg, int *result,u8 reg16b) {
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,2,slave_addr<<1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,3,reg&0xff);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,4,(reg>>8)&0xff);
  u8 d=0;
  if (reg16b)
    d=0x8;
  else
    d=0xa;

  d=d|(octopus_bus<<4);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,d);
  u8 resu=0;
  while (resu==0) {
    res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,0,&resu);
  }
  u8 r0,r1;
  res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,1,&r0);
  res+=i2c_read(i2c_fd_snsr,MACHXO2_ADDR,2,&r1);
  *result=(r1<<8)|r0;
  return (res!=0) ||(resu==0x02);
}



int i2c_write_octopus_bus(int i2c_fd_snsr,u8 octopus_bus,u8 slave_addr, u8 reg, u8 data,u8 reg16b) {
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,2,slave_addr<<1);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,3,reg&0xff);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,4,(reg>>8)&0xff);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,5,data&0xff);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,6,(data>>8)&0xff);

  u8 d=0x4;
  if (reg16b==0)
    d=0x6;
  d=d|(octopus_bus<<4);
  res+=i2c_write(i2c_fd_snsr,MACHXO2_ADDR,1,d);
  
  u8 result=0;
  while (result==0) {
    i2c_read(i2c_fd_snsr,MACHXO2_ADDR,0,&result);
  }
  return (res!=0) || (result==0x02);
}

int i2c_read_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, u8 *result,u8 reg16b) {
  int res =  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,slave_addr<<1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,reg&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,(reg>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,6,optical_bus,0);

  u8 d=0;
  if (reg16b)
    d=0xc;
  else
    d=0xe;
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,d,0);
  u8 resu=0;
  while (resu==0) {
    res += i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,&resu,0);
  }
  if (resu==0x02)
    logger("ERROR","I2C ACK failed on optical bus read");
  res+=i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,result,0);
  return (res!=0) || ((resu)==0x02);
}

int i2c_read_word_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, int *result,u8 reg16b) {
  int res =  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,slave_addr<<1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,reg&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,(reg>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,6,optical_bus,0);

  u8 d=0;
  if (reg16b)
    d=0x8;
  else
    d=0xa;
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,d,0);

  u8 resu=0;
  while (resu==0) {
    res += i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,&resu,0);
  }
  if (resu==0x02)
    logger("ERROR","I2C ACK failed on read word of optical bus");

  u8 r0,r1;
  res+=i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,&r0,0);
  res+=i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,&r1,0);
  int a = r1<<8;

  *result=(a|r0);
  return (res!=0) || ((resu)==0x02);
}

int i2c_write_optical_bus(int i2c_fd_snsr, u8 optical_bus,u8 slave_addr, u8 reg, u8 data,u8 reg16b) {
  int res =  i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,1,slave_addr<<1,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,2,reg&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,3,(reg>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,4,data&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,5,(data>>8)&0xff,0);
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,6,optical_bus,0);
  u8 d=0;
  if (reg16b)
    d=0x4;
  else
    d=0x6;
  res += i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,d,0);

  u8 resu=0;
  while (resu==0) {
    res += i2c_read_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR,0,&resu,0);
  }
  if (resu==0x02) {
    logger("ERROR","I2C ACK failed on optical bus write ");
    //    printf("optical write failure bus =%d addr=%d reg=%d\n",optical_bus,slave_addr,reg);
  }
  return (res!=0) || ((resu)==0x02);
}




float  readTemperature(int i2c_fd_snsr,u8 sensor,u8 number,u8 local) {
  u8 lsb=0;
  u8 msb=0;

  //FOR TMP461
  u8 reg1=0x1;
  u8 reg2=0x10;
  if (local) {
    reg1= 0;
    reg2= 0x15;
  }
  u8 addr;

  if (sensor==RAIL_2V7_INTERMEDIATE) {
    addr=0x4d;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg2,&lsb,0);
    return twos_complement((msb<<4)|(lsb>>4),12)*0.0625;
  }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_N) {
    if (number==0)
      addr = 0x48;
    else
      addr = 0x49;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_N) {
    if (number==0)
      addr = 0x4b;
    else
      addr = 0x4c;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_S) {
    if (number==0)
      addr = 0x48;
    else
      addr = 0x49;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_S) {
    if (number==0)
      addr = 0x4b;
    else
      addr = 0x4c;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==VUP) {
    u8 addr=0x4a;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,NORTH_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==K7) {
    u8 addr=0x4a;
    int res = i2c_read_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr,reg1,&msb,0);
    res+= i2c_read_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_0V85_VCCINT_VUP) {
    u8 reg=0;
    if (number==0) { 
      addr = 0x4d;
      reg  = 0x8d;
    }
    else if (number==1) {
      addr = 0x4d;
      reg  = 0x8e;
    }
    else if (number==2) {
      addr = 0x4e;
      reg  = 0x8d;
    }
    else if (number==3) {
      addr = 0x4e;
      reg  = 0x8e;
    }
    else if (number==4) {
      addr = 0x4f;
      reg  = 0x8d;
    }
    else if (number==5) {
      addr = 0x4f;
      reg  = 0x8e;
    }
    int data;
    int res = i2c_read_word_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr,reg,&data,0);
    logger("EVENT","VCCINT Res = %d",res);
    //reverse bytes
    int byte1 = data&0xff;
    int byte0 = (data>>8)& 0xff;
    int raw =(byte0<<8)|byte1;
    //linear 11
    int Y = 0x7ff &raw;
    //twos complement
    if (Y & (1<<10))
      Y=Y-(1<<11);
    int N=(0xf800 &raw)>>11;
    float exponent;
    if (N & (1<<4)) {
      N=N-(1<<5);
      exponent = 1.0/(2<<(-N));
    }
    else {
      exponent = (2<<(N));
    }
    return Y*exponent;

  }
  else if (sensor==RAIL_OPTICAL_3V3_G0) {
    u8 addr=0x48;
    int res = i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,addr,reg1,&msb,0);
    res+= i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_OPTICAL_3V3_G1) {
    u8 addr=0x48;
    int res = i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,addr,reg1,&msb,0);
    res+= i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_OPTICAL_3V3_G2) {
    u8 addr=0x48;
    int res = i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,addr,reg1,&msb,0);
    res+= i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_OPTICAL_3V3_G3) {
    u8 addr=0x48;
    int res = i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,addr,reg1,&msb,0);
    res+= i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else if (sensor==RAIL_OPTICAL_3V3_G4) {
    u8 addr=0x48;
    int res = i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,addr,reg1,&msb,0);
    res+= i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,addr,reg2,&lsb,0);
    return (twos_complement((msb<<4)|(lsb>>4),12)*0.0625);
  }
  else {
    return -1.0;
    
  }


}
void configure_octopus_voltage_sensors(int i2c_fd_snsr) {
  int res=i2c_write_octopus_bus(i2c_fd_snsr,NORTH_BUS,0x1d, 0x0b,0x02,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,NORTH_BUS,0x1f, 0x0b,0x02,0);
  res+i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,0x1d, 0x0b,0x02,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,0x1f, 0x0b,0x02,0);

  res=i2c_write_octopus_bus(i2c_fd_snsr,NORTH_BUS,0x1d, 0x3,0xff,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,NORTH_BUS,0x1f, 0x3,0xff,0);
  res+i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,0x1d, 0x3,0xff,0);
  res+=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,0x1f, 0x3,0xff,0);


}
void configure_qsfpdd_voltage_sensors(int i2c_fd_snsr) {
  int res=i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_MON_BUS,0x1d, 0x00,0x00,0);
  res=i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_MON_BUS,0x1d, 0x0b,0x02,0);
}


//set interrupt thresholds for temperature! Needed during powerwn up!

void  configure_qsfpdd_temperature_sensors(int i2c_fd_snsr,u8 sensor,u8 number,u8 local,u8 remote) {
  u8 lsb=0;
  u8 msb=0;
  u8 addr;
  u8 bus;
    
  if (sensor==RAIL_OPTICAL_3V3_G0) {
    bus=OPTICAL_G0_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G1) {
    bus=OPTICAL_G1_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G2) {
    bus=OPTICAL_G2_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G3) {
    bus=OPTICAL_G3_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G4) {
    bus=OPTICAL_G4_BUS;
    addr=0x48;
  }
  int res = i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x21,10,0);
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x19,local,0);
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,addr,0x20,remote,0);
}

void  configure_octopus_temperature_sensors(int i2c_fd_snsr,u8 sensor,u8 number,u8 local,u8 remote) {
  u8 lsb=0;
  u8 msb=0;
  u8 addr;
  u8 bus;
    

  if (sensor==RAIL_2V7_INTERMEDIATE) {
    addr=0x4d;
    bus=NORTH_BUS;
  }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_N) {
    bus=NORTH_BUS;
    if (number==0)
      addr = 0x48;
    else
      addr = 0x49;
   }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_N) {
    bus=NORTH_BUS;
    if (number==0)
      addr = 0x4b;
    else
      addr = 0x4c;
   }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_S) {
    bus=SOUTH_BUS;

    if (number==0)
      addr = 0x48;
    else
      addr = 0x49;
   }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_S) {
    bus=SOUTH_BUS;

    if (number==0)
      addr = 0x4b;
    else
      addr = 0x4c;
 
  }
  else if (sensor==VUP) {
    bus=NORTH_BUS;
    addr=0x4a;
  }
  else if (sensor==K7) {
    bus=SOUTH_BUS;
    addr=0x4a;
  }

  else if (sensor==RAIL_OPTICAL_3V3_G0) {
    bus=OPTICAL_G0_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G1) {
    bus=OPTICAL_G1_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G2) {
    bus=OPTICAL_G2_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G3) {
    bus=OPTICAL_G3_BUS;
    addr=0x48;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G4) {
    bus=OPTICAL_G4_BUS;
    addr=0x48;
  }
  int res = i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x21,10,0);
  res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x19,local,0);
  res+= i2c_write_octopus_bus(i2c_fd_snsr,bus,addr,0x20,remote,0);
}



float  readVoltage(int i2c_fd_snsr,u8 sensor) {
  u8 addr=0;
  u8 bus=0;
  u8 board=3;
  u8 ch=0;
  float factor=0;
  if (sensor==RAIL_12V0) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1d;
    ch=0;
    factor=7.72;
  }
  else if (sensor==RAIL_3V3_STANDBY) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1d;
    ch=2;
    factor=2.0;
  }
  else if (sensor==RAIL_3V3_SI5395J) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1d;
    ch=4;
    factor=2.0;
  }
  else if (sensor==RAIL_1V8_SI5395J_XO2) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1d;
    ch=7;
    factor=1.0;
  }
  else if (sensor==RAIL_2V5_OSC_NE) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1f;
    ch=0;
    factor=2.0;
  }
  else if (sensor==RAIL_1V8_MGTVCCAUX_VUP_N) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1f;
    ch=1;
    factor=1.0;
  }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_N) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1f;
    ch=2;
    factor=1.0;
  }
  else if (sensor==RAIL_2V7_INTERMEDIATE) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1f;
    ch=3;
    factor=2.0;
  }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_N) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1f;
    ch=4;
    factor=1.0;
  }
  else if (sensor==RAIL_2V5_OSC_NW) {
    bus=NORTH_BUS;
    board=0;
    addr=0x1f;
    ch=5;
    factor=2.0;
  }
  else if (sensor==RAIL_1V0_VCCINT_K7) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=0;
    factor=1.0;
  }
  else if (sensor==RAIL_2V5_OSC_K7) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=1;
    factor=2.0;
  }
  else if (sensor==RAIL_1V2_MGTAVTT_K7) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=2;
    factor=1.0;
  }
  else if (sensor==RAIL_1V0_MGTAVCC_K7) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=3;
    factor=1.0;
  }
  else if (sensor==RAIL_0V675_DDR_VTT) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=4;
    factor=1.0;
  }
  else if (sensor==RAIL_1V35_DDR) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=5;
    factor=1.0;
  }
  else if (sensor==RAIL_1V8_VCCAUX_K7) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=6;
    factor=1.0;
  }
  else if (sensor==RAIL_2V5_OSC_SE) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1d;
    ch=7;
    factor=2.0;
  }
  else if (sensor==RAIL_1V8_MGTVCCAUX_VUP_S) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1f;
    ch=0;
    factor=1.0;
  }
  else if (sensor==RAIL_0V9_MGTAVCC_VUP_S) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1f;
    ch=1;
    factor=1.0;
  }
  else if (sensor==RAIL_1V2_MGTAVTT_VUP_S) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1f;
    ch=2;
    factor=1.0;
  }
  else if (sensor==RAIL_0V85_VCCINT_VUP) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1f;
    ch=3;
    factor=1.0;
  }
  else if (sensor==RAIL_1V8_VCCAUX_VUP) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1f;
    ch=4;
    factor=1.0;
  }
  else if (sensor==RAIL_2V5_OSC_SW) {
    bus=SOUTH_BUS;
    board=0;
    addr=0x1f;
    ch=5;
    factor=2.0;
  }
  else if (sensor==RAIL_OPTICAL_3V3_STANDBY) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=0;
    factor=11.0;
  }
  else if (sensor==RAIL_OPTICAL_12V0) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=1;
    factor=11.0;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G0) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=2;
    factor=11.0;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G1) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=3;
    factor=11.0;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G2) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=4;
    factor=11.0;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G3) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=5;
    factor=11.0;
  }
  else if (sensor==RAIL_OPTICAL_3V3_G4) {
    bus=OPTICAL_MON_BUS;
    board=1;
    addr=0x1d;
    ch=6;
    factor=11.0;
  }
  if (board==0) {
    u8 not_ready=1;
    int res=i2c_read_octopus_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    while (not_ready) {
      usleep(10000);
      int res=i2c_read_octopus_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    }
    res=i2c_write_octopus_bus(i2c_fd_snsr,bus,addr, 0x9,1,0);
    not_ready=1;  
    res+=i2c_read_octopus_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    while (not_ready) {
      usleep(10000);
      res+=i2c_read_octopus_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    }
    int result;
    res=i2c_read_word_octopus_bus(i2c_fd_snsr,bus,addr, 0x20+ch,&result,0);
    return 2.56*result*factor/65536.0;
  }
  else if (board==1) {
    u8 not_ready=1;
    int res=i2c_read_optical_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    while (not_ready) {
      usleep(10000);
      res=i2c_read_optical_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    }
    res=i2c_write_optical_bus(i2c_fd_snsr,bus,addr, 0x9,1,0);
    res=i2c_read_optical_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    while (not_ready) {
      usleep(10000);
      res=i2c_read_optical_bus(i2c_fd_snsr,bus,addr, 0xc,&not_ready,0);
    }
    int result;
    res=i2c_read_word_optical_bus(i2c_fd_snsr,bus,addr, 0x20+ch,&result,0);
    return 2.56*result*factor/65536.0;


  }
  else {
    return -1.0;
  }

}

void check_rail(int i2c_fd_snsr,u8 sensor,float min,float max) {
  usleep(500000);
  float v = readVoltage(i2c_fd_snsr,sensor);
  if (v<min||v>max) {
    emergency_power_down_octopus(i2c_fd_snsr);
    logger("ERROR", "Failed to Power On the board, Powering Down");
  }
  //Also always check temperature of FPGAs in case of weird sorts that could overheat it
  float t = readTemperature(i2c_fd_snsr,VUP,0,0);
  if (t>60.0) {
    emergency_power_down_octopus(i2c_fd_snsr);
    logger("ERROR", "VUP too hot while powering up, Powering Down");
  }
  t = readTemperature(i2c_fd_snsr,K7,0,0);
  if (t>60.0) {
    emergency_power_down_octopus(i2c_fd_snsr);
    logger("ERROR", "K7 too hot while powering up, Powering Down");
  }   
}

                   
void emergency_power_down_octopus(int i2c_fd_snsr) {
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0);
}

void emergency_power_down_qsfpdd_module(int i2c_fd_snsr) {
  int res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 7, 0x0,0);
}

  

void power_down_octopus(int i2c_fd_snsr) {
  //disable latches
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,10,1);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1f);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0xf);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x7);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x3);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x0);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0xf);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x7);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x3);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x1);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x0);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x3f);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1f);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0xf);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x7);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x3);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1);
  usleep(100000);
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x0);
  usleep(100000);
}
void activate_vcc_int_channel(int i2c_fd_snsr,u8 addr,u8 channel) {
  int res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x00,channel,0);
  res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x01,0x80,0);
  res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x03,0x00,0);
  res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x00,0x00,0);
}

void deactivate_vcc_int_channel(int i2c_fd_snsr,u8 addr,u8 channel) {
  int res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x00,channel,0);
  res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x01,0x00,0);
  res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x03,0x00,0);
  res=i2c_write_octopus_bus(i2c_fd_snsr,SOUTH_BUS,addr, 0x00,0x00,0);

}

    
void power_up_octopus(int i2c_fd_snsr) {

  //MASK ADC alarms since we have PGOOD anyway
  configure_octopus_voltage_sensors(i2c_fd_snsr);

  //set shutdown temperature limits for local sensor (PCB) and remote (device
  // 1 TMP per regulator (2 for VTT/VCC rails) and then FPGAs
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_2V7_INTERMEDIATE,0,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_N,0,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_N,1,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_S,0,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_0V9_MGTAVCC_VUP_S,1,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_N,0,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_N,1,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_S,0,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,RAIL_1V2_MGTAVTT_VUP_S,1,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,VUP,0,95,100);
  configure_octopus_temperature_sensors(i2c_fd_snsr,K7,0,95,100);


  //disable internal safety system
  int res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,10,1);
  //power up

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1);
  usleep(100000);

 
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x7);
  usleep(100000);



  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0xf);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x1f);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x3f);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0x7f);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,7,0xff);
  usleep(100000);



  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x1);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x3);
  usleep(100000);



  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x7);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0xf);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,9,0x1f);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x3);
  usleep(100000);

  
  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x7);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0xf);
  usleep(100000);


  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x1f);
  usleep(100000);

  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,8,0x3f);
  usleep(100000);

  //Deactivate 2 out of 6 LTM4700 channels for now (4d,4f)
  activate_vcc_int_channel(i2c_fd_snsr,0x4d,0);
  deactivate_vcc_int_channel(i2c_fd_snsr,0x4d,1);
  activate_vcc_int_channel(i2c_fd_snsr,0x4e,0);
  activate_vcc_int_channel(i2c_fd_snsr,0x4e,1);
  activate_vcc_int_channel(i2c_fd_snsr,0x4f,0);
  deactivate_vcc_int_channel(i2c_fd_snsr,0x4f,1);
  usleep(300000);


  //keep internal safety system disabled for now since 
  //the PGOOD is 0 and trips the board -because of the deactivation
  //of the channels above
  //  res = i2c_write(i2c_fd_snsr,MACHXO2_ADDR,10,0);
}




void power_up_qsfpdd_module(int i2c_fd_snsr) {

  //  configure_qsfpdd_voltage_sensors(i2c_fd_snsr);
  //check that standalone power and 12V0 exists 
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_12V0,11,13);  
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_3V3_STANDBY,3.2,3.4);  
  int res = i2c_write_octopus_bus(i2c_fd_snsr,OPTICAL_BUS,OPTICAL_ADDR, 7, 0x1f,0);
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_3V3_G0,3.1,3.4);  
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_3V3_G1,3.1,3.4);  
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_3V3_G2,3.1,3.4);  
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_3V3_G3,3.1,3.4);  
  //  check_rail(i2c_fd_snsr,RAIL_OPTICAL_3V3_G4,3.1,3.4);  
  usleep(100000);
  configure_optical_io(i2c_fd_snsr);
  usleep(100000);

}

void power_down_qsfpdd_module(int i2c_fd_snsr) {
  emergency_power_down_qsfpdd_module(i2c_fd_snsr);

}


void configure_optical_io(int i2c_fd_snsr) {
  //write I/O register configuration
  int res=0;
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x20, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x20, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x20, 0x06, 0xef,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x20, 0x07, 0xee,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x21, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x21, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x21, 0x06, 0xff,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x21, 0x07, 0xff,0); 

  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x20, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x20, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x20, 0x06, 0xef,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x20, 0x07, 0xee,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x21, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x21, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x21, 0x06, 0xff,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x21, 0x07, 0xff,0); 

  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x20, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x20, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x20, 0x06, 0xef,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x20, 0x07, 0xee,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x21, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x21, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x21, 0x06, 0xff,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x21, 0x07, 0xff,0); 

  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x20, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x20, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x20, 0x06, 0xef,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x20, 0x07, 0xee,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x21, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x21, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x21, 0x06, 0xff,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x21, 0x07, 0xff,0); 

  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x20, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x20, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x20, 0x06, 0xef,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x20, 0x07, 0xee,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x21, 0x02, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x21, 0x03, 0x0,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x21, 0x06, 0xff,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x21, 0x07, 0xff,0); 
}

int detect_optics(int i2c_fd_snsr) {
  int optics=0;
  int res=0;
  u8 r0,r1;
  r0=0;
  r1=0;
  int mask=0;
  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x20,0x0,&r0,0);  
  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G0_BUS,0x20,0x1,&r1,0);  

  mask=(r1<<8)|r0;
  optics=optics|(((mask>>13)&0x1));
  optics=optics|(((mask>>9)&0x1)<<1);
  optics=optics|(((mask>>5)&0x1)<<2);

  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x20, 0x0,&r0,0);  
  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G1_BUS,0x20, 0x1, &r1,0);  


  mask=(r1<<8)|r0;
  optics=optics|(((mask>>13)&0x1)<<3);
  optics=optics|(((mask>>9)&0x1)<<4);
  optics=optics|(((mask>>5)&0x1)<<5);

  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x20, 0x0,&r0,0);  
  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G2_BUS,0x20, 0x1, &r1,0);  

  mask=(r1<<8)|r0;
  optics=optics|(((mask>>13)&0x1)<<6);
  optics=optics|(((mask>>9)&0x1)<<7);
  optics=optics|(((mask>>5)&0x1)<<8);

  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x20, 0x0,&r0,0);  
  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G3_BUS,0x20, 0x1, &r1,0);  

  mask=(r1<<8)|r0;
  optics=optics|(((mask>>13)&0x1)<<9);
  optics=optics|(((mask>>9)&0x1)<<10);
  optics=optics|(((mask>>5)&0x1)<<11);

  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x20, 0x0,&r0,0);  
  res+=i2c_read_optical_bus(i2c_fd_snsr,OPTICAL_G4_BUS,0x20, 0x1, &r1,0);  
  mask=(r1<<8)|r0;
  optics=optics|(((mask>>13)&0x1)<<12);
  optics=optics|(((mask>>9)&0x1)<<13);
  optics=optics|(((mask>>5)&0x1)<<14);
  return ((~optics)&0x7fff);

}



u8 select_qsfpdd_module(int i2c_fd_snsr,u8 cage) {
  u8 bus=0;
  u8 port=0;
  if (cage==0) {
    bus=OPTICAL_G0_BUS;
    port=14;
  }
  if (cage==1) {
    bus=OPTICAL_G0_BUS;
    port=10;
  }
  if (cage==2) {
    bus=OPTICAL_G0_BUS;
    port=6;
  }
  if (cage==3) {
    bus=OPTICAL_G1_BUS;
    port=14;
  }
  if (cage==4) {
    bus=OPTICAL_G1_BUS;
    port=10;
  }
  if (cage==5) {
    bus=OPTICAL_G1_BUS;
    port=6;
  }

  if (cage==6) {
    bus=OPTICAL_G2_BUS;
    port=14;
  }
  if (cage==7) {
    bus=OPTICAL_G2_BUS;
    port=10;
  }
  if (cage==8) {
    bus=OPTICAL_G2_BUS;
    port=6;
  }
  if (cage==9) {
    bus=OPTICAL_G3_BUS;
    port=14;
  }
  if (cage==10) {
    bus=OPTICAL_G3_BUS;
    port=10;
  }
  if (cage==11) {
    bus=OPTICAL_G3_BUS;
    port=6;
  }
  if (cage==12) {
    bus=OPTICAL_G4_BUS;
    port=14;
  }
  if (cage==13) {
    bus=OPTICAL_G4_BUS;
    port=10;
  }
  if (cage==14) {
    bus=OPTICAL_G4_BUS;
    port=6;
  }

  u8 r0,r1;
  int res=0;
  res+= i2c_read_optical_bus(i2c_fd_snsr,bus,0x20, 0x06, &r0,0); 
  res+= i2c_read_optical_bus(i2c_fd_snsr,bus,0x20, 0x07, &r1,0); 
  int mask = (r1<<8)|(r0);
  mask=mask|0x444f;
  mask=mask ^ (1<<port);
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,0x20, 0x06, mask&0xff,0); 
  res+= i2c_write_optical_bus(i2c_fd_snsr,bus,0x20, 0x07, (mask>>8)&0xff,0); 
  return bus;

}


void write_qsfpdd_module(int i2c_fd_snsr,u8 cage,u8 addr,u8 data) {
  u8 bus = select_qsfpdd_module(i2c_fd_snsr,cage);
  int res= i2c_write_optical_bus(i2c_fd_snsr,bus,0x50, addr, data,0); 
  
}
u8 read_qsfpdd_module(int i2c_fd_snsr,u8 cage,u8 addr) {
  u8 bus = select_qsfpdd_module(i2c_fd_snsr,cage);
  u8 r=0;
  int res= i2c_read_optical_bus(i2c_fd_snsr,bus,0x50, addr, &r,0); 
  return r;
}

float qsfpddTemperature(int i2c_fd_snsr) {
  int mask = detect_optics(i2c_fd_snsr);
  u8 i=0;
  float temp=25.0;
  for (i=0;i<15;++i) {
    if ((mask>>i)&0x1) {
      u8 t1 = read_qsfpdd_module(i2c_fd_snsr,i,14);
      u8 t2 = read_qsfpdd_module(i2c_fd_snsr,i,15);
      int t = ((t1<<8)|t2)&0xffff;
      if (t&(1<<15)) {
	t=(1<<16)-t;
      }
      float tt = t/256.0;
      if( tt>temp)
	temp=tt;
    }
  }
  return temp;
}

void update_leds(int i2c_fd_snsr,int original,int current) {
  int mask=0;
  u8 bus=0;
  u8 port=0;
  u8 command;
  u8 cage;
  for (cage=0;cage<15;++cage) {
    u8 wasThere =original & (1<<cage);
    u8 isThere  =current & (1<<cage);
    command=0;
    if (wasThere==0 && isThere!=0)
      command=0x4;
    if (wasThere!=0 && isThere!=0)
      command=0x2;

    if (wasThere!=0 && isThere==0)
      command=0x1;

    if (cage==0) {
      bus=OPTICAL_G0_BUS;
      port=0;
      mask=0x7;
    }
    else if (cage==1) {
      bus=OPTICAL_G0_BUS;
      port=6;
      mask=0x01c0;
      
    }
    else if (cage==2) {
      bus=OPTICAL_G0_BUS;
      port=12;
      mask=0x7000;
      
    }
    else if (cage==3) {
      bus=OPTICAL_G1_BUS;
      port=0;
      mask=0x7;
      
    }
    else if (cage==4) {
      bus=OPTICAL_G1_BUS;
      port=6;
      mask=0x01c0;

    }
    else if (cage==5) {
      bus=OPTICAL_G1_BUS;
      port=12;
      mask=0x7000;

    }

    else if (cage==6) {
      bus=OPTICAL_G2_BUS;
      port=0;
      mask=0x7;

    }
    else if (cage==7) {
      bus=OPTICAL_G2_BUS;
      port=6;
      mask=0x01c0;

    }
    else if (cage==8) {
      bus=OPTICAL_G2_BUS;
      port=12;
      mask=0x7000;

    }
    else if (cage==9) {
      bus=OPTICAL_G3_BUS;
      port=0;
      mask=0x7;

    }
    else if (cage==10) {
      bus=OPTICAL_G3_BUS;
      port=6;
      mask=0x01c0;

    }
    else if (cage==11) {
      bus=OPTICAL_G3_BUS;
      port=12;
      mask=0x7000;

    }
    else if (cage==12) {
      bus=OPTICAL_G4_BUS;
      port=0;
      mask=0x7;

    }
    else if (cage==13) {
      bus=OPTICAL_G4_BUS;
      port=6;
      mask=0x01c0;

    }
    else if (cage==14) {
      bus=OPTICAL_G4_BUS;
      port=12;
      mask=0x7000;
    }
    u8 r0,r1;
    int res=0;
    res+= i2c_read_optical_bus(i2c_fd_snsr,bus,0x21, 0x06, &r0,0); 
    res+= i2c_read_optical_bus(i2c_fd_snsr,bus,0x21, 0x07, &r1,0); 
    int cfg = (r1<<8)|(r0);
    cfg=cfg|mask;
    cfg=cfg ^ (command<<port);
    res+= i2c_write_optical_bus(i2c_fd_snsr,bus,0x21, 0x06, cfg&0xff,0); 
    res+= i2c_write_optical_bus(i2c_fd_snsr,bus,0x21, 0x07, (cfg>>8)&0xff,0); 
    
  }
}
