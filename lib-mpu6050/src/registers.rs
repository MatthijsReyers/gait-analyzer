

pub const XG_OFFS_TC: u8 = 0x00; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
pub const YG_OFFS_TC: u8 = 0x01; //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
pub const ZG_OFFS_TC: u8 = 0x02; //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

pub const XA_OFFS_H: u8 = 0x06; //[15:0] XA_OFFS
pub const XA_OFFS_L_TC: u8 = 0x07;
pub const YA_OFFS_H: u8 = 0x08; //[15:0] YA_OFFS
pub const YA_OFFS_L_TC: u8 = 0x09;
pub const ZA_OFFS_H: u8 = 0x0A; //[15:0] ZA_OFFS
pub const ZA_OFFS_L_TC: u8 = 0x0B;

pub const XG_OFFS_USRH: u8 = 0x13; //[15:0] XG_OFFS_USR
pub const XG_OFFS_USRL: u8 = 0x14;
pub const YG_OFFS_USRH: u8 = 0x15; //[15:0] YG_OFFS_USR
pub const YG_OFFS_USRL: u8 = 0x16;
pub const ZG_OFFS_USRH: u8 = 0x17; //[15:0] ZG_OFFS_USR
pub const ZG_OFFS_USRL: u8 = 0x18;

pub const SMPLRT_DIV: u8 = 0x019;

pub const CONFIG: u8 = 0x01A;
pub const GYRO_CONFIG: u8 = 0x01B;
pub const ACCEL_CONFIG: u8 = 0x01C;

pub const FF_THR: u8 = 0x1D;
pub const FF_DUR: u8 = 0x1E;
pub const MOT_THR: u8 = 0x1F;
pub const MOT_DUR: u8 = 0x20;
pub const ZERO_MOT_THR: u8 = 0x21;
pub const ZERO_MOT_DUR: u8 = 0x22;

pub const FIFO_EN: u8 = 0x23;

pub const I2C_MST_CTRL: u8 = 0x24;
pub const I2C_SLV0_ADDR: u8 = 0x025;
pub const I2C_SLV0_REG: u8 = 0x026;
pub const I2C_SLV0_CTRL: u8 = 0x027;

pub const INT_PIN_CFG: u8 = 0x037;
pub const INT_ENABLE: u8 = 0x038;
pub const DMP_INT_STATUS: u8 = 0x039;
pub const INT_STATUS: u8 = 0x03A;

pub const ACCEL_XOUT_H: u8 = 0x03B;
pub const ACCEL_XOUT_L: u8 = 0x03C;
pub const ACCEL_YOUT_H: u8 = 0x03D;
pub const ACCEL_YOUT_L: u8 = 0x03E;
pub const ACCEL_ZOUT_H: u8 = 0x03F;
pub const ACCEL_ZOUT_L: u8 = 0x040;

pub const TEMP_OUT_H: u8 = 0x041;
pub const TEMP_OUT_L: u8 = 0x042;

pub const GYRO_XOUT_H: u8 = 0x043;
pub const GYRO_XOUT_L: u8 = 0x044;
pub const GYRO_YOUT_H: u8 = 0x045;
pub const GYRO_YOUT_L: u8 = 0x046;
pub const GYRO_ZOUT_H: u8 = 0x047;
pub const GYRO_ZOUT_L: u8 = 0x048;

pub const USER_CTRL: u8 = 0x06A;
pub const PWR_MGMT_1: u8 = 0x06B;
pub const PWR_MGMT_2: u8 = 0x06C;

pub const DMP_BANK_SEL: u8 = 0x6D;
pub const DMP_MEM_START_ADDR: u8 = 0x6E;
pub const DMP_MEM_R_W: u8 = 0x6F;
pub const DMP_CFG_1: u8 = 0x70;
pub const DMP_CFG_2: u8 = 0x71;

pub const FIFO_COUNT_H: u8 = 0x072;
pub const FIFO_COUNT_L: u8 = 0x073;
pub const FIFO_R_W: u8 = 0x074;

pub const WHO_AM_I: u8 = 0x075;
