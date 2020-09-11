use super::i2c::{read_register, write_register, I2CError};
use core::future::Future;
use core::ops::Add;
use core::slice;
use cortex_m::asm;

fn modify_bits(orig: u8, mask: u8, new: u8) -> u8 {
    return (orig | !mask) | new;
}

// ACC GYRO
// WHOAMI
const WHOAMI_ADDR: u8 = 0x0F;
const WHOAMI_BYTES: usize = 1;

// CTRL_REG1_G
const CTRL_1G_ADDR: u8 = 0x10;
const CTRL_1G_BYTES: usize = 1;
const CTRL_1G_RESET: u8 = 0;

const ODR_G_SHIFT: u8 = 5;
const ODR_G_MASK: u8 = 0b111 << ODR_G_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
enum GyroODR {
    PowerDown = 0b000 << ODR_G_SHIFT,
    Hz14_9 = 0b001 << ODR_G_SHIFT,
    Hz59_5 = 0b010 << ODR_G_SHIFT,
    Hz119 = 0b011 << ODR_G_SHIFT,
    Hz238 = 0b100 << ODR_G_SHIFT,
    Hz476 = 0b101 << ODR_G_SHIFT,
    Hz952 = 0b110 << ODR_G_SHIFT,
}

const FS_G_SHIFT: u8 = 2;
const FS_G_MASK: u8 = 0b11 << FS_G_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum GyroFS {
    DPS245 = 0b00 << FS_G_SHIFT,
    DPS500 = 0b10 << FS_G_SHIFT,
    DPS2000 = 0b11 << FS_G_SHIFT,
}

// OUT_TEMP_L and OUT_TEMP_H
const OUT_TEMP_ADDR: u8 = 0x15;
const OUT_TEMP_BYTES: usize = 2;

// OUT_XYZ_G
const OUT_XYZG_ADDR: u8 = 0x18;
const OUT_XYZG_BYTES: usize = 6;

// CTRL_REG6_XL
const CTRL_6XL_ADDR: u8 = 0x20;
const CTRL_6XL_BYTES: usize = 1;
const CTRL_6XL_RESET: u8 = 0;

const ODR_XL_SHIFT: u8 = 5;
const ODR_XL_MASK: u8 = 0b111 << ODR_XL_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
enum AccelODR {
    PowerDown = 0b000 << ODR_XL_SHIFT,
    Hz14_9 = 0b001 << ODR_XL_SHIFT,
    Hz59_5 = 0b010 << ODR_XL_SHIFT,
    Hz119 = 0b011 << ODR_XL_SHIFT,
    Hz238 = 0b100 << ODR_XL_SHIFT,
    Hz476 = 0b101 << ODR_XL_SHIFT,
    Hz952 = 0b110 << ODR_XL_SHIFT,
}

const FS_XL_SHIFT: u8 = 2;
const FS_XL_MASK: u8 = 0b11 << FS_XL_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum AccelFS {
    G2 = 0b00 << FS_XL_SHIFT,
    G16 = 0b01 << FS_XL_SHIFT,
    G4 = 0b10 << FS_XL_SHIFT,
    G8 = 0b11 << FS_XL_SHIFT,
}

const OUT_XYZXL_ADDR: u8 = 0x28;
const OUT_XYZXL_BYTES: usize = 6;

// MAG
// WHOAMI
const WHOAMI_M_ADDR: u8 = 0x0F;
const WHOAMI_M_BYTES: usize = 1;

pub struct AccelReading {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
}

pub struct GyroReading {
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
}

struct MagReading {
    pub mx: f32,
    pub my: f32,
    pub mz: f32,
}

#[derive(Copy, Clone)]
pub struct Settings {
    acc_gyro_addr: u8,
    mag_addr: u8,
    acc_odr: AccelODR,
    gyro_odr: GyroODR,
    accel_fs: AccelFS,
    gyro_fs: GyroFS,
}

impl Default for Settings {
    fn default() -> Settings {
        return Settings {
            acc_gyro_addr: 0x6B,
            mag_addr: 0x1E,
            acc_odr: AccelODR::Hz952,
            gyro_odr: GyroODR::Hz952,
            accel_fs: AccelFS::G8,
            gyro_fs: GyroFS::DPS500,
        };
    }
}

impl Settings {
    pub async fn init(self) -> Result<LSM, LSMError> {
        // Check that the accgyro is detected
        let mut who_am_i = [0; WHOAMI_BYTES];
        if let Err(error) = read_register(self.acc_gyro_addr, WHOAMI_ADDR, &mut who_am_i).await {
            match error {
                I2CError::TooManyBytes => panic!("Not possible"),
                // If the accgyro is not found
                I2CError::PeripheralError => return Err(LSMError::BusError),
            }
        }

        // If the accgyro is not responding correctly
        if who_am_i[0] != 0x68 {
            return Err(LSMError::SensorNotFound);
        }

        let mut who_am_i_m = [0; WHOAMI_M_BYTES];
        if let Err(error) = read_register(self.mag_addr, WHOAMI_M_ADDR, &mut who_am_i_m).await {
            match error {
                I2CError::TooManyBytes => panic!("Not possible"),
                // If the mag is not found
                I2CError::PeripheralError => return Err(LSMError::BusError),
            }
        }

        // If the mag is not responding correctly
        if who_am_i_m[0] != 0x3D {
            return Err(LSMError::SensorNotFound);
        }

        // Perform initialization
        let mut ctrl_1g = modify_bits(CTRL_1G_RESET, ODR_G_MASK, self.gyro_odr as u8);
        ctrl_1g = modify_bits(ctrl_1g, FS_G_MASK, self.gyro_fs as u8);
        write_register(self.acc_gyro_addr, CTRL_1G_ADDR, &[ctrl_1g]).await;

        let mut ctrl_6xl = modify_bits(CTRL_6XL_RESET, ODR_XL_MASK, self.acc_odr as u8);
        ctrl_6xl = modify_bits(ctrl_6xl, FS_XL_MASK, self.accel_fs as u8);
        write_register(self.acc_gyro_addr, CTRL_6XL_ADDR, &[ctrl_6xl]).await;
        return Ok(LSM { settings: self });
    }
}

#[derive(Debug)]
pub enum LSMError {
    SensorNotFound,
    BusError,
}

pub struct LSM {
    settings: Settings,
}

impl LSM {
    pub async fn acceleration(&self) -> Result<AccelReading, I2CError> {
        let mut xyz_xl = [0; OUT_XYZXL_BYTES];
        read_register(self.settings.acc_gyro_addr, OUT_XYZXL_ADDR, &mut xyz_xl).await?;
        return Ok(AccelReading {
            ax: ((xyz_xl[0] as i16) | ((xyz_xl[1] as i16) << 8)) as f32 * 8.0 / 32768.0,
            ay: ((xyz_xl[2] as i16) | ((xyz_xl[3] as i16) << 8)) as f32 * 8.0 / 32768.0,
            az: ((xyz_xl[4] as i16) | ((xyz_xl[5] as i16) << 8)) as f32 * 8.0 / 32768.0,
        });
    }
}
