// TODO: Match all the naming exactly with datasheet

use super::i2c::{read_register, write_register, I2CError};
use core::future::Future;
use core::ops::Add;
use core::slice;
use cortex_m::asm;

fn modify_bits(orig: u8, mask: u8, new: u8) -> u8 {
    return (orig & !mask) | new;
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
pub enum GyroODR {
    PowerDown = 0b000 << ODR_G_SHIFT,
    HZ14_9 = 0b001 << ODR_G_SHIFT,
    HZ59_5 = 0b010 << ODR_G_SHIFT,
    HZ119 = 0b011 << ODR_G_SHIFT,
    HZ238 = 0b100 << ODR_G_SHIFT,
    HZ476 = 0b101 << ODR_G_SHIFT,
    HZ952 = 0b110 << ODR_G_SHIFT,
}

const FS_G_SHIFT: u8 = 3;
const FS_G_MASK: u8 = 0b11 << FS_G_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum GyroFS {
    DPS245 = 0b00 << FS_G_SHIFT,
    DPS500 = 0b01 << FS_G_SHIFT,
    DPS2000 = 0b11 << FS_G_SHIFT,
}

fn gyro_fs_to_numeric(fs: GyroFS) -> f32 {
    return match fs {
        GyroFS::DPS245 => 8.75 / 1000.0,
        GyroFS::DPS500 => 17.50 / 1000.0,
        GyroFS::DPS2000 => 70.0 / 1000.0,
    };
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
pub enum AccODR {
    PowerDown = 0b000 << ODR_XL_SHIFT,
    HZ14_9 = 0b001 << ODR_XL_SHIFT,
    HZ59_5 = 0b010 << ODR_XL_SHIFT,
    HZ119 = 0b011 << ODR_XL_SHIFT,
    HZ238 = 0b100 << ODR_XL_SHIFT,
    HZ476 = 0b101 << ODR_XL_SHIFT,
    HZ952 = 0b110 << ODR_XL_SHIFT,
}

const FS_XL_SHIFT: u8 = 3;
const FS_XL_MASK: u8 = 0b11 << FS_XL_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum AccFS {
    G2 = 0b00 << FS_XL_SHIFT,
    G4 = 0b10 << FS_XL_SHIFT,
    G8 = 0b11 << FS_XL_SHIFT,
    G16 = 0b01 << FS_XL_SHIFT,
}

fn acc_fs_to_numeric(fs: AccFS) -> f32 {
    return match fs {
        AccFS::G2 => 0.061 / 1000.0,
        AccFS::G4 => 0.122 / 1000.0,
        AccFS::G8 => 0.244 / 1000.0,
        AccFS::G16 => 0.732 / 1000.0,
    };
}

// CTRL_REG8
const CTRL_REG8_ADDR: u8 = 0x22;
const CTRL_REG8_BYTES: usize = 1;
const CTRL_REG8_RESET: u8 = 0b00000100;

const SW_RESET_SHIFT: u8 = 0;
const SW_RESET_MASK: u8 = 0b1 << SW_RESET_SHIFT;
const SW_RESET: u8 = SW_RESET_MASK;

// OUT_XYZ_XL
const OUT_XYZXL_ADDR: u8 = 0x28;
const OUT_XYZXL_BYTES: usize = 6;

// MAG
// WHOAMI
const WHOAMI_M_ADDR: u8 = 0x0F;
const WHOAMI_M_BYTES: usize = 1;

// CTRL_REG1_M
const CTRL_1M_ADDR: u8 = 0x20;
const CTRL_1M_BYTES: u8 = 1;
const CTRL_1M_RESET: u8 = 0b00010000;

const TEMP_COMP_SHIFT: u8 = 7;
const TEMP_COMP_MASK: u8 = 0b1 << TEMP_COMP_SHIFT;
const TEMP_COMP_ENABLED: u8 = TEMP_COMP_MASK;

const XY_OM_SHIFT: u8 = 5;
const XY_OM_MASK: u8 = 0b11 << XY_OM_SHIFT;

const FAST_ODR_SHIFT: u8 = 1;
const FAST_ODR_MASK: u8 = 0b1 << FAST_ODR_SHIFT;
const FAST_ODR_ENABLED: u8 = FAST_ODR_MASK;

const ODR_M_SHIFT: u8 = 2;
const ODR_M_MASK: u8 = 0b111 << ODR_M_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum MagODR {
    HZ0_625 = 0b000 << ODR_M_SHIFT,
    HZ1_25 = 0b001 << ODR_M_SHIFT,
    HZ2_5 = 0b010 << ODR_M_SHIFT,
    HZ5 = 0b011 << ODR_M_SHIFT,
    HZ10 = 0b100 << ODR_M_SHIFT,
    HZ20 = 0b101 << ODR_M_SHIFT,
    HZ40 = 0b110 << ODR_M_SHIFT,
    HZ80 = 0b111 << ODR_M_SHIFT,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum XYOperativeMode {
    LowPower = 0b00 << XY_OM_SHIFT,
    MediumPerformance = 0b01 << XY_OM_SHIFT,
    HighPerformance = 0b10 << XY_OM_SHIFT,
    UltraHighPerformance = 0b11 << XY_OM_SHIFT,
}

// CTRL_REG2_M
const CTRL_2M_ADDR: u8 = 0x21;
const CTRL_2M_BYTES: u8 = 1;
const CTRL_2M_RESET: u8 = 0;

const SOFT_RST_SHIFT: u8 = 2;
const SOFT_RST_MASK: u8 = 0b1 << SOFT_RST_SHIFT;
const SOFT_RST: u8 = SOFT_RST_MASK;

const FS_M_SHIFT: u8 = 5;
const FS_M_MASK: u8 = 0b11 << FS_M_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum MagFS {
    G4 = 0b00 << FS_M_SHIFT,
    G8 = 0b01 << FS_M_SHIFT,
    G12 = 0b10 << FS_M_SHIFT,
    G16 = 0b11 << FS_M_SHIFT,
}

fn mag_fs_to_numeric(fs: MagFS) -> f32 {
    return match fs {
        MagFS::G4 => 0.14 / 1000.0,
        MagFS::G8 => 0.29 / 1000.0,
        MagFS::G12 => 0.43 / 1000.0,
        MagFS::G16 => 0.58 / 1000.0,
    };
}

// CTRL_REG3_M
const CTRL_3M_ADDR: u8 = 0x22;
const CTRL_3M_BYTES: u8 = 1;
const CTRL_3M_RESET: u8 = 0b00000011;

const OP_MODE_SHIFT: u8 = 0;
const OP_MODE_MASK: u8 = 0b11 << OP_MODE_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
enum MagOperatingMode {
    Continuous = 0b00 << OP_MODE_SHIFT,
    Single = 0b01 << OP_MODE_SHIFT,
    PowerDown = 0b11 << OP_MODE_SHIFT,
}

// CTRL_REG4_M
const CTRL_4M_ADDR: u8 = 0x23;
const CTRL_4M_BYTES: u8 = 1;
const CTRL_4M_RESET: u8 = 0;

const Z_OM_SHIFT: u8 = 2;
const Z_OM_MASK: u8 = 0b11 << Z_OM_SHIFT;
#[repr(u8)]
#[derive(Copy, Clone)]
enum ZOperativeMode {
    LowPower = 0b00 << Z_OM_SHIFT,
    MediumPerformance = 0b01 << Z_OM_SHIFT,
    HighPerformance = 0b10 << Z_OM_SHIFT,
    UltraHighPerformance = 0b11 << Z_OM_SHIFT,
}

// OUT_XYZ_M
const OUT_XYZM_ADDR: u8 = 0x28;
const OUT_XYZM_BYTES: usize = 6;

pub struct AccReading {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
}

pub struct GyroReading {
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
}

pub struct MagReading {
    pub mx: f32,
    pub my: f32,
    pub mz: f32,
}

#[derive(Copy, Clone)]
pub struct Settings {
    acc_gyro_addr: u8,
    mag_addr: u8,
    acc_odr: AccODR,
    gyro_odr: GyroODR,
    mag_odr: MagODR,
    acc_fs: AccFS,
    gyro_fs: GyroFS,
    mag_fs: MagFS,
}

impl Default for Settings {
    fn default() -> Settings {
        return Settings {
            acc_gyro_addr: 0x6B,
            mag_addr: 0x1E,
            acc_odr: AccODR::HZ952,
            gyro_odr: GyroODR::HZ952,
            mag_odr: MagODR::HZ80,
            acc_fs: AccFS::G16,
            gyro_fs: GyroFS::DPS500,
            mag_fs: MagFS::G4,
        };
    }
}

impl Settings {
    pub fn set_acc_odr(&mut self, odr: AccODR) -> &mut Settings {
        self.acc_odr = odr;
        return self;
    }
    pub fn set_gyro_odr(&mut self, odr: GyroODR) -> &mut Settings {
        self.gyro_odr = odr;
        return self;
    }
    pub fn set_mag_odr(&mut self, odr: MagODR) -> &mut Settings {
        self.mag_odr = odr;
        return self;
    }
    pub fn set_acc_fs(&mut self, fs: AccFS) -> &mut Settings {
        self.acc_fs = fs;
        return self;
    }
    pub fn set_gyro_fs(&mut self, fs: GyroFS) -> &mut Settings {
        self.gyro_fs = fs;
        return self;
    }
    pub fn set_mag_fs(&mut self, fs: MagFS) -> &mut Settings {
        self.mag_fs = fs;
        return self;
    }
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

        // Check that the mag is detected
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

        // Sensors are present, reset before initializing
        write_register(self.acc_gyro_addr, CTRL_REG8_ADDR, &[SW_RESET])
            .await
            .unwrap();
        write_register(self.mag_addr, CTRL_2M_ADDR, &[SOFT_RST])
            .await
            .unwrap();

        // Perform initialization
        let mut ctrl_1g = modify_bits(CTRL_1G_RESET, ODR_G_MASK, self.gyro_odr as u8);
        ctrl_1g = modify_bits(ctrl_1g, FS_G_MASK, self.gyro_fs as u8);
        write_register(self.acc_gyro_addr, CTRL_1G_ADDR, &[ctrl_1g])
            .await
            .unwrap();

        let mut ctrl_6xl = modify_bits(CTRL_6XL_RESET, ODR_XL_MASK, self.acc_odr as u8);
        ctrl_6xl = modify_bits(ctrl_6xl, FS_XL_MASK, self.acc_fs as u8);
        write_register(self.acc_gyro_addr, CTRL_6XL_ADDR, &[ctrl_6xl])
            .await
            .unwrap();

        let mut ctrl_1m = modify_bits(
            CTRL_1M_RESET,
            XY_OM_MASK,
            XYOperativeMode::HighPerformance as u8,
        );
        ctrl_1m = modify_bits(ctrl_1m, TEMP_COMP_MASK, TEMP_COMP_ENABLED);
        ctrl_1m = modify_bits(ctrl_1m, FAST_ODR_MASK, FAST_ODR_ENABLED);
        ctrl_1m = modify_bits(ctrl_1m, ODR_M_MASK, self.mag_odr as u8);
        write_register(self.mag_addr, CTRL_1M_ADDR, &[ctrl_1m])
            .await
            .unwrap();

        let ctrl_2m = modify_bits(CTRL_2M_RESET, FS_M_MASK, self.mag_fs as u8);
        write_register(self.mag_addr, CTRL_2M_ADDR, &[ctrl_2m])
            .await
            .unwrap();

        let ctrl_4m = modify_bits(
            CTRL_4M_RESET,
            Z_OM_MASK,
            ZOperativeMode::UltraHighPerformance as u8,
        );
        write_register(self.mag_addr, CTRL_4M_ADDR, &[ctrl_4m])
            .await
            .unwrap();

        let ctrl_3m = modify_bits(CTRL_3M_RESET, OP_MODE_MASK, MagOperatingMode::Continuous as u8);
        write_register(self.mag_addr, CTRL_3M_ADDR, &[ctrl_3m])
            .await
            .unwrap();

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
    pub async fn acceleration(&self) -> Result<AccReading, I2CError> {
        let mut xyz_xl = [0; OUT_XYZXL_BYTES];
        read_register(self.settings.acc_gyro_addr, OUT_XYZXL_ADDR, &mut xyz_xl).await?;
        return Ok(AccReading {
            ax: i16::from_le_bytes([xyz_xl[0], xyz_xl[1]]) as f32
                * acc_fs_to_numeric(self.settings.acc_fs),
            ay: i16::from_le_bytes([xyz_xl[2], xyz_xl[3]]) as f32
                * acc_fs_to_numeric(self.settings.acc_fs),
            az: i16::from_le_bytes([xyz_xl[4], xyz_xl[5]]) as f32
                * acc_fs_to_numeric(self.settings.acc_fs),
        });
    }
    pub async fn angular_rate(&self) -> Result<GyroReading, I2CError> {
        let mut xyz_g = [0; OUT_XYZG_BYTES];
        read_register(self.settings.acc_gyro_addr, OUT_XYZG_ADDR, &mut xyz_g).await?;
        return Ok(GyroReading {
            gx: i16::from_le_bytes([xyz_g[0], xyz_g[1]]) as f32
                * gyro_fs_to_numeric(self.settings.gyro_fs),
            gy: i16::from_le_bytes([xyz_g[2], xyz_g[3]]) as f32
                * gyro_fs_to_numeric(self.settings.gyro_fs),
            gz: i16::from_le_bytes([xyz_g[4], xyz_g[5]]) as f32
                * gyro_fs_to_numeric(self.settings.gyro_fs),
        });
    }
    pub async fn magnetic(&self) -> Result<MagReading, I2CError> {
        let mut xyz_m = [0; OUT_XYZM_BYTES];
        read_register(self.settings.mag_addr, OUT_XYZM_ADDR, &mut xyz_m).await?;
        return Ok(MagReading {
            mx: i16::from_le_bytes([xyz_m[0], xyz_m[1]]) as f32
                * mag_fs_to_numeric(self.settings.mag_fs),
            my: i16::from_le_bytes([xyz_m[2], xyz_m[3]]) as f32
                * mag_fs_to_numeric(self.settings.mag_fs),
            mz: i16::from_le_bytes([xyz_m[4], xyz_m[5]]) as f32
                * mag_fs_to_numeric(self.settings.mag_fs),
        });
    }
}
