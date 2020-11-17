use super::i2c::{read_register, write_register, I2CError};
use core::future::Future;
use core::ops::Add;
use core::slice;
use cortex_m::asm;

fn modify_bits(orig: u8, mask: u8, new: u8) -> u8 {
    return (orig & !mask) | new;
}

// DIG (Trimming)
// Every value in this range is 2 bytes
// [LSB, MSB] layout
const DIG_T_P_ADDR: u8 = 0x88;
const DIG_T_P_BYTES: usize = 12 * 2;

// ID
const ID_ADDR: u8 = 0xD0;
const ID_BYTES: usize = 1;

// PRESSURE
// CTRL_MEAS
const CTRL_MEAS_ADDR: u8 = 0xF4;
const CTRL_MEAS_BYTES: usize = 1;
const CTRL_MEAS_RESET: u8 = 0x0;

const MODE_OFFSET: u8 = 0;
const MODE_MASK: u8 = 0b11 << MODE_OFFSET;

#[repr(u8)]
#[derive(Copy, Clone)]
enum Mode {
    Sleep = 0b00 << MODE_OFFSET,
    Forced = 0b01 << MODE_OFFSET,
    Normal = 0b11 << MODE_OFFSET,
}

const OSRSP_OFFSET: u8 = 2;
const OSRSP_MASK: u8 = 0b111 << OSRSP_OFFSET;

#[repr(u8)]
#[derive(Copy, Clone)]
enum PressOversample {
    PressDisabled = 0 << OSRSP_OFFSET,
    X1 = 0b001 << OSRSP_OFFSET,
    X2 = 0b010 << OSRSP_OFFSET,
    X4 = 0b011 << OSRSP_OFFSET,
    X8 = 0b100 << OSRSP_OFFSET,
    X16 = 0b101 << OSRSP_OFFSET,
}

const OSRST_OFFSET: u8 = 5;
const OSRST_MASK: u8 = 0b111 << OSRST_OFFSET;

#[repr(u8)]
#[derive(Copy, Clone)]
enum TempOversample {
    TempDisabled = 0b000 << OSRST_OFFSET,
    X1 = 0b001 << OSRST_OFFSET,
    X2 = 0b010 << OSRST_OFFSET,
    X4 = 0b011 << OSRST_OFFSET,
    X8 = 0b100 << OSRST_OFFSET,
    X16 = 0b101 << OSRST_OFFSET,
}

// CONFIG
const CONFIG_ADDR: u8 = 0xF5;
const CONFIG_BYTES: usize = 1;
const CONFIG_RESET: u8 = 0x0;

const TSB_OFFSET: u8 = 5;
const TSB_MASK: u8 = 0b111 << TSB_OFFSET;

#[repr(u8)]
#[derive(Copy, Clone)]
enum StandbyDuration {
    MS0_5 = 0b000 << TSB_OFFSET,
    MS62_5 = 0b001 << TSB_OFFSET,
    MS125 = 0b010 << TSB_OFFSET,
    MS250 = 0b011 << TSB_OFFSET,
    MS500 = 0b100 << TSB_OFFSET,
    MS1000 = 0b101 << TSB_OFFSET,
    MS2000 = 0b110 << TSB_OFFSET,
    MS4000 = 0b111 << TSB_OFFSET,
}

const FILTER_OFFSET: u8 = 2;
const FILTER_MASK: u8 = 0b111 << FILTER_OFFSET;

#[repr(u8)]
#[derive(Copy, Clone)]
enum Filter {
    OFF = 0b000 << FILTER_OFFSET,
    X2 = 0b001 << FILTER_OFFSET,
    X4 = 0b010 << FILTER_OFFSET,
    X8 = 0b011 << FILTER_OFFSET,
    X16 = 0b100 << FILTER_OFFSET,
}

// PRESS AND TEMP
// Layout is [P_MSB, P_LSB, P_XLSB, T_MSB, T_LSB, T_XLSB]
const PRESS_TEMP_ADDR: u8 = 0xF7;
const PRESS_TEMP_BYTES: usize = 2 * 3;

pub struct PressTempReading {
    pub temp: f32,
    pub press: f32,
}

struct TrimmingCoeffs {
    t1: u16,
    t2: i16,
    t3: i16,
    p1: u16,
    p2: i16,
    p3: i16,
    p4: i16,
    p5: i16,
    p6: i16,
    p7: i16,
    p8: i16,
    p9: i16,
}

#[derive(Copy, Clone)]
pub struct Settings {
    addr: u8,
    press_oversample: PressOversample,
    temp_oversample: TempOversample,
    standby_duration: StandbyDuration,
    filter: Filter,
}

impl Default for Settings {
    fn default() -> Settings {
        return Settings {
            addr: 0x77,
            press_oversample: PressOversample::X16,
            temp_oversample: TempOversample::X2,
            standby_duration: StandbyDuration::MS125,
            filter: Filter::X16,
        };
    }
}

impl Settings {
    pub async fn init(self) -> Result<BMP, BMPError> {
        // Try to detect the sensor
        let mut id = [0; ID_BYTES];
        if let Err(error) = read_register(self.addr, ID_ADDR, &mut id).await {
            match error {
                I2CError::TooManyBytes => panic!("Not possible"),
                // If the sensor is not found
                I2CError::PeripheralError => return Err(BMPError::BusError),
            }
        }

        // If the sensor is not responding correctly
        if id[0] != 0x58 {
            return Err(BMPError::SensorNotFound);
        }

        // Perform initialization
        // Soft reset
        write_register(self.addr, 0xE0, &[0xB6]).await.unwrap();
        // This isn't great, 2ms delay required.
        // Should reimplement this with a timer future.
        asm::delay(8_000_000);

        let mut ctrl_meas = modify_bits(CTRL_MEAS_RESET, MODE_MASK, Mode::Normal as u8);
        ctrl_meas = modify_bits(ctrl_meas, OSRSP_MASK, self.press_oversample as u8);
        ctrl_meas = modify_bits(ctrl_meas, OSRST_MASK, self.temp_oversample as u8);
        write_register(self.addr, CTRL_MEAS_ADDR, &[ctrl_meas])
            .await
            .unwrap();

        let mut config = modify_bits(CONFIG_RESET, FILTER_MASK, self.filter as u8);
        config = modify_bits(config, TSB_MASK, self.standby_duration as u8);
        write_register(self.addr, CONFIG_ADDR, &[config])
            .await
            .unwrap();

        let mut trimming = [0; DIG_T_P_BYTES];
        read_register(self.addr, DIG_T_P_ADDR, &mut trimming).await;
        return Ok(BMP {
            settings: self,
            trimming: TrimmingCoeffs {
                t1: (trimming[1] as u16) << 8 | (trimming[0] as u16),
                t2: (trimming[3] as i16) << 8 | (trimming[2] as i16),
                t3: (trimming[5] as i16) << 8 | (trimming[4] as i16),
                p1: (trimming[7] as u16) << 8 | (trimming[6] as u16),
                p2: (trimming[9] as i16) << 8 | (trimming[8] as i16),
                p3: (trimming[11] as i16) << 8 | (trimming[10] as i16),
                p4: (trimming[13] as i16) << 8 | (trimming[12] as i16),
                p5: (trimming[15] as i16) << 8 | (trimming[14] as i16),
                p6: (trimming[17] as i16) << 8 | (trimming[16] as i16),
                p7: (trimming[19] as i16) << 8 | (trimming[18] as i16),
                p8: (trimming[21] as i16) << 8 | (trimming[20] as i16),
                p9: (trimming[23] as i16) << 8 | (trimming[22] as i16),
            },
        });
    }
}

#[derive(Debug)]
pub enum BMPError {
    SensorNotFound,
    BusError,
}

pub struct BMP {
    settings: Settings,
    trimming: TrimmingCoeffs,
}

impl BMP {
    pub async fn pressure_temperature(&self) -> Result<PressTempReading, I2CError> {
        let mut press_temp = [0; PRESS_TEMP_BYTES];
        read_register(self.settings.addr, PRESS_TEMP_ADDR, &mut press_temp).await?;

        // The unsigned pressure and temperature
        // Can't use integer from bytes methods because XLSB is not a full byte
        let u_press = ((press_temp[0] as u32) << 12)
            | ((press_temp[1] as u32) << 4)
            | ((press_temp[2] as u32) >> 4);
        let u_temp = ((press_temp[3] as u32) << 12)
            | ((press_temp[4] as u32) << 4)
            | ((press_temp[5] as u32) >> 4);

        // Trim the temperature and pressure
        // Adaptation of the datasheet description
        let temp_fine = {
            let var1 = ((u_temp as f32) / 16384.0 - (self.trimming.t1 as f32) / 1024.0)
                * (self.trimming.t2 as f32);
            let var2 = ((u_temp as f32) / 131072.0 - (self.trimming.t1 as f32) / 8192.0)
                * ((u_temp as f32) / 131072.0 - (self.trimming.t1 as f32) / 8192.0)
                * (self.trimming.t3 as f32);
            var1 + var2
        };

        let press = {
            let mut var1 = (temp_fine / 2.0) - 64000.0;
            let mut var2 = var1 * var1 * ((self.trimming.p6 as f32) / 32768.0);
            var2 = var2 + var1 * (self.trimming.p5 as f32) / 2.0;
            var2 = var2 / 4.0 + (self.trimming.p4 as f32) * 65536.0;
            var1 = ((self.trimming.p3 as f32) * var1 * var1 / 524288.0
                + (self.trimming.p2 as f32) * var1)
                / 524288.0;
            var1 = (1.0 + var1 / 32768.0) * (self.trimming.p1 as f32);
            let mut p = 1048576.0 - (u_press as f32);
            p = (p - var2 / 4096.0) * 6250.0 / var1;
            var1 = (self.trimming.p9 as f32) * p * p / 2147483648.0;
            var2 = p * (self.trimming.p8 as f32) / 32768.0;
            p + (var1 + var2 + (self.trimming.p7 as f32)) / 16.0
        };

        return Ok(PressTempReading {
            press,
            temp: temp_fine / 5120.0,
        });
    }
}
