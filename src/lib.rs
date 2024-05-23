#![no_std]

mod bits;

use bits::{raw_reading_to_i16, raw_reading_to_i32, raw_reading_to_u16};
use defmt::Format;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use libm::powf;

pub const BMP180_I2C_ADDR: u8 = 0x77;

pub const BMP180_REGISTER_RESET: u8 = 0xE0;
pub const BMP180_REGISTER_CTL: u8 = 0xF4;
pub const BMP180_CMD_TEMP: u8 = 0x2E;
pub const BMP180_REGISTER_TEMP_MSB: u8 = 0xF6;
pub const BMP180_CMD_PRESSURE: u8 = 0x34;
pub const BMP180_REGISTER_PRESSURE_MSB: u8 = 0xF6;
pub const BMP180_REGISTER_AC1MSB: u8 = 0xaa;

#[derive(Copy, Clone)]
pub enum PressureMode {
    BMP180UltraLowPower,
    BMP180Standard,
    BMP180HighResolution,
    BMP180UltraHighResolution,
}

impl PressureMode {
    pub fn get_mode_value(self) -> u8 {
        match self {
            PressureMode::BMP180UltraLowPower => 0,
            PressureMode::BMP180Standard => 1,
            PressureMode::BMP180HighResolution => 2,
            PressureMode::BMP180UltraHighResolution => 3,
        }
    }
    pub fn mode_delay(self) -> u32 {
        match self {
            PressureMode::BMP180UltraLowPower => 5,
            PressureMode::BMP180Standard => 8,
            PressureMode::BMP180HighResolution => 14,
            PressureMode::BMP180UltraHighResolution => 26,
        }
    }
}
#[derive(Copy, Clone, Default)]
pub struct CalibrationCoefficients {
    ac1: i16,
    ac2: i16,
    ac3: i16,
    ac4: u16,
    ac5: u16,
    ac6: u16,
    b1: i16,
    b2: i16,
    mb: i16,
    mc: i16,
    md: i16,
}

#[derive(Debug, Format)]
pub enum BMP180Error<E> {
    CalibrationError,
    RawReadingError,
    GroundLevelNotSet,
    I2cError(I2cBusError<E>),
}

pub struct BMP180<I, D> {
    i2c_bus: I2cBus<I>,
    delay: D,
    coeff: CalibrationCoefficients,
    pressure_precision: PressureMode,
    ground_level_pressure: Option<f32>,
}

impl<I, D, E> BMP180<I, D>
where
    I: I2c<Error = E>,
    D: DelayNs,
    E: core::fmt::Debug,
{
    /// Create sensor accessor for BMP180 on the provided i2c bus path
    pub fn new(i2c: I, delay: D, pressure_precision: PressureMode) -> Result<Self, BMP180Error<E>> {
        let i2c_bus = I2cBus::new(i2c);
        Ok(Self {
            i2c_bus,
            delay,
            coeff: Default::default(),
            pressure_precision,
            ground_level_pressure: None,
        })
    }

    pub async fn init(&mut self) -> Result<(), BMP180Error<E>> {
        self.coeff = self.get_calibration().await?;
        Ok(())
    }

    pub async fn calculate_ground_level(&mut self) -> Result<(), BMP180Error<E>> {
        let mut p = 0f32;

        for _ in [0..10] {
            p += self.pressure_pa().await?;
        }

        p /= 10.0;

        self.ground_level_pressure = Some(p);

        Ok(())
    }

    pub async fn calculate_altitude(&mut self) -> Result<f32, BMP180Error<E>> {
        if self.ground_level_pressure.is_none() {
            return Err(BMP180Error::GroundLevelNotSet);
        }

        let g_pressure = self.ground_level_pressure.unwrap();
        let current_pressure = self.pressure_pa().await?;

        let altitude = 44330.0 * (1.0 - powf(current_pressure / g_pressure, 1.0 / 5.255));

        Ok(altitude)
    }

    pub async fn pressure_pa(&mut self) -> Result<f32, BMP180Error<E>> {
        let tadc = self.read_raw_temperature().await?;
        let padc = self.read_raw_pressure(self.pressure_precision).await?;
        let b5 = self.coeff.calculate_b5(tadc);
        let real_pressure = calculate_real_pressure(padc, b5, self.coeff, self.pressure_precision);
        Ok(real_pressure as f32)
    }

    pub async fn pressure_hpa(&mut self) -> Result<f32, BMP180Error<E>> {
        match self.pressure_pa().await {
            Ok(x) => Ok(x / 100_f32),
            Err(e) => Err(e),
        }
    }

    pub async fn temperature_celsius(&mut self) -> Result<f32, BMP180Error<E>> {
        let reading = self.read_raw_temperature().await?;
        let b5 = self.coeff.calculate_b5(reading);
        let t = (b5 + 8) >> 4;
        Ok((t as f32) / 10_f32)
    }

    pub async fn pressure_kpa(&mut self) -> Result<f32, BMP180Error<E>> {
        match self.pressure_pa().await {
            Ok(x) => Ok(x / 1000_f32),
            Err(e) => Err(e),
        }
    }

    pub async fn get_sea_level_pressure(&mut self, altitude: u16) -> Result<f32, BMP180Error<E>> {
        let p = self.pressure_pa().await?;

        let res = p / 1.0 - powf(altitude as f32 / 44330.0, 5.255);

        Ok(res)
    }

    pub async fn reset(&mut self) -> Result<(), BMP180Error<E>> {
        self.i2c_bus
            .write_byte(BMP180_REGISTER_RESET, 0xB6)
            .await
            .map_err(map_i2c_error)?;
        Ok(())
    }

    async fn read_raw_temperature(&mut self) -> Result<i16, BMP180Error<E>> {
        // fist we need read temp needed for further pressure calculations
        self.i2c_bus
            .write_byte(BMP180_REGISTER_CTL, BMP180_CMD_TEMP)
            .await
            .map_err(map_i2c_error)?;

        // maximum conversion time is 5ms
        self.delay.delay_ms(5).await;
        // Read uncompensated temperature (two registers)
        // i2c gets LittleEndian we need BigEndian
        let mut buf = [0_u8; 2];
        self.i2c_bus
            .read_bytes(BMP180_REGISTER_TEMP_MSB, &mut buf)
            .await
            .map_err(map_i2c_error)?;

        // we have raw temp data in tadc.
        let tadc: i16 = raw_reading_to_i16(&buf[..], 0);
        // info!("Raw temp data: {}", tadc);

        Ok(tadc)
    }

    async fn read_raw_pressure(&mut self, mode: PressureMode) -> Result<i32, BMP180Error<E>> {
        // now lets get pressure
        let offset = mode.get_mode_value();
        self.i2c_bus
            .write_byte(BMP180_REGISTER_CTL, BMP180_CMD_PRESSURE + (offset << 6))
            .await
            .map_err(map_i2c_error)?;
        self.delay.delay_ms(mode.mode_delay()).await;

        let mut p_buf = [0_u8; 3];
        self.i2c_bus
            .read_bytes(BMP180_REGISTER_PRESSURE_MSB, &mut p_buf)
            .await
            .map_err(map_i2c_error)?;

        let mut padc: i32 = raw_reading_to_i32(&p_buf, 0);
        padc >>= 8 - offset;
        Ok(padc)
    }

    async fn get_calibration(&mut self) -> Result<CalibrationCoefficients, BMP180Error<E>> {
        let mut buf: [u8; 22] = [0; 22];
        self.i2c_bus
            .read_bytes(BMP180_REGISTER_AC1MSB, &mut buf)
            .await
            .map_err(map_i2c_error)?;

        Ok(CalibrationCoefficients {
            ac1: raw_reading_to_i16(&buf, 0),
            ac2: raw_reading_to_i16(&buf, 2),
            ac3: raw_reading_to_i16(&buf, 4),
            ac4: raw_reading_to_u16(&buf, 6),
            ac5: raw_reading_to_u16(&buf, 8),
            ac6: raw_reading_to_u16(&buf, 10),
            b1: raw_reading_to_i16(&buf, 12),
            b2: raw_reading_to_i16(&buf, 14),
            mb: raw_reading_to_i16(&buf, 16),
            mc: raw_reading_to_i16(&buf, 18),
            md: raw_reading_to_i16(&buf, 20),
        })
    }
}

impl CalibrationCoefficients {
    pub fn calculate_b5(self, raw_temp: i16) -> i32 {
        let x1 = (((raw_temp as i32) - (self.ac6 as i32)) * (self.ac5 as i32)) >> 15;
        let x2 = ((self.mc as i32) << 11) / (x1 + (self.md as i32));
        x1 + x2
    }
}

fn calculate_real_pressure(
    up: i32,
    b5: i32,
    coeff: CalibrationCoefficients,
    oss: PressureMode,
) -> i32 {
    let b6: i32 = b5 - 4000;
    let mut x1: i32 = ((coeff.b2 as i32) * ((b6 * b6) >> 12)) >> 11;
    let mut x2: i32 = ((coeff.ac2 as i32) * b6) >> 11;
    let mut x3: i32 = x1 + x2;
    let b3 = ((((coeff.ac1 as u32) * 4 + x3 as u32) << oss.get_mode_value()) + 2) / 4;
    x1 = ((coeff.ac3 as i32) * b6) >> 13;
    x2 = ((coeff.b1 as i32) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    let b4 = ((coeff.ac4 as u32) * (x3 as u32 + 32768)) >> 15;
    let b7 = (up as u32 - b3) * (50000u32 >> oss.get_mode_value());
    let p = if b7 < 0x80000000 {
        ((b7 * 2) / b4) as i32
    } else {
        ((b7 / b4) * 2) as i32
    };

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;

    p + ((x1 + x2 + 3791) >> 4)
}

#[derive(Debug, Format)]
pub enum I2cBusError<E> {
    I2c(E),
}

pub struct I2cBus<I> {
    i2c: I,
}

impl<I, E> I2cBus<I>
where
    I: I2c<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    /// Writes byte to register
    pub async fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), I2cBusError<E>> {
        self.i2c
            .write(BMP180_I2C_ADDR, &[reg, byte])
            .await
            .map_err(I2cBusError::I2c)?;
        Ok(())
    }

    /// Enables bit n at register address reg
    pub async fn write_bit(
        &mut self,
        reg: u8,
        bit_n: u8,
        enable: bool,
    ) -> Result<(), I2cBusError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        self.write_byte(reg, byte[0]).await
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub async fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), I2cBusError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        self.write_byte(reg, byte[0]).await
    }

    /// Read bit n from register
    pub async fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, I2cBusError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    /// Read bits at register reg, starting with bit start_bit, until start_bit+length
    pub async fn read_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
    ) -> Result<u8, I2cBusError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(bits::get_bits(byte[0], start_bit, length))
    }

    /// Reads byte from register
    pub async fn read_byte(&mut self, reg: u8) -> Result<u8, I2cBusError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(BMP180_I2C_ADDR, &[reg], &mut byte)
            .await
            .map_err(I2cBusError::I2c)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2cBusError<E>> {
        self.i2c
            .write_read(BMP180_I2C_ADDR, &[reg], buf)
            .await
            .map_err(I2cBusError::I2c)?;
        Ok(())
    }
}

fn map_i2c_error<E: core::fmt::Debug>(error: I2cBusError<E>) -> BMP180Error<E> {
    BMP180Error::I2cError(error)
}
