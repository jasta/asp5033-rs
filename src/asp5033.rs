use crate::simple_cursor::SimpleCursor;
use core::fmt::Debug;
use embedded_hal_async::i2c::{AddressMode, I2c, SevenBitAddress};
use measurements::{Pressure, Temperature};

pub const DEFAULT_ADDRESS: SevenBitAddress = 0x6d;
pub const ALTERNATE_ADDRESS: SevenBitAddress = 0x6c;

const REG_ID_GET: u8 = 0x01;
const REG_ID_SET: u8 = 0xa4;
const WHOAMI_DEFAULT_ID: u8 = 0x00;
const WHOAMI_RECHECK_ID: u8 = 0x66;

const REG_CMD: u8 = 0x30;
const CMD_MEASURE: u8 = 0x0a;
const CMD_MASK_SENSOR_READY: u8 = 0x08;

const REG_MEASUREMENTS: u8 = 0x06;

const PRESSURE_SCALE: f32 = 1.0 / 128.0;
const TEMP_SCALE: f32 = 1.0 / 256.0;

pub struct Asp5033<I2C, A = SevenBitAddress> {
    i2c: I2C,
    address: A,
}

impl<I2C> Asp5033<I2C, SevenBitAddress>
where
        I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_address(i2c, DEFAULT_ADDRESS)
    }
}

impl<I2C, A> Asp5033<I2C, A>
where
        I2C: I2c<A>,
        A: AddressMode + Copy,
{
    pub fn new_with_address(i2c: I2C, address: A) -> Self {
        Self {
            i2c,
            address,
        }
    }

    pub async fn perform_sensor_id_check(&mut self) -> Result<(), SensorIdCheckError<I2C::Error>> {
        let id = self.write_read_one(&[REG_ID_SET]).await?;
        if id != WHOAMI_DEFAULT_ID && id != WHOAMI_RECHECK_ID {
            return Err(SensorIdCheckError::UnexpectedId { got: id, failed_operation: REG_ID_SET });
        }

        self.i2c.write(self.address, &[REG_ID_SET, WHOAMI_RECHECK_ID]).await?;

        let id = self.write_read_one(&[REG_ID_GET]).await?;
        if id != WHOAMI_RECHECK_ID {
            return Err(SensorIdCheckError::UnexpectedId { got: id, failed_operation: REG_ID_GET });
        }

        Ok(())
    }

    /// Request that the sensor begin a one-off measurement.
    ///
    /// This must be called one sampling interval before [Self::read_measurement] else an error
    /// may occur.
    pub async fn request_measurement(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[REG_CMD, CMD_MEASURE]).await?;
        Ok(())
    }

    /// Check if the previously scheduled measurement cycle is complete.  Normally this can be
    /// skipped for efficiency by simply waiting sufficiently between the measurement cycle being
    /// started and then the measurements sampled.  It is unknown what exactly this interval must
    /// be according to the sensor but in my testing 100Hz sampling seems to work without errors.
    pub async fn is_measurement_ready(&mut self) -> Result<bool, I2C::Error> {
        let status = self.write_read_one(&[REG_CMD]).await?;
        Ok((status & CMD_MASK_SENSOR_READY) != 0)
    }

    /// Read sensor values from a prior measurement cycle.
    pub async fn read_measurement(&mut self) -> Result<CombinedMeasurement, I2C::Error> {
        let mut combined = [0u8; 5];
        self.i2c.write_read(self.address, &[REG_MEASUREMENTS], &mut combined).await?;

        let mut cursor = SimpleCursor::new(&combined);
        let raw_pressure = cursor.read_i24();
        let pressure = raw_pressure as f32 * PRESSURE_SCALE;

        let raw_temp = cursor.read_i16();
        let temperature = raw_temp as f32 * TEMP_SCALE;

        Ok(CombinedMeasurement {
            pressure: Pressure::from_pascals(pressure as f64),
            temperature: Temperature::from_celsius(temperature as f64),
        })
    }

    async fn write_read_one(&mut self, write: &[u8]) -> Result<u8, I2C::Error> {
        let mut one = [0u8];
        self.i2c.write_read(self.address, write, &mut one).await?;
        Ok(one[0])
    }
}

#[derive(thiserror::Error, Debug)]
pub enum SensorIdCheckError<E: Debug> {
    #[error("Unexpected id: got={got}, failed_operation={failed_operation:?}")]
    UnexpectedId {
        got: u8,
        failed_operation: u8,
    },

    #[error("Bus error: {0:?}")]
    BusError(#[from] E),
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct CombinedMeasurement {
    pub pressure: Pressure,
    pub temperature: Temperature,
}