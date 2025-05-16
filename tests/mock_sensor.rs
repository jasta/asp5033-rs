use anyhow::Error;
use asp5033::asp5033::{Asp5033, CombinedMeasurement, DEFAULT_ADDRESS};
use embedded_hal_mock::eh1::i2c::Mock;
use embedded_hal_mock::eh1::i2c::Transaction;
use measurements::{Pressure, Temperature};

const REG_ID_GET: u8 = 0x01;
const REG_ID_SET: u8 = 0xa4;
const WHOAMI_DEFAULT_ID: u8 = 0x00;
const WHOAMI_RECHECK_ID: u8 = 0x66;

const REG_CMD: u8 = 0x30;
const CMD_MEASURE: u8 = 0x0a;
const CMD_MASK_SENSOR_READY: u8 = 0x08;

const REG_PRESSURE_DATA: u8 = 0x06;
const REG_TEMP_DATA: u8 = 0x09;

const PRESSURE_SCALE: f32 = 1.0 / 128.0;
const TEMP_SCALE: f32 = 1.0 / 256.0;

#[tokio::test]
pub async fn test_single_measurement() -> anyhow::Result<()> {
    let mut i2c = Mock::new([]);

    let mut driver = Asp5033::new(i2c.clone());

    i2c.update_expectations(&confirm_id_transactions());
    driver.perform_sensor_id_check().await
            .map_err(Error::msg)?;

    let expected = CombinedMeasurement {
        pressure: Pressure::from_bars(1.0),
        temperature: Temperature::from_celsius(50.0),
    };
    i2c.update_expectations(&measurement_transactions(&expected));
    driver.request_measurement().await
            .map_err(Error::msg)?;
    let is_ready = driver.is_measurement_ready().await
            .map_err(Error::msg)?;
    assert!(is_ready);

    let actual = driver.read_measurement().await
            .map_err(Error::msg)?;
    assert_eq!(actual, expected);

    i2c.done();

    Ok(())
}

fn confirm_id_transactions() -> Vec<Transaction> {
    vec![
        Transaction::write_read(DEFAULT_ADDRESS, vec![REG_ID_SET], vec![WHOAMI_DEFAULT_ID]),
        Transaction::write(DEFAULT_ADDRESS, vec![REG_ID_SET, WHOAMI_RECHECK_ID]),
        Transaction::write_read(DEFAULT_ADDRESS, vec![REG_ID_GET], vec![WHOAMI_RECHECK_ID])
    ]
}

fn measurement_transactions(value: &CombinedMeasurement) -> Vec<Transaction> {
    let raw_pressure = value.pressure.as_pascals() / (PRESSURE_SCALE as f64);
    let raw_temp = value.temperature.as_celsius() / (TEMP_SCALE as f64);

    let mut raw_value = vec![];
    let raw_pressure_bytes = (raw_pressure as i32).to_be_bytes();
    assert_eq!(raw_pressure_bytes[0], 0);
    raw_value.extend_from_slice(&raw_pressure_bytes[1..]);
    let raw_temp_bytes = (raw_temp as i16).to_be_bytes();
    raw_value.extend_from_slice(&raw_temp_bytes);

    vec![
        Transaction::write(DEFAULT_ADDRESS, vec![REG_CMD, CMD_MEASURE]),
        Transaction::write_read(DEFAULT_ADDRESS, vec![REG_CMD], vec![CMD_MASK_SENSOR_READY]),
        Transaction::write_read(DEFAULT_ADDRESS, vec![REG_PRESSURE_DATA], raw_value),
    ]
}