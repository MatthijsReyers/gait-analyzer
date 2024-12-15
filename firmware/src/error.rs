use core::{error::Error, fmt};

use bleps::ad_structure::AdvertisementDataError;
use esp_hal::i2c::master::Error as I2cError;

#[derive(Debug)]
pub enum AppError
{
    I2c(I2cError),
    Bluetooth(bleps::Error),
    BluetoothAdvertisement(AdvertisementDataError),
    BluetoothDisconnected,
}

impl Error for AppError {}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::I2c(err) => write!(f, "I2c communication error: {:?}", err),
            Self::Bluetooth(err) => write!(f, "Bluetooth error: {:?}", err),
            Self::BluetoothAdvertisement(err) => write!(f, "Bluetooth advertisement error: {:?}", err),
            Self::BluetoothDisconnected => write!(f, "Bluetooth got disconnected"),
        }
    }
}

impl From<bleps::Error> for AppError
{
    fn from(err: bleps::Error) -> Self {
        AppError::Bluetooth(err)
    }
}

impl From<AdvertisementDataError> for AppError
{
    fn from(err: AdvertisementDataError) -> Self {
        AppError::BluetoothAdvertisement(err)
    }
}

impl From<I2cError> for AppError
{
    fn from(err: I2cError) -> Self {
        AppError::I2c(err)
    }
}
