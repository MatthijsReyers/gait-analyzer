use core::{error::Error, fmt};

use bleps::ad_structure::AdvertisementDataError;


#[derive(Debug)]
pub enum AppError
{
    Bluetooth(bleps::Error),
    BluetoothAdvertisement(AdvertisementDataError),
    BluetoothDisconnected,
}

impl Error for AppError {}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
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
