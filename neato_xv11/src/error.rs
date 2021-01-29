use std::fmt::{Display, Formatter, Result};
use std::error::Error;
use std::io::Error as IoError;

use serial::Error as SerialError;

#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};

/// ## Summary
/// 
/// A driver error.
/// These errors are usually very serious.
/// 
#[derive(Debug)]
pub enum LidarDriverError {
    // Checksum error occured. The associated value is the packet index.
    Checksum(usize),
    // Unable to configure serial port.
    Configure(SerialError),
    // Unable to open serial port.
    OpenSerialPort(SerialError),
    // A resync is required.
    ResyncRequired,
    // Serial read error.
    SerialRead(IoError),
    // Unable to set timeout.
    SetTimeout(SerialError),
}

impl Display for LidarDriverError {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            LidarDriverError::Checksum(index) => write!(f, "A checksum error occured at packet index {}", index),
            LidarDriverError::Configure(_) => write!(f, "Unable to configure serial port"),
            LidarDriverError::OpenSerialPort(_) => write!(f, "Unable to open serial port"),
            LidarDriverError::ResyncRequired => write!(f, "Resync required"),
            LidarDriverError::SerialRead(_) => write!(f, "Unable to read from serial port"),
            LidarDriverError::SetTimeout(_) => write!(f, "Unable to set serial port timeout"),
        }
    }
}

impl Error for LidarDriverError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            LidarDriverError::Configure(e) => Some(e),
            LidarDriverError::OpenSerialPort(e) => Some(e),
            LidarDriverError::SerialRead(e) => Some(e),
            LidarDriverError::SetTimeout(e) => Some(e),
            _ => None,
        }
    }
}

impl PartialEq for LidarDriverError {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (LidarDriverError::Checksum(first), LidarDriverError::Checksum(second)) => first == second,
            _ => false
        }
    }
}

/// ## Summary
/// 
/// A LIDAR reading error. 
/// This occurs when the LIDAR reports that the data is erroneous or unreliable, 
/// which typically happens if the LIDAR is attempting to scan a far surface.
/// 
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub enum LidarReadingError {
    // The Invalid Data Error flag was set. The associated value is the error code.
    InvalidDataError(i32),
    // The Signal Strength Warning flag was set.
    SignalStrengthWarning,
}