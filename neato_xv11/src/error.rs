use std::fmt;
use std::io;

use serial;

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
    Configure(serial::Error),
    // Unable to open serial port
    OpenSerialPort(serial::Error),
    // A resync is required.
    ResyncRequired,
    // Serial read error.
    SerialRead(io::Error),
    // Unable to set timeout.
    SetTimeout(serial::Error),
}

impl fmt::Display for LidarDriverError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            LidarDriverError::Checksum(size) => write!(f, "checksum error occured at packet index {}", size),
            LidarDriverError::Configure(_) => write!(f, "unable to configure serial port"),
            LidarDriverError::OpenSerialPort(_) => write!(f, "unable to open serial port"),
            LidarDriverError::ResyncRequired => write!(f, "resync required"),
            LidarDriverError::SerialRead(_) => write!(f, "unable to read from serial port"),
            LidarDriverError::SetTimeout(_) => write!(f, "unable to set serial port timeout"),
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
    InvalidDataError(u32),
    // The Signal Strength Warning flag was set.
    SignalStrengthWarning,
}
