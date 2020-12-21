use std::fmt;
use std::io;

/// ## Summary
/// 
/// A driver error. 
/// These errors are usually very serious.
/// 
#[derive(Debug)]
pub enum DriverError {
    // Checksum error occured. The associated value is the packet index.
    Checksum(usize),
    // A resync is required.
    ResyncRequired,
    // Serial read error.
    SerialRead(io::Error),
}

impl fmt::Display for DriverError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            DriverError::Checksum(size) => write!(f, "checksum error occured at packet index {}", size),
            DriverError::ResyncRequired => write!(f, "resync required"),
            DriverError::SerialRead(_) => write!(f, "unable to read from serial port"),
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
pub enum LidarReadingError {
    // The Invalid Data Error flag was set. The associated value is the error code.
    InvalidDataError(u32),
    // The Signal Strength Warning flag was set.
    SignalStrengthWarning,
}
