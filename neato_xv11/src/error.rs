

#[derive(Debug, PartialEq)]
pub enum DriverErrorType {
    // Checksum error occured. The associated value is the packet index.
    ChecksumError(usize),
    // A resync is required.
    ResyncRequired,
}

#[derive(Debug)]
pub enum LidarReadingErrorType {
    // No warnings or errors reported.
    None,
    // The Invalid Data Error flag was set. The associated value is the error code.
    InvalidDataError(u32),
    // The Signal Strength Warning flag was set.
    SignalStrengthWarning,
}
