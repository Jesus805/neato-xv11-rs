use std::fmt::Display;

use super::data::LidarPacket;

/// ## Summary
///
/// Messages sent to the LIDAR driver.
///
pub enum LidarDriverCommand {
    // Pause LIDAR reading.
    Pause,
    // Run LIDAR.
    Run,
    // Stop LIDAR.
    Stop,
}

impl Display for LidarDriverCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            LidarDriverCommand::Pause => write!(f, "Pause"),
            LidarDriverCommand::Run => write!(f, "Run"),
            LidarDriverCommand::Stop => write!(f, "Stop"),
        }
    }
}

/// ## Summary
/// 
/// Messages received from the LIDAR driver.
/// 
#[derive(Debug)]
pub enum LidarDriverMessage {
    // A LIDAR packet (4 readings).
    Packet(LidarPacket),
    // The LIDAR is shutting down.
    Shutdown,
}