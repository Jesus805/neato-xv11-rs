use super::error::LidarReadingError;

#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};

/// ## Summary
///
/// A LIDAR distance reading.
///
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct LidarReading {
    // Index of the reading.
    pub index: usize,
    // Distance in millimeters.
    pub distance: i32,
    // Quality of the reading.
    pub quality: i32,
    // Error reported in reading.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub error: Option<LidarReadingError>,
}

impl LidarReading {
    /// ## Summary
    /// 
    /// Initialize a new LIDAR reading.
    /// 
    /// ## Parameters
    /// 
    /// index: Index of the reading.
    /// 
    /// distance: Distance in millimeters.
    /// 
    /// quality: Quality of the reading.
    /// 
    /// error: Error reported in reading.
    /// 
    pub(crate) fn new(index: usize,
                      distance: i32,
                      quality: i32,
                      error: Option<LidarReadingError>) -> Self {
        LidarReading {
            index,
            distance,
            quality,
            error,
        }
    }
}

/// ## Summary
///
/// A decoded LIDAR packet containing four distance readings.
///
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct LidarPacket {
    // Collection of four readings.
    pub readings: Vec<LidarReading>,
    // LIDAR spin speed (RPM).
    pub speed: f64,
}

impl LidarPacket {
    /// ## Summary
    /// 
    /// Initialize a new decoded LIDAR message.
    /// 
    /// ## Parameters
    /// 
    /// readings: Collection of four readings.
    /// 
    /// speed: LIDAR spin speed (RPM).
    /// 
    pub(crate) fn new(readings: Vec<LidarReading>, speed: f64) -> Self {
        LidarPacket {
            readings,
            speed,
        }
    }
}
