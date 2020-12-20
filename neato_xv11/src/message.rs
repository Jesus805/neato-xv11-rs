use super::error::LidarReadingErrorType;

#[allow(dead_code)]
#[derive(Debug)]
pub struct LidarReading {
    index: usize,
    distance: u32,
    quality: u32,
    error: LidarReadingErrorType,
}

impl LidarReading {
    pub(crate) fn new(index: usize,
                      distance: u32,
                      quality: u32,
                      error: LidarReadingErrorType) -> Self {
        LidarReading {
            index,
            distance,
            quality,
            error,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct LidarMessage {
    readings: Vec<LidarReading>,
    speed: f64,
}

impl LidarMessage {
    pub(crate) fn new(readings: Vec<LidarReading>, speed: f64) -> Self {
        LidarMessage {
            readings,
            speed,
        }
    }
}

