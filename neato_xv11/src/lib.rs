pub mod error;
pub mod message;
mod driver;

pub mod prelude {
    pub use crate::error::{DriverErrorType, LidarReadingErrorType};
    pub use crate::message::{LidarReading, LidarMessage};
}

pub use crate::driver::*;
