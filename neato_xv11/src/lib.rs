pub mod error;
pub mod message;
mod driver;

pub mod prelude {
    pub use crate::error::{DriverError, LidarReadingError};
    pub use crate::message::{LidarReading, LidarMessage};
}

pub use crate::driver::*;
