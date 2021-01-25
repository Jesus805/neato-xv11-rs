mod driver;
mod test;
pub mod data;
pub mod error;
pub mod message;

pub mod prelude {
    pub use crate::data::{LidarReading, LidarPacket};
    pub use crate::error::{LidarDriverError, LidarReadingError};
    pub use crate::message::{LidarDriverCommand, LidarDriverMessage};
}

pub use driver::*;