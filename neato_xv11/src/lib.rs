mod driver;
pub mod error;
pub mod message;
pub mod command;

pub mod prelude {
    pub use crate::command::LidarDriverCommand;
    pub use crate::error::{LidarDriverError, LidarReadingError};
    pub use crate::message::{LidarReading, LidarMessage};
}

pub use driver::*;
