use super::error::{DriverError, LidarReadingError};
use super::message::{LidarReading, LidarMessage};
use serial::prelude::*;
use std::ffi::OsStr;
use std::sync::mpsc::Sender;
use std::time::Duration;
use std::vec::Vec;

/// Default Neato XV-11 LIDAR settings.
const SETTINGS: serial::PortSettings = serial::PortSettings {
    baud_rate: serial::Baud115200,
    char_size: serial::CharSize::Bits8,
    parity: serial::Parity::ParityNone,
    stop_bits: serial::StopBits::Stop1,
    flow_control: serial::FlowControl::FlowNone,
};

/// ## Summary
///
/// Calculate the checksum using the first 20 bytes of the packet.
///
/// ## Remarks
///
/// The slice must be 20 bytes in size.
///
fn checksum(data : &[u8]) -> u32 {
    let mut chk32 : u32 = 0;

    for i in 0..10 {
        // Group the data by word, little-endian
        let lsb = data[2 * i] as u32;
        let msb = data[2 * i + 1] as u32;
        let val = (msb << 8) | lsb;
        // compute the checksum on 32 bits
        chk32 = (chk32 << 1) + val;
    }

    // Wrap around to fit into 15 bits
    let mut check_sum = (chk32 & 0x7FFF) + (chk32 >> 15);
    // Truncate to 15 bits
    check_sum &= 0x7FFF;
    // Return the checksum
    check_sum
}

/// ## Summary
///
/// Parse encoded LIDAR packet.
///
fn parse(buffer: &[u8; 22]) -> Result<LidarMessage, DriverError> {
    let mut readings = Vec::with_capacity(4);

    // Packet index | Range = [0,89].
    let index = buffer[1] as usize;
    let index = index - 0xA0;

    // Lidar Speed.
    let msb = buffer[3] as u32;
    let lsb = buffer[2] as u32;
    let speed = ((msb << 8) | lsb) as f64 / 64.0;

    // Verify the packet's integrity.
    let msb = buffer[21] as u32;
    let lsb = buffer[20] as u32;

    // Generate the expected checksum.
    let expected_checksum = (msb << 8) | lsb;
    let calc_checksum = checksum(&buffer[0..20]);

    if calc_checksum != expected_checksum {
        // Checksum error occured. The data is corrupted.
        return Err(DriverError::Checksum(index));
    }

    for i in 1..5 {
        let byte_index = 4 * i;
        let reading_index = 4 * index + i - 1;

        // The first 2 bytes are two flags + distance.
        let msb = buffer[byte_index + 1] as u32;
        let lsb = buffer[byte_index] as u32;
        let distance = (msb << 8) | lsb;

        // The next 2 bytes are the reliability (higher # = more reliable reading).
        let msb = buffer[byte_index + 3] as u32;
        let lsb = buffer[byte_index + 2] as u32;
        let quality = (msb << 8) | lsb;

        if distance & 0x8000 > 0 {
            // Invalid data flag triggered. LSB contains error code.
            let error_code = distance & 0x00FF;
            readings.push(LidarReading::new(reading_index, distance, quality, Some(LidarReadingError::InvalidDataError(error_code))));
        }
        else if distance & 0x4000 > 0 {
            // Signal strength warning flag triggered. Remove flag before recording.
            let distance = distance & 0x3FFF;
            readings.push(LidarReading::new(reading_index, distance, quality, Some(LidarReadingError::SignalStrengthWarning)));
        }
        else {
            // No flag triggered. Write distance to readings.
            readings.push(LidarReading::new(reading_index, distance, quality, None));
        }
    }
    
    Ok(LidarMessage::new(readings, speed))
}

/// ## Summary
///
/// Read from the serial port. Send read errors to the async channel.
///
/// ## Parameters
///
/// port: The port to read from.
///
/// buffer: The buffer to read to. The size of the slice will be the read size.
/// 
/// tx: Send channel to write to in the event of a read error.
///
fn read<T: SerialPort>(port: &mut T, mut buffer: &mut [u8], tx: &Sender<Result<LidarMessage, DriverError>>) -> Result<(), ()> {
    port.read_exact(&mut buffer).map_err(|e| {
        // Consume error into wrapper.
        let serial_error = DriverError::SerialRead(e);
        // Report error to the calling program.
        tx.send(Err(serial_error)).unwrap();
    })
}

/// ## Summary
///
/// Synchronizes by finding the header of a NeatoXV-11 LIDAR data packet.
///
/// ## Parameters
///
/// port: The port to read from.
///
/// buffer: The buffer to read to.
/// 
/// tx: Send channel to write to in the event of a read error.
///
fn sync<T: SerialPort>(mut port: &mut T, buffer: &mut [u8; 22], tx: &Sender<Result<LidarMessage, DriverError>>) {
    loop {
        // Read 1 byte until '0xFA' is found.
        if let Err(_) = read::<T>(&mut port, &mut buffer[0..1], &tx) {
            break;
        }

        if buffer[0] != 0xFA {
            continue;
        }

        // Read the remaining 21 bytes.
        if let Err(_) = read::<T>(&mut port, &mut buffer[1..], &tx) {
            break;
        }

        // Ensure that the next byte is a valid index.
        if buffer[1] < 0xA0 || buffer[1] > 0xF9 {
            continue;
        }
        
        // In sync, break out of loop.
        break;
    }
}


/// ## Summary
///
/// Begin reading LIDAR data.
/// 
/// ## Parameters
/// 
/// port_name: The port name to open.
///
/// ## Remarks
///
/// 22 byte packet format:
/// [0xFA, 1-byte index, 2-byte speed, [2-byte flags/distance, 2-byte quality] * 4, 2-byte checksum]
/// All multi-byte values are little endian (except speed which is big endian)
///
/// ## Example
///
/// ```no_run
/// # use neato_xv11;
/// # use std::sync::mpsc::channel;
/// 
/// let (tx, rx) = channel();
/// neato_xv11::run("/dev/serial0", tx);
/// ```
pub fn run<T: AsRef<OsStr> + ?Sized> (port_name: &T, tx: Sender<Result<LidarMessage, DriverError>>) {
    // Initialize the serial port.
    let mut port = serial::open(port_name).unwrap();
    port.set_timeout(Duration::from_secs(1)).unwrap();
    port.configure(&SETTINGS).unwrap();
    
    // Temporary buffer to hold packet data.
    let mut buffer : [u8; 22] = [0; 22];
    // If true, synchronization is required.
    let mut needs_sync = true;

    loop {
        // Sleep for 1 millisecond.
        std::thread::sleep(Duration::from_millis(1));

        // Clear buffer
        for element in buffer.iter_mut() {
            *element = 0;
        }

        if needs_sync {
            // Synchronize to ensure every 22 bytes is a valid packet.
            sync(&mut port, &mut buffer, &tx);
            needs_sync = false;
        }
        else {
            if let Err(_) = read(&mut port, &mut buffer, &tx) {
                break;
            }
            
            if buffer[0] != 0xFA || buffer[1] < 0xA0 || buffer[1] > 0xF9 {
                // The first byte is not '0xFA' or the second byte isn't a valid index.
                // Resync required.
                tx.send(Err(DriverError::ResyncRequired)).unwrap();

                needs_sync = true;
                continue;
            }
        }

        let result = parse(&buffer);
        
        tx.send(result).unwrap();
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const PACKET: [u8; 22] = [0xFA, 0xB1, 0xE3, 0x49, 0xE4, 0x00, 0xE1, 0x05, 0xE2, 0x00, 0x34,
                              0x06, 0xE0, 0x00, 0x25, 0x06, 0xDF, 0x00, 0x84, 0x06, 0xF6, 0x6B];

    const BAD_CHECKSUM: [u8; 22] = [0xFA, 0xB1, 0xE3, 0x49, 0xE4, 0x00, 0xE1, 0x05, 0xE2, 0x00, 0x34,
                                    0x06, 0xE0, 0x00, 0x25, 0x06, 0xDF, 0x00, 0x84, 0x06, 0xA6, 0xCE];


    #[test]
    fn checksum_fn_should_be_correct() {
        // Arrange
        let expected_checksum = 0x6BF6;
        // Act
        let actual_checksum = checksum(&PACKET[..20]);
        // Assert
        assert_eq!(expected_checksum, actual_checksum);
    }
    
    #[test]
    fn parse_with_correct_checksum_should_return_ok() {
        // Act
        let actual_result = parse(&PACKET);
        // Assert
        assert!(actual_result.is_ok());
    }
    
    /*
    #[test]
    fn parse_with_incorrect_checksum_should_return_error() {
        // Arrange
        let expected_result = DriverError::Checksum(0x11);
        // Act
        let actual_result = parse(&BAD_CHECKSUM).expect_err("Checksum Error expected");
        // Assert
        if actual_result == DriverError::Checksum {

        }
        assert_eq!(actual_result, expected_result);
    }
    */
}
