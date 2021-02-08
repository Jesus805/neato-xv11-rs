use std::ffi::OsStr;
use std::sync::mpsc::{Sender, Receiver, TryRecvError};
use std::time::Duration;
use std::vec::Vec;

#[cfg(feature = "log")]
use log::{info, warn, error};

use serial::prelude::*;

use super::prelude::*;


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
pub(crate) fn calc_checksum(data : &[u8]) -> u32 {
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
pub(crate) fn parse_packet(buffer: &[u8; 22]) -> Result<LidarDriverMessage, LidarDriverError> {
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
    let calc_checksum = calc_checksum(&buffer[0..20]);

    if calc_checksum != expected_checksum {
        #[cfg(feature = "log")]
        error!("A checksum error occured. The data is corrupted");

        // Checksum error occured. The data is corrupted.
        return Err(LidarDriverError::Checksum(index));
    }

    for i in 1..5 {
        let byte_index = 4 * i;
        let reading_index = 4 * index + i - 1;

        // The first 2 bytes are two flags + distance.
        let msb = buffer[byte_index + 1] as i32;
        let lsb = buffer[byte_index] as i32;
        let distance = (msb << 8) | lsb;

        // The next 2 bytes are the reliability (higher # = more reliable reading).
        let msb = buffer[byte_index + 3] as i32;
        let lsb = buffer[byte_index + 2] as i32;
        let quality = (msb << 8) | lsb;

        if distance & 0x8000 > 0 {
            // Invalid data flag triggered. LSB contains error code.
            let error_code = distance & 0x00FF;
            readings.push(LidarReading::new(reading_index, distance, quality, Some(LidarReadingError::InvalidDataError(error_code))));
        } else if distance & 0x4000 > 0 {
            // Signal strength warning flag triggered. Remove flag before recording.
            let distance = distance & 0x3FFF;
            readings.push(LidarReading::new(reading_index, distance, quality, Some(LidarReadingError::SignalStrengthWarning)));
        } else {
            // No flag triggered. Write distance to readings.
            readings.push(LidarReading::new(reading_index, distance, quality, None));
        }
    }
    
    Ok(LidarDriverMessage::Packet(LidarPacket::new(readings, speed)))
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
fn read<T: SerialPort>(port: &mut T, mut buffer: &mut [u8], tx: &Sender<Result<LidarDriverMessage, LidarDriverError>>) -> Result<(), ()> {
    port.read_exact(&mut buffer).map_err(|e| {
        #[cfg(feature = "log")]
        error!("Unable to read from serial port. {}", e);

        // Consume error into wrapper.
        let serial_error = LidarDriverError::SerialRead(e);
        
        // Report error to the calling program.
        // We don't care about the result since being unable to read is a fatal error.
        let _ = send_message(&tx, Err(serial_error));
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
fn sync<T: SerialPort>(mut port: &mut T, buffer: &mut [u8; 22], tx: &Sender<Result<LidarDriverMessage, LidarDriverError>>) -> Result<(), ()> {
    loop {
        // Sleep for 1 millisecond.
        std::thread::sleep(Duration::from_micros(100));
        
        // Read 1 byte until '0xFA' is found.
        if let Err(_) = read::<T>(&mut port, &mut buffer[0..1], &tx) {
            return Err(());
        }

        if buffer[0] != 0xFA {
            continue;
        }

        // Read the remaining 21 bytes.
        if let Err(_) = read::<T>(&mut port, &mut buffer[1..], &tx) {
            return Err(());
        }

        // Ensure that the next byte is a valid index.
        if buffer[1] < 0xA0 || buffer[1] > 0xF9 {
            continue;
        }
        
        // In sync, break out of loop.
        return Ok(());
    }
}

fn send_message(tx: &Sender<Result<LidarDriverMessage, LidarDriverError>>, result: Result<LidarDriverMessage, LidarDriverError>) -> Result<(), ()> {
    #[cfg(feature = "log")]
    return tx.send(result).map_err(|e| {
        error!("Unable to send message. {}", e);
    });
    
    #[cfg(not(feature = "log"))]
    return tx.send(result).map_err(|_| {});
}

/// ## Summary
/// 
/// Begin reading LIDAR data.
/// 
/// ## Parameters
/// 
/// port_name: The port name to open.
/// 
/// tx: Sends decoded LIDAR messages or error encountered.
/// 
/// rx: Receives commands from the calling program.
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
/// # use std::thread;
/// # use std::sync::mpsc::channel;
/// # use neato_xv11;
/// 
/// // Create a message channel.
/// let (message_tx, message_rx) = channel();
/// // Create a command channel.
/// let (command_tx, command_rx) = channel();
/// 
/// thread::spawn(move || {
///     neato_xv11::run("/dev/serial0", message_tx, command_rx);
/// });
/// ```
pub fn run<T: AsRef<OsStr> + ?Sized> (port_name: &T, tx: Sender<Result<LidarDriverMessage, LidarDriverError>>, rx: Receiver<LidarDriverCommand>) {
    let mut port;
    
    // Open the serial port.
    match serial::open(port_name) {
        Ok(p) => {
            #[cfg(feature = "log")]
            info!("Successfully opened serial port");

            // Initialize the serial port.
            port = p
        },
        Err(err) => {
            #[cfg(feature = "log")]
            error!("Unable to open serial port. {}", err);

            // Unable to open the serial port.
            let _ = send_message(&tx, Err(LidarDriverError::OpenSerialPort(err)));
            return;
        }
    }

    // Set the timeout.
    if let Err(err) = port.set_timeout(Duration::from_secs(1)) {
        #[cfg(feature = "log")]
        error!("Unable to set timeout. {}", err);
        
        // Unable to set the timeout.
        let _ = send_message(&tx, Err(LidarDriverError::SetTimeout(err)));
        return;
    }

    #[cfg(feature = "log")]
    info!("Successfully set the timeout");

    // Configure the serial port.
    if let Err(err) = port.configure(&SETTINGS) {
        #[cfg(feature = "log")]
        error!("Unable to configure serial port. {}", err);

        // Unable to configure the port.
        let _ = send_message(&tx, Err(LidarDriverError::Configure(err)));
        return;
    }

    #[cfg(feature = "log")]
    info!("Successfully configured the serial port");
    
    // Temporary buffer to hold packet data.
    let mut buffer : [u8; 22] = [0; 22];
    // Dictates if synchronization is required.
    let mut needs_sync = true;
    // Prevents the driver from reading from the serial port.
    let mut is_paused = false;

    loop {
        // Sleep for 1 millisecond.
        std::thread::sleep(Duration::from_millis(1));

        // Try to receive a command message from the main thread.
        match rx.try_recv() {
            Ok(cmd) => {
                #[cfg(feature = "log")]
                info!("Received command {}", cmd);

                match cmd {
                    LidarDriverCommand::Run => is_paused = false,
                    LidarDriverCommand::Pause => is_paused = true,
                    LidarDriverCommand::Stop => break,
                }
            },
            Err(err) => {
                match err {
                    TryRecvError::Empty => {}
                    TryRecvError::Disconnected => {
                        #[cfg(feature = "log")]
                        error!("Command channel disconnected");
                        break;
                    },
                }
            }
        }

        if is_paused {
            // Skip reading from serial.
            continue;
        }

        // Clear buffer
        for element in buffer.iter_mut() {
            *element = 0;
        }

        if needs_sync {
            // Synchronize to ensure every 22 bytes is a valid packet.
            if let Err(_) = sync(&mut port, &mut buffer, &tx) {
                #[cfg(feature = "log")]
                error!("Unable to sync");

                // Error syncing.
                break;
            }
            needs_sync = false;
        }
        else {
            // Read 22 bytes from serial.
            if let Err(_) = read(&mut port, &mut buffer, &tx) {
                // Error reading from serial.
                break;
            }
            
            if buffer[0] != 0xFA || buffer[1] < 0xA0 || buffer[1] > 0xF9 {
                // The first byte is not '0xFA' or the second byte isn't a valid index.
                // Resync required.
                #[cfg(feature = "log")]
                warn!("Corrupted data, resync required.");

                if let Err(_) = send_message(&tx, Err(LidarDriverError::ResyncRequired)) {
                    // Sending a message to the calling program failed, shutdown the driver.
                    break;
                } else {
                    needs_sync = true;
                    continue;
                }
            }
        }

        let result = parse_packet(&buffer);
        
        if let Err(_) = send_message(&tx, result) {
            // Sending a message to the calling program failed, shutdown the driver.
            break;
        }
    }

    #[cfg(feature = "log")]
    info!("Shutting down lidar.");

    let _ = send_message(&tx, Ok(LidarDriverMessage::Shutdown));
}