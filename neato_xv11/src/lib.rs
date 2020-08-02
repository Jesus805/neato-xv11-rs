use std::io::prelude::*;
use serial::prelude::*;
use std::ffi::OsStr;
use std::time::Duration;

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
/// Perform the checksum calculation on the first 20 bytes of the packet.
///
/// ## Remarks
///
/// The slice must be 20 bytes in size.
///
fn checksum(data : &[u8]) -> u32 {
    let mut chk32 : u32 = 0;

    for i in 0..10 {
        // Group the data by word, little-endian
        let lsb : u32 = data[2 * i].into();
        let msb : u32 = data[2 * i + 1].into();
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
/// Synchronizes by finding the beginning of a packet.
///
fn sync<T: SerialPort>(port : &mut T, buffer : &mut [u8; 22]) {
    loop {
        // Read 1 byte until 0xFA is found
        port.read_exact(&mut buffer[0..1]).unwrap();
        if buffer[0] != 0xFA {
            continue;
        }

        // Read the remaining 21 bytes
        port.read_exact(&mut buffer[1..]).unwrap();
        // Ensure that the next byte is a valid index
        if buffer[1] < 0xA0 || buffer[1] > 0xF9 {
            continue;
        }
        
        // In sync, break from loop
        break;
    }
}

/// An implementation of the Neato XV-11 LIDAR.
pub struct NeatoXV11Lidar {
    /// The LIDAR readings. 
    /// Each reading is represented as a (Distance, Reliability) tuple.
    pub readings: [(i32, i32); 360],
}

impl NeatoXV11Lidar {
    /// ## Summary
    ///
    /// Initialize a new instance of NeatoXV11Lidar.
    ///
    /// ## Example
    ///
    /// ```no_run
    /// # use neato_xv11::NeatoXV11Lidar;
    ///
    /// let lidar = neato_xv11::NeatoXV11Lidar::new();
    /// ```
    pub fn new() -> Self {
        NeatoXV11Lidar {
            readings: [(0,0); 360]
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
    /// All multi-byte values are little endian.
    ///
    /// ## Example
    ///
    /// ```no_run
    /// # use neato_xv11::NeatoXV11Lidar;
    ///
    /// let mut lidar = NeatoXV11Lidar::new();
    /// lidar.run("/dev/serial0");
    /// ```
    pub fn run<T: AsRef<OsStr> + ?Sized> (&mut self, port_name: &T) {
        let mut port = serial::open(port_name).unwrap();
        port.set_timeout(Duration::from_secs(1)).unwrap();
        port.configure(&SETTINGS).unwrap();
        
        let mut buffer : [u8; 22] = [0; 22];
        let mut needs_sync = true;

        loop {
            // Sleep for 1 millisecond
            std::thread::sleep(Duration::from_millis(1));

            if needs_sync {
                // Synchronize to ensure every 22 bytes is a valid packet
                sync(&mut port, &mut buffer);
                needs_sync = false;
            }
            else {
                port.read_exact(&mut buffer).unwrap();
                
                if buffer[0] != 0xFA ||
                   buffer[1] < 0xA0 || buffer[1] > 0xF9 {
                    // println!("Got {:X?} for the first two bytes, resync required.", buffer[0..2]);

                    // If the first byte is not '0xFA' or the second byte isn't a valid index; Resync
                    needs_sync = true;
                    continue;
                }
            }

            // println!("{:X?}", buffer);
            
            // Packet index | Range = [0,89]
            let index : usize = buffer[1].into();
            let index = index - 0xA0;
            // println!("Index: {}", index);

            // Verify the packet's integrity
            let msb : u32 = buffer[21].into();
            let lsb : u32 = buffer[20].into();
            
            // Generate the expected checksum
            let expected_checksum = (msb << 8) | lsb;
            let calc_checksum = checksum(&buffer[0..20]);

            if calc_checksum != expected_checksum {
                // Checksum error occured
                for i in 0..4 {
                    self.readings[4 * index + i].0 = 0;
                    self.readings[4 * index + i].1 = -3;
                }
                continue;
            }

            for i in 1..5 {
                let byte_index = 4 * i;

                // The first 2 bytes are two flags + distance
                let msb : i32 = buffer[byte_index + 1].into();
                let lsb : i32 = buffer[byte_index].into();
                let distance = (msb << 8) | lsb;

                // The next 2 bytes are the reliability (higher # = more reliable reading)
                let msb : i32 = buffer[byte_index + 3].into();
                let lsb : i32 = buffer[byte_index + 2].into();
                let quality = (msb << 8) | lsb;

                if distance & 0x8000 > 0 {
                    // Invalid data flag triggered. LSB contains error code
                    self.readings[4 * index + i - 1].0 = distance & 0x00FF;
                    self.readings[4 * index + i - 1].1 = -1;
                    continue;
                }
                else if distance & 0x4000 > 0 {
                    // Signal strength warning flag triggered
                    self.readings[4 * index + i - 1].0 = distance & 0x4000;
                    self.readings[4 * index + i - 1].1 = -2;
                    continue;
                }
                else {
                    // No flag triggered. Write distance to readings
                    self.readings[4 * index + i - 1].0 = distance;
                    self.readings[4 * index + i - 1].1 = quality;
                }
            }
            
            /*
            if index == 89 {
                print!("[");
                for reading in self.readings.iter() {
                    print!("({},{}), ", reading.0, reading.1);
                }
                println!("]\n\n");
            }
            */
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_should_be_correct() {
        // Arrange
        let data : [u8; 22] = [0xFA, 0xB1, 0xE3, 0x49, 0xE4, 0x00, 0xE1, 0x05, 0xE2, 0x00, 0x34,
                               0x06, 0xE0, 0x00, 0x25, 0x06, 0xDF, 0x00, 0x84, 0x06, 0xF6, 0x6B];
        let expected_checksum = 0x6BF6;
        // Act
        let actual_checksum = checksum(&data[..20]);
        // Assert
        assert_eq!(expected_checksum, actual_checksum);
    }
}
