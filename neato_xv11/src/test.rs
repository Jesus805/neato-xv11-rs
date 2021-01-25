#[cfg(test)]
mod tests {
    use crate::driver::*;
    use crate::error::LidarDriverError;

    const PACKET: [u8; 22] = [0xFA, 0xB1, 0xE3, 0x49, 0xE4, 0x00, 0xE1, 0x05, 0xE2, 0x00, 0x34,
                              0x06, 0xE0, 0x00, 0x25, 0x06, 0xDF, 0x00, 0x84, 0x06, 0xF6, 0x6B];

    const PACKET_CHECKSUM: u32 = 0x6BF6;

    const BAD_CHECKSUM: [u8; 22] = [0xFA, 0xB1, 0xE3, 0x49, 0xE4, 0x00, 0xE1, 0x05, 0xE2, 0x00, 0x34,
                                    0x06, 0xE0, 0x00, 0x25, 0x06, 0xDF, 0x00, 0x84, 0x06, 0xA6, 0xCE];

    #[test]
    fn checksum_fn_should_be_correct() {
        // Arrange
        let expected_checksum = PACKET_CHECKSUM;
        // Act
        let actual_checksum = calc_checksum(&PACKET[..20]);
        // Assert
        assert_eq!(expected_checksum, actual_checksum);
    }
    
    #[test]
    fn parse_with_correct_checksum_should_return_ok() {
        // Act
        let actual_result = parse_packet(&PACKET);
        // Assert
        assert!(actual_result.is_ok());
    }
    
    #[test]
    fn parse_with_incorrect_checksum_should_return_error() {
        // Arrange
        let expected_result = LidarDriverError::Checksum(0x11);
        // Act
        let actual_result = parse_packet(&BAD_CHECKSUM);
        // Assert
        assert_eq!(expected_result, actual_result.unwrap_err());
    }
}