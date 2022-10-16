const PACKET_START: [u8; 2] = [0x4D, 0x39]; // M9

enum Type {
    GetRequest = 0x47,  // G
    GetResponse = 0x52, // R
    SetRequest = 0x53,  // S
    SetAck = 0x4B,      // K
    SetError = 0x45,    // E
}

fn calculate_crc(data: &[u8]) -> u8 {
    simple_crc::simple_crc8(data, 0x07, 0x00, false, false, 0x00)
}
