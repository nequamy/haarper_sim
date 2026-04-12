pub fn encode(accel: [f32; 3], gyro: [f32; 3], quat: [f32; 4]) -> [u8; 20] {
    let mut res = [0u8; 20];

    res[0..2].copy_from_slice(&((accel[0] * 100.0).round() as i16).to_le_bytes());
    res[2..4].copy_from_slice(&((accel[1] * 100.0).round() as i16).to_le_bytes());
    res[4..6].copy_from_slice(&((accel[2] * 100.0).round() as i16).to_le_bytes());

    res[6..8].copy_from_slice(
        &((gyro[0] * (180.0 / std::f32::consts::PI) * 16.0).round() as i16).to_le_bytes(),
    );
    res[8..10].copy_from_slice(
        &((gyro[1] * (180.0 / std::f32::consts::PI) * 16.0).round() as i16).to_le_bytes(),
    );
    res[10..12].copy_from_slice(
        &((gyro[2] * (180.0 / std::f32::consts::PI) * 16.0).round() as i16).to_le_bytes(),
    );

    res[12..14].copy_from_slice(&((quat[3] * 16384.0).round() as i16).to_le_bytes());
    res[14..16].copy_from_slice(&((quat[0] * 16384.0).round() as i16).to_le_bytes());
    res[16..18].copy_from_slice(&((quat[1] * 16384.0).round() as i16).to_le_bytes());
    res[18..20].copy_from_slice(&((quat[2] * 16384.0).round() as i16).to_le_bytes());

    res
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_bno055() {
        let res = encode([1.0, 0.0, 9.81], [0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0]);

        assert_eq!(
            res,
            [
                0x64, 0x00, // ax = 1.0
                0x00, 0x00, // ay = 0.0
                0xD5, 0x03, // az = 9.81
                0x00, 0x00, // gx = 0.0
                0x00, 0x00, // gy = 0.0
                0x5C, 0x00, // gz = 0.1
                0x00, 0x40, // qw = 1.0
                0x00, 0x00, // qx = 0.0
                0x00, 0x00, // qy = 0.0
                0x00, 0x00, // qz = 0.0
            ]
        )
    }
}
