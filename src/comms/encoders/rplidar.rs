use crate::sensors::lidar::LidarScan;

pub fn encode(scan: &LidarScan) -> Vec<u8> {
    let n = scan.ranges.len();
    let mut buf = Vec::with_capacity(2 + 8 + n * 8);

    buf.extend_from_slice(&(n as u16).to_le_bytes());
    buf.extend_from_slice(&((scan.timestamps * 1_000_000.0) as u64).to_le_bytes());

    for i in 0..n {
        buf.extend_from_slice(&scan.angles[i].to_le_bytes());
        buf.extend_from_slice(&scan.ranges[i].to_le_bytes());
    }

    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_two_points() {
        let scan = LidarScan {
            ranges: vec![1.5, 12.0],
            angles: vec![0.0, std::f32::consts::FRAC_PI_2],
            timestamps: 123.456,
        };

        let buf = encode(&scan);

        assert_eq!(buf.len(), 26);
        assert_eq!(buf[0..2], [0x02, 0x00]);
        assert_eq!(buf[2..10], 123.456_f64.to_bits().to_le_bytes());
        assert_eq!(buf[10..14], 0.0_f32.to_le_bytes());
        assert_eq!(buf[14..18], 1.5_f32.to_le_bytes());
        assert_eq!(buf[18..22], std::f32::consts::FRAC_PI_2.to_le_bytes());
        assert_eq!(buf[22..26], 12.0_f32.to_le_bytes());
    }
}
