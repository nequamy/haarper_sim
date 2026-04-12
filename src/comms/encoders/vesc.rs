use crate::physics::{battery::BatteryState, motor::MotorState};

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

pub fn encode_get_values(motor: &MotorState, battery: &BatteryState) -> Vec<u8> {
    let mut payload = vec![0u8; 65];
    payload[0] = 0x04;

    payload[1..3].copy_from_slice(&(350u16).to_be_bytes());
    payload[3..5].copy_from_slice(&(400u16).to_be_bytes());
    payload[5..9].copy_from_slice(&((motor.current * 100.0) as i32).to_be_bytes());
    payload[9..13].copy_from_slice(&((motor.current * motor.duty * 100.0) as i32).to_be_bytes());
    payload[21..23].copy_from_slice(&((motor.duty * 1000.0) as i16).to_be_bytes());
    payload[23..27].copy_from_slice(&(motor.rpm as i32).to_be_bytes());
    payload[27..29].copy_from_slice(&((battery.v_terminal * 10.0) as u16).to_be_bytes());

    let crc = crc16_ccitt(&payload);

    let mut frame = Vec::with_capacity(70);
    frame.push(0x02);
    frame.push(65);
    frame.extend_from_slice(&payload);
    frame.push((crc >> 8) as u8);
    frame.push((crc & 0xFF) as u8);
    frame.push(0x03);

    frame
}

pub fn decode_set_duty(frame: &[u8]) -> Option<f32> {
    if frame.len() < 2 || frame[0] != 0x02 || frame[frame.len() - 1] != 0x03 {
        return None;
    }

    let payload_len = frame[1] as usize;
    if frame.len() < 2 + payload_len + 3 || frame[2] != 0x05 || payload_len < 5 {
        return None;
    }

    let payload = &frame[2..2 + payload_len];
    let crc_received = ((frame[2 + payload_len] as u16) << 8) | (frame[3 + payload_len] as u16);
    let crc_calc = crc16_ccitt(payload);
    if crc_received != crc_calc {
        return None;
    }

    let duty_raw = i32::from_be_bytes([payload[1], payload[2], payload[3], payload[4]]);
    Some(duty_raw as f32 / 100000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_known() {
        let crc = crc16_ccitt(b"123456789");
        assert_eq!(crc, 0x31C3);
    }

    #[test]
    fn test_encode_frame_structure() {
        let motor = MotorState {
            current: 10.0,
            rpm: 5000.0,
            voltage: 12.0,
            duty: 0.5,
        };
        let battery = BatteryState {
            soc: 0.9,
            v_terminal: 11.5,
            i_draw: 10.0,
            r_internal: 0.02,
        };

        let frame = encode_get_values(&motor, &battery);

        assert_eq!(frame[0], 0x02);
        assert_eq!(frame[1], 65);
        assert_eq!(frame[2], 0x04);
        assert_eq!(frame[frame.len() - 1], 0x03);
        assert_eq!(frame.len(), 70);

        let payload = &frame[2..67];
        let crc = crc16_ccitt(payload);
        assert_eq!(frame[67], (crc >> 8) as u8);
        assert_eq!(frame[68], (crc & 0xFF) as u8);
    }

    #[test]
    fn test_decode_set_duty_roundtrip() {
        let duty_raw: i32 = 50000;
        let mut payload = vec![0x05u8];
        payload.extend_from_slice(&duty_raw.to_be_bytes());
        let crc = crc16_ccitt(&payload);

        let mut frame = vec![0x02, payload.len() as u8];
        frame.extend_from_slice(&payload);
        frame.push((crc >> 8) as u8);
        frame.push((crc & 0xFF) as u8);
        frame.push(0x03);

        let duty = decode_set_duty(&frame).unwrap();
        assert!((duty - 0.5).abs() < 0.001);
    }
}
