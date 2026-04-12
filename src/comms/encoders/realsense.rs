pub fn encode_rgb(rgba: &[u8], width: u32, height: u32) -> Vec<u8> {
    let n = (width * height) as usize;
    let mut rgb = Vec::with_capacity(n * 3);
    for i in 0..n {
        rgb.push(rgba[i * 4]);
        rgb.push(rgba[i * 4 + 1]);
        rgb.push(rgba[i * 4 + 2]);
    }
    rgb
}

pub fn encode_depth(_width: u32, _height: u32) -> Vec<u8> {
    vec![0u8; 320 * 240 * 2]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_rgb_strip_alpha() {
        let rgba = vec![255, 128, 64, 255, 0, 0, 0, 128];
        let rgb = encode_rgb(&rgba, 2, 1);
        assert_eq!(rgb, vec![255, 128, 64, 0, 0, 0]);
        assert_eq!(rgb.len(), 6);
    }
}
