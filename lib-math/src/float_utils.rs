
/// Rust implementation of the Arduino's standard lib' map function. This function scales a given
/// float from one range to another.
/// 
/// Note that this function does not constrain values to within the range, because out-of-range 
/// values are sometimes intended and useful. 
/// 
/// 
pub fn map_range(value: f64, from_low: f64, from_high: f64, to_low: f64, to_high: f64) -> f64 {
    (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
}

/// Take the absolute value of an integer
/// 
pub fn abs(x: f64) -> f64 {
    if x < 0.0 {
        return x * -1.0;
    }
    x
}
