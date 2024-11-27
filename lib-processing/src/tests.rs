
use crate::*;

/// When almost no changes happen (i.e. accelerometer and gyroscope detect almost no movement)
/// applying one step worth of time should leave the state of the algorithm nearly unchanged.
/// 
#[test]
pub fn no_movement_unchanged() {
    let mut algo = ProcessingAlgorithm::new();
    algo.prev_time = 381531544000;

    algo.step(
        381538727000, 
        Vector::new(0.00066615, 0.0003829, 0.0003297), 
        Vector::new(0.001266615, 0.0063829, 0.00334767), 
    );

    let unchanged = Quaternion::identity();

    assert!(
        libm::fabsf(algo.orientation.x - unchanged.x) < 0.1,
        "{} != {}", algo.orientation.x, unchanged.x
    );
    assert!(
        libm::fabsf(algo.orientation.y - unchanged.y) < 0.1,
        "{} != {}", algo.orientation.y, unchanged.y
    );
    assert!(
        libm::fabsf(algo.orientation.z - unchanged.z) < 0.1,
        "{} != {}", algo.orientation.z, unchanged.z
    );
    assert!(
        libm::fabsf(algo.orientation.w - unchanged.w) < 0.1,
        "{} != {}", algo.orientation.w, unchanged.w
    );
}