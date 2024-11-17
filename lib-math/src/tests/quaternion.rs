use core::f32::consts::PI;

use crate::*;

#[test]
fn test_standing_up_gravity() {
    // This is a test to check

    // Gravity acceleration in device frame when device is standing straight up.
    let device_gravity = Vector { x: 0.0, y: -1.0, z: 0.0 };

    // Gravity acceleration in world space.
    let expected_world_gravity = Vector { x: 0.0, y: 0.0, z: 1.0 };

    // Orientation of device 
    let device_quat = Quaternion::from(
        EulerAngles::new(0.0, 0.0, PI / -2.0)
    );

    // Rotate the vector using the quaternion
    let world_gravity = device_quat.rotate(&device_gravity);

    println!("{:?}", expected_world_gravity);
    println!("{:?}", world_gravity);

    // Check if the result matches the expectation (with a small tolerance)
    assert!(world_gravity.approx_eq(&expected_world_gravity, 0.0001));
}

#[test]
fn test_rotate_vector_negative_90_deg_x_axis() {
    // Define a quaternion for a 90-degree rotation around the X-axis
    let rotation_quat = Quaternion::from(
        EulerAngles::new(0.0, 0.0, PI / 2.0)
    );

    // Define +Y in local/device space
    let input_vector = Vector { x: 0.0, y: 1.0, z: 0.0 };

    // Rotate the vector using the quaternion
    let rotated_vector = rotation_quat.rotate(&input_vector);

    // Should give +X in world space.
    let expected_vector = Vector { x: 0.0, y: 0.0, z: 1.0 };

    println!("{:?}", rotated_vector);
    println!("{:?}", expected_vector);

    // Check if the result matches the expectation (with a small tolerance)
    assert!(rotated_vector.approx_eq(&expected_vector, 0.0001));
}

#[test]
fn test_from_to_euler_angles_consistency() {
    // Test a variety of Euler angles
    let test_cases = vec![
        (0.0, 0.0, 0.0),                    // Identity rotation
        (PI / 4.0, 0.0, 0.0),               // 45 degrees roll
        (0.0, PI / 4.0, 0.0),               // 45 degrees pitch
        (0.0, 0.0, PI / 4.0),               // 45 degrees yaw
        (PI / 4.0, PI / 4.0, PI / 4.0),     // All axes
        (-PI / 1.2, PI / 3.0, PI / 5.6),    // Mixed signs
    ];

    for (yaw, pitch, roll) in test_cases {
        let euler_in = EulerAngles::new(yaw, pitch, roll);

        // Create a quaternion from Euler angles
        let quat = Quaternion::from(euler_in);

        // Convert back to Euler angles
        let euler_out = EulerAngles::from(quat);

        println!("{:?}", euler_in);
        println!("{:?}", euler_out);

        // Check that the result matches the original input within a small tolerance
        assert!(euler_in.approx_eq(&euler_out, 0.0001));
    }
}
