use crate::*;

#[test]
fn test_cross_product_orthogonal_vectors() {
    let v1 = Vector { x: 1.0, y: 0.0, z: 0.0 };
    let v2 = Vector { x: 0.0, y: 1.0, z: 0.0 };
    let result = v1.cross(&v2);
    let expected = Vector { x: 0.0, y: 0.0, z: 1.0 };
    assert!(result.approx_eq(&expected, 1e-6));
}

#[test]
fn test_cross_product_parallel_vectors() {
    let v1 = Vector { x: 1.0, y: 2.0, z: 3.0 };
    let v2 = Vector { x: 2.0, y: 4.0, z: 6.0 };
    let result = v1.cross(&v2);
    let expected = Vector { x: 0.0, y: 0.0, z: 0.0 };
    assert!(result.approx_eq(&expected, 1e-6));
}

#[test]
fn test_cross_product_arbitrary_vectors() {
    let v1 = Vector { x: 3.0, y: -3.0, z: 1.0 };
    let v2 = Vector { x: 4.0, y: 9.0, z: 2.0 };
    let result = v1.cross(&v2);
    let expected = Vector { x: -15.0, y: -2.0, z: 39.0 };
    assert!(result.approx_eq(&expected, 1e-6));
}