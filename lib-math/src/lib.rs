#![no_std]

pub mod float_utils;
pub use float_utils::*;

pub mod vector;
pub use vector::*;

pub mod quaternion;
pub use quaternion::*;

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
