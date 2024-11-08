use std::{env, fs::{self, File}, io::Write, path::Path};
use math::{Quaternion, Vector};
use mpu6050::dmp::DMPPacket;

static RESULTS_DIR: &str = "analysis";

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        panic!("Expected CSV file to read data from");
    }

    // Open the input CSV file.
    let in_path = Path::new(&args[1]);
    let in_file = File::open(in_path).unwrap();
    
    // Every input CSV file gets its own folder in the results directory.
    let out_dir = in_path.file_name().unwrap().to_str().unwrap().to_string().replace(".csv", "");
    let out_dir = format!("{}/{}", RESULTS_DIR, out_dir);
    fs::create_dir_all(&out_dir).unwrap();
    
    // Create a file to save the gravity vector calculations to.
    let mut gravity_file = File::create(format!("{}/{}", out_dir, "gravity.csv")).unwrap();
    gravity_file.write(b"time,x,y,z\n").unwrap();

    // Loop over every line the in the input CSV.
    let mut reader = csv::Reader::from_reader(in_file);
    for result in reader.deserialize::<Vec<f32>>() {
        let record = result.unwrap();

        let time = record[0];
        let packet = DMPPacket {
            gyro: Vector { x: record[7], y: record[8], z: record[9], },
            accel: Vector { x: record[10], y: record[11], z: record[12], },
            quaternion: Quaternion { w: record[13], x: record[14], y: record[15], z: record[16], },
        };

        let gravity = packet.get_gravity();
        gravity_file.write(format!(
            "{},{},{},{}\n", 
            time, gravity.x, gravity.y, gravity.z
        ).as_bytes()).unwrap();
    }
}
