use std::{env, fs::{self, File}, io::Write, path::Path};

use math::*;
use processing::*;

static RESULTS_DIR: &str = "analysis";

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        panic!("Expected CSV file to read data from");
    }

    env_logger::init();

    fs::create_dir_all(RESULTS_DIR).unwrap();

    // Open the input CSV file.
    let in_path = Path::new(&args[1]);
    let in_file = File::open(in_path).unwrap();
    
    let out_dir = format!("{}/{}", RESULTS_DIR, in_path.file_name().unwrap().to_str().unwrap());

    let mut sensor_fusion = SensorFusion::new();
    let mut step_detection = StepDetection::new();

    let mut algo_file = File::create(out_dir.replace(".csv", "_algo.csv")).unwrap();
    algo_file.write(sensor_fusion.get_csv_header().as_bytes()).unwrap();

    let mut angles_file = File::create(out_dir.replace(".csv", "_angles.csv")).unwrap();
    angles_file.write(b"time,").unwrap();
    angles_file.write(b"fusion.yaw,fusion.pitch,fusion.roll,").unwrap();
    angles_file.write(b"gyro.yaw,gyro.pitch,gyro.roll,").unwrap();
    angles_file.write(b"accel.yaw,accel.pitch,accel.roll,").unwrap();
    angles_file.write(b"gyro.x,gyro.y,gyro.z,gyro.w\n").unwrap();

    let mut steps_file = File::create(out_dir.replace(".csv", "_steps.csv")).unwrap();
    steps_file.write(b"start,peak,finished,peak.x,peak.y,peak.z,").unwrap();
    steps_file.write(b"velocity.x,velocity.y,velocity.z\n").unwrap();

    // Loop over every line the in the input CSV.
    let mut reader = csv::Reader::from_reader(in_file);
    for result in reader.deserialize::<Vec<f32>>() {

        let row = result.unwrap();

        let time = row[0] as i64;
        let gyro = Vector::new(row[1], row[2], row[3]);
        let accel = Vector::new(row[4], row[5], row[6]);

        sensor_fusion.update(time, &accel, &gyro);
        step_detection.update(&mut sensor_fusion, &accel);
        
        algo_file.write(sensor_fusion.get_csv_state().as_bytes()).unwrap();

        let fusion_angles = EulerAngles::from(&sensor_fusion.orientation);
        let gyro_angles = EulerAngles::from(&sensor_fusion.gyro_orientation);
        let accel_angles = EulerAngles::from(&sensor_fusion.accel_orientation);

        angles_file.write(format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            time,
            fusion_angles.yaw * RAD_TO_DEG,
            fusion_angles.pitch * RAD_TO_DEG,
            fusion_angles.roll * RAD_TO_DEG,
            gyro_angles.yaw * RAD_TO_DEG,
            gyro_angles.pitch * RAD_TO_DEG,
            gyro_angles.roll * RAD_TO_DEG,
            accel_angles.yaw * RAD_TO_DEG,
            accel_angles.pitch * RAD_TO_DEG,
            accel_angles.roll * RAD_TO_DEG,
            sensor_fusion.gyro_orientation.x,
            sensor_fusion.gyro_orientation.y,
            sensor_fusion.gyro_orientation.z,
            sensor_fusion.gyro_orientation.w,
        ).as_bytes()).unwrap();

        if step_detection.step_is_done() {
            steps_file.write(format!(
                "{},{},{}\n",
                step_detection.start_time.unwrap(),
                step_detection.peak_time,
                step_detection.stop_time.unwrap(),
                // step_detection.peak_pos.x,
                // step_detection.peak_pos.y,
                // step_detection.peak_pos.z,
            ).as_bytes()).unwrap();
        }
    }

    if !step_detection.step_is_done() && step_detection.start_time.is_some() {
        steps_file.write(format!(
            "{},{},{},{},{},{},{},{},{}\n",
            step_detection.start_time.unwrap(),
            step_detection.peak_time,
            step_detection.stop_time.unwrap_or(sensor_fusion.prev_time),
            step_detection.peak_position.x,
            step_detection.peak_position.y,
            step_detection.peak_position.z,
            step_detection.peak_velocity.x,
            step_detection.peak_velocity.y,
            step_detection.peak_velocity.z,
        ).as_bytes()).unwrap();
    }
}
