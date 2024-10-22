

fn print_data(data: &Vec<i32>) {
    for number in data {
        println!("{}", number);
    }
}

fn update_data(data: &mut Vec<i32>) {
    data[2] = 5;
}


fn main() {

    let mut data: Vec<i32> = vec![1,2,3,4,5];

    print_data(&data);

    update_data(&mut data);

    print_data(&data);
}

