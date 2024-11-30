use embedded_sdmmc::{TimeSource, Timestamp};

pub struct RtcSource {}
impl<'d, 'a> TimeSource for RtcSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }   
    }
}
