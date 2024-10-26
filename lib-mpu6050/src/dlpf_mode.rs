
/// DLPF (Digital Low Pass Filter) mode, this determines the highest frequency that is not filtered
/// out. 
/// 
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DLPFMode {
    Bw256Hz = 0x00,
    Bw188Hz = 0x01,
    Bw98Hz = 0x02,
    Bw42Hz = 0x03,
    Bw20Hz = 0x04,
    Bw10Hz = 0x05,
    Bw5Hz = 0x06,
}
