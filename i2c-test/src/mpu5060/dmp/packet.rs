
pub const DMP_PACKET_SIZE: u16 = 42;

#[derive(Debug, Clone, Copy)]
pub struct DMPPacket {

}

impl DMPPacket {
    pub fn from_bytes(bs: [ u8 ; DMP_PACKET_SIZE as usize ]) -> Self {
        log::debug!("packet: {:?}", bs);
        DMPPacket {
            
        }
    }
}