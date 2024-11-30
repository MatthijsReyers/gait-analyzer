use core::cmp::min;


pub struct ByteWriter<'a> {
    pub buf: &'a mut [u8],
    pub head: usize,
}

impl<'a> ByteWriter<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        ByteWriter { buf, head: 0 }
    }

    pub fn reset(&mut self) {
        for x in 0 .. self.head {
            self.buf[x] = 0;
        }
        self.head = 0;
    }

    pub fn get_msg(&self) -> &[u8] {
        &self.buf[0..self.head]
    }

    /// Removes the last `n` characters from the buffer.
    /// 
    pub fn remove(&mut self, n: usize) {
        self.head -= min(self.head, n);
    }
}

impl core::fmt::Write for ByteWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let max = self.buf.len();
        for (i, &b) in self.buf[self.head .. max].iter_mut().zip(s.as_bytes().iter())
        {
            *i = b;
        }
        self.head = usize::min(max, self.head + s.as_bytes().len());
        Ok(())
    }
}
