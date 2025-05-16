// TODO: Maybe binread is a better option here
pub struct SimpleCursor<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> SimpleCursor<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub fn read_i24(&mut self) -> i32 {
        let buf = self.take_slice(3);
        i32::from_be_bytes([0, buf[0], buf[1], buf[2]])
    }

    pub fn read_i16(&mut self) -> i16 {
        let buf = self.take_slice(2);
        i16::from_be_bytes([buf[0], buf[1]])
    }

    fn take_slice(&mut self, len: usize) -> &'a [u8] {
        let ret = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        ret
    }
}

#[cfg(test)]
mod tests {
    use crate::simple_cursor::SimpleCursor;

    #[test]
    pub fn test_happy_path() {
        let data = [1, 2, 3, 4, 5];
        let mut cursor = SimpleCursor::new(&data);
        assert_eq!(cursor.read_i24(), 0x10203);
        assert_eq!(cursor.read_i16(), 0x405);
    }

    #[test]
    #[should_panic]
    pub fn test_overflow_panics() {
        let data = [0, 1, 2, 3, 4, 5];
        let mut cursor = SimpleCursor::new(&data);
        assert_eq!(cursor.read_i24(), 0);
        cursor.read_i24();
    }
}