#![cfg_attr(not(test), no_std)]

pub struct RingBuf<'a> {
    buffer: &'a mut [u8],
    front: usize,
    back: usize,
    full: bool,
}

impl<'a> RingBuf<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            buffer,
            front: 0,
            back: 0,
            full: false,
        }
    }

    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.buffer.len()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        if self.full {
            self.capacity()
        } else if self.back < self.front {
            self.back + self.capacity() - self.front
        } else {
            self.back - self.front
        }
    }

    #[must_use]
    pub fn available(&self) -> usize {
        self.capacity() - self.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.front == self.back && !self.full
    }

    #[must_use]
    pub fn is_full(&self) -> bool {
        self.full
    }

    pub fn clear(&mut self) {
        self.front = 0;
        self.back = 0;
        self.full = false;
    }

    #[must_use]
    pub fn readable(&self) -> (&[u8], &[u8]) {
        if self.is_empty() {
            (&self.buffer[..0], &self.buffer[..0])
        } else if self.front < self.back {
            (&self.buffer[self.front..self.back], &self.buffer[..0])
        } else {
            (&self.buffer[self.front..], &self.buffer[..self.back])
        }
    }

    pub fn complete_read(&mut self, len: usize) {
        debug_assert!(len <= self.len());
        self.front += len;
        if self.front >= self.capacity() {
            self.front -= self.capacity();
        }
        if len > 0 {
            self.full = false;
        }
    }

    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        let (first, second) = self.readable();
        if second.is_empty() {
            let len = first.len().min(buf.len());
            buf[..len].copy_from_slice(&first[..len]);
            len
        } else {
            let first_len = first.len().min(buf.len());
            buf[..first_len].copy_from_slice(&first[..first_len]);
            if first_len < first.len() {
                first_len
            } else {
                let second_len = second.len().min(buf.len() - first_len);
                buf[first_len..][..second_len].copy_from_slice(&second[..second_len]);
                first_len + second_len
            }
        }
    }

    pub fn writable(&mut self) -> (&mut [u8], &mut [u8]) {
        if self.is_empty() {
            let (second, first) = self.buffer.split_at_mut(self.back);
            (first, second)
        } else if self.front < self.back {
            let (before_front, tail) = self.buffer.split_at_mut(self.front);
            let (_readable, after_back) = tail.split_at_mut(self.back - self.front);
            (after_back, before_front)
        } else {
            (&mut self.buffer[self.back..self.front], &mut [])
        }
    }

    pub fn complete_write(&mut self, len: usize) {
        debug_assert!(len <= self.available());
        self.back += len;
        if self.back >= self.capacity() {
            self.back -= self.capacity();
        }
        if len > 0 {
            self.full = self.front == self.back;
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> usize {
        let (first, second) = self.writable();
        if second.is_empty() {
            let len = first.len().min(buf.len());
            first[..len].copy_from_slice(&buf[..len]);
            len
        } else {
            let first_len = first.len().min(buf.len());
            first[..first_len].copy_from_slice(&buf[..first_len]);
            if first_len < first.len() {
                first_len
            } else {
                let second_len = second.len().min(buf.len() - first_len);
                second[..second_len].copy_from_slice(&buf[first_len..][..second_len]);
                first_len + second_len
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        let mut buffer = [0; 10];
        let mut ring = RingBuf::new(&mut buffer);
        assert!(ring.is_empty());
        assert!(!ring.is_full());
        assert_eq!(ring.len(), 0);
        assert_eq!(ring.available(), 10);
        assert_eq!(ring.capacity(), 10);

        let (f, s) = ring.writable();
        assert_eq!(f.len(), 10);
        assert_eq!(s.len(), 0);
        f[..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF][..]);
        ring.complete_write(4);
        assert_eq!(ring.len(), 4);

        let (f, s) = ring.readable();
        assert_eq!(f.len(), 4);
        assert_eq!(s.len(), 0);
        assert_eq!(f, [0xDE, 0xAD, 0xBE, 0xEF]);
        ring.complete_read(4);
        assert!(ring.is_empty());

        let (f, s) = ring.writable();
        assert_eq!(f.len(), 6);
        assert_eq!(s.len(), 4);
        f[..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF][..]);
        ring.complete_write(4);
        assert_eq!(ring.len(), 4);

        ring.clear();
        assert_eq!(ring.len(), 0);
    }
}
