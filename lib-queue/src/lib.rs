// #![no_std]

use core::fmt;
use core::iter::Iterator;
use core::iter::ExactSizeIterator;

pub struct Queue<T: Sized, const SIZE: usize> {
    pub(crate) data: [Option<T>; SIZE],
    
    /// Amount of items currently in the queue.
    pub(crate) size: usize,

    /// Index of the oldest item in the queue (i.e. the next item to be taken from the queue)
    pub(crate) bottom: usize,
}

impl<T: Sized, const SIZE: usize> Queue<T, SIZE> {
    const NONE_T: Option<T> = None;

    pub const fn new() -> Queue<T, SIZE> {
        Queue {
            data: [Self::NONE_T; SIZE],
            size: 0,
            bottom: 0,
        }
    }

    /// Pushes an element to the top of the queue if there is space available. Note that if the 
    /// queue is full already the item will simply be dropped and lost.
    /// 
    pub fn push(&mut self, item: T) {
        self.try_push(item);
    }

    /// Tries to push an element to the top of the queue and returns `None` if it was successful or
    /// returns the item again if there was no space left on the queue. 
    /// 
    pub fn try_push(&mut self, item: T) -> Option<T> {
        if self.size < SIZE {
            self.data[(self.bottom + self.size) % SIZE] = Some(item);
            self.size += 1;
            None
        } else {
            Some(item)
        }
    }

    /// Removes the top element (i.e. the last one pushed) from the queue or returns None if the
    /// queue was empty. Note that this is essentially using the queue like a stack, to get the 
    /// oldest item in the queue you want to use `next()`.
    /// 
    pub fn pop(&mut self) -> Option<T> {
        if self.size > 0 {
            let item = self.data[(self.bottom + self.size - 1) % SIZE].take();
            self.size = (self.size + SIZE - 1) % SIZE;
            return item;
        }
        None
    }
}

impl<T: Sized, const SIZE: usize> Default for Queue<T, SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Sized + core::fmt::Debug, const SIZE: usize> fmt::Debug for Queue<T, SIZE> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // write!(f, "Queue [size: {}]", self.size)
        write!(f, "Queue {:?}", self.data)
    }
}

impl<T: Sized, const SIZE: usize> Iterator for Queue< T, SIZE> {
    type Item = T;

    /// Removes the oldest element from the queue or returns None if the queue is empty.
    /// 
    fn next(&mut self) -> Option<T> {
        if self.size > 0 {
            let item = self.data[self.bottom].take();
            self.bottom = (self.bottom + 1) % SIZE;
            self.size -= 1; 
            return item;
        }
        None
    }
}

impl<T: Sized, const SIZE: usize> ExactSizeIterator for Queue<T, SIZE> {
    fn len(&self) -> usize {
        self.size
    }
}

#[cfg(test)]
mod tests 
{
    use super::*;

    #[test]
    pub fn test_try_push() {
        let mut queue = Queue::<u32, 15>::new();
        for i in 0..15 {
            let result = queue.try_push(i);
            assert_eq!(result, None);
            assert_eq!(queue.len(), (i+1) as usize);
        }

        assert_eq!(queue.len(), 15);

        // Fails on a full queue
        let result = queue.try_push(12345);
        assert_eq!(result, Some(12345));

        assert_eq!(queue.len(), 15);

        for i in 0..15 {
            let item = queue.next();
            assert_eq!(Some(i), item);
            assert_eq!(queue.len(), 14 - i as usize);
        }
    }


    #[test]
    pub fn test_queue_empty_after_take() {
        let number = 42;
        let mut queue = Queue::<u32, 10>::new();
        queue.push(number);
        let item = queue.next();
        assert_eq!(Some(number), item);
        assert_eq!(queue.len(), 0);
    }

    #[test]
    pub fn test_queue_empty_after_pop() {
        let number = 666;
        let mut queue = Queue::<u32, 10>::new();
        queue.push(number);
        assert_eq!(queue.len(), 1);
        let item = queue.pop();
        assert_eq!(Some(number), item);
        assert_eq!(queue.len(), 0);
    }

    #[test]
    pub fn test_take_10_items() {
        let mut queue = Queue::<u32, 12>::new();

        for i in 0..10 {
            queue.push(i);
            assert_eq!(queue.len(), (i+1) as usize);
        }

        for i in 0..10 {
            let item = queue.next();
            assert_eq!(Some(i), item);
            assert_eq!(queue.len(), 9 - i as usize);
        }
    }

    #[test]
    pub fn test_take_20_items() {
        let mut queue = Queue::<u32, 15>::new();

        for i in 0..10 {
            queue.push(i);
            assert_eq!(queue.len(), (i+1) as usize);
        }

        assert_eq!(queue.len(), 10);

        for i in 0..10 {
            let item = queue.next();
            assert_eq!(Some(i), item);
            assert_eq!(queue.len(), 9 - i as usize);
        }

        assert_eq!(queue.len(), 0);

        for i in 10..20 {
            queue.push(i);
            println!("{:?}", queue);
            assert_eq!(queue.len(), (i-9) as usize);
        }

        assert_eq!(queue.len(), 10);

        for i in 10..20 {
            let item = queue.next();
            assert_eq!(item, Some(i));
            assert_eq!(queue.len(), (9 - (i - 10)) as usize);
        }

        assert_eq!(queue.len(), 0);
    }

    
    #[test]
    pub fn test_pop_10_items() {
        let mut queue = Queue::<u32, 12>::new();

        for i in 0..10 {
            queue.push(i);
            assert_eq!(queue.len(), (i+1) as usize);
        }

        for i in (0..10).rev() {
            let item = queue.pop();
            assert_eq!(Some(i), item);
            assert_eq!(queue.len(), i as usize);
        }
    }

    #[test]
    pub fn test_take_and_pop_20_items() {
        let mut queue = Queue::<u32, 20>::new();
        queue.bottom = 6;

        for i in 0..20 {
            queue.push(i);
            assert_eq!(queue.len(), (i+1) as usize);
        }

        assert_eq!(queue.len(), 20);

        for i in 0..10 {
            
            let popped_item = queue.pop();
            assert_eq!(Some(19-i), popped_item);

            let next_item = queue.next();
            assert_eq!(Some(i), next_item);

            assert_eq!(queue.len(), (20 - (i+1)*2) as usize);
        }
    }
}
