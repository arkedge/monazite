#![no_std]

use core::sync::atomic::{AtomicPtr, Ordering};

pub struct AtomicOnceCell<T> {
    ptr: AtomicPtr<T>,
}

impl<T> AtomicOnceCell<T> {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ptr: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn try_get(&self) -> Option<&T> {
        let ptr = self.ptr.load(Ordering::SeqCst);
        if ptr.is_null() {
            None
        } else {
            Some(unsafe { &*ptr })
        }
    }

    /// # Panics
    /// Panics if the cell is not initialized.
    #[inline]
    pub fn get(&self) -> &T {
        let ptr = self.ptr.load(Ordering::SeqCst);
        assert!(
            !ptr.is_null(),
            "{} is not initialized",
            core::any::type_name::<T>()
        );
        unsafe { &*ptr }
    }

    pub fn set<'a>(&'a self, new: &'a T) -> &'a T {
        self.ptr
            .store(core::ptr::from_ref::<T>(new).cast_mut(), Ordering::SeqCst);
        new
    }
}

impl<T> Default for AtomicOnceCell<T> {
    fn default() -> Self {
        Self::new()
    }
}
