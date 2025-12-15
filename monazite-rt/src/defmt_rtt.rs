use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::{interrupt, register};
use rtt_target::UpChannel;

static mut CHANNEL: Option<UpChannel> = None;

#[defmt::global_logger]
struct Logger;

pub fn init(channel: UpChannel) {
    unsafe { CHANNEL = Some(channel) }
}

/// Global logger lock.
static TAKEN: AtomicBool = AtomicBool::new(false);
static INTERRUPTS_ACTIVE: AtomicBool = AtomicBool::new(false);
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let primask = register::primask::read();
        interrupt::disable();

        assert!(
            !TAKEN.load(Ordering::Relaxed),
            "defmt logger taken reentrantly"
        );

        // no need for CAS because interrupts are disabled
        TAKEN.store(true, Ordering::Relaxed);

        INTERRUPTS_ACTIVE.store(primask.is_active(), Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        #[allow(static_mut_refs)]
        unsafe {
            ENCODER.start_frame(do_write);
        }
    }

    unsafe fn flush() {}

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        #[allow(static_mut_refs)]
        ENCODER.end_frame(do_write);

        TAKEN.store(false, Ordering::Relaxed);
        if INTERRUPTS_ACTIVE.load(Ordering::Relaxed) {
            // re-enable interrupts
            interrupt::enable();
        }
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        #[allow(static_mut_refs)]
        ENCODER.write(bytes, do_write);
    }
}

fn do_write(bytes: &[u8]) {
    unsafe {
        #[allow(static_mut_refs)]
        if let Some(c) = &mut CHANNEL {
            c.write(bytes);
        }
    };
}
