//! temperature sensor driver
use core::sync::atomic::AtomicU16;

use c2a_monazite_thermometer_bind::Thermometer as ThermometerBind;
use cortex_m::interrupt::Mutex;
use hal::{
    adc::{Disabled, Enabled},
    dma::{
        dma::{DmaConfig, Stream7},
        DBTransfer, PeripheralToMemory, Transfer,
    },
    pac::{ADC3, ADC3_COMMON, DMA2},
    signature::{TS_CAL_110, TS_CAL_30, VDDA_CALIB},
};
use stm32h7xx_hal as hal;

pub const CHANNELS: usize = 1;

type AdcToBufferTxfer<S> =
    Transfer<S, hal::adc::Adc<ADC3, Enabled>, PeripheralToMemory, &'static mut [u16], DBTransfer>;

pub struct Thermometer<S = Stream7<DMA2>>
where
    S: hal::dma::traits::Stream,
{
    buf: &'static [AtomicU16; CHANNELS],
    // if we don't store the transfer, it will be dropped and the DMA will stop
    _transfer: Mutex<AdcToBufferTxfer<S>>,
}

impl<S> Thermometer<S>
where
    S: hal::dma::traits::Stream<Config = DmaConfig> + hal::dma::traits::DoubleBufferedStream,
{
    pub fn new(
        buf: &'static mut [u16; CHANNELS],
        adc: hal::adc::Adc<ADC3, Disabled>,
        dma_stream: S,
    ) -> Self {
        let mut channel = hal::adc::Temperature::new();
        channel.enable(&adc);

        let mut adc = adc.enable();
        adc.set_resolution(hal::adc::Resolution::SixteenBit);

        let dma_config = DmaConfig::default()
            .memory_increment(false)
            .circular_buffer(true);
        let dma_buf = unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr(), buf.len()) };
        let mut transfer: Transfer<_, _, _, _, _> =
            Transfer::init(dma_stream, adc, dma_buf, None, dma_config);

        transfer.start(|adc| {
            // enable temperature sensor
            unsafe {
                (*ADC3_COMMON::ptr())
                    .ccr
                    .modify(|_, w| w.vsenseen().set_bit());
            }

            adc.start_conversion_dma(&mut channel, hal::adc::AdcDmaMode::Circular);
        });

        let buf = unsafe { &*buf.as_mut_ptr().cast::<[AtomicU16; CHANNELS]>() };
        Self {
            buf,
            _transfer: Mutex::new(transfer),
        }
    }

    pub fn get_raw_value(&self) -> u16 {
        self.buf[0].load(core::sync::atomic::Ordering::Relaxed)
    }

    #[allow(clippy::cast_precision_loss)]
    pub fn get_temperature(&self) -> f32 {
        const V_REFINT: f32 = 2.5; // 2.5V
        const V_REFINT_CAL: f32 = VDDA_CALIB as f32 / 1000.0; // 3.3V

        let raw = u32::from(self.get_raw_value());
        let val_ts = raw as f32 * V_REFINT / V_REFINT_CAL;
        let cal = (110.0 - 30.0) / f32::from(TS_CAL_110::read() - TS_CAL_30::read());

        // temperature
        cal * (val_ts - f32::from(TS_CAL_30::read())) + 30.0
    }
}

impl ThermometerBind for Thermometer {
    fn value(&self) -> f32 {
        self.get_temperature()
    }
}
