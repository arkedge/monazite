use core::sync::atomic::AtomicU16;

use c2a_monazite_adc_bind::{
    Error, InputChannelId, TestChannelId, INPUT_CHANNEL_NUM, TEST_CHANNEL_NUM,
};
use cortex_m::interrupt::Mutex;
use hal::{
    adc::Enabled,
    dma::{
        dma::{DmaConfig, Stream7},
        DBTransfer, PeripheralToMemory, Transfer,
    },
    hal::adc::Channel,
    pac::{ADC2, DAC, DMA1},
    rcc::ResetEnable,
};
use stm32h7xx_hal as hal;

pub const TOTAL_CHANNEL_NUM: usize = TEST_CHANNEL_NUM + INPUT_CHANNEL_NUM;

struct DacOut1;
impl DacOut1 {
    pub fn enable(self) -> Self {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.mcr.modify(|_, w| w.mode1().variant(0b11));
        dac.cr.modify(|_, w| w.en1().set_bit());

        self
    }
    #[allow(clippy::unused_self)]
    pub fn set_value(&self, val: u16) {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.dhr12r1.write(|w| w.dacc1dhr().variant(val));
    }
}
impl Channel<ADC2> for DacOut1 {
    type ID = u8;
    fn channel() -> Self::ID {
        // RM0433 spec p931, Figure 139. ADC2 connectivity
        16
    }
}

struct DacOut2;
impl DacOut2 {
    pub fn enable(self) -> Self {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.mcr.modify(|_, w| w.mode2().variant(0b11));
        dac.cr.modify(|_, w| w.en2().set_bit());

        self
    }
    #[allow(clippy::unused_self)]
    pub fn set_value(&self, val: u16) {
        let dac = unsafe { &(*DAC::ptr()) };
        dac.dhr12r2.write(|w| w.dacc2dhr().variant(val));
    }
}
impl Channel<ADC2> for DacOut2 {
    type ID = u8;
    fn channel() -> Self::ID {
        // RM0433 spec p931, Figure 139. ADC2 connectivity
        17
    }
}

type AdcToBufferTxfer<S> =
    Transfer<S, hal::adc::Adc<ADC2, Enabled>, PeripheralToMemory, &'static mut [u16], DBTransfer>;

pub struct Adc<S = Stream7<DMA1>>
where
    S: hal::dma::traits::Stream,
{
    buf: &'static [AtomicU16; TOTAL_CHANNEL_NUM],
    // if we don't store the transfer, it will be dropped and the DMA will stop
    _transfer: Mutex<AdcToBufferTxfer<S>>,
}

impl<S> Adc<S>
where
    S: hal::dma::traits::Stream<Config = DmaConfig> + hal::dma::traits::DoubleBufferedStream,
{
    pub fn new<CH1, CH2, CH3>(
        buf: &'static mut [u16; TOTAL_CHANNEL_NUM],
        adc: hal::adc::Adc<ADC2, Enabled>,
        dac: hal::rcc::rec::Dac12,
        (ch1, ch2, ch3): (CH1, CH2, CH3),
        dma_stream: S,
    ) -> Self
    where
        CH1: Channel<ADC2, ID = u8>,
        CH2: Channel<ADC2, ID = u8>,
        CH3: Channel<ADC2, ID = u8>,
    {
        const TWELVE_BIT_MAX: u16 = 0xFFF;

        let dma_config = DmaConfig::default()
            .memory_increment(true)
            .circular_buffer(true);
        let dma_buf = unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr(), buf.len()) };
        let mut transfer: Transfer<_, _, _, _, _> =
            Transfer::init(dma_stream, adc, dma_buf, None, dma_config);

        // Enable DAC clocks and reset
        let _ = dac.enable().reset();
        // Enable DAC without buffer
        let dacout1 = DacOut1.enable();
        let dacout2 = DacOut2.enable();
        dacout1.set_value(TWELVE_BIT_MAX / 2);
        dacout2.set_value(TWELVE_BIT_MAX);

        transfer.start(|adc| {
            let reg = adc.inner_mut();
            let mut adc = DmaAdc::new(reg);
            adc.unshift_channel(ch3);
            adc.unshift_channel(ch2);
            adc.unshift_channel(ch1);
            adc.unshift_channel(dacout2);
            adc.unshift_channel(dacout1);
            adc.start();
        });

        let buf = unsafe { &*buf.as_mut_ptr().cast::<[AtomicU16; TOTAL_CHANNEL_NUM]>() };
        Self {
            buf,
            _transfer: Mutex::new(transfer),
        }
    }

    pub fn read_ch(&self, ch: u8) -> u16 {
        self.buf[TEST_CHANNEL_NUM + ch as usize].load(core::sync::atomic::Ordering::Relaxed)
    }

    pub fn read_test_ch(&self, ch: u8) -> u16 {
        self.buf[ch as usize].load(core::sync::atomic::Ordering::Relaxed)
    }
}

struct DmaAdc<'a> {
    reg: &'a mut ADC2,
    count: u8,
}

impl<'a> DmaAdc<'a> {
    fn new(reg: &'a mut ADC2) -> Self {
        reg.cfgr
            .modify(|_, w| unsafe { w.res().bits(hal::adc::Resolution::TwelveBitV.into()) });
        reg.cfgr.modify(|_, w| w.dmngt().dma_circular());
        reg.cfgr
            .modify(|_, w| w.cont().set_bit().discen().clear_bit());
        reg.cfgr.modify(|_, w| w.ovrmod().overwrite());
        reg.cfgr2.modify(|_, w| w.lshift().variant(0));

        Self { reg, count: 0 }
    }

    fn start(self) -> &'a mut ADC2 {
        let reg = &self.reg;
        reg.sqr1.modify(|_, w| w.l().variant(self.count - 1));
        reg.cr.modify(|_, w| w.adstart().set_bit());
        self.reg
    }

    fn unshift_channel<P: Channel<ADC2, ID = u8>>(&mut self, _: P) {
        let reg = &self.reg;
        let chan = P::channel();
        reg.pcsel
            .modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() | (1 << chan)) });
        let sqr1_high = reg.sqr1.read().sq4().bits();
        let sqr2_high = reg.sqr2.read().sq8().bits();
        let sqr3_high = reg.sqr3.read().sq13().bits();
        reg.sqr1
            .modify(|r, w| unsafe { w.bits((r.bits() | u32::from(chan)) << 6) });
        reg.sqr2
            .modify(|r, w| unsafe { w.bits((r.bits() << 6) | u32::from(sqr1_high)) });
        reg.sqr3
            .modify(|r, w| unsafe { w.bits((r.bits() << 6) | u32::from(sqr2_high)) });
        reg.sqr4
            .modify(|r, w| unsafe { w.bits((r.bits() << 6) | u32::from(sqr3_high)) });

        self.set_chan_smp(chan, 0b111);
        self.count += 1;
    }

    fn set_chan_smp(&mut self, chan: u8, t: u8) {
        let reg = &self.reg;
        if chan <= 9 {
            reg.smpr1.modify(|_, w| match chan {
                0 => w.smp0().bits(t),
                1 => w.smp1().bits(t),
                2 => w.smp2().bits(t),
                3 => w.smp3().bits(t),
                4 => w.smp4().bits(t),
                5 => w.smp5().bits(t),
                6 => w.smp6().bits(t),
                7 => w.smp7().bits(t),
                8 => w.smp8().bits(t),
                9 => w.smp9().bits(t),
                _ => unreachable!(),
            });
        } else {
            reg.smpr2.modify(|_, w| match chan {
                10 => w.smp10().bits(t),
                11 => w.smp11().bits(t),
                12 => w.smp12().bits(t),
                13 => w.smp13().bits(t),
                14 => w.smp14().bits(t),
                15 => w.smp15().bits(t),
                16 => w.smp16().bits(t),
                17 => w.smp17().bits(t),
                18 => w.smp18().bits(t),
                19 => w.smp19().bits(t),
                _ => unreachable!(),
            });
        }
    }
}

impl<S> c2a_monazite_adc_bind::Adc for Adc<S>
where
    S: hal::dma::traits::Stream<Config = DmaConfig> + hal::dma::traits::DoubleBufferedStream + Send,
{
    fn initialize(&self) -> Result<(), Error> {
        Ok(())
    }

    fn get_value(&self, ch: InputChannelId) -> u16 {
        self.read_ch(ch.into())
    }

    fn get_test_value(&self, ch: TestChannelId) -> u16 {
        self.read_test_ch(ch.into())
    }
}
