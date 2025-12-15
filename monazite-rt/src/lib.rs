#![no_std]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::inline_always)]

mod adc;
mod btmgr;
mod ccsds;
mod gpio;
mod iflash;
mod thermometer;
mod wdt;

mod defmt_rtt;
mod perf;
mod ramecc;
mod resources;
mod uart;
#[macro_use]
mod macros;
mod pins;

use core::mem::MaybeUninit;

use hal::{
    dma::{ConstDBTransfer, DBTransfer, PeripheralToMemory},
    pac,
};
use panic_probe as _; // panic handler
use rtt_target as _;
use rtt_target::rprintln;

use c2a_monazite_adc_bind::{Adc as AdcBind, C2A_MONAZITE_ADC};
use c2a_monazite_btmgr_bind::{Btmgr as BtmgrBind, C2A_MONAZITE_BTMGR};
use c2a_monazite_ccsds_bind::{Ccsds as CcsdsBind, C2A_MONAZITE_CCSDS};
use c2a_monazite_gpio_bind::{Gpio as GpioBind, C2A_MONAZITE_GPIO};
use c2a_monazite_iflash_bind::{Iflash as IflashBind, C2A_MONAZITE_IFLASH};
use c2a_monazite_ramecc_bind::{Ramecc as RameccBind, C2A_MONAZITE_RAMECC};
use c2a_monazite_thermometer_bind::{Thermometer as ThermometerBind, C2A_MONAZITE_THERMOMETER};
use c2a_monazite_uart_bind::{Uart as UartBind, C2A_MONAZITE_UART};
use c2a_monazite_wdt_bind::{Wdt as WdtBind, C2A_MONAZITE_WDT};
use cortex_m::{
    peripheral::{syst::SystClkSource, DWT},
    singleton,
};
use hal::dma::{dma::DmaConfig, MemoryToPeripheral, Transfer};
use hal::gpio::{Input, Output};
use hal::prelude::*;
use hal::rcc::rec::AdcClkSel;
use stm32h7xx_hal as hal;

use bootmeta::{BootMeta, FlashOptionBytes};
use uart::DirectUartArray;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use super::{
        enable_cache, iflash, init_adc, init_btmgr, init_c2a, init_ccsds, init_dbgmcu, init_gpio,
        init_iflash, init_perf_counter, init_ramecc, init_rtt, init_thermometer, init_uart,
        init_wdt, perf, resources, DirectUartArray, SystClkSource,
    };

    #[shared]
    struct Shared {
        direct_uarts: &'static DirectUartArray,
        #[lock_free]
        ram_scrubber: crate::ramecc::RamScrubber,
        #[lock_free]
        perf_count: perf::Counter,
    }
    #[local]
    struct Local {
        iflash: &'static iflash::Iflash,
        perf_win: perf::Window,
    }

    // clippy's bug???
    #[allow(clippy::mut_mut)]
    #[allow(clippy::cast_possible_truncation)]
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        init_rtt();

        defmt::println!("starting...");

        let mut cp = ctx.core; // Arm Core Peripherals
        let dp = ctx.device; // Device(STM32) Peripherals

        enable_cache(&cp.MPU, &mut cp.SCB, &mut cp.CPUID);

        let (perf_win, perf_count) = init_perf_counter(&mut cp.DCB, &mut cp.DWT);

        let mut res = resources::Resources::new(dp);

        init_wdt(res.wdt);

        let ram_scrubber = init_ramecc(res.ramecc);

        init_btmgr(res.btmgr);

        let iflash = init_iflash(res.iflash);

        init_gpio(res.gpio);

        init_ccsds(&mut res.shared);

        let direct_uarts = init_uart(res.direct_uart, &mut res.shared);

        init_dbgmcu(res.dbgmcu);

        init_adc(res.adc, &mut res.shared);

        init_thermometer(res.thermometer, &mut res.shared);

        cp.SYST.disable_interrupt();
        cp.SYST.disable_counter();
        cp.SYST.set_clock_source(SystClkSource::Core);
        cp.SYST
            .set_reload(res.shared.clocks.sys_ck().to_MHz() * (cp.SYST.calib.read() & 0xFFFF));
        cp.SYST.enable_interrupt();
        cp.SYST.enable_counter();

        defmt::println!("started.");

        (
            Shared {
                direct_uarts,
                ram_scrubber,
                perf_count,
            },
            Local { iflash, perf_win },
        )
    }

    #[idle(shared = [])]
    fn idle(_ctx: idle::Context) -> ! {
        init_c2a();
        loop {
            unsafe {
                c2a_core::C2A_core_main();
            }
        }
    }

    #[task(binds = SysTick, local = [perf_win], shared = [ram_scrubber, perf_count])]
    fn sys_tick(ctx: sys_tick::Context) {
        unsafe {
            c2a_core::system::time_manager::TMGR_count_up_master_clock();
        }

        ctx.shared.ram_scrubber.tick();

        let perf_win = ctx.local.perf_win;
        perf_win.tick(ctx.shared.perf_count);
        if perf_win.ticks >= 1000 {
            defmt::println!(
                "interrupt cycles: {} cycles / {} ticks (max. {} cycles/tick)",
                perf_win.sum,
                perf_win.ticks,
                perf_win.max
            );
            perf_win.reset();
        }
    }

    #[task(binds = DMA1_STR0, shared = [direct_uarts, perf_count])]
    fn txfer0_dma(mut ctx: txfer0_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[0].0.complete_read();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA2_STR0, shared = [direct_uarts, perf_count])]
    fn rxfer0_dma(mut ctx: rxfer0_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[0].1.complete_write();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA1_STR1, shared = [direct_uarts, perf_count])]
    fn txfer1_dma(mut ctx: txfer1_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[1].0.complete_read();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA2_STR1, shared = [direct_uarts, perf_count])]
    fn rxfer1_dma(mut ctx: rxfer1_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[1].1.complete_write();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA1_STR2, shared = [direct_uarts, perf_count])]
    fn txfer2_dma(mut ctx: txfer2_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[2].0.complete_read();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA2_STR2, shared = [direct_uarts, perf_count])]
    fn rxfer2_dma(mut ctx: rxfer2_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[2].1.complete_write();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA1_STR3, shared = [direct_uarts, perf_count])]
    fn txfer3_dma(mut ctx: txfer3_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[3].0.complete_read();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA2_STR3, shared = [direct_uarts, perf_count])]
    fn rxfer3_dma(mut ctx: rxfer3_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[3].1.complete_write();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA1_STR4, shared = [direct_uarts, perf_count])]
    fn txfer4_dma(mut ctx: txfer4_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[4].0.complete_read();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA2_STR4, shared = [direct_uarts, perf_count])]
    fn rxfer4_dma(mut ctx: rxfer4_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[4].1.complete_write();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA1_STR5, shared = [direct_uarts, perf_count])]
    fn txfer5_dma(mut ctx: txfer5_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[5].0.complete_read();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = DMA2_STR5, shared = [direct_uarts, perf_count])]
    fn rxfer5_dma(mut ctx: rxfer5_dma::Context) {
        ctx.shared.perf_count.begin();
        ctx.shared.direct_uarts.lock(|direct_uarts| {
            direct_uarts[5].1.complete_write();
        });
        ctx.shared.perf_count.end();
    }

    #[task(binds = FLASH, shared = [perf_count], local = [iflash])]
    fn flash(ctx: flash::Context) {
        ctx.shared.perf_count.begin();
        ctx.local.iflash.poll();
        ctx.shared.perf_count.end();
    }

    #[task(binds = RAMECC, shared = [ram_scrubber])]
    fn ramecc(ctx: ramecc::Context) {
        ctx.shared.ram_scrubber.handling_interrupt();
    }
}

/// キャッシュを有効化する
///
/// この関数は、SRAM1,2以外のSRAM領域に対してキャッシュを有効化します。
/// SRAM1,2はDMAバッファとして使用されるため、キャッシュを無効化します。
fn enable_cache(mpu: &pac::MPU, scb: &mut pac::SCB, cpuid: &mut pac::CPUID) {
    // Configure MPU to disable CPU cache on SRAM1,2 which is used for DMA buffers
    configure_mpu(mpu);
    // Enable caches on all other SRAM regions except SRAM1,2
    scb.enable_icache();
    scb.enable_dcache(cpuid);
}

/// キャッシュを無効化する範囲を MPU のレジスタ経由で指定．
///
/// [https://www.st.com/resource/ja/programming_manual/pm0253-stm32f7-series-and-stm32h7-series-cortexm7-processor-programming-manual-stmicroelectronics.pdf](https://www.st.com/resource/ja/programming_manual/pm0253-stm32f7-series-and-stm32h7-series-cortexm7-processor-programming-manual-stmicroelectronics.pdf)
fn configure_mpu(mpu: &pac::MPU) {
    unsafe {
        mpu.ctrl.write(0); // Disable MPU
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // Configure internal SRAM1 and SRAM2 as non-cacheable
        {
            mpu.rnr.write(0); // Use region 0 for internal SRAM1 settings
            mpu.rbar.write(
                0x3000_0000, // internal SRAM1 base address
            );
            let rasr
            = 0b11 << 24 // AP = Full Access
            | 0b1 << 18 // S = Sharable
            | 0b1 << 16 // B = Bufferable
            | 17 << 1 // SIZE = 128KiB (SRAM1) + 128KiB (SRAM2)
            | 0b1 << 0// Enable
            ;
            mpu.rasr.write(rasr);
        }

        let ctrl
        = 0b1 << 2 // PRIVDEFENA = Enable privileged access to default memory map
        | 0b1 << 0 // ENABLE     = Enable MPU
        ;
        mpu.ctrl.write(ctrl);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

fn init_rtt() {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 512
                mode: NoBlockSkip
                name: "C2A"
            }
            1: {
                size: 512
                mode: NoBlockSkip
                name: "defmt"
            }
        }
    };
    defmt_rtt::init(channels.up.1);
}

fn init_perf_counter(dcb: &mut pac::DCB, dwt: &mut pac::DWT) -> (perf::Window, perf::Counter) {
    dcb.enable_trace();
    dwt.enable_cycle_counter();
    assert!(DWT::cycle_counter_enabled());

    let perf_win = perf::Window::default();
    let perf_count = perf::Counter::default();
    (perf_win, perf_count)
}

fn init_power_reset_and_clocks(
    syscfg: &pac::SYSCFG,
    pwr: pac::PWR,
    rcc: pac::RCC,
) -> hal::rcc::Ccdr {
    fn enable_sram123(rcc: &hal::pac::RCC) {
        rcc.ahb2enr.modify(|_, w| {
            w.sram1en()
                .set_bit()
                .sram2en()
                .set_bit()
                .sram3en()
                .set_bit()
        });
    }

    fn enable_vrefbuf(rcc: &hal::pac::RCC) {
        rcc.apb4enr.modify(|_, w| w.vrefen().set_bit());
    }

    let pwr = pwr.constrain();
    let pwrcfg = pwr.freeze();

    enable_sram123(&rcc);
    enable_vrefbuf(&rcc);

    let rcc = rcc.constrain();
    let mut ccdr = rcc.sys_ck(400.MHz()).hclk(100.MHz()).freeze(pwrcfg, syscfg);
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);
    ccdr
}

fn init_wdt(res: resources::Wdt) {
    let wdt = wdt::Wdt::new(res.iwdg);
    let wdt = singleton!(: wdt::Wdt = wdt).unwrap();
    let wdt = singleton!(: &dyn WdtBind = wdt).unwrap();
    C2A_MONAZITE_WDT.set(wdt);
}

fn init_ramecc(res: resources::Ramecc) -> ramecc::RamScrubber {
    #[link_section = ".sram1.eccbuf"]
    static mut ECC_DST_BUFFER: MaybeUninit<[u32; 1]> = MaybeUninit::uninit();
    // This buf is not referenced by multiple sources simultaneously.
    #[allow(static_mut_refs)]
    let ecc_dst_buf = unsafe { ECC_DST_BUFFER.write([0; 1]) };

    let ecc_stats = singleton!(: ramecc::EccStats = ramecc::EccStats::default()).unwrap();
    let ecc_stats = &*ecc_stats;
    let ramecc = singleton!(: &dyn RameccBind = ecc_stats).unwrap();
    C2A_MONAZITE_RAMECC.set(ramecc);

    ramecc::RamScrubber::new(
        ecc_dst_buf,
        ecc_stats,
        (res.ramecc1, res.ramecc2, res.ramecc3),
        res.mdma_s0,
    )
}

fn init_btmgr(res: resources::Btmgr) {
    let bootmeta = BootMeta::new(res.rtc);
    let flash_option_bytes = unsafe { FlashOptionBytes::new() };
    let btmgr = btmgr::Btmgr::new(bootmeta, flash_option_bytes);
    let btmgr = singleton!(: btmgr::Btmgr = btmgr).unwrap();
    let btmgr = singleton!(: &dyn BtmgrBind = btmgr).unwrap();
    C2A_MONAZITE_BTMGR.set(btmgr);
}

#[allow(clippy::similar_names)]
fn init_iflash(res: resources::Iflash) -> &'static iflash::Iflash {
    #[no_mangle]
    #[link_section = ".sram1.iflashbuf"]
    static mut IFLASH_BUF: MaybeUninit<[u8; 128]> = MaybeUninit::uninit();
    // singleton! guarantees that only one instance can reference at a time.
    #[allow(static_mut_refs)]
    let iflash_buf = unsafe { IFLASH_BUF.write([0; 128]) };

    let iflash = iflash::Iflash::new(res.flash, iflash_buf);
    let iflash = singleton!(: iflash::Iflash = iflash).unwrap();
    let iflash = &*iflash;
    {
        let iflash = singleton!(: &dyn IflashBind = iflash).unwrap();
        C2A_MONAZITE_IFLASH.set(iflash);
    }
    iflash
}

fn init_gpio(
    resources::Gpio {
        output_pins,
        input_pins,
    }: resources::Gpio,
) {
    let output_ports = {
        let output_ports = seq_macro::seq!(N in 0..9 {[
            #(output_pins.N.into_push_pull_output().erase(),)*
        ]});
        singleton!(: [hal::gpio::ErasedPin<Output>; 9] = output_ports).unwrap()
    };
    let input_ports = {
        let input_ports = seq_macro::seq!(N in 0..6 {[
            #(input_pins.N.into_input().erase(),)*
        ]});
        singleton!(: [hal::gpio::ErasedPin<Input>; 6] = input_ports).unwrap()
    };

    let gpio = gpio::Gpio::new(output_ports, input_ports);
    let gpio = singleton!(: gpio::Gpio = gpio).unwrap();
    let gpio = singleton!(: &dyn GpioBind = gpio).unwrap();
    C2A_MONAZITE_GPIO.set(gpio);
}

fn init_adc(res: resources::Adc, shared: &mut resources::Shared) {
    #[link_section = ".sram1.adcbuf"]
    static mut ADC_BUFFER: MaybeUninit<[u16; crate::adc::TOTAL_CHANNEL_NUM]> =
        MaybeUninit::uninit();
    // singleton! guarantees that only one instance can reference at a time.
    #[allow(static_mut_refs)]
    let adc_buf = unsafe { ADC_BUFFER.write([0; crate::adc::TOTAL_CHANNEL_NUM]) };
    let adc2 = hal::adc::Adc::adc2(
        res.adc2,
        4.MHz(),
        &mut shared.delay,
        res.ccdrp_adc12,
        &shared.clocks,
    );
    let adc2 = adc2.enable();
    let adc_ch1 = res.pins.agc_inc;
    let adc_ch2 = res.pins.agc_coh;
    let adc_ch3 = res.pins.freq_err;
    let adc = adc::Adc::new(
        adc_buf,
        adc2,
        res.ccdrp_dac12,
        (adc_ch1, adc_ch2, adc_ch3),
        res.dma1s7,
    );
    let adc = singleton!(: adc::Adc = adc).unwrap();
    let adc = singleton!(: &dyn AdcBind = adc).unwrap();
    C2A_MONAZITE_ADC.set(adc);
}

fn init_thermometer(res: resources::Thermometer, shared: &mut resources::Shared) {
    // adc3 for thermometer
    #[link_section = ".sram1.thermobuf"]
    static mut THERMO_BUFFER: MaybeUninit<[u16; 1]> = MaybeUninit::uninit();
    // singleton! guarantees that only one instance can reference at a time.
    #[allow(static_mut_refs)]
    let buf = unsafe { THERMO_BUFFER.write([0; 1]) };
    let adc3 = hal::adc::Adc::adc3(
        res.adc3,
        4.MHz(),
        &mut shared.delay,
        res.ccdrp_adc3,
        &shared.clocks,
    );
    let thermometer = thermometer::Thermometer::new(buf, adc3, res.dma2s7);
    let thermometer = singleton!(: thermometer::Thermometer = thermometer).unwrap();
    let thermometer = singleton!(: &dyn ThermometerBind = thermometer).unwrap();
    C2A_MONAZITE_THERMOMETER.set(thermometer);
}

#[allow(clippy::similar_names)]
fn init_uart(
    direct: resources::DirectUart,
    shared: &mut resources::Shared,
) -> &'static DirectUartArray {
    let direct_uarts = {
        // DIRECT_UART_NUM is too long name, so use CH_NUM instead.
        use uart::DIRECT_UART_NUM as CH_NUM;
        type TxBufMem = MaybeUninit<[u8; 4096]>;
        type RxBufMem = MaybeUninit<[u8; 8192]>; // UARTの受信バッファサイズに合わせる

        #[link_section = ".sram1.uartbuf"]
        static mut DIRECT_UART_TX_BUFFERS: [TxBufMem; CH_NUM] = [TxBufMem::uninit(); CH_NUM];
        #[link_section = ".sram2.uartbuf"]
        static mut DIRECT_UART_RX_BUFFERS: [RxBufMem; CH_NUM] = [RxBufMem::uninit(); CH_NUM];

        let direct_uarts = direct_uart!(direct, shared,
            DIRECT_UART_TX_BUFFERS, DIRECT_UART_RX_BUFFERS,
            {
                0: USART1,
                1: USART2,
                2: USART3,
                3: UART4,
                4: UART5,
                5: UART7,
            }
        );
        singleton!(: DirectUartArray = direct_uarts).unwrap()
    };

    let uart = uart::Uart::new(direct_uarts);
    let uart = singleton!(: uart::Uart = uart).unwrap();
    let uart = singleton!(: &dyn UartBind = uart).unwrap();
    C2A_MONAZITE_UART.set(uart);
    direct_uarts
}

fn init_ccsds(_shared: &mut resources::Shared) {
    let ccsds = ccsds::Ccsds::new();
    let ccsds = singleton!(: ccsds::Ccsds = ccsds).unwrap();
    let ccsds = singleton!(: &dyn CcsdsBind = ccsds).unwrap();
    C2A_MONAZITE_CCSDS.set(ccsds);
}

#[allow(clippy::needless_pass_by_value)]
fn init_dbgmcu(res: resources::Dbgmcu) {
    res.dbgmcu.apb4fz1.write(|w| {
        w.dbg_wdglsd1().set_bit(); // disable WDT in debug mode
        w
    });
}

fn init_c2a() {
    unsafe {
        c2a_core::system::watchdog_timer::WDT_init();
        c2a_core::system::time_manager::TMGR_init(); // Time Manager
    }

    unsafe {
        c2a_core::C2A_core_init();
    }

    // TaskDispatcherでの大量のアノマリを避けるために、一度時刻を初期化する。
    unsafe {
        c2a_core::system::time_manager::TMGR_clear();
    }
    rprintln!("C2A_init: TMGR_init done.");

    // FIXME: BTMGR_set_next_boot_bank(0) の呼び出しタイミングは user 側で定義させたい
    c2a_monazite_btmgr_bind::BTMGR_set_next_boot_bank(0);
}
