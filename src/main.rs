#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod board;

use embassy_rp as hal;
use embassy_rp::block::ImageDef;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;
use embassy_rp::flash;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::usb;
use embassy_rp::watchdog::{ResetReason, Watchdog};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::Builder;
use heapless::Vec;
use static_cell::StaticCell;
use core::sync::atomic::{AtomicBool, Ordering};
use smart_leds::RGB8;

use light_machine::Word;
use fluxpilot_firmware::program::default_program;
use fluxpilot_firmware::usb_io::{io_loop, PliotShared};
use fluxpilot_firmware::usb_vendor::{VendorClass, VendorReceiver, VendorSender};
use fluxpilot_firmware::flash_storage::FlashStorage;
use pliot::Pliot;

mod build_constants {
    include!(concat!(env!("OUT_DIR"), "/memory_consts.rs"));
}

use build_constants::FLASH_SIZE;


// Panic handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<peripherals::PIO0>;
});

const MAX_ARGS: usize = 3;
const MAX_RESULT: usize = 3;
const PROGRAM_BLOCK_SIZE: usize = 64;
const UI_BLOCK_SIZE: usize = 128;
const INCOMING_MESSAGE_CAP: usize = 2048;
const OUTGOING_MESSAGE_CAP: usize = 1048;
const NUM_LEDS: usize = 1024;
const FRAME_TARGET_MS: u64 = 16;
const PROGRAM_BUFFER_SIZE: usize = 1024;
const USB_RECEIVE_BUF_SIZE: usize = 2048;
const STACK_SIZE: usize = 100;
const WATCHDOG_RESET_THRESHOLD: u32 = 3;
const WATCHDOG_PERIOD_MS: u64 = 2_000;
const WATCHDOG_FEED_MS: u64 = 500;
const GLOBALS_SIZE: usize = 2048;
const WATCHDOG_SCRATCH_MAGIC: u32 = u32::from_le_bytes(*b"WDT0");

type FlashDriver = flash::Flash<'static, peripherals::FLASH, flash::Blocking, FLASH_SIZE>;
type StorageImpl = FlashStorage<FlashDriver>;
type SharedState = PliotShared<
    'static,
    'static,
    StorageImpl,
    MAX_ARGS,
    MAX_RESULT,
    PROGRAM_BLOCK_SIZE,
    UI_BLOCK_SIZE,
    STACK_SIZE,
>;

static PROGRAM_BUFFER: StaticCell<[u16; PROGRAM_BUFFER_SIZE]> = StaticCell::new();
static GLOBALS: StaticCell<[u16; GLOBALS_SIZE]> = StaticCell::new();
static FLASH_STORAGE: StaticCell<FlashStorage<FlashDriver>> = StaticCell::new();
static USB_RECEIVE_BUF: StaticCell<[u8; USB_RECEIVE_BUF_SIZE]> = StaticCell::new();
static RAW_MESSAGE_BUFF: StaticCell<Vec<u8, INCOMING_MESSAGE_CAP>> = StaticCell::new();
static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 64]> = StaticCell::new();
static USB_BOS_DESCRIPTOR: StaticCell<[u8; 64]> = StaticCell::new();
static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static LED_BUFFER: StaticCell<[RGB8; NUM_LEDS]> = StaticCell::new();
static PLIOT_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, SharedState>> = StaticCell::new();
static USB_CONNECTED: AtomicBool = AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let reset_reason = watchdog.reset_reason();
    let mut reset_count = if watchdog.get_scratch(1) == WATCHDOG_SCRATCH_MAGIC {
        watchdog.get_scratch(0)
    } else {
        0
    };
    if reset_reason == Some(ResetReason::TimedOut) {
        reset_count = reset_count.saturating_add(1);
    } else {
        reset_count = 0;
    }
    watchdog.set_scratch(1, WATCHDOG_SCRATCH_MAGIC);
    watchdog.set_scratch(0, reset_count);
    let clear_program = reset_count >= WATCHDOG_RESET_THRESHOLD;
    if clear_program {
        watchdog.set_scratch(0, 0);
    }

    let mut pio = Pio::new(p.PIO0, Irqs);
    let program = PioWs2812Program::new(&mut pio.common);
    if board::LED_DATA_3V3_GPIO != 15 {
        panic!("LED data pin must be GPIO15 for PIO WS2812");
    }
    let mut led_driver = PioWs2812::<_, 0, NUM_LEDS, Grb>::new(
        &mut pio.common,
        pio.sm0,
        p.DMA_CH0,
        p.PIN_15,
        &program,
    );

    if board::LED_B_GPIO != 18 {
        panic!("LED blue pin must be GPIO18 for heartbeat");
    }
    let onboard_blue = Output::new(p.PIN_18, Level::High);
    
    let data = LED_BUFFER.init([RGB8::default(); NUM_LEDS]);
    boot_led_smoke(&mut led_driver, data).await;

    let usb = p.USB;
    let driver = usb::Driver::new(usb, Irqs);

    //let mut config = embassy_usb::Config::new(0x2E8A, 0x000A);
    let mut config = embassy_usb::Config::new(0x4348, 0x55e0);
    
    config.manufacturer = Some("Pimoroni");
    config.product = Some("Plasma 2350");
    config.serial_number = Some("plasma2350");
    config.device_class = 0xFF;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

    let config_descriptor = USB_CONFIG_DESCRIPTOR.init([0; 64]);
    let bos_descriptor = USB_BOS_DESCRIPTOR.init([0; 64]);
    let control_buf = USB_CONTROL_BUF.init([0; 64]);
    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [],
        control_buf,
    );
    let class = VendorClass::new(&mut builder, 64);
    let usb = builder.build();

    let globals = GLOBALS.init([0u16; GLOBALS_SIZE]);
    let program_buffer = PROGRAM_BUFFER.init([0u16; PROGRAM_BUFFER_SIZE]);
    let storage = {
        use pliot::StorageError;

        let flash = FlashDriver::new_blocking(p.FLASH);
        let flash_base = flash::FLASH_BASE as usize;
        let mut storage = match FlashStorage::new(flash, flash_base) {
            Ok(storage) => storage,
            Err(_) => {
                panic!("flash storage init failed");
            }
        };
        if clear_program {
            if storage.format().is_err() {
                panic!("flash storage watchdog reset format failed");
            }
        }
        match storage.load_header() {
            Ok(()) => {}
            Err(StorageError::InvalidHeader { location: _ }) => {
                if storage.probe_write_read().is_err() {
                    panic!("flash storage probe failed");
                }
                if storage.format().is_err() {
                    panic!("flash storage format failed");
                }
                if storage.load_header().is_err() {
                    panic!("flash storage header reload failed");
                }
            }
            Err(_) => {
                panic!("flash storage header load failed");
            }
        }

        if clear_program || storage.is_empty() {
            write_default_program(&mut storage, program_buffer);
        }
        FLASH_STORAGE.init(storage)
    };

    let shared = PLIOT_SHARED.init(Mutex::new(PliotShared {
        pliot: Pliot::new(storage, globals.as_mut_slice()),
        stack: Vec::new(),
    }));
    {
        let mut guard = shared.lock().await;
        let PliotShared { pliot, stack } = &mut *guard;
        if pliot.init(stack).is_err() {
            panic!("pliot init failed");
        }
    }

    let (usb_sender, usb_receiver) = class.split();
    
    let _ = spawner.spawn(usb_device_task(usb));
    let _ = spawner.spawn(io_task(usb_receiver, usb_sender, shared));
    let _ = spawner.spawn(heartbeat_task(onboard_blue));
    let _ = spawner.spawn(watchdog_task(watchdog));

    led_loop_pio::<
        _,
        _,
        0,
        NUM_LEDS,
        MAX_ARGS,
        MAX_RESULT,
        PROGRAM_BLOCK_SIZE,
        UI_BLOCK_SIZE,
        STACK_SIZE,
        FRAME_TARGET_MS,
    >(&mut led_driver, data, shared)
    .await;
}

async fn boot_led_smoke<P, const SM: usize, const NUM_LEDS: usize>(
    writer: &mut PioWs2812<'static, P, SM, NUM_LEDS, Grb>,
    data: &mut [RGB8; NUM_LEDS],
) where
    P: embassy_rp::pio::Instance,
{
    data.fill(RGB8::default());
    set_debug_led(writer, data, 0, (16, 0, 0)).await;
    Timer::after_millis(150).await;

}

async fn set_debug_led<P, const SM: usize, const NUM_LEDS: usize>(
    writer: &mut PioWs2812<'static, P, SM, NUM_LEDS, Grb>,
    data: &mut [RGB8; NUM_LEDS],
    index: usize,
    rgb: (u8, u8, u8)
) where
    P: embassy_rp::pio::Instance,
{
    if let Some(led) = data.get_mut(index) {
        *led = rgb.into()
    }
    writer.write(data).await;
    Timer::after_millis(150).await;
}
#[embassy_executor::task]
async fn usb_device_task(
    mut usb: embassy_usb::UsbDevice<'static, usb::Driver<'static, peripherals::USB>>,
) {
    usb.run().await;
}

#[embassy_executor::task]
async fn io_task(
    mut receiver: VendorReceiver<'static, usb::Driver<'static, peripherals::USB>>,
    mut sender: VendorSender<'static, usb::Driver<'static, peripherals::USB>>,
    shared: &'static Mutex<CriticalSectionRawMutex, SharedState>,
) {
    let usb_buf = USB_RECEIVE_BUF.init([0u8; USB_RECEIVE_BUF_SIZE]);
    let frame = RAW_MESSAGE_BUFF.init(Vec::new());
    loop {
        USB_CONNECTED.store(false, Ordering::Relaxed);
        receiver.wait_connection().await;
        sender.wait_connection().await;
        USB_CONNECTED.store(true, Ordering::Relaxed);
        frame.clear();
        let _ = io_loop::<
            _,
            _,
            MAX_ARGS,
            MAX_RESULT,
            PROGRAM_BLOCK_SIZE,
            UI_BLOCK_SIZE,
            STACK_SIZE,
            USB_RECEIVE_BUF_SIZE,
            INCOMING_MESSAGE_CAP,
            OUTGOING_MESSAGE_CAP,
        >(&mut receiver, &mut sender, shared, usb_buf, frame)
        .await;
        USB_CONNECTED.store(false, Ordering::Relaxed);
    }
}

fn write_default_program(
    storage: &mut StorageImpl,
    program_buffer: &mut [u16; PROGRAM_BUFFER_SIZE],
) {
    let program_len = match default_program(program_buffer) {
        Ok(length) => length,
        Err(_) => {
            panic!("default program build failed");
        }
    };
    let Some(program) = program_buffer.get(..program_len) else {
        panic!("default program bounds invalid");
    };
    if storage.write_program(program).is_err() {
        panic!("default program flash write failed");
    }
}

async fn led_loop_pio<
    P,
    S,
    const SM: usize,
    const NUM_LEDS: usize,
    const MAX_ARGS: usize,
    const MAX_RESULT: usize,
    const PROGRAM_BLOCK_SIZE: usize,
    const UI_BLOCK_SIZE: usize,
    const STACK_SIZE: usize,
    const FRAME_TARGET_MS: u64,
>(
    writer: &mut PioWs2812<'static, P, SM, NUM_LEDS, Grb>,
    data: &mut [RGB8; NUM_LEDS],
    shared: &'static Mutex<
        CriticalSectionRawMutex,
        PliotShared<'static, 'static, S, MAX_ARGS, MAX_RESULT, PROGRAM_BLOCK_SIZE, UI_BLOCK_SIZE, STACK_SIZE>,
    >,
) -> ! where
    P: embassy_rp::pio::Instance,
    S: pliot::Storage,
{
    let tick: u16 = 0u16;
    loop {
        let start_time = Instant::now();
        {
            let mut guard = shared.lock().await;
            let PliotShared { pliot, stack } = &mut *guard;
            let machine_count = match pliot.machine_count() {
                Ok(count) => count,
                Err(_) => {
                    continue;
                }
            };
            let seed_stack =
                |stack: &mut Vec<Word, STACK_SIZE>, red: u8, green: u8, blue: u8| -> bool {
                    stack.clear();
                    if stack.push(red as Word).is_err() {
                        return false;
                    }
                    if stack.push(green as Word).is_err() {
                        stack.clear();
                        return false;
                    }
                    if stack.push(blue as Word).is_err() {
                        stack.clear();
                        return false;
                    }
                    true
                };
            for (i, led) in data.iter_mut().enumerate() {
                let mut red = 0u8;
                let mut green = 0u8;
                let mut blue = 0u8;
                if !seed_stack(stack, red, green, blue) {
                    continue;
                }
                let tick = Instant::now().as_millis() as u16;
                for machine_number in 0..machine_count {
                    match pliot.get_led_color(machine_number, i as u16, tick, stack) {
                        Ok((next_red, next_green, next_blue)) => {
                            red = next_red;
                            green = next_green;
                            blue = next_blue;
                            if !seed_stack(stack, red, green, blue) {
                                break;
                            }
                        }
                        Err(_) => {
                            stack.clear();
                            break;
                        }
                    }
                }
                *led = (red, green, blue).into();
            }
        }

        let connected = USB_CONNECTED.load(Ordering::Relaxed);
        if let Some(status) = data.last_mut() {
            if connected {
                *status = (0, 16, 0).into();
            } else if tick & 0x10 == 0 {
                *status = (16, 0, 0).into();
            } else {
                *status = RGB8::default();
            }
        }
        writer.write(data).await;

        let wait_duration = match Duration::from_millis(FRAME_TARGET_MS)
            .checked_sub(start_time.elapsed())
        {
            Some(d) => d,
            None => Duration::from_millis(0),
        };

        Timer::after(wait_duration).await;
    }
}

#[embassy_executor::task]
async fn heartbeat_task(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after_millis(40).await;
        led.set_high();
        Timer::after_millis(960).await;
    }
}

#[embassy_executor::task]
async fn watchdog_task(mut watchdog: Watchdog) {
    watchdog.pause_on_debug(true);
    watchdog.start(Duration::from_millis(WATCHDOG_PERIOD_MS));
    loop {
        Timer::after_millis(WATCHDOG_FEED_MS).await;
        watchdog.feed();
    }
}




// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"fluxpilot_plasma2350"),
    embassy_rp::binary_info::rp_program_description!(
        c"your program description"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// End of file
