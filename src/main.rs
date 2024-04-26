#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::{MicrosDurationU64},
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    uart::{config::Config, TxRxPins, Uart},
};

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    init_heap();

    esp_println::logger::init_logger_from_env();

    let mut timer = esp_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks, None).timer0;

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio2.into_floating_input(),
    );
    let mut uart1 = Uart::new_with_config(peripherals.UART1, Config::default(), Some(pins), &clocks, None);

    loop {
        timer.start(MicrosDurationU64::from_ticks(1000000));
        timer.wait();
        uart1.write_bytes("\nHello, world!".as_bytes()).unwrap();
    }
}
