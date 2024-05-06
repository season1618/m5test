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

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

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
        uart1.write_bytes("Hello, world!\n".as_bytes()).unwrap();
    }
}
