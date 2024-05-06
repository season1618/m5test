#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::{Delay, MicrosDurationU64},
    gpio::{GpioPin, IO, Unknown},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
};

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::digital::{OutputPin, ErrorType, PinState};
use embedded_hal_bus::spi::ExclusiveDevice;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
// use stm32f4xx_hal::{
//     prelude::*,
//     spi::{Mode, NoMiso, Phase, Polarity},
//     timer::Channel,
// };

// #[derive(Default)]
// pub struct DummyOutputPin;
// impl ErrorType for DummyOutputPin {
//     type Error = ();
// }

// impl OutputPin for DummyOutputPin {
//     fn set_low(&mut self) -> Result<(), Self::Error> {
//         Ok(())
//     }
//     fn set_high(&mut self) -> Result<(), Self::Error> {
//         Ok(())
//     }
//     fn set_state(&mut self, _state: PinState) -> Result<(), Self::Error> {
//         Ok(())
//     }
// }

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let mut clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    let mut timer = esp_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks, None).timer0;

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // --- ili9341 ---
    let lcd_dc = io.pins.gpio1.into_push_pull_output(); // tekitou
    
    let sclk = io.pins.gpio12;
    let miso = io.pins.gpio11;
    let mosi = io.pins.gpio13;
    let cs = io.pins.gpio10;

    let mut lcd_spi = Spi::new(
        peripherals.SPI2,
        100.kHz(),
        SpiMode::Mode0,
        // &mut peripheral_clock_control,
        &mut clocks,
    ).with_pins(Some(sclk), Some(mosi), Some(miso), None as Option<GpioPin<Unknown, 10>>);

    let mut lcd_spi = ExclusiveDevice::new(lcd_spi, cs.into_push_pull_output(), delay).unwrap();

    let spi_iface = SPIInterface::new(lcd_spi, lcd_dc);
    // let dummy_reset = DummyOutputPin::default();
    let dummy_reset = io.pins.gpio2.into_push_pull_output(); // tekitou
    

    let mut lcd = Ili9341::new(
        spi_iface,
        dummy_reset,
        &mut delay,
        Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
    .unwrap();

    // Create a new character style
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);

    // Create a text at position (20, 30) and draw it using the previously defined style
    Text::with_alignment(
        "First line\nSecond line",
        Point::new(20, 30),
        style,
        Alignment::Center,
    )
    .draw(&mut lcd)
    .unwrap();
    // ---

    loop {
        timer.start(MicrosDurationU64::from_ticks(1000000));
        timer.wait();
    }
}
