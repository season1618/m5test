#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
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
use embedded_hal_bus::spi::ExclusiveDevice;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let mut clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sck = io.pins.gpio18.into_push_pull_output();
    let mosi = io.pins.gpio23.into_push_pull_output();
    let miso = io.pins.gpio38;
    let cs = io.pins.gpio5.into_push_pull_output();
    let lcd_dc = io.pins.gpio15.into_push_pull_output();
    let reset = io.pins.gpio33.into_push_pull_output(); // tekitou

    let lcd_spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &mut clocks)
        .with_sck(sck)
        .with_mosi(mosi)
        .with_miso(miso);
    let lcd_spi = ExclusiveDevice::new(lcd_spi, cs, delay).unwrap();

    let spi_iface = SPIInterface::new(lcd_spi, lcd_dc);

    let mut lcd = Ili9341::new(
        spi_iface,
        reset,
        &mut delay,
        Orientation::PortraitFlipped,
        DisplaySize240x320,
    ).unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);

    Text::with_alignment(
        "First line\nSecond line",
        Point::new(20, 30),
        style,
        Alignment::Center,
    )
    .draw(&mut lcd)
    .unwrap();

    loop {
    }
}
