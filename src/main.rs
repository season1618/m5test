#![no_std]
#![no_main]

use esp_backtrace as _;
use esp32_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{IO, GpioPin, Output, PushPull},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
};

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb666,
    prelude::*,
    text::{Alignment, Text},
};
use mipidsi::{
    Builder,
    options::{ColorInversion, ColorOrder},
};
use axp192::Axp192;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let mut clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sck = io.pins.gpio18;
    let mosi = io.pins.gpio23;
    let miso = io.pins.gpio38;
    let cs = io.pins.gpio5;
    let lcd_dc = io.pins.gpio15.into_push_pull_output();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    let mut axp = Axp192::new(i2c);
    m5sc2_init(&mut axp, &mut delay).unwrap();

    let lcd_spi = Spi::new(
        peripherals.SPI2,
        sck,
        mosi,
        miso,
        cs,
        400u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &mut clocks
    );

    let spi_iface = SPIInterfaceNoCS::new(lcd_spi, lcd_dc);

    let mut lcd = Builder::ili9342c_rgb666(spi_iface)
        .with_display_size(320, 240)
        .with_color_order(ColorOrder::Bgr)
        .with_invert_colors(ColorInversion::Inverted)
        .init(&mut delay, None::<GpioPin<Output<PushPull>, 0>>) // tekitou
        .unwrap();

    // lcd.clear(Rgb666::RED).unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, Rgb666::BLACK);

    Text::with_alignment(
        "Hello, world!",
        Point::new(20, 30),
        style,
        Alignment::Center,
    )
    .draw(&mut lcd)
    .unwrap();

    loop {
    }
}

fn m5sc2_init<I2C, E>(axp: &mut axp192::Axp192<I2C>, delay: &mut Delay) -> Result<(), E>
where
    I2C: embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    // Default setup for M5Stack Core 2
    axp.set_dcdc1_voltage(3350)?; // Voltage to provide to the microcontroller (this one!)

    axp.set_ldo2_voltage(3300)?; // Peripherals (LCD, ...)
    axp.set_ldo2_on(true)?;

    axp.set_ldo3_voltage(2000)?; // Vibration motor
    axp.set_ldo3_on(false)?;

    axp.set_dcdc3_voltage(2800)?; // LCD backlight
    axp.set_dcdc3_on(true)?;

    axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Power LED
    axp.set_gpio1_output(false)?; // In open drain modes, state is opposite to what you might
                                  // expect

    axp.set_gpio2_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Speaker
    axp.set_gpio2_output(true)?;

    axp.set_key_mode(
        // Configure how the power button press will work
        axp192::ShutdownDuration::Sd4s,
        axp192::PowerOkDelay::Delay64ms,
        true,
        axp192::LongPress::Lp1000ms,
        axp192::BootTime::Boot512ms,
    )?;

    axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput)?; // LCD reset control

    axp.set_battery_voltage_adc_enable(true)?;
    axp.set_battery_current_adc_enable(true)?;
    axp.set_acin_current_adc_enable(true)?;
    axp.set_acin_voltage_adc_enable(true)?;

    // Actually reset the LCD
    axp.set_gpio4_output(false)?;
    axp.set_ldo3_on(true)?; // Buzz the vibration motor while intializing ¯\_(ツ)_/¯
    delay.delay_ms(100u32);
    axp.set_gpio4_output(true)?;
    axp.set_ldo3_on(false)?;
    delay.delay_ms(100u32);

    Ok(())
}
