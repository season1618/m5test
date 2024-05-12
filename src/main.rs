#![no_std]
#![no_main]

use core::fmt::Write;
use core::format_args;

use esp_backtrace as _;
use esp32_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{IO, GpioPin, Output, PushPull},
    i2c::{self, I2C},
    peripherals::{self, Peripherals},
    prelude::*,
    spi::{Spi, SpiMode},
    uart::{config, TxRxPins, Uart}
};
use esp_println::println;

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

const CONFIG: u8 = 0x01;
const PWR_MGMT_1: u8 = 0x6B;
const ACCEL: u8 = 0x3B;
const TEMP: u8 = 0x41;
const GYRO: u8 = 0x43;
const SLAVE_ADDR: u8 = 0x68;

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
    let dc = io.pins.gpio15.into_push_pull_output();

    let config = config::Config {
        baudrate: 115200,
        data_bits: config::DataBits::DataBits8,
        parity: config::Parity::ParityNone,
        stop_bits: config::StopBits::STOP1,
    };
    
    let txrxpins = TxRxPins::new_tx_rx(
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio3.into_floating_input(),
    );
    
    let mut uart = Uart::new_with_config(
        peripherals.UART0,
        Some(config),
        Some(txrxpins),
        &clocks,
        &mut system.peripheral_clock_control,
    );

    uart.write_str("hello, world\n").unwrap();

    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let i2c_ptr = &mut i2c as *mut I2C<_>;
    let mpu = unsafe { &mut *i2c_ptr as &mut I2C<_> };
    mpu6886_init(mpu, &mut delay).unwrap();

    let mut axp = Axp192::new(i2c);
    axp192_init(&mut axp, &mut delay).unwrap();

    let spi = Spi::new(
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
    let spi_iface = SPIInterfaceNoCS::new(spi, dc);
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
        if let Ok(ch) = uart.read() {
            uart.write_bytes(&mut [ch]).unwrap();
        }

        let mut accel_buf = [0; 6];
        let mut gyro_buf = [0; 6];
        let mut temp_buf = [0; 2];

        mpu.write(SLAVE_ADDR, &[ACCEL]).unwrap();
        mpu.read(SLAVE_ADDR, &mut accel_buf).unwrap();
        mpu.write(SLAVE_ADDR, &[GYRO]).unwrap();
        mpu.read(SLAVE_ADDR, &mut gyro_buf).unwrap();
        mpu.write(SLAVE_ADDR, &[TEMP]).unwrap();
        mpu.read(SLAVE_ADDR, &mut temp_buf).unwrap();

        let accel = (concat(&accel_buf[0..2]) / 16384.0, concat(&accel_buf[2..4]) / 16384.0, concat(&accel_buf[4..6]) / 16384.0);
        let gyro = (concat(&gyro_buf[0..2]) / 131.0, concat(&gyro_buf[2..4]) / 131.0, concat(&gyro_buf[4..6]) / 131.0);
        let temp = concat(&temp_buf) / 326.8 + 25.0;

        println!("accel (g m / s^2): {:5.2?}", accel);
        println!("gyro (degree / s): {:5.2?}", gyro);
        println!("temp  (degree C) : {temp}");
        println!("");

        delay.delay_ms(1000u32);
    }
}

fn concat(arr: &[u8]) -> f32 {
    (((arr[0] as u16) << 8 | arr[1] as u16) as i16) as f32
}

fn mpu6886_init<'a>(mpu: &mut I2C<'a, peripherals::I2C0>, delay: &mut Delay) -> Result<(), i2c::Error> {
    mpu.write(SLAVE_ADDR, &[PWR_MGMT_1, 0b1_0_0_0_0_000])?; // reset
    delay.delay_ms(1000u32);
    mpu.write(SLAVE_ADDR, &[PWR_MGMT_1, 0b0_0_0_0_0_001])?; // enable gyroscope
    mpu.write(SLAVE_ADDR, &[CONFIG, 0b0_0_0_00_0_01])?;

    Ok(())
}

fn axp192_init<I2C, E>(axp: &mut Axp192<I2C>, delay: &mut Delay) -> Result<(), E>
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
