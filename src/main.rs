#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{self, Adc},
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::prelude::_embedded_hal_adc_OneShot;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use usb_device::{
    class_prelude::{InterfaceNumber, UsbBus, UsbBusAllocator, UsbClass},
    endpoint::EndpointIn,
    prelude::{UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::SerialPort;

const BATCH_SIZE: usize = 32;

struct BulkClass<'a, B: UsbBus> {
    ep_in: EndpointIn<'a, B>,
    iface: InterfaceNumber,
}

impl<'a, B: UsbBus> BulkClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            ep_in: alloc.bulk(64),
            iface: alloc.interface(),
        }
    }

    pub fn write(&self, bytes: &[u8]) -> Result<usize, UsbError> {
        self.ep_in.write(bytes)
    }
}

impl<'a, B: UsbBus> UsbClass<B> for BulkClass<'a, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface(self.iface, 0xff, 0x00, 0x00)?;
        writer.endpoint(&self.ep_in)?;

        Ok(())
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus_allocator = UsbBusAllocator::new(usb_bus);

    let mut bulk = BulkClass::new(&usb_bus_allocator);

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus_allocator);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus_allocator, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .build();

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let mut gpio26 = pins.gpio26.into_floating_input();

    loop {
        usb_dev.poll(&mut [&mut bulk, &mut serial]);

        let mut measures = [0 as u8; BATCH_SIZE * 2];

        for i in 0..BATCH_SIZE {
            let counts: u16 = adc.read(&mut gpio26).unwrap_or(0);
            let first_byte_index = i * 2;
            let second_byte_index = first_byte_index + 1;

            measures[first_byte_index] = (counts >> 8).try_into().unwrap_or(255);
            measures[second_byte_index] = (counts & 255).try_into().unwrap_or(255);

            match bulk.write(&measures) {
                Ok(_) => (),
                Err(_) => {
                    serial.write(b"ERROR");
                    ()
                }
            };
        }
    }
}

// End of file
