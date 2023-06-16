#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        self,
        multicore::{Multicore, Stack},
        timer::{Alarm, Alarm0},
        Timer,
    },
};

use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use fugit::ExtU32;
use usb_device::{
    class_prelude::{InterfaceNumber, UsbBus, UsbBusAllocator, UsbClass},
    endpoint::EndpointIn,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::SerialPort;

use hal::pac::{interrupt, Interrupt::TIMER_IRQ_0};

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

const BATCH_SIZE: usize = 32;
static mut CORE1_STACK: Stack<1024> = Stack::new();

static mut ALARM: Option<Alarm0> = None;
static mut USB_ALLOCATOR: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_DEV: Option<UsbDevice<'_, hal::usb::UsbBus>> = None;
static mut USB_CLASS_LIST: Option<(
    SerialPort<'_, hal::usb::UsbBus>,
    BulkClass<'_, hal::usb::UsbBus>,
)> = None;

#[interrupt]
fn TIMER_IRQ_0() {
    unsafe {
        match (&mut ALARM, &mut USB_DEV, &mut USB_CLASS_LIST) {
            (Some(alarm), Some(usb_device), Some((serial, bulk))) => {
                alarm.clear_interrupt();
                usb_device.poll(&mut [serial, bulk]);
                alarm.enable_interrupt();
                let _ = alarm.schedule(5.millis());
            }
            (Some(alarm), _, _) => {
                alarm.clear_interrupt();
                alarm.enable_interrupt();
                let _ = alarm.schedule(5.millis());
            }
            _ => (),
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let adc = pac.ADC;
    let mut resets = pac.RESETS;
    let mut ppb = pac.PPB;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut sio = hal::Sio::new(pac.SIO);

    let usb_bus = hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut resets,
    );

    let mut timer = Timer::new(pac.TIMER, &mut resets);

    let mut mc = Multicore::new(&mut pac.PSM, &mut ppb, &mut sio.fifo);
    let alarm = timer.alarm_0().unwrap();

    unsafe {
        ALARM = Some(alarm);
    }

    // Set up the USB driver
    unsafe {
        USB_ALLOCATOR = Some(UsbBusAllocator::new(usb_bus));

        USB_ALLOCATOR.as_mut().and_then(|allocator| {
            let bulk = BulkClass::new(allocator);

            // Set up the USB Communications Class Device driver
            let serial = SerialPort::new(allocator);

            // Create a USB device with a fake VID and PID
            let usb_dev = UsbDeviceBuilder::new(allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .build();

            USB_DEV = Some(usb_dev);
            USB_CLASS_LIST = Some((serial, bulk));

            Some(())
        });
    }

    unsafe {
        ALARM.as_mut().and_then(|alarm| {
            alarm.enable_interrupt();
            match alarm.schedule(5.millis()) {
                _ => (),
            };
            Some(())
        });
    }

    unsafe { pac::NVIC::unmask(TIMER_IRQ_0) }

    unsafe {
        resets.reset.modify(|r, w| w.bits(r.bits()).adc().set_bit());
        resets
            .reset
            .modify(|r, w| w.bits(r.bits()).adc().clear_bit());
        adc.cs.write_with_zero(|w| w.en().set_bit());
        while !adc.cs.read().ready().bit() {}
        pac.PADS_BANK0.gpio[26].write(|w| {
            w.od()
                .set_bit()
                .ie()
                .clear_bit()
                .pue()
                .clear_bit()
                .pde()
                .clear_bit()
        });
        adc.cs.modify(|r, w| w.bits(r.bits()).ainsel().bits(0));
        adc.fcs.write_with_zero(|w| w.en().set_bit());
        adc.cs
            .modify(|r, w| w.bits(r.bits()).start_many().set_bit());
    }

    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let pac = unsafe { pac::Peripherals::steal() };
        let mut sio = hal::Sio::new(pac.SIO);

        loop {
            while adc.fcs.read().empty().bit() {}
            let reading0 = adc.fifo.read().val().bits();

            while adc.fcs.read().empty().bit() {}
            let reading1 = adc.fifo.read().val().bits();

            let data: u32 =
                reading0.try_into().unwrap_or(0) << 16 | reading1.try_into().unwrap_or(0);
            sio.fifo.write_blocking(data);
        }
    });

    let mut measures = [0u8; BATCH_SIZE * 2];

    loop {
        for i in 0..BATCH_SIZE / 2 {
            let data = sio.fifo.read_blocking();
            let reading0: u16 = (data >> 16).try_into().unwrap_or(0);
            let reading1: u16 = (data & 0xffff).try_into().unwrap_or(0);

            let begin = i * 4;
            let end = begin + 4;

            measures[begin..end].copy_from_slice(&[
                (reading0 >> 8).try_into().unwrap_or(255),
                (reading0 & 255).try_into().unwrap_or(255),
                (reading1 >> 8).try_into().unwrap_or(255),
                (reading1 & 255).try_into().unwrap_or(255),
            ]);
        }

        unsafe {
            let _ = &USB_CLASS_LIST.as_mut().unwrap().1.write(&measures);
        }
    }
}

// End of file
