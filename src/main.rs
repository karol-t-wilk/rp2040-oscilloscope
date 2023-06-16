#![no_std]
#![no_main]

use core::{borrow::BorrowMut, fmt::Write};

use bsp::{
    entry,
    hal::{
        self,
        multicore::{Multicore, Stack},
        sio::Spinlock0,
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
const CIRCULAR_BUFFER_SIZE: usize = 128;
static mut CORE1_STACK: Stack<1024> = Stack::new();

struct ReadingsBuffer {
    readings_buffer: [u16; CIRCULAR_BUFFER_SIZE],
    readings_head: usize,
    usb_head: usize,
}

impl ReadingsBuffer {
    pub const fn new() -> Self {
        Self {
            readings_buffer: [0; CIRCULAR_BUFFER_SIZE],
            readings_head: 0,
            usb_head: 0,
        }
    }

    pub fn can_send_batch(&self) -> bool {
        let updated_readings_head = if self.readings_head < self.usb_head {
            self.readings_head + CIRCULAR_BUFFER_SIZE
        } else {
            self.readings_head
        };

        updated_readings_head - self.usb_head >= BATCH_SIZE
    }

    pub fn can_read_adc(&self) -> bool {
        let updated_usb_head = if self.usb_head < self.readings_head {
            self.usb_head + CIRCULAR_BUFFER_SIZE
        } else {
            self.usb_head
        };

        updated_usb_head - self.readings_head > 1
    }

    pub fn push_adc_reading(&mut self, reading: u16) {
        self.readings_buffer[self.readings_head] = reading;
        self.readings_head += 1;
        self.readings_head %= CIRCULAR_BUFFER_SIZE;
    }

    pub fn pop_readings(&mut self, measures: &mut [u8]) {
        for i in 0..BATCH_SIZE {
            let first_byte_index = i * 2;
            let second_byte_index = first_byte_index + 1;

            let counts = self.readings_buffer[(self.usb_head + i) % CIRCULAR_BUFFER_SIZE];

            measures[first_byte_index] = (counts >> 8).try_into().unwrap_or(255);
            measures[second_byte_index] = (counts & 255).try_into().unwrap_or(255);
        }

        self.usb_head += BATCH_SIZE;
        self.usb_head %= CIRCULAR_BUFFER_SIZE;
    }
}

static mut READINGS_BUFFER: ReadingsBuffer = ReadingsBuffer::new();

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

    let _readings_thread = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || loop {
        while adc.fcs.read().empty().bit() {}

        let _lock = Spinlock0::claim();

        unsafe {
            while READINGS_BUFFER.can_read_adc() && !adc.fcs.read().empty().bit() {
                READINGS_BUFFER.push_adc_reading(adc.fifo.read().val().bits())
            }
        }
    });

    let mut measures = [0u8; BATCH_SIZE * 2];

    loop {
        let lock = Spinlock0::claim();

        unsafe {
            if !READINGS_BUFFER.can_send_batch() {
                continue;
            }

            READINGS_BUFFER.pop_readings(&mut measures);

            drop(lock);

            let _ = &USB_CLASS_LIST.as_mut().unwrap().1.write(&measures);
        }
    }
}

// End of file
