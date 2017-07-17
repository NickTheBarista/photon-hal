//! Low level bindings to the Particle HAL

#![allow(non_camel_case_types)]
#![deny(warnings)]
#![no_std]

extern crate cty;
extern crate photon_core;
extern crate static_ref;

use core::{ops, slice, ptr};

pub mod cloud;
pub mod ll;

use cty::{c_char, c_uchar, c_uint};

#[repr(C)]
pub struct String {
    /// the actual char array
    buffer: *mut c_char,
    /// the array length minus one (for the '\0')
    capacity: c_uint,
    /// the String length (not counting the '\0')
    len: c_uint,
    /// unused, for future features
    flags: c_uchar,
}

impl ops::Deref for String {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        unsafe { slice::from_raw_parts(self.buffer, self.len as usize) }
    }
}

pub struct UsbSerial;

impl UsbSerial {
    /// Enables the serial channel with the specified `baud_rate`
    pub fn begin(&self, baud_rate: u32) {
        unsafe { ll::USB_USART_Init(baud_rate) }
    }

    /// Writes binary data to the serial port
    pub fn write(&self, byte: u8) {
        unsafe {
            ll::USB_USART_Send_Data(byte);
        }
    }
}

pub enum PinMode {
    Input,
    InputPulldown,
    InputPullup,
    Output,
}

pub const LED: D7 = D7;

pub struct D0;
pub struct D1;
pub struct D2;
pub struct D3;
pub struct D4;
pub struct D5;
pub struct D6;
pub struct D7;

pub struct A0;
pub struct A1;
pub struct A2;
pub struct A3;
pub struct A4;
pub struct A5;

macro_rules! pin_mode {
    ($pin:ident, $i:expr) => {
        impl $pin {
            pub fn pin_mode(&self, mode: PinMode) {
                match mode {
                    PinMode::Input => unsafe {
                        ll::HAL_Pin_Mode($i, ll::PinMode::INPUT)
                    },
                    PinMode::InputPulldown => unsafe {
                        ll::HAL_Pin_Mode($i, ll::PinMode::INPUT_PULLDOWN)
                    },
                    PinMode::InputPullup => unsafe {
                        ll::HAL_Pin_Mode($i, ll::PinMode::INPUT_PULLUP)
                    },
                    PinMode::Output => unsafe {
                        ll::HAL_Pin_Mode($i, ll::PinMode::OUTPUT)
                    },
                }
            }
        }
    }
}

pin_mode!(D0, 0);
pin_mode!(D1, 1);
pin_mode!(D2, 2);
pin_mode!(D3, 3);
pin_mode!(D4, 4);
pin_mode!(D5, 5);
pin_mode!(D6, 6);
pin_mode!(D7, 7);

macro_rules! digital_write {
    ($pin:ident, $i:expr) => {
        impl $pin {
            pub fn high(&self) {
                unsafe {
                    ll::HAL_GPIO_Write($i, 1)
                }
            }

            pub fn low(&self) {
                unsafe {
                    ll::HAL_GPIO_Write($i, 0)
                }
            }
        }
    }
}

digital_write!(D0, 0);
digital_write!(D1, 1);
digital_write!(D2, 2);
digital_write!(D3, 3);
digital_write!(D4, 4);
digital_write!(D5, 5);
digital_write!(D6, 6);
digital_write!(D7, 7);

/// Returns the ID of the Particle device
pub fn device_id() -> String {
    unsafe { ll::spark_deviceID() }
}

/// Waits for `ms` milliseconds
///
/// **WARNING** WiFi won't be serviced during this delay. The `App.delay_ms`
/// method should be preferred over this function.
pub fn delay_ms(ms: u32) {
    unsafe { ll::HAL_Delay_Milliseconds(ms) }
}

/// Waits for `us` microseconds
pub fn delay_us(us: u32) {
    unsafe { ll::HAL_Delay_Microseconds(us) }
}

/// Returns the current microseconds
pub fn micros() -> u32 {
    unsafe { ll::HAL_Timer_Get_Micro_Seconds() }
}

//////////////////////////////////////////////////

// FIXME: maybe split this out into a separate source file?
// FIXME: most of the simple logic below is stolen from the Particle firmware:
// https://github.com/spark/firmware/blob/656851a06a0ae659fdb98885218d2256ce5caac4/wiring/src/spark_wiring_rgb.cpp
// Not sure if there are licensing issues here.

pub struct RGBLed;

impl RGBLed {
    /// Return whether or not the RGB LED is under user control (true) or system control (false)
    pub fn controlled(&self) -> bool {
        unsafe { ll::LED_RGB_IsOverRidden() }
    }

    /// Set control of the RGB LED
    pub fn control(&self, override_control: bool) {
        if override_control == self.controlled() {
            return;
        } else if override_control {
            unsafe { ll::LED_Signaling_Start() }
        } else {
            unsafe { ll::LED_Signaling_Stop() }
        }
    }

    /// Set the color of the RGB LED
    pub fn color(&self, r: u16, g: u16, b: u16) {
        if self.controlled() {
            unsafe { ll::HAL_Led_Rgb_Set_Values(r, g, b, ptr::null_mut()) }
        }
    }

    /// Get the brightness of the RGB LED
    pub fn get_brightness() -> u8 {
        unsafe { ll::Get_LED_Brightness() }
    }

    /// Set the brightness of the RGB LED, and optionally update the LED's
    /// state immediately.
    pub fn set_brightness(&self, brightness: u8, update: bool) {
        unsafe { ll::LED_SetBrightness(brightness) };

        if update {
            unsafe { ll::LED_On(ll::LED_RGB) }
        }
    }

    // FIXME: untested
    // FIXME: I'd rather pass D4 instead of 4 into the pin arguments here,
    //        but I guess each Dx is a separate type. Not sure how to do that.
    pub fn mirror_to(&self, rpin: ll::pin_t, gpin: ll::pin_t, bpin: ll::pin_t,
                            invert: bool, bootloader: bool) {
        unsafe {
            ll::HAL_Core_Led_Mirror_Pin(ll::LED_RED as u8 + ll::LED_MIRROR_OFFSET,
                rpin, invert as u32, bootloader as u8, ptr::null_mut());
            ll::HAL_Core_Led_Mirror_Pin(ll::LED_GREEN as u8 + ll::LED_MIRROR_OFFSET,
                gpin, invert as u32, bootloader as u8, ptr::null_mut());
            ll::HAL_Core_Led_Mirror_Pin(ll::LED_BLUE as u8 + ll::LED_MIRROR_OFFSET,
                bpin, invert as u32, bootloader as u8, ptr::null_mut())
        }
    } 

    // TODO:
    // - change handler stuff
}
