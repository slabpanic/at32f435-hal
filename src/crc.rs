//! CRC calculation unit (RM 10): the AT32F435 CRC supports the standard
//! fixed CRC-32 polynomial (0x4C11DB7, init 0xFFFFFFFF) plus programmable
//! 16/8/7-bit polynomials, input bit reversal and output word reversal.
//!
//! Feed 32-bit words into DT and read the running result back from DT (the
//! register holds the accumulation; CDT/IDT are the common/initial-data
//! registers). This module provides a blocking `embedded-hal-friendly` API:
//!
//! ```ignore
//! let mut crc = hal::crc::Crc::new(p.CRC, hal::crc::Config::default());
//! crc.reset();
//! let c = crc.feed_words(&[0x1234_5678, 0xDEAD_BEEF]);
//! ```
//!
//! Note: the AT32 CRC unit is the "wire world" CRC-32/MPEG-2-style unit
//! (fixed poly 0x04C11DB7, init 0xFFFFFFFF, no final XOR, MSB-first by
//! default) - that matches CRC-32/MPEG-2 for word-aligned input.

use embassy_hal_internal::Peri;

use crate::pac;

/// CRC unit configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// Polynomial size (CTRL.POLY_SIZE). The fixed 32-bit polynomial is used
    /// for [`PolySize::Bits32`]; smaller sizes use the POLY register.
    pub poly_size: PolySize,
    /// Input data reversal (CTRL.REVID, applied to each word fed in).
    pub reverse_input: ReverseInput,
    /// Output reversal (CTRL.REVOD: word reversal of the result).
    pub reverse_output: bool,
    /// Initial data value (IDT register; 0xFFFFFFFF = the standard CRC-32
    /// init). Reset semantics: CTRL.RST restores the IDT value.
    pub initial_data: u32,
    /// Programmable polynomial value for 16/8/7-bit sizes (POLY register).
    pub polynomial: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            poly_size: PolySize::Bits32,
            reverse_input: ReverseInput::None,
            reverse_output: false,
            initial_data: 0xFFFF_FFFF,
            polynomial: 0x04C1_1DB7,
        }
    }
}

/// Polynomial width (CTRL.POLY_SIZE).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum PolySize {
    /// 32-bit (the fixed CRC-32 polynomial, POLY register unused).
    #[default]
    Bits32,
    /// 16-bit (`POLY[15:0]`).
    Bits16,
    /// 8-bit (`POLY[7:0]`).
    Bits8,
    /// 7-bit (`POLY[6:0]`).
    Bits7,
}

impl PolySize {
    const fn code(self) -> u8 {
        match self {
            PolySize::Bits32 => 0,
            PolySize::Bits16 => 1,
            PolySize::Bits8 => 2,
            PolySize::Bits7 => 3,
        }
    }

    /// Width in bits.
    pub const fn bits(self) -> u32 {
        match self {
            PolySize::Bits32 => 32,
            PolySize::Bits16 => 16,
            PolySize::Bits8 => 8,
            PolySize::Bits7 => 7,
        }
    }
}

/// Input data reversal (CTRL.REVID).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum ReverseInput {
    /// No effect.
    #[default]
    None,
    /// Bit reversal by byte.
    Byte,
    /// Bit reversal by half-word.
    HalfWord,
    /// Bit reversal by word.
    Word,
}

impl ReverseInput {
    const fn code(self) -> u8 {
        match self {
            ReverseInput::None => 0,
            ReverseInput::Byte => 1,
            ReverseInput::HalfWord => 2,
            ReverseInput::Word => 3,
        }
    }
}

/// CRC unit handle (blocking; the unit is a shared singleton).
pub struct Crc {
    _peri: Peri<'static, crate::peripherals::CRC>,
}

impl Crc {
    /// Configures the unit with `config` and resets the accumulator to the
    /// initial data value.
    pub fn new(peri: Peri<'static, crate::peripherals::CRC>, config: Config) -> Self {
        <crate::peripherals::CRC as crate::peripheral::ClockControl>::enable_and_reset();
        let r = unsafe { &*pac::CRC::PTR };
        if config.poly_size != PolySize::Bits32 {
            r.poly().write(|w| w.set(config.polynomial));
        }
        r.idt().write(|w| w.set(config.initial_data));
        // REVOD (bit 7) has no PAC writer field - write the whole register.
        // Clears POLY_SIZE[4:3] + REVID[6:5] + REVOD[7]; RST (bit 0) is a
        // write-1 strobe and is left alone.
        let mut bits = r.ctrl().read().bits();
        bits &= !(0b11111 << 3);
        bits |= ((config.poly_size.code() as u32) << 3)
            | ((config.reverse_input.code() as u32) << 5)
            | (u32::from(config.reverse_output) << 7);
        r.ctrl().write(|w| unsafe { w.bits(bits) });
        r.ctrl().modify(|_, w| w.rst().set_bit());
        Self { _peri: peri }
    }

    /// Resets the accumulator to the initial data value (CTRL.RST).
    pub fn reset(&mut self) {
        let r = unsafe { &*pac::CRC::PTR };
        r.ctrl().modify(|_, w| w.rst().set_bit());
    }

    /// Feeds one 32-bit word and returns the running CRC.
    pub fn feed_word(&mut self, word: u32) -> u32 {
        let r = unsafe { &*pac::CRC::PTR };
        r.dt().write(|w| w.set(word));
        r.dt().read().bits()
    }

    /// Feeds a slice of 32-bit words and returns the accumulated CRC.
    pub fn feed_words(&mut self, words: &[u32]) -> u32 {
        let mut v = 0;
        for w in words {
            v = self.feed_word(*w);
        }
        v
    }

    /// Feeds a byte slice: the AT32 DT register accepts full 32-bit words,
    /// so the bytes are packed big-endian into words (byte 0 feeds first =
    /// the word's MSB) and the tail is padded with zero bytes at the LSB end.
    /// With the default configuration this matches the byte-oriented
    /// CRC-32/MPEG-2 reference (e.g. b"123456789" -> 0x0376E6E7).
    pub fn feed_bytes(&mut self, bytes: &[u8]) -> u32 {
        let mut chunks = bytes.chunks_exact(4);
        let mut v = 0;
        for c in &mut chunks {
            let w = u32::from_be_bytes([c[0], c[1], c[2], c[3]]);
            v = self.feed_word(w);
        }
        let rem = chunks.remainder();
        if !rem.is_empty() {
            let mut buf = [0u8; 4];
            buf[..rem.len()].copy_from_slice(rem);
            let w = u32::from_be_bytes(buf);
            v = self.feed_word(w);
        }
        v
    }

    /// Reads the current CRC value (without feeding data).
    pub fn value(&self) -> u32 {
        unsafe { &*pac::CRC::PTR }.dt().read().bits()
    }

    /// Sets the initial-data register (IDT; used by CTRL.RST).
    pub fn set_initial_data(&mut self, v: u32) {
        unsafe { &*pac::CRC::PTR }.idt().write(|w| w.set(v));
    }

    /// Reads/writes the 8-bit common data register (CDT; free scratch byte
    /// intended for storing a checksum byte alongside a data block).
    pub fn set_common_data(&mut self, v: u8) {
        // CDT is byte-accessible only (u8 register width).
        unsafe { &*pac::CRC::PTR }
            .cdt()
            .write(|w| unsafe { w.bits(v as u32) });
    }

    pub fn common_data(&self) -> u8 {
        unsafe { &*pac::CRC::PTR }.cdt().read().bits() as u8
    }
}

/// Reference CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no final xor,
/// MSB-first) computed in software - used by the unit tests as the golden
/// value for the default hardware configuration.
#[cfg(test)]
mod tests {
    #[test]
    fn sw_reference_matches_known_vector() {
        // CRC-32/MPEG-2("123456789") = 0x0376E6E7 (well-known check value)
        let data = b"123456789";
        let mut crc: u32 = 0xFFFF_FFFF;
        for &b in data {
            crc ^= (b as u32) << 24;
            for _ in 0..8 {
                if crc & 0x8000_0000 != 0 {
                    crc = (crc << 1) ^ 0x04C1_1DB7;
                } else {
                    crc <<= 1;
                }
            }
        }
        assert_eq!(crc, 0x0376_E6E7);
    }
}
