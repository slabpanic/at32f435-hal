//! FLASH controller driver (RM 5): sector/block erase, word programming,
//! verify, and option-bytes (user system data) reads.
//!
//! The AT32F435/437 main memory is split into bank1 and bank2 (bank2 exists on
//! the 960K+ densities only); each bank has its own UNLOCK/STS/CTRL/ADDR
//! register set inside the same register block. Sector size is 4 KB on the
//! 448K/960K/1984K/4032K densities and 2 KB on the 256K/1024K densities; the
//! total size is read from the device feature register F_SIZE (@0x1FFF_F7E0,
//! in KB) at runtime.
//!
//! WARNING (RM): the unlock sequence must write KEY1 then KEY2 in order; a
//! wrong sequence raises a bus error and locks the controller until reset.
//!
//! ```ignore
//! let mut flash = hal::flash::Flash::new(p.FLASH);
//! flash.unlock();
//! // SAFETY: The linker reserves this sector exclusively for application data.
//! let result = unsafe { flash.blocking_write_sector(reserved_sector, b"hello") };
//! flash.lock();
//! result?;
//! ```

use embassy_hal_internal::Peri;

use crate::pac;

/// Unlock key 1 (FLASH_UNLOCK / FLASH_UNLOCK2 / FLASH_USD_UNLOCK).
const KEY1: u32 = 0x4567_0123;
/// Unlock key 2.
const KEY2: u32 = 0xCDEF_89AB;

/// Main memory base address.
pub const FLASH_BASE: u32 = 0x0800_0000;

// Raw user system data in flash (RM table 5-7). FLASH_USD is a decoded mirror.
const USD_BASE: usize = 0x1FFF_C000;

/// Generous busy-wait bound for one flash operation (an erase can take
/// hundreds of ms; the loop body is a few cycles at <= 288 MHz).
const OP_TIMEOUT: u32 = 1 << 30;

/// Operation error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum FlashError {
    /// The controller reports a pending operation that did not clear in time.
    Busy,
    /// Program to a location that was not erased (PRGMERR).
    ProgramError,
    /// Erase/program protection blocks the operation (EPPERR).
    ProtectionError,
    /// Address outside main memory (or misaligned where required).
    OutOfBounds,
    /// The controllers are still locked (call `unlock()` first).
    Locked,
}

/// Option-bytes snapshot (loaded from the user system data area at every
/// reset, read from the FLASH_USD / FLASH_EPPS0 / FLASH_EPPS1 registers).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Usd {
    /// Flash access protection active (FAP bit: true = access-protected).
    pub fap: bool,
    /// System setting byte (SSB, bit layout per RM table 5-7; use the
    /// accessors below for the individual options).
    pub ssb: u8,
    /// User data 0 / 1.
    pub user_d0: u8,
    pub user_d1: u8,
    /// Raw erase/program protection bytes 0-7 (1 = unprotected, 0 = protected).
    pub epps0: u32,
    pub epps1: u32,
    /// A USD byte/its complement mismatched (the loader forced both to 0xFF).
    pub usderr: bool,
}

impl Usd {
    /// nWDT_ATO_EN (SSB bit 0): false = hardware watchdog auto-start enabled.
    pub fn wdt_auto_start_disabled(&self) -> bool {
        self.ssb & (1 << 0) != 0
    }
    /// nDEPSLP_RST (SSB bit 1): false = deep-sleep entry generates a reset.
    pub fn deepsleep_no_reset(&self) -> bool {
        self.ssb & (1 << 1) != 0
    }
    /// nSTDBY_RST (SSB bit 2): false = standby entry generates a reset.
    pub fn standby_no_reset(&self) -> bool {
        self.ssb & (1 << 2) != 0
    }
    /// BTOPT (SSB bit 3): force boot from bank 1 when set (RM table 5-7).
    pub fn boot_opt(&self) -> bool {
        self.ssb & (1 << 3) != 0
    }
}

/// Main-memory layout, derived from the F_SIZE feature register.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FlashLayout {
    /// Total size in bytes.
    pub total_size: u32,
    /// Sector size in bytes (2048 or 4096).
    pub sector_size: u32,
    /// First address of bank2 (None = single-bank density).
    pub bank2_start: Option<u32>,
}

impl FlashLayout {
    /// Reads the layout from the device feature register.
    ///
    /// Panics if the device reports an unrecognized flash geometry.
    pub fn detect() -> Self {
        let total_kb = unsafe { (0x1FFF_F7E0 as *const u16).read_volatile() };
        match total_kb {
            256 => Self {
                total_size: 256 * 1024,
                sector_size: 2048,
                bank2_start: None,
            },
            448 => Self {
                total_size: 448 * 1024,
                sector_size: 4096,
                bank2_start: None,
            },
            960 => Self {
                total_size: 960 * 1024,
                sector_size: 4096,
                bank2_start: Some(0x0808_0000),
            },
            1024 => Self {
                total_size: 1024 * 1024,
                sector_size: 2048,
                bank2_start: Some(0x0808_0000),
            },
            1984 => Self {
                total_size: 1984 * 1024,
                sector_size: 4096,
                bank2_start: Some(0x0810_0000),
            },
            4032 => Self {
                total_size: 4032 * 1024,
                sector_size: 4096,
                bank2_start: Some(0x0820_0000),
            },
            _ => panic!("unknown flash size: {total_kb} KiB"),
        }
    }

    fn contains_range(&self, addr: u32, len: usize) -> bool {
        addr.checked_sub(FLASH_BASE)
            .and_then(|offset| (offset as usize).checked_add(len))
            .is_some_and(|end| end <= self.total_size as usize)
    }

    /// Last valid main-memory address (inclusive).
    pub fn end_addr(&self) -> u32 {
        FLASH_BASE + self.total_size - 1
    }

    /// Bank1 range (inclusive).
    pub fn bank1_range(&self) -> (u32, u32) {
        match self.bank2_start {
            Some(b2) => (FLASH_BASE, b2 - 1),
            None => (FLASH_BASE, self.end_addr()),
        }
    }

    /// Number of sectors.
    pub fn num_sectors(&self) -> u32 {
        self.total_size / self.sector_size
    }

    /// Start address of sector `n` (numbered from the start of main memory,
    /// continuing across the bank1/bank2 boundary).
    pub fn sector_addr(&self, n: u32) -> u32 {
        FLASH_BASE + n * self.sector_size
    }
}

/// FLASH controller handle. All operations are blocking (a flash operation
/// stalls any concurrent flash read on the same bank anyway).
pub struct Flash {
    layout: FlashLayout,
    _peri: Peri<'static, crate::peripherals::FLASH>,
}

impl Flash {
    /// Takes the FLASH singleton.
    pub fn new(peri: Peri<'static, crate::peripherals::FLASH>) -> Self {
        Self {
            layout: FlashLayout::detect(),
            _peri: peri,
        }
    }

    /// The detected memory layout.
    pub fn layout(&self) -> FlashLayout {
        self.layout
    }

    /// The currently loaded option bytes.
    pub fn usd(&self) -> Usd {
        let flash = unsafe { &*pac::FLASH::PTR };
        let usd = flash.usd().read();
        // The SSB byte (USD-area bits 7:0) is spread over USD register bits
        // [9:2] (nWDT_ATO_EN is the byte's bit 0). The PAC exposes it as
        // individual fields; reassemble it from the register value.
        let raw = usd.bits();
        Usd {
            fap: usd.fap().bit_is_set(),
            ssb: ((raw >> 2) & 0xFF) as u8,
            user_d0: usd.user_d0().bits(),
            user_d1: usd.user_d1().bits(),
            epps0: flash.epps0().read().epps().bits(),
            epps1: flash.epps1().read().epps().bits(),
            usderr: usd.usderr().bit_is_set(),
        }
    }

    /// Unlocks main flash in both banks, when present.
    pub fn unlock(&mut self) {
        let flash = unsafe { &*pac::FLASH::PTR };
        if flash.ctrl().read().oplk().bit_is_set() {
            flash.unlock().write(|w| unsafe { w.bits(KEY1) });
            flash.unlock().write(|w| unsafe { w.bits(KEY2) });
        }
        if self.layout.bank2_start.is_some() && flash.ctrl2().read().oplk().bit_is_set() {
            flash.unlock2().write(|w| unsafe { w.bits(KEY1) });
            flash.unlock2().write(|w| unsafe { w.bits(KEY2) });
        }
    }

    /// Erases the entire user system data area and programs one option byte.
    /// Offsets are in halfwords. Offsets 0 and 1 are FAP and SSB. Offsets
    /// 2..=7 cover user data and the first four EPP bytes. The flash controller
    /// stores the complement in the high byte.
    ///
    /// # Safety
    ///
    /// The caller must account for the erase of all other option bytes and
    /// user data. These bytes control boot, flash protection, watchdog and
    /// SRAM configuration. Changing readout protection can erase main flash
    /// and reset the device. The caller must select a valid value for the target byte.
    pub unsafe fn blocking_usd_erase_program(
        &mut self,
        offset: usize,
        value: u8,
    ) -> Result<(), FlashError> {
        if offset >= 8 {
            return Err(FlashError::OutOfBounds);
        }
        let flash = &*pac::FLASH::PTR;
        // Do not start a second operation on a busy controller.
        self.wait_done(FLASH_BASE)?;
        self.unlock();
        if flash.ctrl().read().usdulks().bit_is_clear() {
            flash.usd_unlock().write(|w| unsafe { w.bits(KEY1) });
            flash.usd_unlock().write(|w| unsafe { w.bits(KEY2) });
        }
        if flash.ctrl().read().usdulks().bit_is_clear() {
            self.lock();
            return Err(FlashError::Locked);
        }

        flash
            .ctrl()
            .modify(|_, w| w.usders().set_bit().erstr().set_bit());
        let erase = self.wait_done(FLASH_BASE);
        flash.ctrl().modify(|_, w| w.usders().clear_bit());
        let result = erase.and_then(|()| {
            flash.ctrl().modify(|_, w| w.usdprgm().set_bit());
            ((USD_BASE + offset * 2) as *mut u16).write_volatile(u16::from(value));
            let program = self.wait_done(FLASH_BASE);
            flash.ctrl().modify(|_, w| w.usdprgm().clear_bit());
            program
        });
        self.lock();
        result
    }

    /// Reads a raw USD halfword, including the complement in the high byte.
    /// Use [`Self::usd`] to read the decoded values loaded at reset.
    pub fn usd_raw(&self, offset: usize) -> u16 {
        assert!(offset < 8, "USD halfword offset must be 0..8");
        unsafe { ((USD_BASE + offset * 2) as *const u16).read_volatile() }
    }

    /// Locks the flash and the user system data area.
    pub fn lock(&mut self) {
        let flash = unsafe { &*pac::FLASH::PTR };
        flash
            .ctrl()
            .modify(|_, w| w.usdulks().clear_bit().oplk().set_bit());
        if self.layout.bank2_start.is_some() {
            flash.ctrl2().modify(|_, w| w.oplk().set_bit());
        }
    }

    /// True while any targeted bank is still locked (OPLK = 1).
    fn is_locked(&self, addr: u32) -> bool {
        let flash = unsafe { &*pac::FLASH::PTR };
        if self.is_bank2(addr) {
            flash.ctrl2().read().oplk().bit_is_set()
        } else {
            flash.ctrl().read().oplk().bit_is_set()
        }
    }

    /// Blocking erase of sector `n`.
    ///
    /// # Safety
    ///
    /// The entire sector must be reserved for the caller. It must contain
    /// no executable code, live references or data owned by another user.
    pub unsafe fn blocking_erase_sector(&mut self, n: u32) -> Result<(), FlashError> {
        if n >= self.layout.num_sectors() {
            return Err(FlashError::OutOfBounds);
        }
        let addr = self.layout.sector_addr(n);
        if self.is_locked(addr) {
            return Err(FlashError::Locked);
        }
        self.wait_done(addr)?;
        let flash = unsafe { &*pac::FLASH::PTR };
        if self.is_bank2(addr) {
            flash.ctrl2().modify(|_, w| w.secers().set_bit());
            flash.addr2().write(|w| unsafe { w.fa().bits(addr) });
            flash.ctrl2().modify(|_, w| w.erstr().set_bit());
        } else {
            flash.ctrl().modify(|_, w| w.secers().set_bit());
            flash.addr().write(|w| unsafe { w.fa().bits(addr) });
            flash.ctrl().modify(|_, w| w.erstr().set_bit());
        }
        let res = self.wait_done(addr);
        self.clear_erase_mode(addr);
        res
    }

    /// Blocking erase of the 64 KB block containing `addr`.
    ///
    /// # Safety
    ///
    /// The caller must own the entire block, with no executable code or live
    /// references into it.
    pub unsafe fn blocking_erase_block(&mut self, addr: u32) -> Result<(), FlashError> {
        self.check_addr(addr)?;
        if self.is_locked(addr) {
            return Err(FlashError::Locked);
        }
        self.wait_done(addr)?;
        let flash = unsafe { &*pac::FLASH::PTR };
        if self.is_bank2(addr) {
            flash.ctrl2().modify(|_, w| w.blkers().set_bit());
            flash.addr2().write(|w| unsafe { w.fa().bits(addr) });
            flash.ctrl2().modify(|_, w| w.erstr().set_bit());
        } else {
            flash.ctrl().modify(|_, w| w.blkers().set_bit());
            flash.addr().write(|w| unsafe { w.fa().bits(addr) });
            flash.ctrl().modify(|_, w| w.erstr().set_bit());
        }
        let res = self.wait_done(addr);
        self.clear_erase_mode(addr);
        res
    }

    /// Blocking word program (32-bit). `addr` must be 4-byte aligned and the
    /// target must be erased (or hold all-1 bits).
    ///
    /// # Safety
    ///
    /// The caller must own the target word, with no executable code, live
    /// references or data belonging to other users in the programmed range.
    pub unsafe fn blocking_program_word(&mut self, addr: u32, word: u32) -> Result<(), FlashError> {
        if !addr.is_multiple_of(4) || !self.layout.contains_range(addr, 4) {
            return Err(FlashError::OutOfBounds);
        }
        if self.is_locked(addr) {
            return Err(FlashError::Locked);
        }
        let flash = unsafe { &*pac::FLASH::PTR };
        let busy = if self.is_bank2(addr) {
            flash.sts2().read().obf().bit_is_set()
        } else {
            flash.sts().read().obf().bit_is_set()
        };
        if busy {
            return Err(FlashError::Busy);
        }
        if self.is_bank2(addr) {
            flash.ctrl2().modify(|_, w| w.fprgm().set_bit());
        } else {
            flash.ctrl().modify(|_, w| w.fprgm().set_bit());
        }
        (addr as *mut u32).write_volatile(word);
        let res = self.wait_done(addr);
        if self.is_bank2(addr) {
            flash.ctrl2().modify(|_, w| w.fprgm().clear_bit());
        } else {
            flash.ctrl().modify(|_, w| w.fprgm().clear_bit());
        }
        res
    }

    /// Blocking byte-buffer write: programs `data` word-by-word starting at
    /// `addr` (4-byte aligned). The tail is padded with 0xFF (legal over an
    /// erased location).
    ///
    /// # Safety
    ///
    /// The caller must own the full programmed range, including any padding,
    /// with no executable code or live references into it.
    pub unsafe fn blocking_write_bytes(
        &mut self,
        addr: u32,
        data: &[u8],
    ) -> Result<(), FlashError> {
        let padded_len = data.len().checked_add(3).ok_or(FlashError::OutOfBounds)? & !3;
        if !addr.is_multiple_of(4) || !self.layout.contains_range(addr, padded_len) {
            return Err(FlashError::OutOfBounds);
        }
        for (index, chunk) in data.chunks(4).enumerate() {
            let mut word = [0xFF; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            self.blocking_program_word(addr + 4 * index as u32, u32::from_le_bytes(word))?;
        }
        Ok(())
    }

    /// Reads bytes from main flash. Panics if the range is outside main flash.
    pub fn read_bytes(&self, addr: u32, buf: &mut [u8]) {
        assert!(
            self.layout.contains_range(addr, buf.len()),
            "read outside main flash"
        );
        for (index, byte) in buf.iter_mut().enumerate() {
            *byte = unsafe { ((addr as usize + index) as *const u8).read_volatile() };
        }
    }

    /// Compares bytes in main flash with `data`. Returns false for an
    /// out-of-range read or a mismatch.
    pub fn verify(&self, addr: u32, data: &[u8]) -> bool {
        self.layout.contains_range(addr, data.len())
            && data.iter().enumerate().all(|(index, &byte)| {
                byte == unsafe { ((addr as usize + index) as *const u8).read_volatile() }
            })
    }

    /// Erase, write and verify one sector.
    ///
    /// # Safety
    ///
    /// The entire sector must be reserved for the caller as described by
    /// [`Self::blocking_erase_sector`].
    pub unsafe fn blocking_write_sector(&mut self, n: u32, data: &[u8]) -> Result<(), FlashError> {
        if data.len() > self.layout.sector_size as usize {
            return Err(FlashError::OutOfBounds);
        }
        self.blocking_erase_sector(n)?;
        let addr = self.layout.sector_addr(n);
        self.blocking_write_bytes(addr, data)?;
        if !self.verify(addr, data) {
            return Err(FlashError::ProgramError);
        }
        Ok(())
    }

    // ---- internals -------------------------------------------------------

    fn check_addr(&self, addr: u32) -> Result<(), FlashError> {
        if self.layout.contains_range(addr, 1) {
            Ok(())
        } else {
            Err(FlashError::OutOfBounds)
        }
    }

    fn is_bank2(&self, addr: u32) -> bool {
        self.layout.bank2_start.is_some_and(|b2| addr >= b2)
    }

    fn wait_done(&self, addr: u32) -> Result<(), FlashError> {
        let flash = unsafe { &*pac::FLASH::PTR };
        let mut left = OP_TIMEOUT;
        if self.is_bank2(addr) {
            while flash.sts2().read().obf().bit_is_set() {
                left -= 1;
                if left == 0 {
                    return Err(FlashError::Busy);
                }
            }
            let sts = flash.sts2().read();
            let err = if sts.prgmerr().bit_is_set() {
                Some(FlashError::ProgramError)
            } else if sts.epperr().bit_is_set() {
                Some(FlashError::ProtectionError)
            } else {
                None
            };
            // rw1c flags: write 1 to clear ODF and the error flags.
            flash.sts2().write(|w| {
                w.odf().set_bit();
                w.prgmerr().set_bit();
                w.epperr().set_bit();
                w
            });
            err.map_or(Ok(()), Err)
        } else {
            while flash.sts().read().obf().bit_is_set() {
                left -= 1;
                if left == 0 {
                    return Err(FlashError::Busy);
                }
            }
            let sts = flash.sts().read();
            let err = if sts.prgmerr().bit_is_set() {
                Some(FlashError::ProgramError)
            } else if sts.epperr().bit_is_set() {
                Some(FlashError::ProtectionError)
            } else {
                None
            };
            flash.sts().write(|w| {
                w.odf().set_bit();
                w.prgmerr().set_bit();
                w.epperr().set_bit();
                w
            });
            err.map_or(Ok(()), Err)
        }
    }

    /// Clears SECERS/BLKERS on the bank that owns `addr` (the official library
    /// clears the mode bit after every erase).
    fn clear_erase_mode(&self, addr: u32) {
        let flash = unsafe { &*pac::FLASH::PTR };
        if self.is_bank2(addr) {
            flash.ctrl2().modify(|_, w| {
                w.secers().clear_bit();
                w.blkers().clear_bit();
                w
            });
        } else {
            flash.ctrl().modify(|_, w| {
                w.secers().clear_bit();
                w.blkers().clear_bit();
                w
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn range_requires_the_full_buffer_to_fit_in_flash() {
        let layout = FlashLayout {
            total_size: 1024 * 1024,
            sector_size: 2048,
            bank2_start: Some(0x0808_0000),
        };
        assert!(layout.contains_range(FLASH_BASE, layout.total_size as usize));
        assert!(layout.contains_range(layout.end_addr(), 1));
        assert!(layout.contains_range(layout.end_addr() + 1, 0));
        assert!(!layout.contains_range(layout.end_addr(), 2));
        assert!(!layout.contains_range(FLASH_BASE - 1, 1));
        assert!(!layout.contains_range(u32::MAX, usize::MAX));
    }
}
