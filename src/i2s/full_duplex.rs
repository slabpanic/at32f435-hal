use super::*;
use crate::peripheral::ClockControl;

// ---------------------------------------------------------------------------
// I2sFullDuplex: main master + EXT slave (I2S2/I2S3 only)
// ---------------------------------------------------------------------------

/// Full-duplex I2S: the main block runs as master in one direction, the EXT
/// block as slave in the other (both share the main block's CK/WS; only the
/// EXT's SD pin is muxed). Only I2S2/I2S3 have an EXT block.
pub struct I2sFullDuplex<'d, M: HasExt> {
    main: I2s<'d, M>,
    _ext_peri: Peri<'d, M::Ext>,
    _ext_sd: Peri<'d, crate::gpio::AnyPin>,
    ext_is_tx: bool,
}

impl<'d, M: HasExt> I2sFullDuplex<'d, M> {
    /// Master TX + EXT slave RX: `sd` transmits, `ext_sd` receives; no MCK.
    #[allow(clippy::too_many_arguments)]
    pub fn new_master_tx(
        main: Peri<'d, M>,
        ext: Peri<'d, M::Ext>,
        sd: Peri<'d, impl I2sSdPin<M>>,
        ext_sd: Peri<'d, impl I2sExtSdPin<M::Ext>>,
        ws: Peri<'d, impl I2sWsPin<M>>,
        ck: Peri<'d, impl I2sCkPin<M>>,
        config: Config,
    ) -> Self {
        Self::new_inner(main, ext, sd, ext_sd, ws, ck, None, false, config)
    }

    /// Master TX + EXT slave RX with the master clock output on `mck`.
    #[allow(clippy::too_many_arguments)]
    pub fn new_master_tx_with_mck(
        main: Peri<'d, M>,
        ext: Peri<'d, M::Ext>,
        sd: Peri<'d, impl I2sSdPin<M>>,
        ext_sd: Peri<'d, impl I2sExtSdPin<M::Ext>>,
        ws: Peri<'d, impl I2sWsPin<M>>,
        ck: Peri<'d, impl I2sCkPin<M>>,
        mck: Peri<'d, impl I2sMckPin<M>>,
        config: Config,
    ) -> Self {
        let mck_pin = new_pin!(
            mck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        Self::new_inner(main, ext, sd, ext_sd, ws, ck, mck_pin, false, config)
    }

    /// Master RX + EXT slave TX: `sd` receives, `ext_sd` transmits; no MCK.
    #[allow(clippy::too_many_arguments)]
    pub fn new_master_rx(
        main: Peri<'d, M>,
        ext: Peri<'d, M::Ext>,
        sd: Peri<'d, impl I2sSdPin<M>>,
        ext_sd: Peri<'d, impl I2sExtSdPin<M::Ext>>,
        ws: Peri<'d, impl I2sWsPin<M>>,
        ck: Peri<'d, impl I2sCkPin<M>>,
        config: Config,
    ) -> Self {
        Self::new_inner(main, ext, sd, ext_sd, ws, ck, None, true, config)
    }

    /// Master RX + EXT slave TX with the master clock output on `mck`.
    #[allow(clippy::too_many_arguments)]
    pub fn new_master_rx_with_mck(
        main: Peri<'d, M>,
        ext: Peri<'d, M::Ext>,
        sd: Peri<'d, impl I2sSdPin<M>>,
        ext_sd: Peri<'d, impl I2sExtSdPin<M::Ext>>,
        ws: Peri<'d, impl I2sWsPin<M>>,
        ck: Peri<'d, impl I2sCkPin<M>>,
        mck: Peri<'d, impl I2sMckPin<M>>,
        config: Config,
    ) -> Self {
        let mck_pin = new_pin!(
            mck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        Self::new_inner(main, ext, sd, ext_sd, ws, ck, mck_pin, true, config)
    }

    #[allow(clippy::too_many_arguments)]
    fn new_inner(
        main: Peri<'d, M>,
        ext: Peri<'d, M::Ext>,
        sd: Peri<'d, impl I2sSdPin<M>>,
        ext_sd: Peri<'d, impl I2sExtSdPin<M::Ext>>,
        ws: Peri<'d, impl I2sWsPin<M>>,
        ck: Peri<'d, impl I2sCkPin<M>>,
        mck_pin: Option<Peri<'d, crate::gpio::AnyPin>>,
        ext_is_tx: bool,
        config: Config,
    ) -> Self {
        // EXT SD direction follows the EXT's role.
        let out_af = || AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None);
        let in_af = || AfType::input(Pull::None);
        let ext_sd_pin = new_pin!(ext_sd, if ext_is_tx { out_af() } else { in_af() });
        // The EXT shares the main block's clock gate; enabling it twice is
        // idempotent, and the reset also just resets the shared gate.
        M::Ext::enable_and_reset();
        let ext_mode = if ext_is_tx {
            Mode::SlaveTx
        } else {
            Mode::SlaveRx
        };
        configure(
            <M::Ext as SealedInstance>::regs(),
            M::Ext::frequency(),
            ext_mode,
            &config,
            false,
        );
        <M::Ext as SealedInstance>::regs()
            .i2sctrl()
            .modify(|_, w| w.en().set_bit());
        // Main block last (per RM: configure both, then enable; the example
        // enables the main after the EXT's DMA setup).
        let main_i2s = I2s::new_inner(
            main,
            sd,
            ws,
            ck,
            mck_pin,
            if ext_is_tx {
                Mode::MasterRx
            } else {
                Mode::MasterTx
            },
            config,
        );
        Self {
            main: main_i2s,
            _ext_peri: ext,
            _ext_sd: ext_sd_pin.unwrap(),
            ext_is_tx,
        }
    }

    /// The main (master) endpoint for direct access (read16/write16 etc.).
    pub fn main(&mut self) -> &mut I2s<'d, M> {
        &mut self.main
    }

    /// Whether the EXT block transmits (true = EXT TX + main RX).
    pub fn ext_is_tx(&self) -> bool {
        self.ext_is_tx
    }

    /// Simultaneous halfword exchange (16-bit data formats): transmits `tx`
    /// and fills `rx` in lockstep (one DT event per side per channel frame).
    /// The first frame after enable carries the DT reset value (0) - it is
    /// received and discarded here, so `rx` aligns with `tx`.
    pub fn blocking_exchange16(&mut self, tx: &[u16], rx: &mut [u16]) -> Result<(), Error> {
        let n = tx.len().max(rx.len());
        let _startup = self.get_hw()?;
        for i in 0..n {
            if let Some(&t) = tx.get(i) {
                self.put_hw(t)?;
            }
            if let Some(r) = rx.get_mut(i) {
                *r = self.get_hw()?;
            }
        }
        self.main.wait_tx_idle()
    }

    /// Simultaneous 24/32-bit sample exchange (two halfwords per frame,
    /// packed/unpacked per protocol). The startup frame (DT reset value) is
    /// received and discarded; see [`I2sFullDuplex::blocking_exchange16`].
    pub fn blocking_exchange32(&mut self, tx: &[u32], rx: &mut [u32]) -> Result<(), Error> {
        let n = tx.len().max(rx.len());
        let format = self.main.config.format;
        let protocol = self.main.config.protocol;
        // Discard the two halfwords of the startup frame.
        for _ in 0..2 {
            self.get_hw()?;
        }
        for i in 0..n {
            if let Some(&t) = tx.get(i) {
                for hw in pack_sample32(t, format, protocol) {
                    self.put_hw(hw)?;
                }
            }
            if let Some(r) = rx.get_mut(i) {
                let mut hw = [0u16; 2];
                for slot in hw.iter_mut() {
                    *slot = self.get_hw()?;
                }
                *r = unpack_sample32(hw, format, protocol);
            }
        }
        self.main.wait_tx_idle()
    }

    fn put_hw(&mut self, hw: u16) -> Result<(), Error> {
        // Paced by the paired get_hw (one RDBF per channel frame); TDBE is
        // vestigial in I2S TX (latch is immediate, hw-verified).
        M::regs().dt().write(|w| unsafe { w.bits(hw) });
        Ok(())
    }

    fn get_hw(&self) -> Result<u16, Error> {
        let r = <M::Ext as SealedInstance>::regs();
        let mut left = DEFAULT_SPIN;
        wait_rdbf(r, &mut left)?;
        Ok(r.dt().read().bits())
    }

    /// Disables both blocks.
    pub fn disable(&mut self) {
        <M::Ext as SealedInstance>::regs()
            .i2sctrl()
            .modify(|_, w| w.en().clear_bit());
        self.main.disable();
    }
}
