//! Link the firmware for the AT32F435CGT7 example board.

use std::{env, fs, path::PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo sets OUT_DIR"));
    fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).expect("copy memory.x");
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rerun-if-changed=memory.x");
}
