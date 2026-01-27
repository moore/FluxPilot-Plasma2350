//! Set up linker scripts for the rp235x-hal examples

use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // The file `memory.x` is loaded by cortex-m-rt's `link.x` script, which
    // is what we specify in `.cargo/config.toml` for Arm builds
    let memory_x = include_bytes!("memory.x");
    let mut f = File::create(out.join("memory.x")).unwrap();
    f.write_all(memory_x).unwrap();
    println!("cargo:rerun-if-changed=memory.x");

    let flash_size = parse_flash_size(memory_x).unwrap_or_else(|message| {
        panic!("failed to parse FLASH size from memory.x: {message}");
    });
    let storage_size = parse_storage_size(memory_x).unwrap_or_else(|message| {
        panic!("failed to parse __storage_size from memory.x: {message}");
    });
    let mut f = File::create(out.join("memory_consts.rs")).unwrap();
    writeln!(f, "pub const FLASH_SIZE: usize = {};", flash_size).unwrap();
    writeln!(f, "pub const STORAGE_SIZE: usize = {};", storage_size).unwrap();

    // The file `rp235x_riscv.x` is what we specify in `.cargo/config.toml` for
    // RISC-V builds
    let rp235x_riscv_x = include_bytes!("rp235x_riscv.x");
    let mut f = File::create(out.join("rp235x_riscv.x")).unwrap();
    f.write_all(rp235x_riscv_x).unwrap();
    println!("cargo:rerun-if-changed=rp235x_riscv.x");

    println!("cargo:rerun-if-changed=build.rs");
}

fn parse_flash_size(memory_x: &[u8]) -> Result<usize, &'static str> {
    let contents = core::str::from_utf8(memory_x).map_err(|_| "memory.x is not valid UTF-8")?;
    for line in contents.lines() {
        let line = line.trim();
        if !line.starts_with("FLASH") {
            continue;
        }
        let length_pos = line.find("LENGTH = ").ok_or("missing LENGTH")?;
        let value = line[(length_pos + "LENGTH = ".len())..]
            .split_whitespace()
            .next()
            .ok_or("missing LENGTH value")?;
        let value = value.trim_end_matches(',');
        return parse_size(value);
    }
    Err("missing FLASH line")
}

fn parse_storage_size(memory_x: &[u8]) -> Result<usize, &'static str> {
    let contents = core::str::from_utf8(memory_x).map_err(|_| "memory.x is not valid UTF-8")?;
    for line in contents.lines() {
        let line = line.trim();
        if !line.starts_with("__storage_size") {
            continue;
        }
        let value = line
            .split('=')
            .nth(1)
            .ok_or("missing __storage_size value")?
            .trim()
            .trim_end_matches(';');
        return parse_size(value);
    }
    Err("missing __storage_size")
}

fn parse_size(value: &str) -> Result<usize, &'static str> {
    let value = value.trim();
    let (number, unit) = match value.chars().last() {
        Some(ch) if ch.is_ascii_alphabetic() => (&value[..value.len() - 1], &value[value.len() - 1..]),
        _ => (value, ""),
    };
    let number: usize = number.trim().parse().map_err(|_| "invalid size number")?;
    let multiplier = match unit {
        "" => 1,
        "K" | "k" => 1024,
        "M" | "m" => 1024 * 1024,
        _ => return Err("unsupported size suffix"),
    };
    number
        .checked_mul(multiplier)
        .ok_or("size overflow")
}
