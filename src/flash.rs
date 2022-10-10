use rp_pico::hal::rom_data::{
    connect_internal_flash, flash_enter_cmd_xip, flash_exit_xip, flash_flush_cache,
    flash_range_erase, flash_range_program,
};

// Flash constants
const FLASH_START: usize = 0x10000000;
const FLASH_SIZE: usize = 2048 * 1024;
const FLASH_SECTOR_SIZE: usize = 4096;
const FLASH_PAGE_SIZE: usize = 256;
const FLASH_BLOCK_SIZE: u32 = 65536;
const FLASH_OFFSET: usize = FLASH_SIZE - FLASH_SECTOR_SIZE;
const FLASH_WR: *mut u8 = (FLASH_START + FLASH_OFFSET) as *mut u8;

// Flash chip commands
const SECTOR_ERASE: u8 = 0x20; // Tested and works with W25Q16JV flash chip
const BLOCK32_ERASE: u8 = 0x52;
const BLOCK64_ERASE: u8 = 0xD8;

/// Write slice data to flash.
///
/// Slice length must be a multiple of 256.
/// Address must be aligned to 4096 bytes.
#[inline(never)]
#[link_section = ".data.ram_code"]
fn write_flash(addr: u32, data: &[u8]) {
    let size = data.len();
    assert!(addr as usize + size <= FLASH_SIZE);
    assert!(addr as usize & (FLASH_SECTOR_SIZE - 1) == 0);
    assert!(size & (FLASH_PAGE_SIZE - 1) == 0);

    cortex_m::interrupt::free(|_| unsafe {
        connect_internal_flash();
        flash_exit_xip();

        flash_range_erase(addr, FLASH_SECTOR_SIZE, FLASH_BLOCK_SIZE, SECTOR_ERASE);
        flash_range_program(addr, data.as_ptr(), size);

        flash_flush_cache();
        flash_enter_cmd_xip();
    });
}

/// Read a slice from flash
///
/// Returned slice may be changed if you rewrite the flash at that address
fn read_flash(addr: usize, size: usize) -> &'static [u8] {
    assert!(addr + size <= FLASH_SIZE);

    let addr = (FLASH_START + addr) as *mut u8;
    let data = cortex_m::interrupt::free(|_| unsafe { core::slice::from_raw_parts(addr, size) });

    data
}
