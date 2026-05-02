MEMORY
{
    /*
     * Low 64 KB — reserved for DMA-visible static memory (descriptors and
     * packet buffers). Anything that the EMAC DMA must read or write MUST
     * land here, never in the high RAM region. Apply via the .dma_bss
     * section (see SECTIONS below) plus `#[link_section = ".dma_bss"]`.
     */
    RAM_DMA : ORIGIN = 0x4FF40000, LENGTH = 0x00010000

    /*
     * Main RAM for code/rodata/data/non-DMA bss/heap/stack. Stops at
     * 0x4FF80000 — the upper 256 KB (0x4FF80000..0x4FFC0000) is the *backing
     * SRAM for the L2 cache* and addresses there behave unpredictably for
     * arbitrary access (writes can be silently dropped, reads return cache
     * tag content, etc.). Stack lives at the top of REGION_STACK so it must
     * not cross 0x4FF80000 either — keeping LENGTH at 0x30000 caps the top
     * exactly there.
     *
     * If `.text` no longer fits, build with `[profile.dev] opt-level = "s"`
     * at the workspace root.
     */
    RAM : ORIGIN = 0x4FF50000, LENGTH = 0x00030000
}

REGION_ALIAS("REGION_TEXT", RAM);
REGION_ALIAS("REGION_RODATA", RAM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

SECTIONS
{
    /*
     * Note: NOT (NOLOAD). Rust statics carry their initial bytes via the ELF
     * — marking the section NOLOAD would leave DMA_RESOURCES uninitialized at
     * runtime (memory contents are whatever pre-existed) and Rust would run
     * over a struct it thinks is zero-initialized but isn't. Loading the
     * zeros via the ELF image takes a few extra KB but is correct.
     */
    .dma_bss : ALIGN(128)
    {
        KEEP(*(.dma_bss .dma_bss.*))
    } > RAM_DMA
}
