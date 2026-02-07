/* LibOpenCM3 uyumlu STM32F412 Linker Script */

/* Kütüphanenin vektör tablosunu kullanması için zorla */
EXTERN(vector_table)
ENTRY(reset_handler)

MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

SECTIONS
{
    .text : {
        *(.vectors)      /* Vektör Tablosu en başta olmalı */
        *(.text*)        /* Kod bloğu */
        . = ALIGN(4);
        *(.rodata*)      /* Read-only veriler */
        . = ALIGN(4);
    } >rom

    .preinit_array : {
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
    } >rom

    .init_array : {
        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
    } >rom

    .fini_array : {
        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;
    } >rom

    /* RAM Başlangıcı */
    .data : {
        . = ALIGN(4);
        _data = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >ram AT >rom

    /* Kütüphanenin RAM'e kopyalama yapabilmesi için kaynak adresi */
    _data_loadaddr = LOADADDR(.data);

    .bss : {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >ram

    /* Stack Pointer en sona */
    _stack = ORIGIN(ram) + LENGTH(ram);
}