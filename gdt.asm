; Title: gdt.asm	FUNCTION: global Table Descriptor	AUTHOR:Ashmead Mohammed
;[bits 16]
GDT:
gdt_null: ;#the mandatory null descriptor
dd 0x0
dd 0x0

GDT_KCode:
gdt_kcode_limit dw (0xFFFFFFFF & 0xFFFF)
gdt_kcode_base_low dw (0x0 & 0xFF)
gdt_kcode_base_mid db (0x0 >> 16) & 0xFF
gdt_kcode_access db 10011010b ; 1 st flags , type flags ;0x9A ; privilege level
gdt_kcode_gran db ((0xFFFFFFFF >> 16) & 0x0F) | (11001111b & 0xF0)
gdt_kcode_base_hi db (0x0 >> 24) & 0xFF

GDT_KData:
gdt_kdata_limit dw (0xFFFFFFFF & 0xFFFF)
gdt_kdata_base_low dw (0x0 & 0xFF)
gdt_kdata_base_mid db (0x0 >> 16) & 0xFF
gdt_kdata_access db 10010010b ; 1 st flags , type flags ;0x92
gdt_kdata_gran db ((0xFFFFFFFF >> 16) & 0x0F) | (11001111b & 0xF0)
gdt_kdata_base_hi db (0x0 >> 24) & 0xFF

GDT_UCode:
gdt_ucode_limit dw (0xFFFFFFFF & 0xFFFF)
gdt_ucode_base_low dw (0x0 & 0xFF)
gdt_ucode_base_mid db (0x0 >> 16) & 0xFF
gdt_ucode_access db 11111010b ; 1 st flags , type flags ;0xFA
gdt_ucode_gran db ((0xFFFFFFFF >> 16) & 0x0F) | (11001111b & 0xF0)
gdt_ucode_base_hi db (0x0 >> 24) & 0xFF

GDT_UData:
gdt_udata_limit dw (0xFFFFFFFF & 0xFFFF)
gdt_udata_base_low dw (0x0 & 0xFF)
gdt_udata_base_mid db (0x0 >> 16) & 0xFF
gdt_udata_access db 11110010b ; 1 st flags , type flags ; 0xF2
gdt_udata_gran db ((0xFFFFFFFF >> 16) & 0x0F) | (11001111b & 0xF0)
gdt_udata_base_hi db (0x0 >> 24) & 0xFF

GDT_End: ;A label at the end of the GDT is so we can have the assembler calculate the size of the GDT for the GDT decriptor ( below )

GDT_Ptr:
dw GDT_End - GDT - 1
dd GDT
; Start address of our GDT
; Define constants for GDT segment descriptor offsets , which
; are what segment registers must contain when in protected mode. For example ,
; when we set DS = 0 x10 in PM , the CPU knows that we mean it to use the
; segment described at offset 0 x10 ( i.e. 16 bytes ) in our GDT , which in our
; case is the DATA segment (0 x0 -> NULL ; 0 x08 -> CODE ; 0 x10 -> DATA )
KCODE_SEG equ GDT_KCode - GDT
KDATA_SEG equ GDT_KData - GDT
UCODE_SEG equ GDT_UCode - GDT
UDATA_SEG equ GDT_UData - GDT
