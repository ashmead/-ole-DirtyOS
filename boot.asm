;#TITLE boot.asm
;#FUNCTION 'ole dirty bootloader / kernel
;#AUTHOR Ashmead Mohammed
[bits 16]
;#From BIOS load at address 0x7C00
[ORG 0x7C00]
jmp begin ;start

;BIOS_PARAMETER_BLOCK:
BIOS_PARAMETER_BLOCK:
	.OEMName db 'SAMACAIR' 
	.Try db 0
	.BytesPerSector dw 512
	.SectorsPerCluster db 1
	.ReservedSectors dw 1
	.NumOfFats db 2
	.NumDirEntries dw 224
	.TotalSectors dw 2880
	.Media db 0xF0
	.SectorsPerFat dw 9
	.SectorsPerTrack dw 18
	.HeadsPerCylinder dw 2
	.HiddenSectors dd 0
	.LongSectors dd 0
	.Drive db 0
	.Unused db 0
	.ExtBootSig db 0x29
	.Serial dd 0x5AAACA10
	.VolumeLabel db 'SAMARACAIRO'
	.FileSystem db 'FAT12   '  

begin:
jmp start

;count times 32 db 0xff

;BIOS_PARAMETER_BLOCK_EXT:
;BIOS_PARAMETER_BLOCK_EXT:
;	.SectorsPerFat dd 0
;	.Flags dw 0
;	.Version dw 0
;	.RootCluster dd 0
;	.InfoCluster dw 0
;	.BackupBoot dw 0
;	.Reserved0 dw 0
;	.Reserved1 dw 0
;	.Reserved2 dw 0
;	.Reserved3 dw 0
;	.Reserved4 dw 0
;	.Reserved5 dw 0
	

;;;;;;;;;;;;;
;;;;INCLUDES;
;;;;;;;;;;;;;
%include 'gdt.asm'

; Types of Address Ranges
; The types of address ranges defined for this function is shown below:
; 1: Available Memory
; 2: Reserved, do not use. (e.g. system ROM, memory-mapped device)
; 3: ACPI Reclaim Memory (usable by OS after reading ACPI tables)
; 4: ACPI NVS Memory (OS is required to save this memory between NVS sessions)
; All other values should be treated as undefined.
struc	MemoryMapEntry
	.base	resq	1	; base address of address range
	.length		resq	1	; length of address range in bytes
	.type		resd	1	; type of address range
	.acpi	resd	1	; reserved
endstruc

;;;;VARIABLES;;;
;mem_errmsg db 'M',0
;load_errmsg db 'K', 0
err_msg db 'E', 0
jmp_msg db 'J', 0

;%define FAT_Addr 0x2000
;%define Root_Dir_Addr 0x3300
;%define Kernel_Addr 0x7E00


boot_info:
;struc multiboot_info
;	at multiboot_info.flags,				dd 0
	.memoryLo: dd 0
	.memoryHi: dd 0
	.bootDevice: dd 0
;	at multiboot_info.cmdLine,				dd 0
;	at multiboot_info.mods_count,			dd 0
;	at multiboot_info.mods_addr,			dd 0
;	at multiboot_info.syms0,				dd 0
;	at multiboot_info.syms1,				dd 0
;	at multiboot_info.syms2,				dd 0
;	at multiboot_info.syms3,				dd 0
	.mmap_length: dd 0
	.mmap_addr: dd 0
;	at multiboot_info.drives_length,		dd 0
;	at multiboot_info.drives_addr,			dd 0
;	at multiboot_info.config_table,			dd 0
;	at multiboot_info.bootloader_name,		dd 0
;	at multiboot_info.apm_table,			dd 0
;	at multiboot_info.vbe_control_info,		dd 0
;	at multiboot_info.vbe_mode_info,		dw 0
;	at multiboot_info.vbe_mode		dw 0
;	at multiboot_info.vbe_interface_seg,	dw 0
;	at multiboot_info.vbe_interface_off,	dw 0
;	at multiboot_info.vbe_interface_len,	dw 0
;endstruc

;;;;;;;;;;;;;;;;;;;;;;
;16 bit print routine;
;SI=0 terminated string
;;;;;;;;;;;;;;;;;;;;;;
print16:
	pusha ;save regs
	mov ah, 0xE	;Print char function
	.Loop1:
		lodsb		;load next byte from string from SI to AL
		or al, al	;Does AL=0?
		jz Puts16Done	;0 terminator exit
		int 10h
		jmp .Loop1	;Repeat till 0 terminator
Puts16Done:
	popa ;restore regs
	ret
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Read disc sectors into memory using int 0x13 ah=02;
;AH = 02h
;AL = number of sectors to read (must be nonzero)
;CH = low eight bits of cylinder number
;CL = sector number 1-63 (bits 0-5) high two bits of cylinder (bits 6-7, hard disk only)
;DH = head number
;DL = drive number (bit 7 set for hard disk)
;ES:BX -> data buffer
;Return:
;CF set on error
;if AH = 11h (corrected ECC error), AL = burst length
;CF clear if successful
;AH = status (see #00234)
;AL = number of sectors transferred (only valid if CF set for some BIOSes)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Sector   = (LBA mod SectorsPerTrack) + 1
;Cylinder = (LBA / SectorsPerTrack) / NumHeads
;Head     = (LBA / SectorsPerTrack) mod NumHeads
load_sect:	; kernel is copied from 2nd sector and loaded at addr 0x7E00
	push ax
	mov ax, 0	; segment to load from
	mov es, ax
	pop ax 
;	mov bx, 0x7E00 ; offset:0x7E00 physical address to put kernel in memory
	mov ah, 2	; BIOS read sector function
;	mov al, 0x40 ;read minimum 0x1 to maximum 0x40 (ie 64) sectors
;system size restrictions, limit kernel size 32768 bytes and 2gb memory
;	mov ch,	00	; begining Cylinder/Track to read
;	mov cl,	02	; begining Sector to read
;	mov dh,	00	; Head to read
	mov dl, [boot_info.bootDevice] ; Drive to read
	int 0x13
	jc load_err  ;Error, try again.
	ret
;	jmp boot_stack_setup ;else setup bootstack
load_err:
	mov si, err_msg
	call print16
	jmp load_sect
ret
;	jmp $


reset_drive:
;	mov si, ax
;	call print16
;	jmp $
	xor ax, ax ; Reset Disk function ax=0
	mov dl, [boot_info.bootDevice] ;Select booted drive
	int 13h				; Reset the drive
	jc reset_drive			; If there was a error, try again.
ret

;;;;;;;;;;;;;;;;;;
;Bootloader START;
;;;;;;;;;;;;;;;;;;
start:
	mov [boot_info.bootDevice], dl ;bootdrive stored in dl at boottime copy to variable mbn_bootdevice

call reset_drive
;load FAT into memory
mov ch, 0 ;Cyl/Trak 
mov cl, 2 ;start at 2nd sector
mov al, 9 ;load 9 sectors
mov dh, 0 ;head to use
mov bx, 0x2000 ;FAT_Addr
call load_sect

call reset_drive
;load Root dir into memory
mov ch, 0 ;Cyl/Trak 
mov cl, 2;start read at 19th sector
mov al, 0xE ;read 14 sectors
mov dh, 1 ;head to use
mov bx, 0x3300 ;Root_Dir_Addr
call load_sect

call reset_drive
;load kernel into memory
mov ch, 0 ;Cyl/Trak 
mov cl, 0x11 ;cyl/trk =1 so that means 18 sectors past. start read at 34th sector
mov al, 0x40 ;read 64 sectors
mov dh, 1 ;head to use
mov bx, 0x7E00 ;Kernel_Addr
call load_sect

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;setup stack for calling functions;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
boot_stack_setup:
	xor ax, ax             ; null registers and segments
;	xor bx, bx
;	xor cx, cx
;	mov ds, ax
;	mov es, ax
	mov	di, 0x5000
;	mov ax, 0x6000
	mov ss, ax   ; stack begins at 0x4000-0x7B00
	mov bp, 0x7B00  ;0xFFFF
	mov sp, bp  ;0xFFFF
;;;;;;;;;;;;;;;;;;
;#Enable A20 gate;
;;;;;;;;;;;;;;;;;;
mov	ax, 2	; set bit 2 (enable a20)
out	0x92, al ;write it

;;;;;;;;;;;;;;;;;;;;;
;;;get system memory;
;;;;;;;;;;;;;;;;;;;;;
;	xor bx, bx
;	xor	cx, cx		;clear all registers. This is for testing later
;	xor	dx, dx
	mov	ax, 0xe801
	int	0x15	
	jc	mem_err
	cmp	ah, 0x86		;unsupported function
	je	mem_err
	cmp	ah, 0x80		;invalid command
	je	mem_err
	jcxz mem_done			;bios may have stored it in ax,bx or cx,dx. test if cx is 0
	mov	ax, cx			;its not, so it should contain mem size; store it
	mov	bx, dx
	jmp mem_done
mem_err:
	mov	ax, -1
	mov	bx, 0
	mov si, err_msg
	call print16
;	jmp $
mem_done: ;Store memory values 
	mov	word [boot_info.memoryHi], bx
	mov word [boot_info.memoryLo], ax

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Get memory map using interrupt 0x15 function 0xe820;
;INT 15 - newer BIOSes - GET SYSTEM MEMORY MAP
;	AX = E820h
;	EAX = 0000E820h
;	EDX = 534D4150h ('SMAP')
;	EBX = continuation value or 00000000h to start at beginning of map
;	ECX = size of buffer for result, in bytes (should be >= 20 bytes)
;	ES:DI -> buffer for result (see #00581)
;Return: CF clear if successful
;	    EAX = 534D4150h ('SMAP')
;	    ES:DI buffer filled
;	    EBX = next offset from which to copy or 00000000h if all done
;	    ECX = actual length returned in bytes
;	CF set on error
;	    AH = error code (86h) (see #00496 at INT 15/AH=80h)
;Notes:	originally introduced with the Phoenix BIOS v4.0, this function is
;	  now supported by most newer BIOSes, since various versions of Windows
;	  call it to find out about the system memory
;	a maximum of 20 bytes will be transferred at one time, even if ECX is
;	  higher; some BIOSes (e.g. Award Modular BIOS v4.50PG) ignore the
;	  value of ECX on entry, and always copy 20 bytes
;	some BIOSes expect the high word of EAX to be clear on entry, i.e.
;	  EAX=0000E820h
;	if this function is not supported, an application should fall back
;	  to AX=E802h, AX=E801h, and then AH=88h
;	the BIOS is permitted to return a nonzero continuation value in EBX
;	  and indicate that the end of the list has already been reached by
;	  returning with CF set on the next iteration
;	this function will return base memory and ISA/PCI memory contiguous
;	  with base memory as normal memory ranges; it will indicate
;	  chipset-defined address holes which are not in use and motherboard
;	  memory-mapped devices, and all occurrences of the system BIOS as
;	  reserved; standard PC address ranges will not be reported
;SeeAlso: AH=C7h,AX=E801h"Phoenix",AX=E881h,MEM xxxxh:xxx0h"ACPI"
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;set destination index to a known address so we know where the memory map is
;edi was set at boot stack setup
;	mov edi, 0x2000 ;BOCHS doesnt run properly when map is placed at lower addresses, QEMU runs as normal
	xor	ebx, ebx
	xor	ebp, ebp ;number of entries stored here
	mov	edx, 'PAMS'								; 'SMAP'
	mov	eax, 0xe820
	mov	ecx, 24								; size of memory map entry struct is 24 bytes
	int	0x15									; get first entry
	jc	.error	
	cmp	eax, 'PAMS'								; bios returns SMAP in eax
	jne	.error
	test	ebx, ebx								; if ebx=0 then list is one entry long; bail out
	je	.error
	jmp	.start
.next_entry:
	mov	edx, 'PAMS'								; some bios' trash this register
	mov	ecx, 24								; entry is 24 bytes
	mov	eax, 0xe820
	int	0x15									; get next entry
.start:
	jcxz	.skip_entry								; if actual returned bytes is 0, skip entry
.notext:
	mov	ecx, [es:di + MemoryMapEntry.length]	; get length (low dword)
	test	ecx, ecx								; if length is 0 skip it
	jne	short .good_entry
	mov	ecx, [es:di + MemoryMapEntry.length + 4]; get length (upper dword)
	jecxz	.skip_entry								; if length is 0 skip it
.good_entry:
	inc	ebp										; increment entry count
	add	di, 24									; point di to next entry in buffer
.skip_entry:
	cmp	ebx, 0									; if ebx returns 0, list is done
	jne	.next_entry								; get next entry
	jmp	.done
.error:
	stc ;set carry flag to signal error
.done:

;store mmap info
;bp contains the number of entries in memory map
mov word [boot_info.mmap_length], bp
mov word [boot_info.mmap_addr], 0x5000 ;address of memmap

;;;;
	mov si, jmp_msg
	call print16
;;;;

;;;#Disable interrupts while setup gdt, idt and enter pmode
cli

;#Load Global Descriptor Table, define pmode segments for code and data
lgdt [GDT_Ptr]

;#Load Interrupt Descriptor Table so div 0, 0 doesnt triple fault ie interrupts #update we do this in the kernel now
;lidt [IDT_Ptr]

;#To switch pmode, set first bit of CR0 control register
mov eax , cr0
or eax , 0x1
mov cr0 , eax

;#far jump new segment ie 32bit code. Forces CPU cache flush
jmp CODE_SEG:init_pm

;;;;;;;;;;;;;;;;;;;;;;;
;#32bit Protected Mode;
;;;;;;;;;;;;;;;;;;;;;;;
;#Init regs and stack in PM to values in GDT data segment
[bits 32]
init_pm:
	mov eax , DATA_SEG
	mov ds , ax
	mov es , ax
	mov fs , ax
	mov gs , ax
	mov ss , ax
;# Update stack at top of free space.
mov ebp, 0x9EFFF
;mov edi , 0xFFFFFFFF
mov esp , ebp

;setup arguments to function call
;mov eax, 0xB002FACE ;my alternative to 1BADB002
;mov eax, kernel_size
;put boot_info on stack so we can retrieve it as an argument to our kernel "function"
push dword [boot_info.memoryLo]
push dword [boot_info.memoryHi]
push dword [boot_info.bootDevice]
push dword [boot_info.mmap_length]
push dword [boot_info.mmap_addr]

;#jump to a known address / call the kernel function
;call CODE_SEG:kernel_begin ;0x7E00
jmp CODE_SEG:0x7E00 ;Kernel_Addr ;0x7E00
;pad with 0 bytes until file is 510 bytes in size
times 510 - ($-$$) db 0
;#BootSector Signature
dw 0xAA55
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Bootloader ends here;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
