;#TITLE: boot.asm	#FUNCTION: Bootloader	#AUTHOR: Ashmead Mohammed
[bits 16]
[ORG 0x7C00]	;#From BIOS load at address 0x7C00
jmp begin
BIOS_PARAMETER_BLOCK:
	.OEMName db 0x53, 0x41, 0x4d, 0x41, 0x43, 0x41, 0x49, 0x52
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
	.VolumeLabel db 0x53, 0x41, 0x4d, 0x41, 0x52, 0x41, 0x43, 0x41, 0x49, 0x52, 0x4f
	.FileSystem db 'FAT12   '  
;BIOS_PARAMETER_BLOCK_EXT:
;	.SectorsPerFat dd 0
;	.Flags dw 0
;	.Version dw 0
;	.RootCluster dd 0
;	.InfoCluster dw 0
;	.BackupBoot dw 0
;	.Reserved1 dw 0
;	.Reserved2 dw 0
;	.Reserved3 dw 0
;	.Reserved4 dw 0
;	.Reserved5 dw 0
;	.Reserved6 dw 0
begin:
jmp start	
;INCLUDES;
%include 'gdt.asm'
;;;;VARIABLES;;;
;mem_errmsg db 'M',0
;load_errmsg db 'K', 0
err_msg db 'E', 0
;jmp_msg db 'J', 0
%define FAT_Addr 0x2000
%define Root_Dir_Addr 0x3300
%define Kernel_Addr 0x7E00
%define MemMapInfoAddr 0x5000

struc	MemoryMapEntry
	.base	resq	1	; base address of address range
	.length		resq	1	; length of address range in bytes
	.type		resd	1	; type of address range  1: AvailableMemory. 2: Reserved, do not use. (sys ROM, memory-mapped device) 3: ACPIReclaimMemory (usable by OS after reading ACPI tables). 4: ACPINVSMemory (OS required to save this memory between NVS sessions). Other values treated as undefined.
	.acpi	resd	1	; reserved
endstruc

boot_info:
;	.flags: dd 0
	.memoryLo: dd 0
	.memoryHi: dd 0
	.bootDevice: dd 0
;	.cmdLine: dd 0
;	.mods_count dd 0
;	.mods_addr dd 0
;	.syms0 dd 0
;	.syms1 dd 0
;	.syms2 dd 0
;	.syms3 dd 0
	.mmap_length: dd 0
	.mmap_addr: dd 0
;	.drives_length dd 0
;	.drives_addr dd 0
;	.config_table: dd 0
;	.bootloader_name dd 0
;	.apm_table dd 0
;	.vbe_control_info dd 0
;	.vbe_mode_info dw 0
;	.vbe_mode dw 0
;	.vbe_interface_seg dw 0
;	.vbe_interface_off dw 0
;	.vbe_interface_len dw 0

print16:	;16 bit print routine SI=0 terminated string
	mov ah, 0xE	;Print char function
	.Loop1:
		lodsb		;load next byte from string from SI to AL
		or al, al	;Does AL=0?
		jz Puts16Done	;0 terminator exit
		int 10h
		jmp .Loop1	;Repeat till 0 terminator
Puts16Done:
	ret

;Sector   = (LBA mod SectorsPerTrack) + 1
;Cylinder = (LBA / SectorsPerTrack) / NumHeads
;Head     = (LBA / SectorsPerTrack) mod NumHeads
load_sect:	;int 0x13 function ah=02 al=num sectors to read ch=cylinder cl=sector number dh=head, dl=drive, es:bx=memory address (data buffer)
	mov ah, 2	; BIOS read sector function
	int 0x13
	jc load_err  ; carry flag set Error, try again.
	ret
load_err:
	mov si, err_msg
	call print16
	jmp load_sect
ret

reset_drive: ;Reset Disk int 0x13 function ah=0, dl=drive to reset (if bit 7 is set hardisk and floppy reset). Status returned in ah. Carry flag set on error
	mov ah, 0
	int 0x13				; Reset the drive
	jc reset_drive			; If there was a error, try again.
ret

start:	;Bootloader START;
mov [boot_info.bootDevice], dl ;bootdrive stored in dl at boottime copy to variable mbn_bootdevice
call reset_drive
mov ch, 0 ;Cyl/Trak 
mov cl, 2 ;start at 2nd sector
mov al, 9 ;load 9 sectors
mov dh, 0 ;head to use
mov bx, FAT_Addr
call load_sect	;load FAT into memory
call reset_drive
mov ch, 0 ;Cyl/Trak 
mov cl, 2;start read at 19th sector
mov al, 0xE ;read 14 sectors
mov dh, 1 ;head to use
mov bx, Root_Dir_Addr
call load_sect	;load Root dir into memory
call reset_drive
mov ch, 0 ;Cyl/Trak 
mov cl, 0x11 ;cyl/trk =1 so that means 18 sectors past. start read at 34th sector
mov al, 0x40 ;read 64 sectors
mov dh, 1 ;head to use
mov bx, Kernel_Addr
call load_sect	;load kernel into memory

boot_stack_setup:	;setup stack for calling functions;
;	xor ax, ax	;null registers and segments
;	xor bx, bx
;	xor cx, cx
;	mov ds, ax
;	mov es, ax
;	mov ss, ax
;	mov bp, 0x7B00	; stack begins at 0x7B00
;	mov sp, bp
mov	ax, 2	;#Enable A20 gate set bit 2 (enable a20)
out	0x92, al ;write to System Control Port A 0x92 
;	xor bx, bx
;	xor	cx, cx		;clear all registers. This is for testing later
;	xor	dx, dx
	mov	ax, 0xe801	;get system memory using int 0x15 function 0xE801
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
	mov si, err_msg
	call print16
	jmp $
mem_done: ;Store memory values 
	mov	word [boot_info.memoryHi], bx
	mov word [boot_info.memoryLo], ax

	mov	edi, MemMapInfoAddr	;set destination index to known address so we know where the memory map is
	xor	ebx, ebx
	xor	ebp, ebp	;number of entries stored here
	mov	edx, 'PAMS'	; 'SMAP'
	mov	eax, 0xe820	;Get memory map using interrupt 0x15 function ax=0xe820, dx='PAMS', cx=size of entry, bp=counter for number of entries, di=memory address to store it at;
	mov	ecx, 24		;size of memory map entry struct is 24 bytes
	int	0x15		;get first entry
	jc	mem_err ;.error		;carry flag set error
	cmp	eax, 'PAMS'	;bios returns SMAP in eax
	jne	 mem_err ;.error
	test ebx, ebx	;if ebx=0 then list is one entry long; bail out
	je	mem_err ;.error
	jmp	.start
.next_entry:
	mov	edx, 'PAMS'	;some bios' trash this register
	mov	ecx, 24		; entry is 24 bytes
	mov	eax, 0xe820
	int	0x15		; get next entry
.start:
	jcxz .skip_entry	;if actual returned bytes is 0, skip entry
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
;.error:
;	stc ;set carry flag to signal error
.done:

mov word [boot_info.mmap_length], bp	;store mmap info, bp contains the number of entries in memory map
mov word [boot_info.mmap_addr], MemMapInfoAddr ;address of memmap
;	mov si, jmp_msg	;Jump to Pmode
;	call print16
cli ;#Disable interrupts to setup gdt and enter pmode. It is re-enabled in the kernel
lgdt [GDT_Ptr]	;#Load Global Descriptor Table, define pmode segments for code and data
mov eax , cr0	;#To switch pmode, set first bit of CR0 control register to 1
or eax , 0x1
mov cr0 , eax
jmp KCODE_SEG:init_pm	;#far jump new segment ie 32bit code. Forces CPU cache flush

[bits 32]	;#32bit Protected Mode
init_pm:	;#Init regs and stack in PM to values in GDT data segment
	mov eax , KDATA_SEG
	mov ds , ax
;	mov es , ax
;	mov fs , ax
;	mov gs , ax
	mov ss , ax
	mov ebp, 0x9F000	;# Update stack at top of free space.
	mov esp , ebp
mov eax, 0xB002FACE ;my alternative to 1BADB002
push dword [boot_info.memoryLo] ;put boot_info on stack so we can retrieve it as argument in the kernel
push dword [boot_info.memoryHi]
push dword [boot_info.bootDevice]
push dword [boot_info.mmap_length]
push dword [boot_info.mmap_addr]
jmp KCODE_SEG:Kernel_Addr ;#jump to a known address / call the kernel function KCODE_SEG:Kernel_Addr ie 0x7E00
times 510 - ($-$$) db 0	;pad with 0 bytes until file is 510 bytes in size
dw 0xAA55	;#BootSector Signature
