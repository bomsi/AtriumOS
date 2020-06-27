; Boot loader for AtriumOS

; Copyright (c) 2017-2020 Mislav Bozicevic
; All rights reserved.

; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:

; * Redistributions of source code must retain the above copyright notice, this
;   list of conditions and the following disclaimer.

; * Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.

; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
; ARE DISCLAIMED.
; IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
; FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
; DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cpu X64

; 16-bit code
[bits 16]

; we are loaded by BIOS at linear address 0x007c00
[org 0x7c00]

; disable interrupts
cli

; canonicalize CS:IP to a known segment:offset pair
; (some BIOSes actually begin execution at 07c0:0000h instead of 0000:7c00h)
jmp    0:next
next:

; set the segment registers to 0
xor    ax, ax
mov    ds, ax
mov    es, ax
mov    fs, ax
mov    gs, ax
mov    ss, ax

; set the stack pointer (top of the real mode stack)
;  the stack can grow (down) to 0x000500, so max size is 2816 bytes (or 176
;  push operations due to the fact that they are word-sized)
mov    sp, 0x1000

; preserve BIOS boot drive number @ boot_drive_addr
%define boot_drive_addr 0x2000
mov    bx, boot_drive_addr
mov    [bx], dx

; enable interrupts
sti

; assume VGA and initialize it in 320x200 resolution with 256 colors (mode 13h)
mov    ah, 0x00
mov    al, 0x13
int    0x10

; enable the A20 gate
mov    ah, 0x24
mov    al, 0x01
int    0x15

; VGA range is A000:0000 - A000:FFFF, so we need to switch the segment
%define vga_segment 0xa000
%define vga_columns 320
%define vga_rows 200
%define vga_color_dark_gray 0x08
mov    ax, vga_segment
mov    es, ax
mov    di, 0x0000
mov    cx, vga_columns * vga_rows
mov    al, vga_color_dark_gray
cld
rep    stosb

; restore drive number
mov    word dx, [boot_drive_addr]

; reset disk system
xor    ax, ax
mov    es, ax
call   reset_disk_system
jc     error_boot_sector

; restore drive number
mov    word dx, [boot_drive_addr]

; read the second stage of boot code from disk
;  (0, 0, 2) ... (0, 0, 18) -> 0x007e00 - 0x00a000 [8704 bytes]
%define boot_code_second_step_addr 0x7e00
xor    ax, ax
mov    es, ax
; sectors to read count
mov    al, 17
; cylinder
mov    ch, 0
; sector
mov    cl, 2
; head
mov    dh, 0
; ES:BX is the buffer address pointer
mov    bx, boot_code_second_step_addr
call   read_sectors
jc     error_boot_sector

; obtain the RAM map
call   detect_memory
jc     error_boot_sector

; jump to the second stage of boot code
jmp    0:boot_code_second_step_addr

; Read sectors from disk.
; Input:
;  AL - number of sectors to read
;  CH - cylinder
;  CL - sector
;  DH - head
;  DL - drive number
;  ES:BX - destination buffer address
; On success, carry flag will be cleared.
read_sectors:
	pusha
	; SI is used as the retry counter
	mov    si, 3

	.read:
	; read sectors function
	mov    ah, 0x02
	int    0x13
	; end on a successful read
	jnc    .end
	; decrement the retry counter
	dec    si
	; end if maximum retry count was exceeded
	jc     .end
	call   reset_disk_system
	; retry if reset succeeded, otherwise end
	jnc    .read

	.end:
	popa
	ret

; Reset the disk system.
; On success, carry flag will be cleared.
reset_disk_system:
	pusha

	; reset disk system function (AH = 0x00)
	xor    ax, ax
	int    0x13

	popa
	ret

; address where the system memory map list will be stored
%define ram_map_addr 0x2005
; address where count of system memory map list items will be stored
%define ram_map_count_addr 0x2003

; Query system address map.
; Output:
;  number of entries (a word) @ ram_map_count_addr
;  entry array (20 bytes per entry) @ ram_map_addr
; On success, carry flag will be cleared.
detect_memory:
	xor    ax, ax
	mov    es, ax
	mov    ax, ram_map_addr
	mov    di, ax
	; use E820h BIOS function
	mov    eax, 0xe820
	; continuation value (zero for first call)
	xor    ebx, ebx
	; signature 'SMAP' used by BIOS to verify the caller is requesting
	; the system map information which will be returned in ES:DI
	mov    edx, 0x534d4150
	; buffer size
	mov    ecx, 20
	; we use BP for count
	xor    bp, bp
	int    0x15
	; carry indicates error here
	jc     short .fail
	; restore the signature if BIOS has overwritten it
	mov    edx, 0x534d4150
	cmp    eax, edx
	jne    short .fail
	test   ebx, ebx
	; if ebx == 0 list has only one entry (useless)
	je     short .fail
	jmp    short .body

	.call:
	mov    eax, 0xe820
	mov    ecx, 20
	int    0x15
	; carry indicates end here
	jc     short .end
	; restore the signature if BIOS has overwritten it
	mov    edx, 0x534d4150

	.body:
	; skip zero-length entry
	jcxz   .skip_entry
	; skip entry if the 64-bit region length is zero
	mov    ecx, [es:di + 8]
	or     ecx, [es:di + 12]
	jz     .skip_entry
	; this is a valid entry, increase count
	inc    bp
	; TODO verify we're not above 1500 to prevent overflow into boot code
	add    di, 20

	.skip_entry:
	test   ebx, ebx
	jne    short .call

	.end:
	mov    [ram_map_count_addr], bp
	clc
	ret

	.fail:
	stc
	ret

; in case of an error in the boot sector code, paint the screen red and halt
%define vga_color_bright_red 0x0c
error_boot_sector:
mov    ax, vga_segment
mov    es, ax
mov    di, 0x0000
mov    cx, vga_columns * vga_rows
mov    al, vga_color_bright_red
cld
rep    stosb

; disable interrupts
cli
halt_boot_sector:
hlt
jmp    halt_boot_sector

; fill the rest of the boot sector with zeroes
times 510 - ($ - $$) db 0

; boot signature
dw 0xaa55

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; second stage boot loader begins here (loaded @ 0x007e00)
;  it will:
;  - copy the OS image from disk to RAM (and draw the progress bar)
;  - switch the CPU to Protected mode
;  - switch the CPU to Long mode

call   draw_progress_bar_frame
call   copy_os_image_to_memory
call   check_cpuid_supported
call   check_long_mode_supported
; we jump to the protected mode without any intention of going back
jmp    switch_to_protected_mode

draw_progress_bar_frame:
	; (60, 147) ... (260, 147)
	%define vga_color_gray 0x07
	mov    ax, vga_segment
	mov    es, ax
	mov    di, 147 * vga_columns + 60
	mov    cx, 260 - 60
	mov    al, vga_color_gray
	rep    stosb

	; (60, 153) ... (260, 153)
	mov    di, 153 * vga_columns + 60
	mov    cx, 260 - 60 + 1
	rep    stosb

	; (60, 147) ... (60, 153)
	mov    cx, 6
	mov    ax, 153 * vga_columns + 60
	.left_vertical:
	sub    ax, vga_columns
	mov    di, ax
	mov    byte [es:di], vga_color_gray
	loop   .left_vertical

	; (260, 147) ... (260, 153)
	mov    cx, 6
	mov    ax, 153 * vga_columns + 260
	.right_vertical:
	sub    ax, vga_columns
	mov    di, ax
	mov    byte [es:di], vga_color_gray
	loop   .right_vertical

	ret

%macro draw_horizontal_line 3
	; (%1, %3) ... (%2, %3)
	; e.g. (60, 150) ... (80, 150)
	mov    ax, vga_segment
	mov    es, ax
	mov    di, %3 * vga_columns + %1
	mov    cx, %2 - %1
	mov    al, vga_color_gray
	rep    stosb
%endmacro

%macro load_sectors 4
	; (%1, %2, 1) ... (%1, %2, %3) -> %4
	; e.g. (0, 1, 1) ... (0, 1, 18) -> 0x00a000 - 0x00c400 [9216 bytes]
	xor    ax, ax
	mov    es, ax
	; sectors to read count
	mov    al, %3
	; cylinder
	mov    ch, %1
	; head
	mov    dh, %2
	; sector
	mov    cl, 1
	; ES:BX is the buffer address pointer
	mov    bx, %4
	call   read_sectors
	jc     error_boot_sector
%endmacro

; Copy OS image from disk to RAM.
;  We assume high density floppy disk geometry:
;  - 80 tracks (cylinders)
;  - 18 sectors per track
;  - 2 heads
;  Read 48 sectors (24576 bytes) from LBA 19 (0, 1, 1) until
;  LBA 66 (1, 1, 12) to address range 0x00a000 - 0x010000.
;  Check out util/lba2chs.py to see how the calculation is performed.
copy_os_image_to_memory:
	; restore drive number
	mov    word dx, [boot_drive_addr]

	; (0, 1, 1) ... (0, 1, 18) -> 0x00a000 - 0x00c400 [9216 bytes]
	load_sectors 0, 1, 18, 0xa000

	draw_horizontal_line 60, 80, 148
	draw_horizontal_line 60, 80, 149
	draw_horizontal_line 60, 80, 150
	draw_horizontal_line 60, 80, 151
	draw_horizontal_line 60, 80, 152

	; (1, 0, 1) ... (1, 0, 18) -> 0x00c400 - 0x00e800 [9216 bytes]
	load_sectors 1, 0, 18, 0xc400

	draw_horizontal_line 80, 100, 148
	draw_horizontal_line 80, 100, 149
	draw_horizontal_line 80, 100, 150
	draw_horizontal_line 80, 100, 151
	draw_horizontal_line 80, 100, 152

	; (1, 1, 1) ... (1, 1, 12) -> 0x00e800 - 0x010000 [6658 bytes]
	load_sectors 1, 1, 12, 0xe800

	draw_horizontal_line 100, 120, 148
	draw_horizontal_line 100, 120, 149
	draw_horizontal_line 100, 120, 150
	draw_horizontal_line 100, 120, 151
	draw_horizontal_line 100, 120, 152

	ret

; Checks to see if CPUID instruction is supported. If it's not, panic.
check_cpuid_supported:
	; push EFLAGS twice on the stack
	pushfd
	pushfd
	; invert the ID bit
	xor    dword [esp], 0x00200000
	; load the EFLAGS with the inverted ID bit
	popfd
	; push EFLAGS again (ID bit may remain inverted)
	pushfd
	; EAX <- EFLAGS
	pop    eax
	; put into EAX changed bits
	xor    eax, [esp]
	; restore original EFLAGS
	popfd
	; ZF will be 1 if ID bit couldn't be changed, meaning
	; that CPUID is not supported
	and    eax, 0x00200000
	; so it's panic time...
	je     error_boot_sector

	ret

; Checks if long mode is supported. If it's not, panic.
check_long_mode_supported:
	; get the "Largest Extended Function Number"
	mov    eax, 0x80000000
	cpuid
	; if no function > 0x80000000, long mode is not supported
	cmp    eax, 0x80000000
	; so it's panic time...
	jbe    error_boot_sector
	; get "Feature Identifiers"
	mov    eax, 0x80000001
	cpuid
	; test if bit 29 is set ("LM: long mode")
	bt     edx, 29
	; if it's not set, panic...
	jnc    error_boot_sector

	ret

; GDT to map the entire "low" memory (<1 MB)
gdt_start:
; offset 0x00
.null_descriptor:
	dq   0
; offset 0x08; for CS (bytes 0 until 1 048 575)
.code_descriptor:
	; segment limit (bits 0-15)
	dw   0xffff
	; base address (bits 0-15)
	dw   0
	; base address (bits 16-23)
	db   0
	;    P      ... segment Present
	;    | DPL  ... Descriptor Privilege Level
	;    | |  S ... descriptor type (0 = System, 1 = code or data)
	;    | |  | Type field: Code (Execute/Read)
	;    | |  | |
	db   1_00_1_1010b
	;    G         ... Granularity (for the segment limit field)
	;    | D/B     ... Default operation size
	;    | | reserved, always set to 0
	;    | | | AVL ... AVaiLable for system use
	;    | | | | segment limit (bits 16-19)
	;    | | | | |
	db   0_1_0_0_1111b
	; base address (bits 24-31)
	db   0
; offset 0x10; for DS, SS, ES, FS, GS (bytes 0 until 1 048 575)
.data_descriptor:
	; segment limit (bits 0-15)
	dw   0xffff
	; base address (bits 0-15)
	dw   0
	; base address (bits 16-23)
	db   0
	;    P      ... segment Present
	;    | DPL  ... Descriptor Privilege Level
	;    | |  S ... descriptor type (0 = System, 1 = code or data)
	;    | |  | Type field: Data (Read/Write)
	;    | |  | |
	db   1_00_1_0010b
	;    G         ... Granularity (for the segment limit field)
	;    | D/B     ... Default operation size
	;    | | reserved, always set to 0
	;    | | | AVL ... AVaiLable for system use
	;    | | | | segment limit (bits 16-19)
	;    | | | | |
	db   0_1_0_0_1111b
	; base address (bits 24-31)
	db   0
gdt_end:

initial_gdt:
	dw   gdt_end - gdt_start - 1
	dd   gdt_start

switch_to_protected_mode:
; disable interrupts
cli
; load the GDT
lgdt   [initial_gdt]

mov    eax, cr0
; set PE (Protection Enable)
or     al, 1
mov    cr0, eax
; set CS to 0x08 (see GDT)
jmp    0x08:protected_mode_entry

%macro pmode_draw_horizontal_line 3
	; (%1, %3) ... (%2, %3)
	; e.g. (60, 150) ... (80, 150)
	mov    edi, 0xa0000 + %3 * vga_columns + %1
	mov    ecx, %2 - %1
	mov    al, vga_color_gray
	rep    stosb
%endmacro

; 32-bit code
[bits 32]
protected_mode_entry:

; set DS, SS, ES, FS, GS to 0x10 (see GDT)
mov    ax, 0x10
mov    ds, ax
mov    ss, ax
mov    es, ax
mov    fs, ax
mov    gs, ax

; reset the stack
mov    ebp, 0x1000
mov    esp, ebp

; update the progress bar
pmode_draw_horizontal_line 120, 140, 148
pmode_draw_horizontal_line 120, 140, 149
pmode_draw_horizontal_line 120, 140, 150
pmode_draw_horizontal_line 120, 140, 151
pmode_draw_horizontal_line 120, 140, 152

; perform identity mapping on the first 1 MB
%define page_tables_base 0x10000
; page_tables_base is the address of PML4
mov    edi, page_tables_base
mov    ecx, 4096
xor    eax, eax
; clear 4096 * 4 bytes for the tables
rep    stosd

%define page_entry_flags 00000000_00000000_00000000_00000011b
;                                                         ||
;                                         Present bit ... P|
;                                      Read/Write bit ...  W

; Page Map Level 4
mov    edi, page_tables_base
lea    eax, [edi + 0x1000]
or     eax, page_entry_flags
; PML4T[0] -> PDPT
mov    dword [edi], eax

; Page Directory Pointer Table
lea    eax, [edi + 0x2000]
or     eax, page_entry_flags
; PDPT[0] -> PDT
mov    dword [edi + 0x1000], eax

; Page Directory
lea    eax, [edi + 0x3000]
or     eax, page_entry_flags
; PDT[0] -> PT
mov    dword [edi + 0x2000], eax

; Page Table
lea    edi, [edi + 0x3000]
mov    eax, page_entry_flags
next_page_table_entry:
mov    [edi], eax
add    eax, 0x1000
add    edi, 8
cmp    eax, 0x100000
jb     short next_page_table_entry

; current memory map:
;  0x0500 -  0x1000 ... stack
;  0x2000 -  0x2002 ... BIOS boot drive identifier
;  0x2003 -  0x2004 ... memory map entry count
;  0x2005 -  0x7bff ... memory map entries (max 1500 entries)
;  0x7c00 - 0x10000 ... boot loader with kernel image (code and data)
; 0x10000 - 0x14000 ... paging tables

mov    eax, cr4
; enable physical-address (PAE) extensions by setting CR4.PAE bit
bts    eax, 5
mov    cr4, eax

; load CR3 with the address of PML4
mov    eax, page_tables_base
mov    cr3, eax

; enable long mode
mov    ecx, 0xc0000080
; 0xc0000080 = EFER MSR number
rdmsr
; set EFER.LME bit
bts    eax, 8
wrmsr

; enable paging to activate long mode
mov    eax, cr0
bts    eax, 31
mov    cr0, eax

jmp    0x08:long_mode_entry

%macro lmode_draw_horizontal_line 3
	mov    rdi, 0xa0000 + %3 * vga_columns + %1
	mov    rcx, %2 - %1
	mov    al, vga_color_gray
	rep    stosb
%endmacro

; 64-bit code
[bits 64]
long_mode_entry:

lmode_draw_horizontal_line 140, 150, 148
lmode_draw_horizontal_line 140, 150, 149
lmode_draw_horizontal_line 140, 150, 150
lmode_draw_horizontal_line 140, 150, 151
lmode_draw_horizontal_line 140, 150, 152

lmode_halt:
hlt
jmp    lmode_halt
