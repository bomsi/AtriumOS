; Boot loader for AtriumOS

; Copyright (c) 2017-2019 Mislav Bozicevic
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

cpu 486

; 16-bit code
[bits 16]

; we are loaded by BIOS at linear address 0x007c00
[org 0x7c00]

; disable interrupts
cli

; canonicalize CS:IP to a known segment:offset pair
; (some BIOSes actually begin execution at 07c0:0000h instead of 0000:7c00h)
jmp 0:next
next:

; set the segment registers to 0
xor ax, ax
mov ds, ax
mov es, ax
mov fs, ax
mov gs, ax
mov ss, ax

; set the stack pointer (top of the real mode stack)
;  the stack can grow (down) to 0x000500, so max size is 2816 bytes (or 176
;  push operations due to the fact that they are word-sized)
mov sp, 0x1000

; preserve BIOS boot drive number @ boot_drive_addr
%define boot_drive_addr 0x2000
mov bx, boot_drive_addr
mov [bx], dx

; enable interrupts
sti

; assume VGA and initialize it in 320x200 resolution with 256 colors (mode 13h)
mov ah, 0x00
mov al, 0x13
int 0x10

; enable the A20 gate
mov ah, 0x24
mov al, 0x01
int 0x15

; VGA range is A000:0000 - A000:FFFF, so we need to switch the segment
%define vga_segment 0xa000
%define vga_columns 320
%define vga_rows 200
%define vga_color_dark_gray 0x08
mov ax, vga_segment
mov es, ax
mov di, 0x0000
mov cx, vga_columns * vga_rows
mov al, vga_color_dark_gray
cld
rep stosb

; restore drive number
mov word dx, [boot_drive_addr]

; reset disk system
xor ax, ax
mov es, ax
call reset_disk_system
jc error_boot_sector

; restore drive number
mov word dx, [boot_drive_addr]

; read the second stage of boot code from disk
;  (0, 0, 2) ... (0, 0, 18) -> 0x007e00 - 0x009d8a [8704 bytes]
%define boot_code_second_step_addr 0x7e00
xor ax, ax
mov es, ax
; sectors to read count
mov al, 17
; cylinder
mov ch, 0x00
; sector
mov cl, 0x02
; head
mov dh, 0x00
; ES:BX is the buffer address pointer
mov bx, boot_code_second_step_addr
call read_sectors
jc error_boot_sector

; obtain the RAM map
call detect_memory
jc error_boot_sector

; jump to the second stage of boot code
jmp 0:boot_code_second_step_addr

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
	mov si, 3

	.read:
	; read sectors function
	mov ah, 0x02
	int 0x13
	; end on a successful read
	jnc .end
	; decrement the retry counter
	dec si
	; end if maximum retry count was exceeded
	jc .end
	call reset_disk_system
	; retry if reset succeeded, otherwise end
	jnc .read

	.end:
	popa
	ret

; Reset the disk system.
; On success, carry flag will be cleared.
reset_disk_system:
	pusha

	; reset disk system function (AH = 0x00)
	xor ax, ax
	int 0x13

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
	xor ax, ax
	mov es, ax
	mov ax, ram_map_addr
	mov di, ax
	; use E820h BIOS function
	mov eax, 0xe820
	; continuation value (zero for first call)
	xor ebx, ebx
	; signature 'SMAP' used by BIOS to verify the caller is requesting
	; the system map information which will be returned in ES:DI
	mov edx, 0x534d4150
	; buffer size
	mov ecx, 20
	; we use BP for count
	xor bp, bp
	int 0x15
	; carry indicates error here
	jc short .fail
	; restore the signature if BIOS has overwritten it
	mov edx, 0x534d4150
	cmp eax, edx
	jne short .fail
	test ebx, ebx
	; if ebx == 0 list has only one entry (useless)
	je short .fail
	jmp short .body

	.call:
	mov eax, 0xe820
	mov ecx, 20
	int 0x15
	; carry indicates end here
	jc short .end
	; restore the signature if BIOS has overwritten it
	mov edx, 0x534d4150

	.body:
	; skip zero-length entry
	jcxz .skip_entry
	; skip entry if the 64-bit region length is zero
	mov ecx, [es:di + 8]
	or ecx, [es:di + 12]
	jz .skip_entry
	; this is a valid entry, increase count
	inc bp
	add di, 20

	.skip_entry:
	test ebx, ebx
	jne short .call

	.end:
	mov [ram_map_count_addr], bp
	clc
	ret

	.fail:
	stc
	ret

; in case of an error in the boot sector code, paint the screen red and halt
%define vga_color_bright_red 0x0c
error_boot_sector:
mov ax, vga_segment
mov es, ax
mov di, 0x0000
mov cx, vga_columns * vga_rows
mov al, vga_color_bright_red
cld
rep stosb

; disable interrupts
cli
halt_boot_sector:
hlt
jmp halt_boot_sector

; fill the rest of the boot sector with zeroes
times 510 - ($ - $$) db 0

; boot signature
dw 0xaa55

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; second stage boot loader begins here (loaded @ 0x007e00)
;  it will:
;  - copy the OS image from disk to RAM (and draw the progress bar)
;  - switch the CPU to protected mode
;  - relocate the kernel to a high address
;  - jump to kernel entry point

call draw_progress_bar_frame
call copy_os_image_to_memory

; TODO jump to the kernel code start
jmp halt_boot_sector

draw_progress_bar_frame:
	; (60, 140) ... (260, 140)
	%define vga_color_gray 0x07
	mov ax, vga_segment
	mov es, ax
	mov di, 140 * vga_columns + 60
	mov cx, 260 - 60
	mov al, vga_color_gray
	rep stosb

	; (60, 160) ... (260, 160)
	mov di, 160 * vga_columns + 60
	mov cx, 260 - 60 + 1
	rep stosb

	; (60, 140) ... (60, 160)
	mov cx, 20
	mov ax, 160 * vga_columns + 60
	.left_vertical:
	sub ax, vga_columns
	mov di, ax
	mov byte [es:di], vga_color_gray
	loop .left_vertical

	; (260, 140) ... (260, 160)
	mov cx, 20
	mov ax, 160 * vga_columns + 260
	.right_vertical:
	sub ax, vga_columns
	mov di, ax
	mov byte [es:di], vga_color_gray
	loop .right_vertical

	ret

; Copy OS image from disk to RAM.
;  We assume high density floppy disk geometry:
;  - 80 tracks (cylinders)
;  - 18 sectors per track
;  - 2 heads
;  For simplicity, 900 sectors will be read one by one into the address range:
;   0x00009d8b - 0x0007a58b (460800 bytes in total).
;  Sectors read: from LBA 19 (0, 1, 1) until LBA 918 (25, 0, 18).
;  Check out util/lba2chs.py to see how the calculation is performed.
;  Buffer at 0x00001100 - 0x00001300 (512 bytes) is used as temporary storage.
copy_os_image_to_memory:
	; TODO
	ret

; fill the rest of the second stage boot loader with zeroes
times (512 + 8074) - ($ - $$) db 0

