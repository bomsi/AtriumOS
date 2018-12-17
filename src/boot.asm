; Boot loader for Atrium OS

; Copyright (c) 2017-2018 Mislav Bozicevic
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

; Assemble with:
; nasm boot.asm -fbin -o atrium.flp

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
mov ax, 0xa000
mov es, ax
mov di, 0x0000
; 320 x 200 = 64000 bytes
mov cx, 0xfa00
; dark gray
mov al, 0x08
cld
rep stosb

; restore drive number
mov word dx, [boot_drive_addr]

; reset disk system
xor ax, ax
mov es, ax
mov ah, 0x00
int 0x13
or ah, ah
jnz error_boot_sector

; restore drive number
mov word dx, [boot_drive_addr]

; read the second stage of boot code from disk
;  (0, 0, 2) ... (0, 0, 18) -> 0x007e00 - 0x009d8a [8074 bytes]
%define boot_code_second_step_addr 0x7e00
xor ax, ax
mov es, ax
mov ah, 0x02
mov al, 17
mov ch, 0x00
mov cl, 0x02
mov dh, 0x00
mov bx, boot_code_second_step_addr
int 0x13
or ah, ah
jnz error_boot_sector

; obtain the RAM map
call detect_memory
jc error_boot_sector

; jump to the second stage of boot code
jmp 0:boot_code_second_step_addr

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
error_boot_sector:
mov ax, 0xa000
mov es, ax
mov di, 0x0000
mov cx, 0xfa00
; bright red
mov al, 0x0c
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

; second stage boot loader begins here (loaded @ 0x007e00)
nop

; TODO switch to protected mode

jmp halt_boot_sector

; fill the rest with zeroes
times 1474560 - ($ - $$) db 0