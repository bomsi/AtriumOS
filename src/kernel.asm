; AtriumOS kernel

; Copyright (c) 2017-2021 Mislav Bozicevic
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

[bits 64]

; Current memory map (prepared by the boot loader):
;  0x0500 -  0x1000 ... stack
;  0x2000 -  0x2002 ... BIOS boot drive identifier
;  0x2003 -  0x2004 ... memory map entry count
;  0x2005 -  0x7bff ... memory map entries (max 1500 entries)
;  0x7c00 - 0x10000 ... boot loader with kernel image (code and data)
; 0x10000 - 0x14000 ... paging tables
; First 1 MB of RAM is identity mapped.

kernel_entry:

; set registers we intend to use to known values
xor    rax, rax
xor    rbx, rbx
xor    rcx, rcx
xor    rdx, rdx
xor    rdi, rdi
xor    rsi, rsi
xor    r8, r8
xor    r9, r9
xor    r10, r10
xor    r11, r11
xor    r12, r12
xor    r13, r13
xor    r14, r14
xor    r15, r15

mov    rbp, 0x1000
mov    rsp, rbp

call   draw_os_name

; TODO initialize 64-bit version of LDT, IDT, TSS

kernel_end:
hlt
jmp    short kernel_end

draw_os_name:
	pushfq
	push   r8
	push   r9
	push   r10
	push   r11
	push   r12

	mov    r8, [bitmap.A]
	mov    r9, 120
	mov    r10, 80
	mov    r11, 0x07
	mov    r12, 0x08
	call   draw
	mov    r8, [bitmap.t]
	add    r9, 9
	call   draw
	mov    r8, [bitmap.r]
	add    r9, 9
	call   draw
	mov    r8, [bitmap.i]
	add    r9, 9
	call   draw
	mov    r8, [bitmap.u]
	add    r9, 9
	call   draw
	mov    r8, [bitmap.m]
	add    r9, 9
	call   draw
	mov    r8, [bitmap.O]
	add    r9, 9
	call   draw
	mov    r8, [bitmap.S]
	add    r9, 9
	call   draw

	pop    r12
	pop    r11
	pop    r10
	pop    r9
	pop    r8
	popfq
	ret

