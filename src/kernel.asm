; AtriumOS kernel

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
xor    rbp, rbp
mov    rsp, 0x1000
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

; TODO initialize 64-bit version of GDT, LDT, IDT, TSS.

kernel_end:
hlt
jmp    short kernel_end

; fill the rest with zeroes
times 1474560 - ($ - $$) db 0
