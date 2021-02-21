; AtriumOS display routines

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

; Example for 'A':
;   0 1 2 3 4 5 6 7
; 0 . . . X X . . . = 0x18
; 1 . . X X X X . . = 0x3c
; 2 . X X . . X X . = 0x66
; 3 . X X . . X X . = 0x66
; 4 . X X X X X X . = 0x7e
; 5 . X X X X X X . = 0x7e
; 6 . X X . . X X . = 0x66
; 7 . X X . . X X . = 0x66
; Little-endian 64-bit:
;  0x66667e7e66663c18

bitmap:
.A: dq 0x66667e7e66663c18
.t: dq 0x183c0c0c3e3e0c0c
.r: dq 0x0c0c0c3c7c6c0000
.i: dq 0x1818181818001818
.u: dq 0xbce6c2c2c2c20000
.m: dq 0x5656567e7e560000
.O: dq 0x187e664242667e18
.S: dq 0x7ceec0f80e06ee7c

; Draws the 8x8 bitmap.
; Input:
;  R8  - bitmap to draw
;  R9  - x coordinate (0 ... 319 - 8, screen width)
;  R10 - y coordinate (0 ... 199 - 8, screen hight)
;  R11 - foreground color (0 ... 255)
;  R12 - background color (0 ... 255)
draw:
        ; save environment
        pushfq
        push   r8
        push   r9
        push   r10
        push   r11
        push   r12
        push   r13
	push   r14
	push   r15
	push   rdx
        push   rcx
        push   rax
        push   rdi
        
        ; normalize parameters to expected range
	cmp    r9, 311
	jbe    short .r9_ok
	mov    r9, 311
	.r9_ok:
	cmp    r10, 191
	jbe    short .r10_ok
	mov    r10, 191
	.r10_ok:
	mov    rax, 0xff
	and    r11, rax
        and    r12, rax
	
	; r13 is used as the indicator which bit to test
        xor    r13, r13
	; rdx is added to y, as we draw row by row
	xor    rdx, rdx
        mov    rcx, 64
        .test_color:
        ; foreground or background color?
        bt     r8, r13
        jc     short .foreground
        mov    rax, r12
        jmp    short .draw_pixel
        .foreground:
        mov    rax, r11
        jmp    short .draw_pixel
        .draw_pixel:
	; rdi = 0xa0000 + y * 320 + x
	;     = 0xa0000 + y * 256 + y * 64 + x
	;     = 0xa0000 + y << 8  + y << 6 + x
	mov    rdi, 0xa0000
	mov    r14, r10
	add    r14, rdx
	shl    r14, 8
	add    rdi, r14
	mov    r14, r10
	add    r14, rdx
	shl    r14, 6
	add    rdi, r14
	add    rdi, r9
	; r13 % 8 is added to x, as we draw column by column
	mov    r15, r13
	and    r15, 0x7
	add    rdi, r15
        stosb
        inc    r13
	mov    r15, r13
	and    r15, 0x7
	; if r13 % 8 == 0, increment y
	test   r15, r15
	jnz    short .y_unchanged
	inc    rdx
	.y_unchanged:
        loop   .test_color
        
	; restore environment
        pop    rdi
        pop    rax
        pop    rcx
	pop    rdx
	pop    r15
	pop    r14
        pop    r13
        pop    r12
        pop    r11
        pop    r10
        pop    r9
        pop    r8
        popfq
        ret

