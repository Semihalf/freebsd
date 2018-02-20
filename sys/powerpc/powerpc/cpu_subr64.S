/*-
 * Copyright (c) 2017-2018 QCM Technologies.
 * Copyright (c) 2017-2018 Semihalf.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include "assym.s"

#include <machine/asm.h>

	.globl	CNAME(power_save_sequence)
	.p2align 3
ENTRY(enter_idle_powerx)
	mfsprg0	%r3			/* Get the pcpu pointer */
	ld	%r3,PC_CURTHREAD(%r3)	/* Get current thread */
	ld	%r3,TD_PCB(%r3)		/* Get PCB of current thread */
	std	%r12,PCB_CONTEXT(%r3)	/* Save the non-volatile GP regs. */
	std	%r13,PCB_CONTEXT+1*8(%r3)
	std	%r14,PCB_CONTEXT+2*8(%r3)
	std	%r15,PCB_CONTEXT+3*8(%r3)
	std	%r16,PCB_CONTEXT+4*8(%r3)
	std	%r17,PCB_CONTEXT+5*8(%r3)
	std	%r18,PCB_CONTEXT+6*8(%r3)
	std	%r19,PCB_CONTEXT+7*8(%r3)
	std	%r20,PCB_CONTEXT+8*8(%r3)
	std	%r21,PCB_CONTEXT+9*8(%r3)
	std	%r22,PCB_CONTEXT+10*8(%r3)
	std	%r23,PCB_CONTEXT+11*8(%r3)
	std	%r24,PCB_CONTEXT+12*8(%r3)
	std	%r25,PCB_CONTEXT+13*8(%r3)
	std	%r26,PCB_CONTEXT+14*8(%r3)
	std	%r27,PCB_CONTEXT+15*8(%r3)
	std	%r28,PCB_CONTEXT+16*8(%r3)
	std	%r29,PCB_CONTEXT+17*8(%r3)
	std	%r30,PCB_CONTEXT+18*8(%r3)
	std	%r31,PCB_CONTEXT+19*8(%r3)

	mfcr	%r16			/* Save the condition register */
	std	%r16,PCB_CR(%r3)
	mflr	%r16			/* Save the link register */
	std	%r16,PCB_LR(%r3)
	std	%r1,PCB_SP(%r3)		/* Save the stack pointer */
	std	%r2,PCB_TOC(%r3)	/* Save the TOC pointer */

	/* Set where we want to jump */
	bl	1f
	.llong power_save_sequence	/* Remember about 8 byte alignment */
1:	mflr	%r3
	ld	%r3,0(%r3)
	mtsrr0	%r3

	/* Set MSR */
	li	%r3,0
	ori	%r3,%r3,(PSL_ME | PSL_RI)
	li	%r8,0x9			/* PSL_SF and PSL_HV */
	insrdi	%r3,%r8,4,0
	mtsrr1	%r3

	rfid

	.p2align 2
CNAME(power_save_sequence):
	bl	1f
	.llong	0x0			/* Playground for power-save sequence */
1:	mflr	%r3

	/* Start power-save sequence */
	std	%r2,0(%r3)
	ptesync
	ld	%r2,0(%r3)
2:	cmpd	%r2,%r2
	bne	2b
	nap
	b	.

ENTRY(memcpy_vector)
	/* Set PCB_VEC flag in case of context switch */
	mfsprg0	%r7			/* Get the pcpu pointer */
	cmpdi	%r7,0			/* SPRG0 is 0 if pcpu not available */
	beq	align_address

	ld	%r7,PC_CURTHREAD(%r7)
	ld	%r7,TD_PCB(%r7)

	lwz	%r8,PCB_FLAGS(%r7)
	ori	%r8,%r8,PCB_VEC
	stw	%r8,PCB_FLAGS(%r7)
	sync

align_address:
	neg	%r0,%r3		/* r0 = ~r3 + 1 */

	/*
	 * Aligning address can't copy more data than we need.
	 * Turncate aligning mask to be not higher than length
	 */
	li	%r7,0x1F
1:	srdi	%r7,%r7,1
	and	%r0,%r7,%r0
	cmpd	%r0,%r5
	bgt	1b

align_address_16:
	clrldi	%r0,%r0,(64-4)
	mtxer	%r0

	/*
	 * lswx instruction loads n=XER[57:63] bytes to ceil(n/4)
	 * registers starting from (in this case) r6
	 * It works only in big endian mode
	 */
	lswx	%r6,0,%r4
	add	%r4,%r4,%r0
	stswx	%r6,0,%r3
	add	%r3,%r3,%r0

	sub	%r5,%r5,%r0
	cmpdi	%r5,16
	blt	copy_tail_15

align_address_128:
	neg	%r0,%r3		/* r0 = ~r3 + 1 */

	li	%r7,0xFF
1:	srdi	%r7,%r7,1
	and	%r0,%r7,%r0
	cmpd	%r0,%r5
	bgt	1b

	mtocrf	0x02,%r0	/* Copy bits 56:59 from r0 to cr6 field */
	clrldi	%r0,%r0,(64-7)

	/* Load some constants for use with lvx/stvx */
	li	%r6,16
	li	%r7,32
	li	%r8,48

align_address_128_vmx:
	/* Align to 32 byte */
	bf	59-32,1f
	lvx	%v0,0,%r4
	addi	%r4,%r4,16
	stvx	%v0,0,%r3
	addi	%r3,%r3,16

	/* Align to 64 byte */
1:	bf	58-32,2f
	lvx	%v0,0,%r4
	lvx	%v1,%r6,%r4
	addi	%r4,%r4,32
	stvx	%v0,0,%r3
	stvx	%v1,%r6,%r3
	addi	%r3,%r3,32

	/* Align to 128 byte */
2:	bf	57-32,3f
	lvx	%v0,0,%r4
	lvx	%v1,%r6,%r4
	lvx	%v2,%r7,%r4
	lvx	%v3,%r8,%r4
	addi	%r4,%r4,64
	stvx	%v0,0,%r3
	stvx	%v1,%r6,%r3
	stvx	%v2,%r7,%r3
	stvx	%v3,%r8,%r3
	addi	%r3,%r3,64

	/* Subtract number of copied bytes */
3:	sub	%r5,%r5,%r0

copy_loop:
	/* Check if we have to copy at least 128 bytes */
	cmpdi	%r5,128
	blt	copy_tail

	li	%r9,64
	li	%r10,80
	li	%r11,96
	li	%r12,112

	/* Copy 128 bytes at once */
	srdi	%r0,%r5,7
	mtctr	%r0
1:	lvx	%v0,0,%r4
	lvx	%v1,%r6,%r4
	lvx	%v2,%r7,%r4
	lvx	%v3,%r8,%r4
	lvx	%v4,%r9,%r4
	lvx	%v5,%r10,%r4
	lvx	%v6,%r11,%r4
	lvx	%v7,%r12,%r4
	addi	%r4,%r4,128
	stvx	%v0,0,%r3
	stvx	%v1,%r6,%r3
	stvx	%v2,%r7,%r3
	stvx	%v3,%r8,%r3
	stvx	%v4,%r9,%r3
	stvx	%v5,%r10,%r3
	stvx	%v6,%r11,%r3
	stvx	%v7,%r12,%r3
	addi	%r3,%r3,128
	bdnz+	1b

copy_tail:
	/* At most 127 bytes left */
	clrldi	%r5,%r5,(64-7)	/* Calculate remainder */
	mtocrf	0x02,%r5

	bf	57-32,1f
	lvx	%v0,0,%r4
	lvx	%v1,%r6,%r4
	lvx	%v2,%r7,%r4
	lvx	%v3,%r8,%r4
	addi	%r4,%r4,64
	stvx	%v0,0,%r3
	stvx	%v1,%r6,%r3
	stvx	%v2,%r7,%r3
	stvx	%v3,%r8,%r3
	addi	%r3,%r3,64

	/* At most 63 bytes left */
1:	bf	58-32,2f
	lvx	%v0,0,%r4
	lvx	%v1,%r6,%r4
	addi	%r4,%r4,32
	stvx	%v0,0,%r3
	stvx	%v1,%r6,%r3
	addi	%r3,%r3,32

	/* At most 31 bytes left */
2:	bf	59-32,copy_tail_15
	lvx	%v0,0,%r4
	addi	%r4,%r4,16
	stvx	%v0,0,%r3
	addi	%r3,%r3,16

copy_tail_15:
	/* At most 15 bytes left */
	clrldi	%r5,%r5,(64-4)	/* Calculate remainder */
	mtxer	%r5

	lswx	%r6,0,%r4
	add	%r4,%r4,%r5
	stswx	%r6,0,%r3
	add	%r3,%r3,%r5

copy_cleanup:
	/* Now we don't need vectors to be saved */
	mfsprg0	%r7			/* Get the pcpu pointer */
	cmpdi	%r7,0			/* SPRG0 is 0 if pcpu not available */
	beq	1f

	ld	%r7,PC_CURTHREAD(%r7)
	ld	%r7,TD_PCB(%r7)

	lwz	%r8,PCB_FLAGS(%r7)
	andi.	%r8,%r8,(~PCB_VEC)@l
	stw	%r8,PCB_FLAGS(%r7)
	sync

1:	blr
