	.syntax unified
	.cpu cortex-m7
	.eabi_attribute 28, 1
	.fpu fpv5-d16
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 2
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.thumb
	.syntax unified
	.file	"sqrt_builtin_check.cpp"
	.text
	.align	2
	.global	_Z12test_builtinf
	.thumb
	.thumb_func
	.type	_Z12test_builtinf, %function
_Z12test_builtinf:
	.fnstart
.LFB0:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	vsqrt.f32	s15, s0
	vcmp.f32	s15, s15
	vmrs	APSR_nzcv, FPSCR
	bne	.L6
	vmov.f32	s0, s15
	bx	lr
.L6:
	push	{r3, lr}
	bl	sqrtf
	pop	{r3, pc}
	.cantunwind
	.fnend
	.size	_Z12test_builtinf, .-_Z12test_builtinf
	.align	2
	.global	_Z8test_stdf
	.thumb
	.thumb_func
	.type	_Z8test_stdf, %function
_Z8test_stdf:
	.fnstart
.LFB1:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	vsqrt.f32	s15, s0
	vcmp.f32	s15, s15
	vmrs	APSR_nzcv, FPSCR
	bne	.L11
	vmov.f32	s0, s15
	bx	lr
.L11:
	push	{r3, lr}
	.save {r3, lr}
	bl	sqrtf
	pop	{r3, pc}
	.fnend
	.size	_Z8test_stdf, .-_Z8test_stdf
	.ident	"GCC: (GNU Tools for ARM Embedded Processors) 5.4.1 20160919 (release) [ARM/embedded-5-branch revision 240496]"
