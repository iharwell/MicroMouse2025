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
	.file	"sqrt_asm_check.cpp"
	.text
	.align	2
	.global	_Z8test_asmf
	.thumb
	.thumb_func
	.type	_Z8test_asmf, %function
_Z8test_asmf:
	.fnstart
.LFB0:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.syntax unified
@ 3 "C:\Users\thene\source\repos\MicroMouse2025\codex_verify\sqrt_asm_check.cpp" 1
	vsqrt.f32 s0, s0
@ 0 "" 2
	.thumb
	.syntax unified
	bx	lr
	.cantunwind
	.fnend
	.size	_Z8test_asmf, .-_Z8test_asmf
	.ident	"GCC: (GNU Tools for ARM Embedded Processors) 5.4.1 20160919 (release) [ARM/embedded-5-branch revision 240496]"
