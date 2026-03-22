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
	.file	"sqrt_teensy_check.cpp"
	.text
	.align	2
	.global	_Z12project_sqrtf
	.thumb
	.thumb_func
	.type	_Z12project_sqrtf, %function
_Z12project_sqrtf:
	.fnstart
.LFB297:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.syntax unified
@ 800 "c:\users\thene\source\repos\micromouse2025\mazemap\mazemap\defines.h" 1
	vsqrt.f32 s0, s0
@ 0 "" 2
	.thumb
	.syntax unified
	bx	lr
	.cantunwind
	.fnend
	.size	_Z12project_sqrtf, .-_Z12project_sqrtf
	.align	2
	.global	_Z10vector_magv
	.thumb
	.thumb_func
	.type	_Z10vector_magv, %function
_Z10vector_magv:
	.fnstart
.LFB298:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	vmov.f32	s0, #2.5e+1
	.syntax unified
@ 800 "c:\users\thene\source\repos\micromouse2025\mazemap\mazemap\defines.h" 1
	vsqrt.f32 s0, s0
@ 0 "" 2
	.thumb
	.syntax unified
	bx	lr
	.cantunwind
	.fnend
	.size	_Z10vector_magv, .-_Z10vector_magv
	.ident	"GCC: (GNU Tools for ARM Embedded Processors) 5.4.1 20160919 (release) [ARM/embedded-5-branch revision 240496]"
