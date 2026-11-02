/*
 * Functions of the C library and of the compiler runtime that PX4 calls the most, but not memcpy()
 * and memset(), which load the ITCM
 */

*libgcc.a:_arm_addsubsf3.o(.text*)
*libgcc.a:_aeabi_ldivmod.o(.text*)
*libgcc.a:_aeabi_uldivmod.o(.text*)
*libgcc.a:_udivmoddi4.o(.text*)

*(.text.sinf .text.cosf .text.tanf .text.asinf .text.acosf .text.atanf .text.atan2f)
*(.text.sqrtf .text.logf .text.expf .text.powf .text.floorf .text.fmodf)
*(.text.__kernel_sinf .text.__kernel_cosf .text.__kernel_tanf .text.__rem_pio2f .text.__math_inexactf)
*(.text.sin .text.cos .text.atan .text.atan2 .text.asin .text.acos .text.cbrt .text.sqrt)
*(.text.__kernel_sin .text.__kernel_cos .text.__rem_pio2 .text.__math_inexact64)
