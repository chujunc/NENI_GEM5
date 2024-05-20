#if ENABLE_M5_TRIGGER
	.align 3
	.globl m5_dumpreset_stats
	m5_dumpreset_stats:
	.long (((0x01) << 26) | ((16) << 21) | ((17) << 16) | (0x42))
	ret ($26)
#endif
#if ENABLE_M5_CKPTS
	.align 3
	.globl m5_checkpoint
	m5_checkpoint:
	.long (((0x01) << 26) | ((16) << 21) | ((17) << 16) | (0x43))
	ret ($26)
#endif
