#ifndef COMPILER_ATTRIBUTES_H_
#define COMPILER_ATTRIBUTES_H_

#ifdef __GNUC__
#define __ITCM_FUNC __attribute__((section(".itcm_text")))
#define __ALWAYS_INLINE __attribute__((always_inline)) inline
#define __NOCACHE_DMA __attribute__((section(".nocache_dma")))
#else
#define __ITCM_FUNC
#define __ALWAYS_INLINE inline
#define __WEAK
#endif

#endif /* COMPILER_ATTRIBUTES_H_ */