/**
 * @file compiler_port.h
 * @brief 编译器兼容宏（内联等基础能力）。
 */
#ifndef __COMPILER_PORT_H__
#define __COMPILER_PORT_H__

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define APP_STATIC_INLINE static __inline
#elif defined(__GNUC__)
#define APP_STATIC_INLINE static inline
#else
#define APP_STATIC_INLINE static inline
#endif

#endif /* __COMPILER_PORT_H__ */
