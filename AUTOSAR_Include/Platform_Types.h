/* Platform_Types.h — AUTOSAR R23-11 Platform Types (simplified for STM32F407) */
#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

typedef unsigned char    uint8;
typedef unsigned short   uint16;
typedef unsigned int     uint32;
typedef unsigned long long uint64;

typedef signed char      sint8;
typedef signed short     sint16;
typedef signed int       sint32;
typedef signed long long sint64;

typedef float            float32;
typedef double           float64;

typedef unsigned char    boolean;

#ifndef TRUE
#define TRUE  ((boolean)1U)
#endif
#ifndef FALSE
#define FALSE ((boolean)0U)
#endif

#endif /* PLATFORM_TYPES_H */
