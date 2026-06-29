/* Std_Types.h — AUTOSAR R23-11 Standard Types (simplified) */
#ifndef STD_TYPES_H
#define STD_TYPES_H

#include "Platform_Types.h"

typedef uint8 Std_ReturnType;

#ifndef E_OK
#define E_OK     ((Std_ReturnType)0x00U)
#endif
#ifndef E_NOT_OK
#define E_NOT_OK ((Std_ReturnType)0x01U)
#endif

typedef struct {
    uint16 vendorID;
    uint16 moduleID;
    uint8  sw_major_version;
    uint8  sw_minor_version;
    uint8  sw_patch_version;
} Std_VersionInfoType;

#endif /* STD_TYPES_H */
