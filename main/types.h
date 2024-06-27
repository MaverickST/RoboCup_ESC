/**
 * \file        types.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>
#include <stdbool.h>

typedef union
{
    uint8_t B;
    struct
    {
        uint8_t uc_data : 1; ///< Data received from the UART console
        uint8_t         : 7; ///< Reserved    
    };
} flags_t;

extern volatile flags_t gFlag;

#endif // __TYPES_H__