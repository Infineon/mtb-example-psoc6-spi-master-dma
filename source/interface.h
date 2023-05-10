/******************************************************************************
* File Name:   interface.h
*
* Description: This file contains all the macros used by the source files in this
*               application.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SOURCE_INTERFACE_H_
#define SOURCE_INTERFACE_H_

#include "cy_pdl.h"
#include "cycfg.h"


/* This code example requires two SPI ports, one as master and the other one 
 * as slave. Set the SPI_MODE macro depending on the kit. Some kits support two 
 * SPI ports which you can configure in SPI_MODE_BOTH whereas some support only
 * one port in which case you can configure the kit in either SPI_MODE_MASTER or 
 * SPI_MODE_SLAVE. See README.md to know more on the kit specific configuration.
 */
#define SPI_MODE_BOTH    0
#define SPI_MODE_MASTER  1
#define SPI_MODE_SLAVE   2

#define SPI_MODE SPI_MODE_MASTER


#if (SPI_MODE == SPI_MODE_BOTH)
    #if defined (TARGET_CYW9P62S1_43012EVB_01) || defined (TARGET_CY8CPROTO_062S3_4343W) || \
    defined (TARGET_CY8CKIT_062S4)
        #warning For the current TARGET, this example requires the SPI_MODE macro to be \
                 configured either in SPI_MODE_MASTER or SPI_MODE_SLAVE, not in SPI_MODE_BOTH.
    #endif
#endif

/* Initialization status */
#define INIT_SUCCESS            (0)
#define INIT_FAILURE            (1)

/* Communication status */
#define TRANSFER_COMPLETE       (0)
#define TRANSFER_FAILURE        (1)
#define TRANSFER_IN_PROGRESS    (2)
#define IDLE                    (3)

/* TX Packet Head and Tail */
#define PACKET_SOP              (0x01UL)
#define PACKET_EOP              (0x17UL)

/* Element index in the packet */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)


#endif /* SOURCE_INTERFACE_H_ */
