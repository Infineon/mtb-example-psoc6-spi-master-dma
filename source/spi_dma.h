/******************************************************************************
* File Name: spi_dma.h
*
* Description: This file contains function prototypes for DMA operation.
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SOURCE_SPI_DMA_H_
#define SOURCE_SPI_DMA_H_

#include "cy_pdl.h"
#include "cycfg.h"


/******************************************************************************
 * Function Prototypes                                                        *
 ******************************************************************************/

uint32_t configure_tx_dma(uint32_t* txBuffer);
void tx_dma_complete(void);
uint32_t configure_rx_dma(uint32_t* rxBuffer);
void rx_dma_complete(void);
void handle_error(void);


/******************************************************************************
 * Extern Variables                                                           *
 ******************************************************************************/

extern volatile bool tx_dma_done;
extern volatile bool rx_dma_done;


#endif /* SOURCE_SPI_DMA_H_ */
