/******************************************************************************
* File Name:   spi_slave.c
*
* Description: This file contains function definitions for SPI Slave.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "spi_slave.h"
#include "interface.h"
#include "spi_dma.h"


#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE))


/******************************************************************************
* Function Name: init_slave
*******************************************************************************
*
* Summary:      This function initializes the SPI Slave based on the
*               configuration done in design.modus file.
*
* Parameters:   None
*
* Return:       (uint32_t) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32_t init_slave(void)
{
    cy_stc_scb_spi_context_t sSPI_context;
    cy_en_scb_spi_status_t init_status;

    /* Configure the SPI block */
    init_status = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &sSPI_context);

    /* If the initialization fails, return failure status */
    if (init_status != CY_SCB_SPI_SUCCESS)
    {
        return(INIT_FAILURE);
    }

    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(sSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    /* Enable the SPI Slave block */
    Cy_SCB_SPI_Enable(sSPI_HW);

    /* Initialization completed */
    return(INIT_SUCCESS);
}


/******************************************************************************
* Function Name: receive_packet
*******************************************************************************
*
* Summary:      This function transfers data from sSPI RX-FIFO to rxBuffer. The 
*               below function enables channel and DMA block to start descriptor 
*               execution process for rxDMA.
*
* Parameters:   None
*
* Return:       None
*
******************************************************************************/
void receive_packet(void)
{
    /* Enable DMA channel to transfer 12 bytes of data from sSPI RX-FIFO to rxBuffer. */
    Cy_DMA_Channel_Enable(rxDma_HW, rxDma_CHANNEL);
}


#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE)) */
