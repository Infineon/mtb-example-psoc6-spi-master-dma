/******************************************************************************
* File Name:   spi_dma.c
*
* Description: This file contains function definitions for DMA operation.
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

#include "cyhal.h"
#include "spi_dma.h"
#include "interface.h"

/*******************************************************************************
 *                       Macro definitions
 ******************************************************************************/

/* Interrupt priority for RXDMA */
#define RXDMA_INTERRUPT_PRIORITY (7u)

/* Interrupt priority for TXDMA */
#define TXDMA_INTERRUPT_PRIORITY (7u)


/* varibale to check the rx dma transection status */
#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE))

volatile bool rx_dma_done = false;

#endif

/* varibale to check the tx dma transection status */
#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER))

volatile bool tx_dma_done = false;

/******************************************************************************
* Function Name: configure_tx_dma
*******************************************************************************
*
* Summary:      This function configure the transmit DMA block 
*
* Parameters:   tx_buffer
*
* Return:       (uint32_t) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32_t configure_tx_dma(uint32_t* tx_buffer)
{
     cy_en_dma_status_t dma_init_status;
     const cy_stc_sysint_t intTxDma_cfg =
     {
         .intrSrc      = txDma_IRQ,
         .intrPriority = 7u
     };
     /* Initialize descriptor */
     dma_init_status = Cy_DMA_Descriptor_Init(&txDma_Descriptor_0, &txDma_Descriptor_0_config);
     if (dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     dma_init_status = Cy_DMA_Channel_Init(txDma_HW, txDma_CHANNEL, &txDma_channelConfig);
     if (dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     /* Set source and destination for descriptor 1 */
     Cy_DMA_Descriptor_SetSrcAddress(&txDma_Descriptor_0, (uint8_t *)tx_buffer);
     Cy_DMA_Descriptor_SetDstAddress(&txDma_Descriptor_0, (void *)&mSPI_HW->TX_FIFO_WR);

      /* Initialize and enable the interrupt from TxDma */
     cyhal_system_set_isr(txDma_IRQ, txDma_IRQ, TXDMA_INTERRUPT_PRIORITY, &tx_dma_complete);
     NVIC_EnableIRQ((IRQn_Type)intTxDma_cfg.intrSrc);

      /* Enable DMA interrupt source. */
     Cy_DMA_Channel_SetInterruptMask(txDma_HW, txDma_CHANNEL, CY_DMA_INTR_MASK);
     /* Enable DMA block to start descriptor execution process */
     Cy_DMA_Enable(txDma_HW);
     return INIT_SUCCESS;
}

/******************************************************************************
* Function Name: tx_dma_complete
*******************************************************************************
*
* Summary:      This function check the tx DMA status
*
* Parameters:   None
*
* Return:       None
*
******************************************************************************/
void tx_dma_complete(void)
{
     /* Check tx DMA status */
     if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_CHANNEL)) &&
         (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_CHANNEL)))
     {
         /* DMA error occurred while TX operations */
         handle_error();
     }

     tx_dma_done = true;
     /* Clear tx DMA interrupt */
     Cy_DMA_Channel_ClearInterrupt(txDma_HW, txDma_CHANNEL);
}

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER)) */


#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE))

/******************************************************************************
* Function Name: configure_rx_dma
*******************************************************************************
*
* Summary:      This function configure the receive DMA block 
*
* Parameters:   rx_buffer
*
* Return:       (uint32_t) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32_t configure_rx_dma(uint32_t* rx_buffer)
{
     cy_en_dma_status_t dma_init_status;
     const cy_stc_sysint_t intRxDma_cfg =
     {
         .intrSrc      = rxDma_IRQ,
         .intrPriority = 7u
     };
     /* Initialize descriptor */
     dma_init_status = Cy_DMA_Descriptor_Init(&rxDma_Descriptor_0, &rxDma_Descriptor_0_config);
     if (dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     dma_init_status = Cy_DMA_Channel_Init(rxDma_HW, rxDma_CHANNEL, &rxDma_channelConfig);
     if (dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     /* Set source and destination for descriptor 1 */
     Cy_DMA_Descriptor_SetSrcAddress(&rxDma_Descriptor_0, (void *)&sSPI_HW->RX_FIFO_RD);
     Cy_DMA_Descriptor_SetDstAddress(&rxDma_Descriptor_0, (uint8_t *)rx_buffer);

      /* Initialize and enable the interrupt from TxDma */
     cyhal_system_set_isr(rxDma_IRQ, rxDma_IRQ, RXDMA_INTERRUPT_PRIORITY, &rx_dma_complete);
     NVIC_EnableIRQ((IRQn_Type)intRxDma_cfg.intrSrc);

      /* Enable DMA interrupt source. */
     Cy_DMA_Channel_SetInterruptMask(rxDma_HW, rxDma_CHANNEL, CY_DMA_INTR_MASK);
     /* Enable channel and DMA block to start descriptor execution process */
     Cy_DMA_Channel_Enable(rxDma_HW, rxDma_CHANNEL);
     Cy_DMA_Enable(rxDma_HW);
     return INIT_SUCCESS;
}

/******************************************************************************
* Function Name: rx_dma_complete
*******************************************************************************
*
* Summary:      This function check the rx DMA status
*
* Parameters:   None
*
* Return:       None
*
******************************************************************************/
void rx_dma_complete(void)
{
    /* Scenario: Inside the interrupt service routine for block DW0 channel 23: */
    if (CY_DMA_INTR_MASK == Cy_DMA_Channel_GetInterruptStatusMasked(rxDma_HW, rxDma_CHANNEL))
    {
        /* Get the interrupt cause */
        cy_en_dma_intr_cause_t cause = Cy_DMA_Channel_GetStatus(rxDma_HW, rxDma_CHANNEL);
        if (CY_DMA_INTR_CAUSE_COMPLETION != cause)
        {
            /* DMA error occurred while RX operations */
            handle_error();
        }
        else
        {
            rx_dma_done = true;
        }

        /* Clear the interrupt */
        Cy_DMA_Channel_ClearInterrupt(rxDma_HW, rxDma_CHANNEL);
    }

}

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE)) */
