/******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates SPI communication between two
* SCB blocks of PSoC 6 device (one configured as Master and another as slave).
* Polling method is used on the master side to confirm data transfer completion.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cyhal_gpio_impl.h"
#include "cycfg.h"
#include "interface.h"
#include "spi_dma.h"
#include "spi_master.h"
#include "spi_slave.h"
#include "cybsp_types.h"

/******************************************************************************
 * Macro definitions                                                          *
 ******************************************************************************/

/* Delay between successive SPI Master command transmissions */
#define CMD_DELAY_MS        (1000)  //in milliseconds

/* Number of elements in the transmit and receive buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS  (3UL)
#define SIZE_OF_ELEMENT     (4UL)

/******************************************************************************
 * Function Prototypes                                                        *
 ******************************************************************************/

/* Function to turn ON or OFF the LED based on the SPI Master command. */
void update_led(uint32_t);

/* Function to handle the error */
void handle_error(void);

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary:  1.  Sets up two SPI blocks - one as master and another as slave
*           2.  SPI master sends commands to the slave to turn LED ON or OFF,
*               every one second.
*           3.  SPI master polls for the data transfer completion.
*           4.  When the required number of bytes is transferred, data received
*               by the slave is checked and LED is turned ON or OFF based on the
*               received command.
*
* Parameters:   None
*
* Return:       Int
*
******************************************************************************/
int main(void)
{
    uint32_t status = 0;

    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();


#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER))

    /* Buffer to hold command packet to be sent to the slave by the master */
    uint32_t  tx_buffer[NUMBER_OF_ELEMENTS];

    /* Local command variable */
    uint32_t cmd = CYBSP_LED_STATE_OFF;

    /* Initialize the SPI Master */
    status = init_master();

    if (INIT_FAILURE == status)
    {
        /* NOTE: This function will block the CPU forever */
        handle_error();
    }

    status = configure_tx_dma(tx_buffer);

    if (INIT_FAILURE == status)
    {
        /* NOTE: This function will block the CPU forever */
        handle_error();
    }

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER)) */


#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE))

    /* Buffer to save the received data by the slave */
    uint32_t  rx_buffer[NUMBER_OF_ELEMENTS];

    /* Initialize the SPI Slave */
    status = init_slave();

    if (INIT_FAILURE == status)
    {
        /* NOTE: This function will block the CPU forever */
        handle_error();
    }

    status = configure_rx_dma(rx_buffer);

    if (INIT_FAILURE == status)
    {
        /* NOTE: This function will block the CPU forever */
        handle_error();
    }

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE)) */


    /* Enable global interrupt */
    __enable_irq();
 
    for (;;)
    {

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER))

        /* Toggle the current LED state */
        cmd = (cmd == CYBSP_LED_STATE_ON) ? CYBSP_LED_STATE_OFF : CYBSP_LED_STATE_ON;

        /* Form the command packet */
        tx_buffer[PACKET_SOP_POS] = PACKET_SOP;
        tx_buffer[PACKET_CMD_POS] = cmd;
        tx_buffer[PACKET_EOP_POS] = PACKET_EOP;

        /* Pass the command packet to the master along with the number of bytes to be
         * sent to the slave.*/
        send_packet();

        /* Wait until master complete the transfer */
        while (false == tx_dma_done) {}
        tx_dma_done = false;

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER)) */


            /* The below code is for slave function. It is implemented in this same code
             * example so that the master function can be tested without the need of one
             * more kit, for kits with 2XSPI ports available. */
#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE))

        if(rx_dma_done)
        {
            /* Check start and end of packet markers */
            if ((rx_buffer[PACKET_SOP_POS] == PACKET_SOP) && (rx_buffer[PACKET_EOP_POS] == PACKET_EOP))
            {
                /* Communication succeeded. Update the LED. */
                update_led(rx_buffer[PACKET_CMD_POS]);
            }
            else
            {
                /* Data was not received correctly and hence Communication failed */
                handle_error();
            }

            rx_dma_done = false;
            /* Get the bytes received by the slave */
            receive_packet();
        }

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_SLAVE)) */

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_MASTER))

        /* Give delay before initiating the next command */
        cyhal_system_delay_ms(CMD_DELAY_MS);
#endif

    }
}

/******************************************************************************
* Function Name: update_led
*******************************************************************************
*
* Summary:      This function updates the LED based on the command received by
*               the SPI Slave from Master.
*
* Parameters:   (uint32_t) led_cmd - command to turn LED ON or OFF
*
* Return:       None
*
******************************************************************************/
void update_led(uint32_t led_cmd)
{
    /* Control the LED. Note that the LED on the supported kits is in active low
       connection. */
    if (CYBSP_LED_STATE_ON == led_cmd)
    {
        /* Turn ON the LED */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
    }
    if (CYBSP_LED_STATE_OFF == led_cmd)
    {
        /* Turn OFF the LED */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
    }
}

/******************************************************************************
* Function Name: handle_error
*******************************************************************************
*
* Summary:      This is a blocking function. It disables the interrupt and waits
*               in an infinite loop. This function is called when an error is
*               encountered during initialization of the blocks or during
*               SPI communication.
*
* Parameters:   None
*
* Return:       None
*
******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}

}
/* [] END OF FILE */
