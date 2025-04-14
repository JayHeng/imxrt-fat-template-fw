/*
 * Copyright 2019, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_iomuxc.h"
#include "fsl_rgpio.h"
#include "fat.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;
/* The PIN status */
volatile bool g_pinSet = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}

void bsp_led_init(void)
{
    /* GPIO configuration on GPIO */
    rgpio_pin_config_t gpio_config = {
        .pinDirection = kRGPIO_DigitalOutput,
        .outputLogic = 0U,
    };
#if defined(SCH_50577_BGA289_EVK)
    RGPIO_PinInit(RGPIO4, 27U, &gpio_config);
    RGPIO_PinInit(RGPIO4, 26U, &gpio_config);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_27_GPIO4_IO27, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_26_GPIO4_IO26, 0U);
#elif defined(SCH_95302_BGA196_FRDM)
    RGPIO_PinInit(RGPIO3, 7U, &gpio_config);
    RGPIO_PinInit(RGPIO2, 11U, &gpio_config);
    RGPIO_PinInit(RGPIO2, 9U, &gpio_config);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_B1_39_GPIO3_IO07, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_B1_11_GPIO2_IO11, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_B1_09_GPIO2_IO09, 0U);
#endif
}

void bsp_led_info(void)
{
#if defined(SCH_50577_BGA289_EVK)
    PRINTF("D6: GREEN LED, check R32, R35, R38\r\n");
    PRINTF("D7: RED LED  , check R33, R36, R39\r\n");
#elif defined(SCH_95302_BGA196_FRDM)
    PRINTF("D4: RGB LED, blue - check R41, R53, R48; green - check R32, R54, R49; red - check R37, R52, R47\r\n");
#endif
}

void bsp_led_toggle(void)
{
#if defined(SCH_50577_BGA289_EVK)
    RGPIO_TogglePinsOutput(RGPIO4, 1UL << 27);
    RGPIO_TogglePinsOutput(RGPIO4, 1UL << 26);
#elif defined(SCH_95302_BGA196_FRDM)
    static uint8_t temp_ctl = 0;
    if (temp_ctl <= 1)
    {
        RGPIO_TogglePinsOutput(RGPIO3, 1UL << 7);
        RGPIO_PinWrite(RGPIO2, 11, 0);
        RGPIO_PinWrite(RGPIO2, 9, 0);
    }
    else if (temp_ctl <= 3)
    {
        RGPIO_PinWrite(RGPIO3, 7, 0);
        RGPIO_TogglePinsOutput(RGPIO2, 1UL << 11);
        RGPIO_PinWrite(RGPIO2, 9, 0);
    }
    else if (temp_ctl <= 5)
    {
        RGPIO_PinWrite(RGPIO3, 7, 0);
        RGPIO_PinWrite(RGPIO2, 11, 0);
        RGPIO_TogglePinsOutput(RGPIO2, 1UL << 9);
    }
    temp_ctl++;
    temp_ctl %= 6;
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Board pin init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Update the core clock */
    SystemCoreClockUpdate();

    FAT_MagicStart(5);

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }

    bsp_led_init();
    bsp_led_info();

    FAT_MagicPass();

    while (1)
    {
        /* Delay 1000 ms */
        SysTick_DelayTicks(1000U);
        bsp_led_toggle();
    }
}
