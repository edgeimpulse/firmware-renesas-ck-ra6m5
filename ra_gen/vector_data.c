/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI3 RXI (Received data full) */
            [1] = sci_uart_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [2] = sci_uart_tei_isr, /* SCI3 TEI (Transmit end) */
            [3] = sci_uart_eri_isr, /* SCI3 ERI (Receive error) */
            [4] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
            [5] = ssi_rxi_isr, /* SSI0 RXI (Receive data full) */
            [6] = ssi_int_isr, /* SSI0 INT (Error interrupt) */
            [7] = gpt_counter_overflow_isr, /* GPT1 COUNTER OVERFLOW (Overflow) */
            [8] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [9] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [10] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [11] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_SCI3_RXI), /* SCI3 RXI (Received data full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_SCI3_TXI), /* SCI3 TXI (Transmit data empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_SCI3_TEI), /* SCI3 TEI (Transmit end) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_SCI3_ERI), /* SCI3 ERI (Receive error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_GPT0_COUNTER_OVERFLOW), /* GPT0 COUNTER OVERFLOW (Overflow) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SSI0_RXI), /* SSI0 RXI (Receive data full) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SSI0_INT), /* SSI0 INT (Error interrupt) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_GPT1_COUNTER_OVERFLOW), /* GPT1 COUNTER OVERFLOW (Overflow) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_IIC0_RXI), /* IIC0 RXI (Receive data full) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_IIC0_TXI), /* IIC0 TXI (Transmit data empty) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_IIC0_TEI), /* IIC0 TEI (Transmit end) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_IIC0_ERI), /* IIC0 ERI (Transfer error) */
        };
        #endif