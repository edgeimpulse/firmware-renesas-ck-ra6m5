/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (12)
#endif
/* ISR prototypes */
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);
void gpt_counter_overflow_isr(void);
void ssi_rxi_isr(void);
void ssi_int_isr(void);
void iic_master_rxi_isr(void);
void iic_master_txi_isr(void);
void iic_master_tei_isr(void);
void iic_master_eri_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_SCI3_RXI ((IRQn_Type) 0) /* SCI3 RXI (Received data full) */
#define SCI3_RXI_IRQn          ((IRQn_Type) 0) /* SCI3 RXI (Received data full) */
#define VECTOR_NUMBER_SCI3_TXI ((IRQn_Type) 1) /* SCI3 TXI (Transmit data empty) */
#define SCI3_TXI_IRQn          ((IRQn_Type) 1) /* SCI3 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI3_TEI ((IRQn_Type) 2) /* SCI3 TEI (Transmit end) */
#define SCI3_TEI_IRQn          ((IRQn_Type) 2) /* SCI3 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI3_ERI ((IRQn_Type) 3) /* SCI3 ERI (Receive error) */
#define SCI3_ERI_IRQn          ((IRQn_Type) 3) /* SCI3 ERI (Receive error) */
#define VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW ((IRQn_Type) 4) /* GPT0 COUNTER OVERFLOW (Overflow) */
#define GPT0_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 4) /* GPT0 COUNTER OVERFLOW (Overflow) */
#define VECTOR_NUMBER_SSI0_RXI ((IRQn_Type) 5) /* SSI0 RXI (Receive data full) */
#define SSI0_RXI_IRQn          ((IRQn_Type) 5) /* SSI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SSI0_INT ((IRQn_Type) 6) /* SSI0 INT (Error interrupt) */
#define SSI0_INT_IRQn          ((IRQn_Type) 6) /* SSI0 INT (Error interrupt) */
#define VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW ((IRQn_Type) 7) /* GPT1 COUNTER OVERFLOW (Overflow) */
#define GPT1_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 7) /* GPT1 COUNTER OVERFLOW (Overflow) */
#define VECTOR_NUMBER_IIC0_RXI ((IRQn_Type) 8) /* IIC0 RXI (Receive data full) */
#define IIC0_RXI_IRQn          ((IRQn_Type) 8) /* IIC0 RXI (Receive data full) */
#define VECTOR_NUMBER_IIC0_TXI ((IRQn_Type) 9) /* IIC0 TXI (Transmit data empty) */
#define IIC0_TXI_IRQn          ((IRQn_Type) 9) /* IIC0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_IIC0_TEI ((IRQn_Type) 10) /* IIC0 TEI (Transmit end) */
#define IIC0_TEI_IRQn          ((IRQn_Type) 10) /* IIC0 TEI (Transmit end) */
#define VECTOR_NUMBER_IIC0_ERI ((IRQn_Type) 11) /* IIC0 ERI (Transfer error) */
#define IIC0_ERI_IRQn          ((IRQn_Type) 11) /* IIC0 ERI (Transfer error) */
#endif /* VECTOR_DATA_H */
