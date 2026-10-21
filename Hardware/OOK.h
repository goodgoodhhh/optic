#ifndef __OOK_H
#define __OOK_H
#include <stdint.h>

#define OOK_BAUD_RATE       115200U
#define OOK_MAX_PAYLOAD     32U
#define OOK_AUDIO_SAMPLES   64U
/* 这套激光接收模块有光输出低：在逐位恢复前反相，不对完整字节取反。 */
#define OOK_RX_INVERTED     1U

typedef struct
{
	uint32_t crc_errors;
	uint32_t uart_errors; /* 保留字段名：现计软件 8N1 起始/停止位错误。 */
	uint32_t tx_drops;
	uint32_t rx_overruns; /* 采样缓冲未及时处理，丢弃残帧并重新同步。 */
	uint32_t rx_dma_errors;
} OOK_Stats;

/* PA2：USART2 TX；PA3：普通 GPIO，由 TIM5 + DMA 采样并软件恢复 8N1。 */
void OOK_Init(void);
void OOK_SetDigitalEnabled(uint8_t enabled);
void OOK_SetTransmitAllowed(uint8_t allowed);
uint8_t OOK_IsSendMode(void);
/* 非阻塞入队；返回 1 为接受，0 为方向不符、队满或参数无效。 */
uint8_t OOK_Send(const uint8_t *data, uint8_t length);
uint8_t OOK_SendAudio(const uint8_t *samples, uint8_t sequence);
void OOK_Tick1ms(void);
uint8_t OOK_TakeMessage(char *buffer, uint8_t buffer_size);
uint8_t OOK_IsTransmitting(void);
void OOK_GetStats(OOK_Stats *stats);
#endif
