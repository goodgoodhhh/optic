#ifndef __VOICE_H
#define __VOICE_H
#include <stdint.h>

#define VOICE_SAMPLE_RATE  8000U

typedef struct
{
	uint32_t rx_packets;
	uint32_t lost_packets;
	uint32_t tx_drops;
	uint32_t rx_drops;
	uint32_t underruns;
	uint32_t dma_errors;
	uint16_t input_level; /* 最近 64 点 ADC 峰峰值，单位为 12 位 ADC 码值。 */
	uint8_t adc_ready;
	uint8_t recording;
	uint8_t playing;
	uint8_t receiving; /* 最近 1 秒内收到 CRC 正确的语音包。 */
} Voice_Stats;

void Voice_Init(void);
void Voice_SetEnabled(uint8_t enabled);
uint8_t Voice_StartTransmit(void);
void Voice_Stop(void);
void Voice_Tick1ms(void);
/* 仅由 OOK 驱动在 CRC 校验通过后调用，每包恰好 64 个无符号 PCM 样本。 */
void Voice_ReceivePacket(const uint8_t *samples, uint8_t sequence);
void Voice_GetStats(Voice_Stats *stats);
#endif
