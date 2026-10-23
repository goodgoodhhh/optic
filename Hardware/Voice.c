#include "stm32f10x.h"
#include "Voice.h"
#include "OOK.h"
#include "Delay.h"

#define AUDIO_HALF       OOK_AUDIO_SAMPLES
#define AUDIO_BUFFER     (AUDIO_HALF * 2U)
#define RX_RING_SIZE     512U
#define RX_RING_MASK     (RX_RING_SIZE - 1U)
#define RX_PREFILL       (AUDIO_HALF * 3U)
#define RX_TIMEOUT_MS    100U
#define RX_NOTICE_MS     1000U
#define DAC_SILENCE      2048U

/* 定时器控制采样时刻，DMA 双半区搬运，OLED 主循环阻塞也不会停采样。
 * PA1: ADC1_IN1，输入必须已调理为以约 1.65 V 为中心的 0~3.3 V 音频。
 * PA4: DAC_OUT1，只接功放的音频输入，不能直接驱动喇叭。 */
static volatile uint16_t adc_buffer[AUDIO_BUFFER];
static uint16_t dac_buffer[AUDIO_BUFFER];
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_read, rx_write;
static volatile uint8_t voice_enabled;
static volatile uint16_t rx_idle_ms;
static volatile uint16_t rx_notice_ms;
static uint8_t tx_sequence, rx_sequence_valid, rx_last_sequence;
static volatile Voice_Stats audio_stats;

/* 下列缓冲/启停辅助函数由临界区内的调用者执行。 */
static void StopCapture(void)
{
	TIM_Cmd(TIM3, DISABLE);
	ADC_ExternalTrigConvCmd(ADC1, DISABLE);
	ADC_DMACmd(ADC1, DISABLE);
	DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_ClearFlag(DMA1_FLAG_GL1);
	audio_stats.recording = 0U;
	audio_stats.input_level = 0U;
}

static void StopPlayback(void)
{
	TIM_Cmd(TIM6, DISABLE);
	DAC_DMACmd(DAC_Channel_1, DISABLE);
	DMA_Cmd(DMA2_Channel3, DISABLE);
	DMA_ClearFlag(DMA2_FLAG_GL3);
	audio_stats.playing = 0U;
	rx_read = rx_write = 0U;
	rx_sequence_valid = 0U;
	rx_idle_ms = 0U;
	/* 触发一次更新，使 DHR 中点真正转入 DOR；仅停定时器会保持末样本。 */
	DAC_SetChannel1Data(DAC_Align_12b_R, DAC_SILENCE);
	TIM_GenerateEvent(TIM6, TIM_EventSource_Update);
}

static void FillDAC(uint16_t offset)
{
	uint16_t i;
	uint8_t empty = 0U;
	for (i = 0U; i < AUDIO_HALF; i++)
	{
		if (rx_read != rx_write)
		{
			/* 8 位无符号 PCM 扩展回 12 位 DAC；128 对应静音中点。 */
			dac_buffer[offset + i] = (uint16_t)((uint16_t)rx_ring[rx_read] << 4);
			rx_read = (rx_read + 1U) & RX_RING_MASK;
		}
		else {dac_buffer[offset + i] = DAC_SILENCE; empty = 1U;}
	}
	if (empty) {audio_stats.underruns++;}
}

static void StartPlayback(void)
{
	FillDAC(0U);
	FillDAC(AUDIO_HALF);
	DMA_SetCurrDataCounter(DMA2_Channel3, AUDIO_BUFFER);
	DMA_ClearFlag(DMA2_FLAG_GL3);
	TIM_SetCounter(TIM6, 0U);
	__DMB();
	audio_stats.playing = 1U;
	DMA_Cmd(DMA2_Channel3, ENABLE);
	DAC_DMACmd(DAC_Channel_1, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
}

void Voice_Init(void)
{
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef timer;
	ADC_InitTypeDef adc;
	DAC_InitTypeDef dac;
	DMA_InitTypeDef dma;
	NVIC_InitTypeDef nvic;
	uint32_t timeout;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM6 | RCC_APB1Periph_DAC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); /* 72/6 = 12 MHz，不超过 ADC 的 14 MHz。 */
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);

	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = 72U - 1U;
	timer.TIM_Period = (1000000U / VOICE_SAMPLE_RATE) - 1U;
	TIM_TimeBaseInit(TIM3, &timer);
	TIM_TimeBaseInit(TIM6, &timer);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

	/* ADC1 由 TIM3 TRGO 触发；F103 不能用 TIM6 TRGO 触发 ADC1 常规组。 */
	ADC_StructInit(&adc);
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_Init(ADC1, &adc);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1U, ADC_SampleTime_55Cycles5);
	ADC_Cmd(ADC1, ENABLE);
	Delay_us(10U);
	ADC_ResetCalibration(ADC1);
	timeout = 1000000U;
	while (ADC_GetResetCalibrationStatus(ADC1) != RESET && timeout != 0U) {timeout--;}
	if (timeout != 0U)
	{
		ADC_StartCalibration(ADC1);
		timeout = 1000000U;
		while (ADC_GetCalibrationStatus(ADC1) != RESET && timeout != 0U) {timeout--;}
		audio_stats.adc_ready = (timeout != 0U);
	}

	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&ADC1->DR;
	dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)adc_buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_BufferSize = AUDIO_BUFFER;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel1, &dma);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, ENABLE);

	DAC_StructInit(&dac);
	dac.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_Init(DAC_Channel_1, &dac);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	/* 高密度 F103 的 DAC1 DMA 请求映射到 DMA2 通道 3。 */
	dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&DAC->DHR12R1;
	dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)dac_buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_Init(DMA2_Channel3, &dma);
	DMA_ITConfig(DMA2_Channel3, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, ENABLE);
	StopPlayback();

	nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2U;
	nvic.NVIC_IRQChannelSubPriority = 0U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = DMA2_Channel3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1U;
	NVIC_Init(&nvic);
}

void Voice_Stop(void)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	StopCapture();
	StopPlayback();
	rx_notice_ms = 0U;
	audio_stats.receiving = 0U;
	__set_PRIMASK(mask);
}

void Voice_SetEnabled(uint8_t enabled)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	voice_enabled = (enabled != 0U);
	StopCapture();
	StopPlayback();
	rx_notice_ms = 0U;
	audio_stats.receiving = 0U;
	__set_PRIMASK(mask);
}

uint8_t Voice_StartTransmit(void)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	if (!voice_enabled || !OOK_IsSendMode() || !audio_stats.adc_ready)
	{
		__set_PRIMASK(mask);
		return 0U;
	}
	StopPlayback();
	StopCapture();
	DMA_SetCurrDataCounter(DMA1_Channel1, AUDIO_BUFFER);
	TIM_SetCounter(TIM3, 0U);
	audio_stats.recording = 1U;
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	__set_PRIMASK(mask);
	return 1U;
}

void Voice_ReceivePacket(const uint8_t *samples, uint8_t sequence)
{
	uint16_t i, queued;
	uint8_t delta;
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	if (!voice_enabled || OOK_IsSendMode()) {__set_PRIMASK(mask); return;}
	rx_idle_ms = 0U;
	audio_stats.rx_packets++;
	/* 即使只收到一包、尚未启动播放，也保留可见的有效接收提示。 */
	rx_notice_ms = RX_NOTICE_MS;
	audio_stats.receiving = 1U;
	if (rx_sequence_valid)
	{
		delta = (uint8_t)(sequence - rx_last_sequence);
		if (delta > 1U && delta < 128U) {audio_stats.lost_packets += delta - 1U;}
	}
	rx_last_sequence = sequence;
	rx_sequence_valid = 1U;
	queued = (rx_write - rx_read) & RX_RING_MASK;
	if (RX_RING_MASK - queued < AUDIO_HALF) {audio_stats.rx_drops++;}
	else
	{
		for (i = 0U; i < AUDIO_HALF; i++)
		{
			rx_ring[rx_write] = samples[i];
			rx_write = (rx_write + 1U) & RX_RING_MASK;
		}
		/* 先缓存 3 包，吸收串口分包与接收时钟的短期抖动。 */
		if (!audio_stats.playing && queued + AUDIO_HALF >= RX_PREFILL) {StartPlayback();}
	}
	__set_PRIMASK(mask);
}

void Voice_Tick1ms(void)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	if (rx_notice_ms != 0U && --rx_notice_ms == 0U)
	{
		audio_stats.receiving = 0U;
	}
	if (voice_enabled && !OOK_IsSendMode() && (audio_stats.playing || rx_sequence_valid))
	{
		if (++rx_idle_ms >= RX_TIMEOUT_MS) {StopPlayback();}
	}
	__set_PRIMASK(mask);
}

void Voice_GetStats(Voice_Stats *stats)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	*stats = audio_stats;
	__set_PRIMASK(mask);
}

/* 每半区 64 点 = 8 ms，转换为 8 位 PCM 后直接交给串口 DMA 发送队列。 */
static void SendCaptureHalf(uint16_t offset)
{
	uint8_t pcm[AUDIO_HALF];
	uint16_t i, value, minimum = 4095U, maximum = 0U;
	for (i = 0U; i < AUDIO_HALF; i++)
	{
		value = adc_buffer[offset + i];
		if (value < minimum) {minimum = value;}
		if (value > maximum) {maximum = value;}
		pcm[i] = (uint8_t)(value >> 4);
	}
	audio_stats.input_level = maximum - minimum;
	if (!OOK_SendAudio(pcm, tx_sequence++)) {audio_stats.tx_drops++;}
}

void DMA1_Channel1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TE1) != RESET)
	{
		audio_stats.dma_errors++;
		StopCapture();
		return;
	}
	if (DMA_GetITStatus(DMA1_IT_HT1) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_HT1);
		if (audio_stats.recording) {SendCaptureHalf(0U);}
	}
	if (DMA_GetITStatus(DMA1_IT_TC1) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		if (audio_stats.recording) {SendCaptureHalf(AUDIO_HALF);}
	}
}

void DMA2_Channel3_IRQHandler(void)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	if (DMA_GetITStatus(DMA2_IT_TE3) != RESET)
	{
		audio_stats.dma_errors++;
		StopPlayback();
	}
	else
	{
		if (DMA_GetITStatus(DMA2_IT_HT3) != RESET)
		{
			DMA_ClearITPendingBit(DMA2_IT_HT3);
			if (audio_stats.playing) {FillDAC(0U);}
		}
		if (DMA_GetITStatus(DMA2_IT_TC3) != RESET)
		{
			DMA_ClearITPendingBit(DMA2_IT_TC3);
			if (audio_stats.playing) {FillDAC(AUDIO_HALF);}
		}
	}
	__set_PRIMASK(mask);
}
