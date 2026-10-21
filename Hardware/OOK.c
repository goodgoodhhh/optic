#include "stm32f10x.h"
#include "OOK.h"
#include "Voice.h"

#define FRAME_TEXT       1U
#define FRAME_AUDIO      2U
#define FRAME_OVERHEAD   7U
#define TX_SLOTS         4U
#define RX_OVERSAMPLE    4U
#define RX_DMA_HALF      512U
#define RX_DMA_COUNT     (RX_DMA_HALF * 2U)

/* 帧：A5 5A TYPE SEQ LEN PAYLOAD CRC低字节 CRC高字节。
 * CRC16-CCITT-FALSE 覆盖 TYPE 至 PAYLOAD，初值 FFFF、多项式 1021。
 * USART2 只负责发送；PA3 电平经软件反相和 8N1 解码后进入同一帧解析器。 */
static uint8_t tx_packet[TX_SLOTS][OOK_AUDIO_SAMPLES + FRAME_OVERHEAD];
static uint8_t tx_length[TX_SLOTS];
static volatile uint8_t tx_head, tx_tail, tx_dma_active;
static volatile uint8_t digital_enabled, transmit_allowed;
static uint8_t text_sequence;
static volatile uint32_t link_ms;
static volatile OOK_Stats link_stats;

static volatile uint8_t rx_state;
static uint8_t rx_type, rx_sequence, rx_length, rx_index;
static uint16_t rx_crc, rx_received_crc;
static uint32_t rx_last_ms;
static uint8_t rx_payload[OOK_AUDIO_SAMPLES];
static volatile char rx_message[OOK_MAX_PAYLOAD + 1U];
static volatile uint8_t rx_message_length, rx_message_ready;

/* TIM5_UP 请求 DMA2 通道2读取 GPIOA->IDR，不使用 USART2 的 RX 输入。
 * 循环采样区的两个半区交替写入；先复制安全半区，再解码，避免边读边覆盖。
 * 静态工作区不占中断栈。115200baud 时半区约 1.11ms，采样约 461.5kHz。 */
static volatile uint16_t rx_samples[RX_DMA_COUNT];
static uint16_t rx_work[RX_DMA_HALF];
static volatile uint8_t sample_state, sample_ticks, sample_idle, sample_bit, sample_byte;
static volatile uint8_t rx_dma_running;

static void ResetSampleDecoder(void)
{
	sample_state = sample_ticks = sample_idle = sample_bit = sample_byte = 0U;
	rx_state = 0U;
}

/* 主循环调用时处于临界区；DMA2 通道2中断本身为最高抢占优先级。 */
static void StopReceiver(void)
{
	TIM_Cmd(TIM5, DISABLE);
	TIM_DMACmd(TIM5, TIM_DMA_Update, DISABLE);
	DMA_Cmd(DMA2_Channel2, DISABLE);
	DMA_ClearFlag(DMA2_FLAG_GL2);
	rx_dma_running = 0U;
	ResetSampleDecoder();
}

static void StartReceiver(void)
{
	ResetSampleDecoder();
	DMA_SetCurrDataCounter(DMA2_Channel2, RX_DMA_COUNT);
	DMA_ClearFlag(DMA2_FLAG_GL2);
	TIM_SetCounter(TIM5, 0U);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	rx_dma_running = 1U;
	DMA_Cmd(DMA2_Channel2, ENABLE);
	TIM_DMACmd(TIM5, TIM_DMA_Update, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

static uint16_t CRC_Update(uint16_t crc, uint8_t byte)
{
	static const uint16_t table[16] = {
		0x0000U, 0x1021U, 0x2042U, 0x3063U,
		0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
		0x8108U, 0x9129U, 0xA14AU, 0xB16BU,
		0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU
	};
	crc ^= (uint16_t)((uint16_t)byte << 8);
	crc = (uint16_t)((crc << 4) ^ table[crc >> 12]);
	return (uint16_t)((crc << 4) ^ table[crc >> 12]);
}

/* 调用者已关闭中断：DMA 只读已发布的完整队列槽位。 */
static void TX_Kick(void)
{
	if (tx_dma_active != 0U || tx_tail == tx_head) {return;}
	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA_ClearFlag(DMA1_FLAG_GL7);
	DMA1_Channel7->CMAR = (uint32_t)(uintptr_t)tx_packet[tx_tail];
	DMA_SetCurrDataCounter(DMA1_Channel7, tx_length[tx_tail]);
	tx_dma_active = 1U;
	DMA_Cmd(DMA1_Channel7, ENABLE);
}

static uint8_t QueueFrame(uint8_t type, uint8_t sequence,
	const uint8_t *data, uint8_t length)
{
	uint32_t mask;
	uint8_t i, next;
	uint16_t crc = 0xFFFFU;
	/* CRC 计算允许高优先级接收中断抢占，不在临界区内逐位计算。 */
	crc = CRC_Update(crc, type);
	crc = CRC_Update(crc, sequence);
	crc = CRC_Update(crc, length);
	for (i = 0U; i < length; i++) {crc = CRC_Update(crc, data[i]);}
	mask = __get_PRIMASK();
	__disable_irq();
	next = (uint8_t)((tx_head + 1U) % TX_SLOTS);
	if (digital_enabled == 0U || transmit_allowed == 0U || next == tx_tail)
	{
		link_stats.tx_drops++;
		__set_PRIMASK(mask);
		return 0U;
	}
	tx_packet[tx_head][0] = 0xA5U;
	tx_packet[tx_head][1] = 0x5AU;
	tx_packet[tx_head][2] = type;
	tx_packet[tx_head][3] = sequence;
	tx_packet[tx_head][4] = length;
	for (i = 0U; i < length; i++) {tx_packet[tx_head][5U + i] = data[i];}
	tx_packet[tx_head][5U + length] = (uint8_t)crc;
	tx_packet[tx_head][6U + length] = (uint8_t)(crc >> 8);
	tx_length[tx_head] = length + FRAME_OVERHEAD;
	__DMB();
	tx_head = next;
	TX_Kick();
	__set_PRIMASK(mask);
	return 1U;
}

static void ParseByte(uint8_t byte)
{
	uint8_t i;
	/* 字节间隔超过 20 ms，残帧作废。状态仅由接收 DMA 中断维护。 */
	if ((uint32_t)(link_ms - rx_last_ms) > 20U) {rx_state = 0U;}
	rx_last_ms = link_ms;
	switch (rx_state)
	{
		case 0:
			if (byte == 0xA5U) {rx_state = 1U;}
			break;
		case 1:
			if (byte == 0x5AU) {rx_state = 2U;}
			else {rx_state = (byte == 0xA5U) ? 1U : 0U;}
			break;
		case 2:
			if (byte != FRAME_TEXT && byte != FRAME_AUDIO)
			{
				rx_state = (byte == 0xA5U) ? 1U : 0U;
				break;
			}
			rx_type = byte;
			rx_crc = CRC_Update(0xFFFFU, byte);
			rx_state = 3U;
			break;
		case 3:
			rx_sequence = byte;
			rx_crc = CRC_Update(rx_crc, byte);
			rx_state = 4U;
			break;
		case 4:
			if ((rx_type == FRAME_TEXT && (byte == 0U || byte > OOK_MAX_PAYLOAD)) ||
				(rx_type == FRAME_AUDIO && byte != OOK_AUDIO_SAMPLES))
			{
				rx_state = (byte == 0xA5U) ? 1U : 0U;
				break;
			}
			rx_length = byte;
			rx_index = 0U;
			rx_crc = CRC_Update(rx_crc, byte);
			rx_state = 5U;
			break;
		case 5:
			rx_payload[rx_index++] = byte;
			rx_crc = CRC_Update(rx_crc, byte);
			if (rx_index == rx_length) {rx_state = 6U;}
			break;
		case 6:
			rx_received_crc = byte;
			rx_state = 7U;
			break;
		default:
			rx_received_crc |= (uint16_t)((uint16_t)byte << 8);
			rx_state = 0U;
			if (rx_received_crc != rx_crc) {link_stats.crc_errors++; break;}
			if (rx_type == FRAME_AUDIO)
			{
				Voice_ReceivePacket(rx_payload, rx_sequence);
			}
			else if (rx_message_ready == 0U)
			{
				for (i = 0U; i < rx_length; i++) {rx_message[i] = (char)rx_payload[i];}
				rx_message[rx_length] = '\0';
				rx_message_length = rx_length;
				rx_message_ready = 1U;
			}
			break;
	}
}

/* 对归一化后的电平解码：空闲1、起始0、数据LSB先发、停止1。
 * 先看到至少一个完整位的空闲，再检测起始沿；每位中心采一次。
 * 停止位中心后立即等下一起始位，支持无额外间隔的连续串口字节。 */
static void DecodeSamples(void)
{
	uint16_t i = 0U, remaining;
	uint8_t bit;
	while (i < RX_DMA_HALF)
	{
		/* 起始沿确定后直接跨到位中心，不逐次处理其余三拍。
		 * 跨越 DMA 半区时保留剩余拍数，避免 OLED/音频负载影响位间隔。 */
		if (sample_state >= 2U)
		{
			remaining = RX_DMA_HALF - i;
			if (sample_ticks > remaining)
			{
				sample_ticks -= (uint8_t)remaining;
				break;
			}
			i += sample_ticks - 1U;
		}
#if OOK_RX_INVERTED
		bit = (uint8_t)((rx_work[i] & GPIO_Pin_3) == 0U);
#else
		bit = (uint8_t)((rx_work[i] & GPIO_Pin_3) != 0U);
#endif
		i++;
		if (sample_state == 0U)
		{
			if (!bit) {sample_idle = 0U;}
			else if (++sample_idle >= RX_OVERSAMPLE) {sample_state = 1U;}
			continue;
		}
		if (sample_state == 1U)
		{
			if (!bit) {sample_state = 2U; sample_ticks = RX_OVERSAMPLE / 2U;}
			continue;
		}
		if (sample_state == 2U)
		{
			if (bit)
			{
				link_stats.uart_errors++;
				sample_state = 0U;
				sample_idle = 1U;
				continue;
			}
			sample_byte = sample_bit = 0U;
			sample_state = 3U;
		}
		else if (sample_state == 3U)
		{
			if (bit) {sample_byte |= (uint8_t)(1U << sample_bit);}
			if (++sample_bit == 8U) {sample_state = 4U;}
		}
		else
		{
			if (bit)
			{
				sample_state = 1U;
				ParseByte(sample_byte);
			}
			else
			{
				link_stats.uart_errors++;
				ResetSampleDecoder();
			}
			continue;
		}
		sample_ticks = RX_OVERSAMPLE;
	}
}

/* DMA 剩余数判断当前在写哪个半区。0 是全满边界，按即将回到半区0处理。 */
static uint8_t SampleHalfIsSafe(uint16_t offset)
{
	uint16_t remaining = DMA_GetCurrDataCounter(DMA2_Channel2);
	if (offset == 0U) {return (uint8_t)(remaining != 0U && remaining <= RX_DMA_HALF);}
	return (uint8_t)(remaining == 0U || remaining > RX_DMA_HALF);
}

static uint8_t CopySampleHalf(uint16_t offset)
{
	uint16_t i;
	uint32_t next_flag = (offset == 0U) ? DMA2_IT_TC2 : DMA2_IT_HT2;
	if (!SampleHalfIsSafe(offset)) {return 0U;}
	for (i = 0U; i < RX_DMA_HALF; i++) {rx_work[i] = rx_samples[offset + i];}
	__DMB();
	/* 复制期间若 DMA 绕回本半区，该快照不可使用，不交给协议/音频层。 */
	return (uint8_t)(SampleHalfIsSafe(offset) && DMA_GetITStatus(next_flag) == RESET);
}

void OOK_Init(void)
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;
	DMA_InitTypeDef dma;
	TIM_TimeBaseInitTypeDef timer;
	NVIC_InitTypeDef nvic;
	RCC_ClocksTypeDef clocks;
	uint32_t timer_ratio, sample_period;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
	/* LISTEN 时 PA2 为低，关闭本机通信灯，避免自干扰。 */
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);
	USART_StructInit(&uart);
	uart.USART_BaudRate = OOK_BAUD_RATE;
	uart.USART_Mode = USART_Mode_Tx; /* PA3 保留普通 GPIO，禁用硬件 RX。 */
	USART_Init(USART2, &uart);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART2, ENABLE);

	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&USART2->DR;
	dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)tx_packet;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_BufferSize = 1U;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel7, &dma);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC | DMA_IT_TE, ENABLE);

	/* USART 的 BRR 对应 PCLK1 下实际位周期；TIM5 时钟为 PCLK1 或 2*PCLK1。
 * 用实际 BRR 推导四倍采样周期，避免分别取整造成采样与发送速率偏差。
 * 当前 PCLK1=36MHz、BRR=312、TIM5=72MHz，因此 ARR=155。 */
	RCC_GetClocksFreq(&clocks);
	timer_ratio = (clocks.PCLK1_Frequency == clocks.HCLK_Frequency) ? 1U : 2U;
	sample_period = ((uint32_t)USART2->BRR * timer_ratio + RX_OVERSAMPLE / 2U) / RX_OVERSAMPLE;
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = 0U;
	timer.TIM_Period = sample_period - 1U;
	TIM_TimeBaseInit(TIM5, &timer);
	TIM_UpdateRequestConfig(TIM5, TIM_UpdateSource_Regular);

	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&GPIOA->IDR;
	dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)rx_samples;
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_BufferSize = RX_DMA_COUNT;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA2_Channel2, &dma);
	DMA_ITConfig(DMA2_Channel2, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, ENABLE);
	nvic.NVIC_IRQChannel = DMA2_Channel2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0U;
	nvic.NVIC_IRQChannelSubPriority = 0U;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2U;
	nvic.NVIC_IRQChannelSubPriority = 1U;
	NVIC_Init(&nvic);

	/* 72 MHz 定时器时钟，提供 1 ms 按键及超时节拍。 */
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = 72U - 1U;
	timer.TIM_Period = 1000U - 1U;
	TIM_TimeBaseInit(TIM2, &timer);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 3U;
	nvic.NVIC_IRQChannelSubPriority = 0U;
	NVIC_Init(&nvic);
	TIM_Cmd(TIM2, ENABLE);
}

/* 切换模式/方向会终止旧帧，接收端用 CRC 和同步头丢弃残帧。 */
static void ResetLink(void)
{
	GPIO_InitTypeDef gpio;
	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA_ClearFlag(DMA1_FLAG_GL7);
	tx_head = tx_tail = tx_dma_active = 0U;
	rx_state = rx_message_ready = 0U;
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = (digital_enabled && transmit_allowed) ? GPIO_Mode_AF_PP : GPIO_Mode_Out_PP;
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	GPIO_Init(GPIOA, &gpio);
	StopReceiver();
	if (digital_enabled && !transmit_allowed) {StartReceiver();}
}

void OOK_SetDigitalEnabled(uint8_t enabled)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	digital_enabled = (enabled != 0U);
	ResetLink();
	__set_PRIMASK(mask);
}

void OOK_SetTransmitAllowed(uint8_t allowed)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	transmit_allowed = (allowed != 0U);
	ResetLink();
	__set_PRIMASK(mask);
}

uint8_t OOK_IsSendMode(void)
{
	return (uint8_t)(digital_enabled && transmit_allowed);
}

uint8_t OOK_Send(const uint8_t *data, uint8_t length)
{
	if (data == 0 || length == 0U || length > OOK_MAX_PAYLOAD) {return 0U;}
	return QueueFrame(FRAME_TEXT, text_sequence++, data, length);
}

uint8_t OOK_SendAudio(const uint8_t *samples, uint8_t sequence)
{
	if (samples == 0) {return 0U;}
	return QueueFrame(FRAME_AUDIO, sequence, samples, OOK_AUDIO_SAMPLES);
}

void OOK_Tick1ms(void) {link_ms++;}

uint8_t OOK_TakeMessage(char *buffer, uint8_t buffer_size)
{
	uint8_t i, length;
	uint32_t mask;
	if (buffer == 0 || buffer_size < 2U) {return 0U;}
	mask = __get_PRIMASK();
	__disable_irq();
	length = rx_message_ready ? rx_message_length : 0U;
	if (length >= buffer_size) {length = buffer_size - 1U;}
	for (i = 0U; i < length; i++) {buffer[i] = rx_message[i];}
	buffer[length] = '\0';
	rx_message_ready = 0U;
	__set_PRIMASK(mask);
	return length;
}

uint8_t OOK_IsTransmitting(void)
{
	return (uint8_t)(OOK_IsSendMode() && (tx_head != tx_tail ||
		tx_dma_active || USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET));
}

void OOK_GetStats(OOK_Stats *stats)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	*stats = link_stats;
	__set_PRIMASK(mask);
}

void DMA2_Channel2_IRQHandler(void)
{
	uint8_t half_ready, full_ready;
	uint16_t offset;
	if (!rx_dma_running) {DMA_ClearFlag(DMA2_FLAG_GL2); return;}
	if (DMA_GetITStatus(DMA2_IT_TE2) != RESET)
	{
		link_stats.rx_dma_errors++;
		StopReceiver();
		StartReceiver();
		return;
	}
	half_ready = (uint8_t)(DMA_GetITStatus(DMA2_IT_HT2) != RESET);
	full_ready = (uint8_t)(DMA_GetITStatus(DMA2_IT_TC2) != RESET);
	if (!half_ready && !full_ready) {return;}
	if (half_ready && full_ready)
	{
		/* 处理超过一个半区期限，不能把覆盖后的样本拼成伪造字节。 */
		link_stats.rx_overruns++;
		StopReceiver();
		StartReceiver();
		return;
	}
	offset = half_ready ? 0U : RX_DMA_HALF;
	DMA_ClearITPendingBit(half_ready ? DMA2_IT_HT2 : DMA2_IT_TC2);
	if (!CopySampleHalf(offset))
	{
		link_stats.rx_overruns++;
		StopReceiver();
		StartReceiver();
		return;
	}
	DecodeSamples();
}

void DMA1_Channel7_IRQHandler(void)
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	if (DMA_GetITStatus(DMA1_IT_TC7) != RESET || DMA_GetITStatus(DMA1_IT_TE7) != RESET)
	{
		if (DMA_GetITStatus(DMA1_IT_TE7) != RESET) {link_stats.tx_drops++;}
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA_ClearFlag(DMA1_FLAG_GL7);
		if (tx_dma_active)
		{
			tx_tail = (uint8_t)((tx_tail + 1U) % TX_SLOTS);
			tx_dma_active = 0U;
			TX_Kick();
		}
	}
	__set_PRIMASK(mask);
}
