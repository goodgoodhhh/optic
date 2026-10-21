#include "stm32f10x.h"
#include "Key.h"
#include "LED.h"
#include "OLED.h"
#include "OOK.h"
#include "Voice.h"
#include <string.h>

#define TEXT_TX_NOTICE_MS 1000U

/* 文字和语音都经过数字 OOK 链路，PB13 始终保持数字通道选择电平。 */
static volatile uint32_t system_ms;
static uint8_t voice_mode;
static uint8_t transmit_mode;
/* 成功入队后保留一秒提示，避免文字发送太快而看不到 BUSY。 */
static uint32_t text_tx_count, text_tx_notice_ms;
static uint8_t text_tx_notice; /* 0：无提示，1：成功入队，2：入队失败。 */
static char last_message[OOK_MAX_PAYLOAD + 1U];

static void ModeOutput_Init(void)
{
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio);
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}

static void UI_Update(void)
{
	Voice_Stats voice;
	OOK_Stats link;
	Voice_GetStats(&voice);
	OOK_GetStats(&link);
	OLED_Clear();
	OLED_Printf(0, 0, OLED_6X8, "OOK SWRX %lu %s", (unsigned long)OOK_BAUD_RATE,
		OOK_RX_INVERTED ? "INV" : "POS");
	OLED_Printf(0, 10, OLED_6X8, "%s %s", voice_mode ? "VOICE" : "TEXT",
		transmit_mode ? "SEND" : "LISTEN");
	if (voice_mode)
	{
		OLED_Printf(0, 20, OLED_6X8, "IN:%s PLAY:%s",
			voice.adc_ready ? (voice.recording ? "ON" : "OFF") : "ERR",
			voice.playing ? "ON" : "OFF");
		OLED_Printf(0, 30, OLED_6X8, "LV:%u RX:%lu", (unsigned int)voice.input_level,
			(unsigned long)voice.rx_packets);
		OLED_Printf(0, 40, OLED_6X8, "CRC:%lu GAP:%lu", (unsigned long)link.crc_errors,
			(unsigned long)voice.lost_packets);
	}
	else
	{
		const char *tx_status = "READY";
		if (!transmit_mode) {tx_status = "LISTEN";}
		else if (OOK_IsTransmitting()) {tx_status = "BUSY";}
		else if (text_tx_notice &&
			(uint32_t)(system_ms - text_tx_notice_ms) < TEXT_TX_NOTICE_MS)
		{
			/* QUEUED 仅表示本机接受发送请求，不代表光路或接收成功。 */
			tx_status = (text_tx_notice == 1U) ? "QUEUED" : "REJECT";
		}
		OLED_Printf(0, 20, OLED_6X8, "TX: %s #%lu", tx_status,
			(unsigned long)text_tx_count);
		OLED_Printf(0, 30, OLED_6X8, "RX: %.20s", last_message);
		OLED_Printf(0, 40, OLED_6X8, "FE:%lu OV:%lu", (unsigned long)link.uart_errors,
			(unsigned long)link.rx_overruns);
	}
	OLED_Printf(0, 54, OLED_6X8, "K1 mode K2 dir K4 stop");
	OLED_Update();
}

int main(void)
{
	uint32_t last_ui_update = 0U;
	uint8_t key, received_length;
	Voice_Stats voice;
	char received[OOK_MAX_PAYLOAD + 1U];
	static const uint8_t hello[] = "HELLO";

	/* 相同固件烧录到两块板；上电为文字 LISTEN，防止两个发射器互相干扰。 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	ModeOutput_Init();
	OLED_Init();
	Key_Init();
	LED_Init();
	OOK_Init();
	Voice_Init();
	OOK_SetDigitalEnabled(1U);
	OOK_SetTransmitAllowed(0U);
	Voice_SetEnabled(0U);
	UI_Update();

	while (1)
	{
		key = Key_GetNum();
		if (key == 1U)
		{
			text_tx_notice = 0U;
			/* K1：切换文字/语音。先停止 DMA，清掉旧模式待发送帧。 */
			Voice_Stop();
			OOK_SetTransmitAllowed(transmit_mode);
			voice_mode ^= 1U;
			Voice_SetEnabled(voice_mode);
		}
		if (key == 2U)
		{
			text_tx_notice = 0U;
			/* K2：半双工方向，语音正在发送时也可立即停止并切到接收。 */
			Voice_Stop();
			transmit_mode ^= 1U;
			OOK_SetTransmitAllowed(transmit_mode);
		}
		if (key == 3U)
		{
			if (!voice_mode)
			{
				if (OOK_Send(hello, sizeof(hello) - 1U))
				{
					text_tx_count++;
					text_tx_notice = 1U;
				}
				else {text_tx_notice = 2U;}
				text_tx_notice_ms = system_ms;
			}
			else if (transmit_mode)
			{
				/* VOICE+SEND 下 K3 开始/停止连续实时采样，无需按住。 */
				Voice_GetStats(&voice);
				if (voice.recording)
				{
					Voice_Stop();
					OOK_SetTransmitAllowed(transmit_mode);
				}
				else {(void)Voice_StartTransmit();}
			}
		}
		if (key == 4U)
		{
			text_tx_notice = 0U;
			/* K4 已移到 PC0：停止音频，回到文字模式并清空显示。 */
			Voice_SetEnabled(0U);
			voice_mode = 0U;
			OOK_SetTransmitAllowed(transmit_mode);
			memset(last_message, 0, sizeof(last_message));
			LED_OFF();
		}

		received_length = OOK_TakeMessage(received, sizeof(received));
		if (received_length != 0U)
		{
			memcpy(last_message, received, received_length + 1U);
			LED_ON();
		}
		if ((uint32_t)(system_ms - last_ui_update) >= 100U)
		{
			last_ui_update = system_ms;
			UI_Update();
		}
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != SET) {return;}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	/* 1 ms 时间服务；音频采样和播放由 TIM3/TIM6、DMA 独立执行。 */
	OOK_Tick1ms();
	Voice_Tick1ms();
	Key_Tick();
	system_ms++;
}
