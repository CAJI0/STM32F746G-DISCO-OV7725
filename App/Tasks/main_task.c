

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "stm32746g_discovery.h"
#include "stm32746g_discovery_camera.h"
#include "stm32746g_discovery_lcd.h"

#include "debug.h"

#define STATUS_STR(ret) ((ret == 0) ? "OK" : "ERROR")

#define CAMERA_RESOLUTION CAMERA_R320x240 //CAMERA_R480x272

#if CAMERA_RESOLUTION == CAMERA_R320x240
#define CAMERA_FRAMEBUFFER  (SDRAM_DEVICE_ADDR+3*SDRAM_BANK_SIZE)
#elif CAMERA_RESOLUTION == CAMERA_R480x272
#define CAMERA_FRAMEBUFFER  LCD_FB_START_ADDRESS
#endif

#define DEBUG_BUF_SIZE 256


static uint8_t DebugWrite(const uint8_t *buf, uint16_t len);
static void DmaInit(void);

static uint8_t DebugBuffer[DEBUG_BUF_SIZE];
static DebugDesc_t DebugDesc;
static UART_HandleTypeDef huart;

static uint8_t cam_frames_cnt = 0;
static uint8_t cam_error_cnt = 0;
static uint32_t cam_lines_cnt = 0;
static uint32_t cam_vsync_cnt = 0;

static DMA_HandleTypeDef hdma_handler;

void vMainTask(void const * argument)
{
	BSP_LED_Init(LED_GREEN);

	BSP_LED_On(LED_GREEN);
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	huart.Init.BaudRate = 115200;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	BSP_COM_Init(COM1, &huart);

	DebugDesc.buffer = DebugBuffer;
	DebugDesc.max_size = DEBUG_BUF_SIZE;
	DebugDesc.write = DebugWrite;
	Debug_Init(&DebugDesc);

	PRINT("Startup\n");

#if CAMERA_RESOLUTION == CAMERA_R320x240
	DmaInit();
#endif

	uint8_t cam_init_status = BSP_CAMERA_Init(CAMERA_RESOLUTION); // CAMERA_R320x240  CAMERA_R480x272
	PRINT("CAMERA INIT: %s\n", STATUS_STR(cam_init_status));

	if (cam_init_status == CAMERA_OK) {
	  BSP_CAMERA_ContinuousStart(CAMERA_FRAMEBUFFER);
	}
	else if (cam_init_status == CAMERA_NOT_DETECTED) {
	  BSP_LCD_DisplayStringAtLine(0, "CAMERA NOT FOUND");
	}

	uint32_t PreviousWakeTime = osKernelSysTick();
	uint32_t cnt = 0;
	uint32_t prev_cam_lines_cnt = 0;
	for(;;) {
		osDelayUntil(&PreviousWakeTime, 1000);
		if (cam_init_status == CAMERA_OK) {
		  char buf[100];
		  snprintf(buf, 100, "FPS: %d, LINES %d", cam_vsync_cnt,  cam_lines_cnt-prev_cam_lines_cnt);
		  PRINT("VSYNC: %d %d, LINES %d\n", cam_vsync_cnt, cam_frames_cnt, cam_lines_cnt-prev_cam_lines_cnt);
		  BSP_LCD_DisplayStringAtLine(0, buf);
	    cam_vsync_cnt = 0;
	    prev_cam_lines_cnt=cam_lines_cnt;
		}
	}
}

static void DmaInit(void)
{
  /*** Configure the DMA ***/
  /* Set the parameters to be configured */
  hdma_handler.Init.Channel             = DMA_CHANNEL_2;
  hdma_handler.Init.Direction           = DMA_MEMORY_TO_MEMORY;
  hdma_handler.Init.PeriphInc           = DMA_PINC_ENABLE;
  hdma_handler.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_handler.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_handler.Init.Mode                = DMA_NORMAL;
  hdma_handler.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_handler.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_handler.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_handler.Init.MemBurst            = DMA_MBURST_SINGLE;
  hdma_handler.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  hdma_handler.Instance = DMA2_Stream2;
  /* NVIC configuration for DMA2 transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  /* Configure the DMA stream */
  HAL_DMA_Init(&hdma_handler);
}

void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_handler);
}

static volatile uint32_t cam_lines_num = 0;

void BSP_CAMERA_FrameEventCallback(void)
{
	cam_frames_cnt++;
}

void BSP_CAMERA_VsyncEventCallback(void)
{
	cam_vsync_cnt++;

	cam_lines_num = 0;
}

void BSP_CAMERA_ErrorCallback(void)
{
	cam_error_cnt++;
}

void BSP_CAMERA_LineEventCallback(void)
{
	cam_lines_cnt++;
#if CAMERA_RESOLUTION == CAMERA_R320x240
	uint32_t src = (CAMERA_FRAMEBUFFER + cam_lines_num * 320 * 2);
	uint32_t dst = (LCD_FB_START_ADDRESS + cam_lines_num * 480 * 2) + (272-240)*480*2 + (480-320);
	HAL_DMA_Start_IT(&hdma_handler, src, dst, 320*2/4);

	//memcpy(LCD_FB_START_ADDRESS + cam_lines_num * 480 * 2, CAMERA_FRAMEBUFFER + cam_lines_num * 320 * 2, 320*2);
	cam_lines_num++;
#endif
}



static uint8_t DebugWrite(const uint8_t *buf, uint16_t len)
{
	HAL_UART_Transmit(&huart, buf, len, 100);
	return 0;
}
