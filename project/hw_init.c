#include "hw_init.h"
#include "dspin.h"

#include "com/iface_usart.h"
#include "com/iface_noop.h"
#include "com/com_fileio.h"
#include "com/datalink.h"

#include "utils/debounce.h"
#include "utils/timebase.h"

#include "bus/event_queue.h"

// ---- Private prototypes --------

static void conf_gpio(void);
static void conf_usart(void);
static void conf_systick(void);
static void conf_irq_prios(void);
static void conf_dspin(void);

// ---- Public functions ----------

/**
 * @brief Initialize hardware resources
 */
void hw_init(void)
{
	conf_gpio();
	conf_usart();
	conf_systick();
	conf_irq_prios();
	conf_dspin();

	// task scheduler subsystem
	timebase_init(15, 15);

	// event and task queues
	queues_init(15, 15);

	// initialize SBMP for ESP8266
	dlnk_init();
}


// ---- Private functions ---------



static void conf_dspin(void)
{
	dSPIN_Peripherals_Init();
	dSPIN_Reset_And_Standby();

	dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;

	/* Structure initialization by default values, in order to avoid blank records */
	dSPIN_Regs_Struct_Reset(&dSPIN_RegsStruct);

	/* Acceleration rate settings to dSPIN_CONF_PARAM_ACC in steps/s2, range 14.55 to 59590 steps/s2 */
	dSPIN_RegsStruct.ACC      = AccDec_Steps_to_Par(dSPIN_CONF_PARAM_ACC);
	/* Deceleration rate settings to dSPIN_CONF_PARAM_DEC in steps/s2, range 14.55 to 59590 steps/s2 */
	dSPIN_RegsStruct.DEC      = AccDec_Steps_to_Par(dSPIN_CONF_PARAM_DEC);
	/* Maximum speed settings to dSPIN_CONF_PARAM_MAX_SPEED in steps/s, range 15.25 to 15610 steps/s */
	dSPIN_RegsStruct.MAX_SPEED    = MaxSpd_Steps_to_Par(dSPIN_CONF_PARAM_MAX_SPEED);
	/* Full step speed settings dSPIN_CONF_PARAM_FS_SPD in steps/s, range 7.63 to 15625 steps/s */
	dSPIN_RegsStruct.FS_SPD   = FSSpd_Steps_to_Par(dSPIN_CONF_PARAM_FS_SPD);

	/* Minimum speed settings to dSPIN_CONF_PARAM_MIN_SPEED in steps/s, range 0 to 976.3 steps/s */
	dSPIN_RegsStruct.MIN_SPEED    = dSPIN_CONF_PARAM_LSPD_BIT | MinSpd_Steps_to_Par(dSPIN_CONF_PARAM_MIN_SPEED);
	/* Acceleration duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_ACC in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_ACC     = Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_ACC);
	/* Deceleration duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_DEC in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_DEC     = Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_DEC);
	/* Run duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_RUN in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_RUN     = Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_RUN);
	/* Hold duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_HOLD in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_HOLD    = Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_HOLD);
	/* Thermal compensation param settings to dSPIN_CONF_PARAM_K_THERM, range 1 to 1.46875 */
	dSPIN_RegsStruct.K_THERM  = KTherm_to_Par(dSPIN_CONF_PARAM_K_THERM);
	/* Intersect speed settings for BEMF compensation to dSPIN_CONF_PARAM_INT_SPD in steps/s, range 0 to 3906 steps/s */
	dSPIN_RegsStruct.INT_SPD  = IntSpd_Steps_to_Par(dSPIN_CONF_PARAM_INT_SPD);
	/* BEMF start slope settings for BEMF compensation to dSPIN_CONF_PARAM_ST_SLP in % step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.ST_SLP   = BEMF_Slope_Perc_to_Par(dSPIN_CONF_PARAM_ST_SLP);
	/* BEMF final acc slope settings for BEMF compensation to dSPIN_CONF_PARAM_FN_SLP_ACC in% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_ACC = BEMF_Slope_Perc_to_Par(dSPIN_CONF_PARAM_FN_SLP_ACC);
	/* BEMF final dec slope settings for BEMF compensation to dSPIN_CONF_PARAM_FN_SLP_DEC in% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_DEC = BEMF_Slope_Perc_to_Par(dSPIN_CONF_PARAM_FN_SLP_DEC);
	/* Stall threshold settings to dSPIN_CONF_PARAM_STALL_TH in mA, range 31.25 to 4000mA */
	dSPIN_RegsStruct.STALL_TH     = StallTh_to_Par(dSPIN_CONF_PARAM_STALL_TH);
	/* Set Config register according to config parameters */
	/* clock setting, switch hard stop interrupt mode, */
	/*  supply voltage compensation, overcurrent shutdown */
	/* slew-rate , PWM frequency */
	dSPIN_RegsStruct.CONFIG   = (uint16_t)dSPIN_CONF_PARAM_CLOCK_SETTING |
								(uint16_t)dSPIN_CONF_PARAM_SW_MODE |
								(uint16_t)dSPIN_CONF_PARAM_VS_COMP |
								(uint16_t)dSPIN_CONF_PARAM_OC_SD |
								(uint16_t)dSPIN_CONF_PARAM_SR |
								(uint16_t)dSPIN_CONF_PARAM_PWM_DIV |
								(uint16_t)dSPIN_CONF_PARAM_PWM_MUL;

	/* Overcurrent threshold settings to dSPIN_CONF_PARAM_OCD_TH in mA */
	dSPIN_RegsStruct.OCD_TH   = dSPIN_CONF_PARAM_OCD_TH;
	/* Alarm settings to dSPIN_CONF_PARAM_ALARM_EN */
	dSPIN_RegsStruct.ALARM_EN     = dSPIN_CONF_PARAM_ALARM_EN;
	/* Step mode and sycn mode settings via dSPIN_CONF_PARAM_SYNC_MODE and dSPIN_CONF_PARAM_STEP_MODE */
	dSPIN_RegsStruct.STEP_MODE    = (uint8_t)dSPIN_CONF_PARAM_SYNC_MODE |
									(uint8_t)dSPIN_CONF_PARAM_STEP_MODE;

	/* Program all dSPIN registers */
	dSPIN_Registers_Set(&dSPIN_RegsStruct);

	dSPIN_Go_Home(); // apply the idle current
}


static void conf_irq_prios(void)
{
	NVIC_SetPriorityGrouping(0); // 0 bits for sub-priority

	// SysTick - highest prio, used for timeouts
	NVIC_SetPriority(SysTick_IRQn, 0); // SysTick - for timeouts
//	NVIC_SetPriority(USART2_IRQn, 6); // USART - datalink
	NVIC_SetPriority(USART1_IRQn, 10); // USART - debug
}


/**
 * @brief Configure GPIOs
 */
static void conf_gpio(void)
{
	GPIO_InitTypeDef gpio_cnf;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// UART1
	GPIO_StructInit(&gpio_cnf);
	gpio_cnf.GPIO_Pin = GPIO_Pin_9;
	gpio_cnf.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio_cnf);

	gpio_cnf.GPIO_Pin = GPIO_Pin_10;
	gpio_cnf.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_cnf);
}


/**
 * @brief Configure USARTs
 */
static void conf_usart(void)
{
	// Debug interface, working as stdout/stderr.
//	debug_iface = usart_iface_init(USART1, 115200, 256, 256);
//	setvbuf(stdout, NULL, _IONBF, 0);
//	setvbuf(stderr, NULL, _IONBF, 0);
//	debug_iface->file = stdout;

	// dev null
	debug_iface = noop_iface_init();
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
	debug_iface->file = stdout;

	// Datalink iface

	// we have only one UART available - will be used for ESP8266
	data_iface = usart_iface_init(USART1, 460800, 256, 256);
}


/**
 * @brief Configure 1 kHz SysTick w/ interrupt
 */
static void conf_systick(void)
{
	SysTick_Config(F_CPU / 1000);
}
