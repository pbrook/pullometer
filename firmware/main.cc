#include "debugf.h"
#include "imu.h"
#include "strain.h"
#include "rf24.h"

void
delay_us(int n)
{
  while (n--)
    DELAY1();
}

void
init_uart(void)
{
  MCG->C1 |= MCG_C1_IRCLKEN_MASK;
  SIM->SOPT2 |= SIM_SOPT2_UART0SRC(3);
  SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
  PORTB->PCR[1] = PORT_PCR_MUX(2);
  PORTB->PCR[2] = PORT_PCR_MUX(2);
  // Rate = 4MHZ / (4 * SBR)
  UART0->BDH = 0;
  UART0->BDL = 9; // 115200
  UART0->C4 = 3; // 4x oversample
  UART0->C1 = 0;
  UART0->C2 = UART0_C2_TE_MASK;
}

extern "C" void
debugf_putc(char c)
{
  if (c == '\n')
    debugf_putc('\r');
  while ((UART0->S1 & UART0_S1_TDRE_MASK) == 0)
    /* no-op */;
  UART0->D = c;
}

static volatile bool pending;
static volatile uint32_t now;

extern "C" void
LPTimer_IRQHandler(void)
{
  LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
  now++;
  pending = true;
}

static void
init_lptimer()
{
  SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;
  LPTMR0->CSR = LPTMR_CSR_TCF_MASK;
  // Run off 4MHz IRC with x4 divider
  MCG->C1 |= MCG_C1_IRCLKEN_MASK;
  MCG->C2 |= MCG_C2_IRCS_MASK;
  LPTMR0->PSR = LPTMR_PSR_PCS(0) | LPTMR_PSR_PRESCALE(1);
  LPTMR0->CMR = 1000;
  LPTMR0->CNR = 0;
  LPTMR0->CSR = LPTMR_CSR_TIE_MASK;
  LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
  NVIC_EnableIRQ(LPTimer_IRQn);
}

extern "C" void
PORTA_IRQHandler()
{
  pending = true;
  NVIC_DisableIRQ(PORTA_IRQn);
}

extern "C" void
PORTB_IRQHandler()
{
  pending = true;
  NVIC_DisableIRQ(PORTB_IRQn);
}

static void
init_irq()
{
  SMC->PMPROT = SMC_PMPROT_AVLP_MASK;
  __enable_irq();
}

static void
idle()
{
  NVIC_EnableIRQ(PORTA_IRQn);
  NVIC_EnableIRQ(PORTB_IRQn);
  __disable_irq();
  if (!pending) {
      __WFI();
  }
  pending = false;
  __enable_irq();
}

void
init(void)
{
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK;
  PORTB->PCR[8] = PORT_PCR_MUX(1);
  FPTB->PDDR |= _BV(8);
  init_uart();
  init_irq();
  init_lptimer();
  init_imu();
  init_strain();
}

volatile int i;

adxl accel;
itg_gyro gyro;

RF24 rf24;

static void
do_tx(int val)
{
  static uint8_t msg[4];
  static uint8_t seq;
  static bool tx_active;
  static int busy_count;
  int rc;

  if (tx_active) {
      rc = rf24.tx_poll();
      if (rc == -1) {
	  busy_count++;
	  return;
      }
      if (rc < 0) {
	  //debugf("TX Failed %d\n", busy_count);
      }
  }
  busy_count = 0;
  msg[0] = seq++;
  msg[1] = val >> 16;
  msg[2] = val >> 8;
  msg[3] = val;
  rf24.tx(0, msg);
  tx_active = 1;
}

int main()
{
  long strain;
  uint32_t last_tick = 0;

  init();
  debugf("Basic\n");
  accel.init();
  debugf("Accel1\n");
  accel.wake();
  debugf("Accel2\n");
  gyro.init();
  gyro.wake();
  debugf("Hello World\n");
  rf24.init();
  rf24.set_address(0xa11be115);
  debugf("RF24\n");
  wake_strain();
  FPTB->PSOR = _BV(8);
  debugf("Ready\n");
  while(1) {
      idle();
      while ((int32_t)(now - last_tick) > 0) {
	  imu_tick();
	  last_tick += IMU_TICK_MS;
      }
      accel.poll();
      gyro.poll();
      strain = poll_strain();
      if (strain != STRAIN_BUSY) {
	  do_tx(strain);
      }
  }
  return 0;
}
