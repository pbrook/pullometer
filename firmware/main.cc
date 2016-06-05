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

static void
init_lptimer()
{
  SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;
  // MCGIRCLK / 1000 = 4kHz
  MCG->C1 |= MCG_C1_IRCLKEN_MASK;
  MCG->C2 |= MCG_C2_IRCS_MASK;
  LPTMR0->CSR = 0;
  LPTMR0->PSR = LPTMR_PSR_PCS(5) | LPTMR_PSR_PRESCALE(0);
  LPTMR0->CSR = LPTMR_CSR_TFC_MASK | LPTMR_CSR_TEN_MASK;
}

void
init(void)
{
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK;
  PORTB->PCR[8] = PORT_PCR_MUX(1);
  FPTB->PDDR |= _BV(8);
  init_lptimer();
  init_uart();
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

  init();
  accel.init();
  gyro.init();
  gyro.sleep();
  debugf("Hello World\n");
  rf24.init();
  rf24.set_address(0xa11be115);
  FPTB->PSOR = _BV(8);
  while(1) {
      if (FPTA->PDIR & _BV(9)) {
	  accel.poll();
      }
      strain = poll_strain();
      if (strain != STRAIN_BUSY) {
	  do_tx(strain);
	//debugf("Strain: %06x\n", strain);
      }
  }
  return 0;
}
