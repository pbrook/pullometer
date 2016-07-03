#include "debugf.h"
#include "imu.h"
#include "strain.h"
#include "rf24.h"

#define ENABLE_UART 1

static volatile bool wake_pending;

void
delay_us(int n)
{
  while (n--)
    DELAY1();
}

void
init_uart(void)
{
#ifdef ENABLE_UART
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
#endif
}

extern "C" void
debugf_putc(char c)
{
#ifdef ENABLE_UART
  if (c == '\n')
    debugf_putc('\r');
  while ((UART0->S1 & UART0_S1_TDRE_MASK) == 0)
    /* no-op */;
  UART0->D = c;
#endif
}

static volatile uint32_t now;

extern "C" void
LPTimer_IRQHandler(void)
{
  LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
  now++;
}

static void
lptimer_wake()
{
  LPTMR0->CSR = LPTMR_CSR_TCF_MASK;
  LPTMR0->CNR = 0;
  LPTMR0->CSR = LPTMR_CSR_TIE_MASK;
  LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
}

static void
lptimer_sleep()
{
  LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;
  LPTMR0->CSR = LPTMR_CSR_TCF_MASK;
}

static void
init_lptimer()
{
  SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;
  // Run off 4MHz IRC with x4 divider
  MCG->C1 |= MCG_C1_IRCLKEN_MASK;
  MCG->C2 |= MCG_C2_IRCS_MASK;
  LPTMR0->CSR = LPTMR_CSR_TCF_MASK;
  LPTMR0->PSR = LPTMR_PSR_PCS(0) | LPTMR_PSR_PRESCALE(1);
  LPTMR0->CMR = 1000;
  NVIC_EnableIRQ(LPTimer_IRQn);
  lptimer_wake();
}

extern "C" void
PORTA_IRQHandler()
{
    uint32_t pcr;
    wake_pending = true;
    // This also clears the pin ISF
    PORTA->PCR[11] &= ~PORT_PCR_IRQC_MASK;
}

static void
init_irq()
{
  SMC->PMPROT = SMC_PMPROT_AVLP_MASK;
  // Gyro wakeup interrupt
  PORTA->PCR[11] = PORT_PCR_MUX(1);
  NVIC_EnableIRQ(PORTA_IRQn);
  __enable_irq();
}

static void
idle()
{
    __WFE();
}

void
init(void)
{
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK;
  PORTB->PCR[8] = PORT_PCR_MUX(1);
  FPTB->PDDR |= _BV(8);
  PORTB->PCR[9] = PORT_PCR_MUX(1);
  FPTB->PDDR |= _BV(9);
  FPTB->PSOR = _BV(9);
  init_uart();
  debugf("UART\n");
  init_irq();
  init_lptimer();
  debugf("LPT\n");
  init_imu();
  debugf("IMU\n");
  init_strain();
  debugf("Strain");
  debugf("CLKDIV1 %08x\n", SIM->CLKDIV1);
  debugf("FCFG1 %08x\n", SIM->FCFG1);
}

volatile int i;

adxl accel;
itg_gyro gyro;

RF24 rf24;

static int base_strain;

static void
deep_sleep()
{
    debugf("Sleeping\n");
    rf24.sleep();
    gyro.sleep();
    sleep_strain();
    accel.sleep();
    __disable_irq();
    lptimer_sleep();

    //FPTB->PCOR = _BV(9);
    // Enable deep sleep mode
    SCB->SCR = 1<<SCB_SCR_SLEEPDEEP_Pos;
    // Set flash clock divider to x5 (800kHz) for VLPR mode
    SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV4(4);
    // Enable VLPR/VLPS mode
    SMC->PMCTRL = SMC_PMCTRL_RUNM(2) | SMC_PMCTRL_STOPM(2);

    while (true) {
        __enable_irq();
        if (accel.poll(true))
            break;
        __disable_irq();
        // Unmask pin change interrupt for accelerometer wakeup
        PORTA->PCR[11] |= PORT_PCR_IRQC(0xc);
        if (!wake_pending)
            __WFI();
        wake_pending = false;
    }

    // Disable deep sleep mode
    SCB->SCR = 0;
    // Disable VLPR/VLPS mode
    SMC->PMCTRL = 0;
    // Set flash clock back to full speed
    SIM->CLKDIV1 &= ~SIM_CLKDIV1_OUTDIV4_MASK;

    FPTB->PSOR = _BV(9);
    __enable_irq();
    lptimer_wake();
    gyro.wake();
    wake_strain();
    accel.wake();
    rf24.wake();
    debugf("Awake\n");
}

static void
send_packet(const uint8_t *data)
{
  static bool tx_active;
  static int fails;
  int rc;

  if (tx_active) {
      rc = rf24.tx_poll();
      if (rc == -1) {
	  return;
      }
      if (rc < 0) {
	  fails++;
          if (fails > 5) {
              deep_sleep();
          }
      } else {
          fails = 0;
      }
  }
  rf24.tx(0, data);
  tx_active = 1;
}

static void
post_data(uint32_t tick, int strain, int16_t angle)
{
  static uint8_t msg[RF24_PACKET_LEN];
  int16_t real_strain;

  real_strain = strain - base_strain;
  msg[0] = tick & 0xff;
  msg[1] = real_strain >> 8;
  msg[2] = real_strain & 0xff;
  msg[3] = angle >> 8;
  msg[4] = angle & 0xff;

  send_packet(msg);
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
  rf24.wake();
  debugf("RF24\n");
  wake_strain();
  FPTB->PCOR = _BV(8);
  debugf("Ready\n");
  while(1) {
      idle();
      while ((int32_t)(now - last_tick) >= IMU_TICK_MS) {
	  imu_tick();
	  last_tick += IMU_TICK_MS;
      }
      accel.poll();
      gyro.poll();
      strain = poll_strain();
      if (strain != STRAIN_BUSY) {
          post_data(last_tick, strain, imu_current_pos());
      }
  }
  return 0;
}
