/* MIT License

Copyright (c) 2020 Ward Ramsdell

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TDWS2811_h
#define TDWS2811_h

#include <Arduino.h>
#include <DMAChannel.h>
#include <FlexIO_t4.h>

#define FLEXMODULE 0

#ifndef DEFAULT_CHANNEL_TYPE
#define DEFAULT_CHANNEL_TYPE GRB
#endif

#define CCM_ANALOG_PLL_VIDEO_BYPASS_MASK (0x10000U)
#define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_VIDEO_NUM_A_MASK (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT (0U)
#define CCM_ANALOG_PLL_VIDEO_NUM_A(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT)) & CCM_ANALOG_PLL_VIDEO_NUM_A_MASK)
#define CCM_ANALOG_PLL_VIDEO_DENOM_B_MASK (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT (0U)
#define CCM_ANALOG_PLL_VIDEO_DENOM_B(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT)) & CCM_ANALOG_PLL_VIDEO_DENOM_B_MASK)
#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_VIDEO_POWERDOWN_MASK (0x1000U)
#define CCM_ANALOG_PLL_VIDEO_ENABLE_MASK (0x2000U)
#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_MISC2_VIDEO_DIV_MASK (0xC0000000U)
#define CCM_ANALOG_PLL_VIDEO_LOCK_MASK (0x80000000U)

class TDWS2811
{
public:
  /// Taken from FastLED:
  /// RGB orderings, used when instantiating controllers to determine what
  /// order the controller should send RGB data out in, RGB being the default
  /// ordering.
  enum EOrder
  {
    RGB = 0012,
    RBG = 0021,
    GRB = 0102,
    GBR = 0120,
    BRG = 0201,
    BGR = 0210
  };

  enum bufferType_t
  {
    ACTIVE,
    INACTIVE
  };

  struct color_t
  {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t white = 0;
  };

  TDWS2811(int num_leds, uint32_t *active_buffer, uint32_t *inactive_buffer)
      : num_leds{num_leds}, frameBuffer{active_buffer, inactive_buffer}
  {
    /* Get a FlexIO channel */
    pFlex = FlexIOHandler::flexIOHandler_list[FLEXMODULE];

    /* Pointer to the port structure in the FlexIO channel */
    p = &pFlex->port();

    /* Pointer to the hardware structure in the FlexIO channel */
    hw = &pFlex->hardware();
  }

  void begin()
  {
    /* Now configure all the things */
    configurePll();
    configureFlexIO();
    configureDma();

    pinMode(13, OUTPUT);
    digitalWrite(13, 1);
  }

  /// Swap active and inactive frame buffers.  This function simply sets the ISR on completion flag of TCD 1 and that ISR handles the rest.
  /// This is to synchronize the buffer swap with the frame blanking period in order to prevent tearing.
  void flipBuffers(void)
  {
    dmaSetting[1].TCD->CSR |= DMA_TCD_CSR_INTMAJOR;
  }

  /// Returns a pointer to the active frame buffer.  Useful for developing more sophisticated buffer writing algorithms
  uint32_t *getActiveBuffer(void)
  {
    return frameBuffer[activeBuffer];
  }

  /// Returns a pointer to the inactive frame buffer.  Useful for developing more sophisticated buffer writing algorithms
  uint32_t *getInactiveBuffer(void)
  {
    return frameBuffer[1 - activeBuffer];
  }

  /// Allows the user to change each channel to RGB, GRB, or GRBW formatting
  void setChannelType(uint8_t channel, EOrder chanType)
  {
    channelType[channel] = chanType;
  }

  int size() { return num_leds; }

  int setLed(uint8_t, uint8_t, color_t, bufferType_t writeBuffer = ACTIVE);
  color_t getLed(uint8_t, uint16_t);

private:
  int num_leds;

  static void _shifterIsr(void);
  static void _dmaIsr(void);
  void shifterIsr(void);
  void dmaIsr(void);
  void configureFlexIO(void);
  void configurePll(void);
  void configureInterrupts(void);
  void configureDma(void);
  void dumpDMA_TCD(DMAChannel *);

  FlexIOHandler *pFlex;
  DMAChannel dmaChannel;
  DMASetting dmaSetting[4];

  volatile uint8_t activeBuffer = 0;
  uint32_t *frameBuffer[2];

  volatile const uint32_t zeros[300] = {0};
  volatile const uint32_t ones = 0xFFFFFFFF;

  static TDWS2811 *pTD;

  IMXRT_FLEXIO_t *p = &pFlex->port();
  const FlexIOHandler::FLEXIO_Hardware_t *hw = &pFlex->hardware();

  EOrder channelType[32] = {DEFAULT_CHANNEL_TYPE};
};

#define TDWS_CLOCK_PIN 2
#define TDWS_CLOCK_FIO_PINSEL 4

#define TDWS_LATCH_PIN 3
#define TDWS_LATCH_FIO_PINSEL 5

#define TDWS_DATA_PIN 4
#define TDWS_DATA_FIO_PINSEL 6

TDWS2811 *TDWS2811::pTD = {nullptr};

void TDWS2811::_dmaIsr(void)
{
  TDWS2811::pTD->dmaIsr();
}

void TDWS2811::dmaIsr(void)
{
  /* Disable interrupts on TCD 1.  If there's a "disable interrupt" function in DMASetting, we should use that, but I don't see one */
  dmaSetting[1].TCD->CSR &= ~DMA_TCD_CSR_INTMAJOR;

  /* Swap the buffer pointer in TCD 0 */
  if (activeBuffer == 0)
  {
    activeBuffer = 1;
    digitalWrite(13, 1);
    dmaSetting[0].sourceBuffer(frameBuffer[1], 24 * num_leds * 4);
  }
  else
  {
    activeBuffer = 0;
    digitalWrite(13, 0);
    dmaSetting[0].sourceBuffer(frameBuffer[0], 24 * num_leds * 4);
  }

  /* Clear the interrupt so we don't get triggered again */
  dmaChannel.clearInterrupt();

  /* Spin for a few cycles.  If we don't do this, the interrupt doesn't clear and we get triggered a second time */
  for (uint8_t i = 0; i < 10; i++)
    __asm__ __volatile__("nop\n\t"); //Some race condition between clearInterrupt() and the return of the ISR.  If we don't delay here, the ISR will fire again.
}

void TDWS2811::configureFlexIO(void)
{
  *portModeRegister(TDWS_DATA_PIN) |= digitalPinToBitMask(TDWS_DATA_PIN);
  *portControlRegister(TDWS_DATA_PIN) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
  // SION + ALT4 (FLEXIO1_FLEXIO6) (IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06)
  *portConfigRegister(TDWS_DATA_PIN) = 0x14;

  *portModeRegister(TDWS_LATCH_PIN) |= digitalPinToBitMask(TDWS_LATCH_PIN);
  *portControlRegister(TDWS_LATCH_PIN) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
  // SION + ALT4 (FLEXIO1_FLEXIO05) (IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05)
  *portConfigRegister(TDWS_LATCH_PIN) = 0x14;

  *portModeRegister(TDWS_CLOCK_PIN) |= digitalPinToBitMask(TDWS_CLOCK_PIN);
  *portControlRegister(TDWS_CLOCK_PIN) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
  // SION + ALT4 (FLEXIO1_FLEXIO04) (IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04)
  *portConfigRegister(TDWS_CLOCK_PIN) = 0x14;

  // Disable flexio1 clock gate see 14.7.24 page 1087
  CCM_CCGR5 &= ~CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);
  // FlexIO1 clock select | Clock divider register pre divider | Clock divider register post divider
  CCM_CDCDR &= ~(CCM_CDCDR_FLEXIO1_CLK_SEL(3) | CCM_CDCDR_FLEXIO1_CLK_PRED(7) | CCM_CDCDR_FLEXIO1_CLK_PODF(7));
  // Derive clock from PLL5 clock 14.7.8 page 1059 | Divide by (4 + 1 = 5) | Divide by (0 + 1 = 1)
  CCM_CDCDR |= CCM_CDCDR_FLEXIO1_CLK_SEL(2) | CCM_CDCDR_FLEXIO1_CLK_PRED(4) | CCM_CDCDR_FLEXIO1_CLK_PODF(0);
  // Enable flexio1 clock gate see 14.7.24 page 1087
  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);

  // Shifter control 50.5.1.14 page 2925
  // SHIFTER0 is configured to output on DIN = pin 8
  IMXRT_FLEXIO1_S.SHIFTCTL[0] =
      // Timer select -> TIMER0 controls logic and shift clock
      FLEXIO_SHIFTCTL_TIMSEL(0) |
      // Timer polarity -> shift on negative edge of shift clock
      FLEXIO_SHIFTCTL_TIMPOL |
      // Shifter pin configuration -> shifter pin output
      FLEXIO_SHIFTCTL_PINCFG(3) |
      // Shifter pin select FLEXIO06 pin (DATA on pin 4)
      FLEXIO_SHIFTCTL_PINSEL(TDWS_DATA_FIO_PINSEL) |
      // Shifter pin polarity -> pin is active high
      (FLEXIO_SHIFTCTL_PINPOL & 0) |
      // Shifter mode -> Transmit mode. Load SHIFTBUF contents into
      // the shifter on expiration of the Timer
      FLEXIO_SHIFTCTL_SMOD(2);
  // SHIFTERS 1-3 do not output to a pin
  IMXRT_FLEXIO1_S.SHIFTCTL[1] =
      FLEXIO_SHIFTCTL_TIMPOL | FLEXIO_SHIFTCTL_SMOD(2);
  IMXRT_FLEXIO1_S.SHIFTCTL[2] =
      FLEXIO_SHIFTCTL_TIMPOL | FLEXIO_SHIFTCTL_SMOD(2);
  IMXRT_FLEXIO1_S.SHIFTCTL[3] =
      FLEXIO_SHIFTCTL_TIMPOL | FLEXIO_SHIFTCTL_SMOD(2);

  // Shifter configuration 50.5.1.15 page 2927
  // SHIFTER0 shifts 1 bit on each clock and has SHIFTER(0+1) as source
  IMXRT_FLEXIO1_S.SHIFTCFG[0] =
      // 1-bit shift on each shift clock
      FLEXIO_SHIFTCFG_PWIDTH(0) |
      // Input source for shifter is output of Shifter N+1
      FLEXIO_SHIFTCFG_INSRC;
  // Same for shifters 1 - 3
  IMXRT_FLEXIO1_S.SHIFTCFG[1] = FLEXIO_SHIFTCFG_INSRC;
  IMXRT_FLEXIO1_S.SHIFTCFG[2] = FLEXIO_SHIFTCFG_INSRC;
  // Illegal input source for shifter 3 ??????
  IMXRT_FLEXIO1_S.SHIFTCFG[3] = FLEXIO_SHIFTCFG_INSRC;

  // Timer configuration 50.5.1.21.4 page 2935
  IMXRT_FLEXIO1_S.TIMCFG[0] =
      // Timer output is logic 1 when enabled, not affected by reset
      FLEXIO_TIMCFG_TIMOUT(0) |
      // Decrement counter on flexio clock, shift clock = timer output
      FLEXIO_TIMCFG_TIMDEC(0) |
      // Timer never resets
      FLEXIO_TIMCFG_TIMRST(0) |
      // Timer never disabled
      FLEXIO_TIMCFG_TIMDIS(0) |
      // Timer enabled on trigger high (shifter 0 status flag)
      FLEXIO_TIMCFG_TIMENA(2) |
      // Stop bit disabled
      FLEXIO_TIMCFG_TSTOP(0) |
      // Start bit disabled
      (FLEXIO_TIMCFG_TSTART & 0);
  IMXRT_FLEXIO1_S.TIMCFG[1] =
      // Timer enabled on timer N-1 enable
      FLEXIO_TIMCFG_TIMENA(1);

  // Timer control 50.5.1.20 page 2932
  // Triggered after loading SHIFTER0 from SHIFTBUF0 50.5.1.16.3 page 2929
  // TIMER0 is configured to output on BCK = pin 10
  IMXRT_FLEXIO1_S.TIMCTL[0] =
      // Trigger select -> triggers on SHIFTER[N=0] status flag (4*N+1)
      FLEXIO_TIMCTL_TRGSEL(1) |
      // Trigger polarity -> trigger active low
      FLEXIO_TIMCTL_TRGPOL |
      // Trigger source -> internal trigger selected
      FLEXIO_TIMCTL_TRGSRC |
      // Timer pin configuration -> timer pin output
      FLEXIO_TIMCTL_PINCFG(3) |
      // Timer pin select -> FLEXIO04 (CLOCK on pin 2)
      FLEXIO_TIMCTL_PINSEL(TDWS_CLOCK_FIO_PINSEL) |
      // Timer pin polarity -> active high
      (FLEXIO_TIMCTL_PINPOL & 0) |
      // Timer mode -> dual 8 bit counters baud mode
      FLEXIO_TIMCTL_TIMOD(1);
  // No trigger is used, timer is enabled on TIMER0
  // TIMER1 is configured to output on BCK = pin 10
  IMXRT_FLEXIO1_S.TIMCTL[1] =
      // Timer pin configuration -> timer pin output
      FLEXIO_TIMCTL_PINCFG(3) |
      // Timer pin select -> FLEXIO05 (LATCH on pin 3)
      FLEXIO_TIMCTL_PINSEL(TDWS_LATCH_FIO_PINSEL) |
      // Timer pin polarity -> active high
      (FLEXIO_TIMCTL_PINPOL & 0) |
      // Timer mode -> single 16-bit counter mode
      FLEXIO_TIMCTL_TIMOD(3);

  // Timer compare 50.5.1.22 page 2937
  // The upper 8 bits configure the number of bits = (cmp[15:8] + 1) / 2
  // Upper 8 bits -> 4 x 32 bits -> 128 * 2 -> 256 - 1 = 0xFF voor 128 bits
  // The lower 8 bits configure baud rate divider = (cmp[ 7:0] + 1) * 2
  // Lower 8 bits -> (0 + 1) * 2 -> divide frequency by 2
  IMXRT_FLEXIO1_S.TIMCMP[0] = 0x0000FF00;
  // Timer compare 50.3.3.3 page 2891
  // Configure baud rate of the shift clock (cmp[15:0] + 1) * 2
  // -> (31 + 1) * 2 -> divide frequency by 64
  // When the counter equals zero and decrements the timer output toggles
  // Pulses generated -> 32 pulses of TIMER0 and 1 pulse of TIMER1
  IMXRT_FLEXIO1_S.TIMCMP[1] = 0x0000001F;

  // Shiftbuffers 1 & 2 get filled by DMA later. Start with all zero's
  // to reset/latch the led's. See 50.5.1.6.3 page 2918 for DMA triggering.
  IMXRT_FLEXIO1_S.SHIFTBUFBIS[0] = 0x00000000;
  IMXRT_FLEXIO1_S.SHIFTBUFBIS[1] = 0x00000000;
  IMXRT_FLEXIO1_S.SHIFTBUFBIS[2] = 0x00000000;
  IMXRT_FLEXIO1_S.SHIFTBUFBIS[3] = 0x00000000;
  // Enable DMA trigger when SHIFTBUF[1 or 2] is loaded onto the shifter
  IMXRT_FLEXIO1_S.SHIFTSDEN |= 0x06;
  // Enable flexio, SHIFTBUF[0] has been written, so TIMER0 will start
  // and other buffers are also ready so no errors in shifting data
  IMXRT_FLEXIO1_S.CTRL = FLEXIO_CTRL_FLEXEN;
}

void TDWS2811::configurePll(void)
{
  /* Set up PLL5 (also known as "PLL_VIDEO" and "PLL_528"), connect to FlexIO1 */
  uint32_t pllVideo;

  /* Disable the Video PLL output before initial Video PLL configuration */
  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_ENABLE_MASK;

  /* Bypass PLL first */
  CCM_ANALOG_PLL_VIDEO = (CCM_ANALOG_PLL_VIDEO & (~CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK)) |
                         CCM_ANALOG_PLL_VIDEO_BYPASS_MASK | CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC(0);

  /* Set numerator and denominator */
  CCM_ANALOG_PLL_VIDEO_NUM = CCM_ANALOG_PLL_VIDEO_NUM_A(2);
  CCM_ANALOG_PLL_VIDEO_DENOM = CCM_ANALOG_PLL_VIDEO_DENOM_B(3);

  /* Set DIV */
  pllVideo = (CCM_ANALOG_PLL_VIDEO & (~(CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK | CCM_ANALOG_PLL_VIDEO_POWERDOWN_MASK))) |
             CCM_ANALOG_PLL_VIDEO_ENABLE_MASK | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(42); //32??

  /* Set the post divider.  To reduce operation by a factor of 4 (for debugging) change from a divide ratio of 2 to 4 */
  pllVideo |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(2);
  // pllVideo |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(0);

  /* Write the PLL divider and post divider to the configuration register */
  CCM_ANALOG_PLL_VIDEO = pllVideo;

  /* Don't remember, TODO: remember */
  CCM_ANALOG_MISC2 = (CCM_ANALOG_MISC2 & (~CCM_ANALOG_MISC2_VIDEO_DIV_MASK)) | CCM_ANALOG_MISC2_VIDEO_DIV(0); //2?

  /* Wait for the PLL to lock.  If it doesn't lock, well, wait some more. */
  while ((CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK_MASK) == 0)
  {
  }

  /* Disable bypass for Video PLL once it's locked. */
  CCM_ANALOG_PLL_VIDEO &= ~(uint32_t(CCM_ANALOG_PLL_VIDEO_BYPASS_MASK));

  /* Disable FlexIO clock gate */
  hw->clock_gate_register &= ~(uint32_t(hw->clock_gate_mask));

  /* Set FLEXIO1_CLK_PRED. Set PODF=0 for full speed operation, PDOF=7 for 1/8 speed operation for debugging */
  CCM_CDCDR = (CCM_CDCDR &
               ~(CCM_CDCDR_FLEXIO1_CLK_SEL(3) | CCM_CDCDR_FLEXIO1_CLK_PRED(7) | CCM_CDCDR_FLEXIO1_CLK_PODF(7))) |
              CCM_CDCDR_FLEXIO1_CLK_SEL(2) | CCM_CDCDR_FLEXIO1_CLK_PRED(4) | CCM_CDCDR_FLEXIO1_CLK_PODF(0);
  //      | CCM_CDCDR_FLEXIO1_CLK_SEL(2) | CCM_CDCDR_FLEXIO1_CLK_PRED(4) | CCM_CDCDR_FLEXIO1_CLK_PODF(7);
}

void TDWS2811::configureDma()
{
  /* Enable DMA trigger on Shifter 1 */
  p->SHIFTSDEN |= 0X00000002;

  /* TCD 0 is responsible for the bulk of the data transfer.  It shuffles data from the frame buffer to Shifter 1 */
  dmaSetting[0].sourceBuffer(frameBuffer[0], 24 * num_leds * 4);
  dmaSetting[0].destination(p->SHIFTBUFBIS[1]);
  dmaSetting[0].replaceSettingsOnCompletion(dmaSetting[1]);

  /* TCD 1 is responsible for setting Shifter 0 (Phase 1) to zero.  I tried "source(zeros[0])", but it doesn't work.  Maybe that's a byte-wide operation?
  Note that the DMA channel is triggered by FlexIO shifter 1.  The only way to acknowledge the trigger is to write to one of Shifter 1's data registers.
  TCD 1 only touches Shifter 0's register, so the DMA channel is immediately triggered again upon TCD 1's completion, moving us directly into TCD 2. */
  dmaSetting[1].sourceBuffer(zeros, 4);
  dmaSetting[1].destination(p->SHIFTBUF[0]);
  dmaSetting[1].replaceSettingsOnCompletion(dmaSetting[2]);

  /* TCD 2 is responsible for the blanking delay.  That delay is, in turn, dictated by the size of the "zeros" array.
  As this TCD executes once per (1.25us) bit period, an array of 40 zeros gets us a 50us delay.*/
  dmaSetting[2].sourceBuffer(zeros, sizeof(zeros));
  dmaSetting[2].destination(p->SHIFTBUF[1]);
  dmaSetting[2].replaceSettingsOnCompletion(dmaSetting[3]);

  /* TCD 3 is responsible for setting Shifter 0 (Phase 1) to ones.  I tried "source(&ones)", but it doesn't work.  Maybe that's a byte-wide operation?
  Note that the DMA channel is triggered by FlexIO shifter 1.  The only way to acknowledge the trigger is to write to one of Shifter 1's data registers.
  TCD 3 only touches Shifter 0's register, so the DMA channel is immediately triggered again upon TCD 3's completion, moving us directly into TCD 0.*/
  dmaSetting[3].sourceBuffer(&ones, 4);
  dmaSetting[3].destination(p->SHIFTBUF[0]);
  dmaSetting[3].replaceSettingsOnCompletion(dmaSetting[0]);

  /* Now set up the DMA channel and initialize it with TCD 0 */
  dmaChannel.disable();
  dmaChannel = dmaSetting[0];
  dmaChannel.triggerAtHardwareEvent(hw->shifters_dma_channel[1]);
  dmaChannel.attachInterrupt(&_dmaIsr);
  /* Needed for the ISR, for reasons relating to using C++ in an embedded environment, which is most foolish */
  pTD = this;
  dmaChannel.enable();
}

int TDWS2811::setLed(uint8_t channel, uint8_t led, color_t color, bufferType_t writeBuffer)
{
  /* Set an LED's color and intensity */
  if (led > num_leds)
  {
    return 0;
  }
  if (channel > 31)
  {
    return 0;
  }

  uint16_t base = 0;
  uint32_t ledVal = 0;
  uint8_t bitCount = 0;
  uint8_t buf = activeBuffer;
  if (writeBuffer == INACTIVE)
    buf = 1 - activeBuffer;

  if (channelType[channel] == RGB)
  {
    bitCount = 24;
    ledVal = (color.red << 24) + (color.green << 16) + (color.blue << 8);
  }

  else if (channelType[channel] == GRB)
  {
    bitCount = 24;
    ledVal = (color.green << 24) + (color.red << 16) + (color.blue << 8);
  }

  // FIXME
  // else if (channelType[channel] == GRBW)
  // {
  //   bitCount = 32;
  //   ledVal = (color.green << 24) + (color.red << 16) + (color.blue << 8) + color.white;
  // }

  else
  {
    return 0;
  }

  uint8_t i;
  uint32_t mask = 1 << channel;

  base = bitCount * led;
  for (i = 0; i < bitCount; i++)
  {
    if (ledVal & 0x80000000)
    {
      frameBuffer[buf][base + i] |= mask;
    }
    else
    {
      frameBuffer[buf][base + i] &= ~mask;
    }
    ledVal = ledVal << 1;
  }

  return 1;
}

typename TDWS2811::color_t TDWS2811::getLed(uint8_t channel, uint16_t led)
{
  /* Returns the color and intensity value of an LED */
  uint32_t mask = 1 << channel;
  uint8_t i, bitCount = 24;
  // FIXME
  // if (channelType[channel] == GRBW)
  //   bitCount = 32;
  uint16_t base = led * bitCount;
  uint32_t out = 0;
  color_t color;
  for (i = 0; i < bitCount; i++)
  {
    out <<= 1;
    if (frameBuffer[activeBuffer][base + i] & mask)
      out += 1;
  }

  switch (channelType[channel])
  {
  case RGB:
    color.blue = out & 0xFF;
    out >>= 8;
    color.green = out & 0xFF;
    out >>= 8;
    color.red = out & 0xFF;
    break;

  case GRB:
    color.blue = out & 0xFF;
    out >>= 8;
    color.red = out & 0xFF;
    out >>= 8;
    color.green = out & 0xFF;
    break;

    // FIXME
    // case GRBW:
    //   color.white = out & 0xFF;
    //   out >>= 8;
    //   color.blue = out & 0xFF;
    //   out >>= 8;
    //   color.red = out & 0xFF;
    //   out >>= 8;
    //   color.green = out & 0xFF;
    //   break;

  default:
    return {0, 0, 0, 0};
  }
  return color;
}

#endif
