# STM32F411 MPU-6500 IMU Driver


This project implements **SPI + DMA communication** with the MPU-6500 6-axis IMU on an STM32F401 microcontroller. The sensor streams 3-axis accelerometer, 3-axis gyroscope, and temperature data. Raw samples are averaged before being output.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Microcontroller | STM32F411 (Black Pill) @ 84 MHz |
| Sensor | MPU-6500 6-axis IMU |
| Communication | SPI1, full-duplex, DMA-driven |
| SPI Mode | Mode 3 (CPOL=1, CPHA=1) |
| SCK | PA5 |
| MISO | PA6 |
| MOSI | PA7 |
| CS | PB0 (software-controlled) |
| MPU INT | PA0 (EXTI0 — data-ready interrupt) |
| TIM2 | Millisecond wall clock (update interrupt) |
| TIM5 | Free-running microsecond counter |

MPU-6500: REGISTER MAP ->   https://www.glynstore.com/content/docs/invensense/RM-MPU-6555A.pdf

MPU-6500: Datasheet -> https://datasheet.octopart.com/MPU-6500-InvenSense-datasheet-138896167.pdf

---

## Project Structure

```
├── main.c              # Entry point, ISR handlers (DMA, EXTI0, EXTI1, TIM2)
├── spi.c / spi.h       # SPI init, DMA-based transact, CS control, RX callback.
├── mpu6500.c / .h      # MPU-6500 init, read/write, scaling, calibration
├── mpuRegister.h       # Full MPU-6500 register map and bit definitions
├── swa.c / swa.h       # Array Averaging Filter + MPU data update logic
├── exti.c / exti.h     # EXTI init for hardware (MPU INT) and software interrupts
├── dma.c / dma.h       # DMA flag clear/check helpers 
├── timer.c / timer.h   # General-purpose timer init, wall clock timer, interrupt enable
├── wallclock.c / .h    # Millisecond/microsecond wall clock, blocking & non-blocking delays
├── boardfile.c / .h    # Board-level pin/DMA/buffer configuration arrays
└── clock_init.c / .h  # System clock config (HSE 25 MHz → PLL → 84 MHz SYSCLK)
```

---

## How the SPI Queue Works

```c
spiTransact(&spiData, txBuffer, length, callbackFn, ctx);
// ├── Copies txBuffer → spiBufferTx
// ├── Asserts CS low
// ├── Starts RX DMA, then TX DMA simultaneously
// └── Sets spiStatus = SPI_PROCESSING  (blocks re-entry)

DMA2_Stream0_IRQHandler()
└── spiProcessRx()                  // RX DMA complete
    ├── Deasserts CS high
    ├── Invokes rxCallBack(void* targetstruct, spiBufferRx, length)
    └── Sets spiStatus = SPI_IDLE
```

## Interrupt Handlers

There are 3 interrupts that drive the entire data pipeline. They fire in order, one after the other.

---

### 1. EXTI0_IRQHandler — MPU-6500 Data Ready

**Trigger:** The MPU-6500 pulls its INT pin high (PA0) every time a new sensor sample is ready inside the chip.

**What it does:**
1. Clears the EXTI0 pending flag
2. Calls `mpu6500Read()` which builds a 15-byte TX buffer (register address + 14 dummy bytes) and hands it to `spiTransact()`
3. `spiTransact()` loads the TX buffer into DMA, pulls CS low, and kicks off both the TX and RX DMA streams simultaneously
4. Returns immediately — the actual data arrives later via the DMA interrupt

This IRQ only starts the SPI transfer. It does not wait for data.

---

### 2. DMA2_Stream0_IRQHandler — SPI RX Transfer Complete

**Trigger:** DMA2 Stream 0 fires when it has finished moving the incoming SPI bytes from the SPI data register into `spiBufferRx` in memory.

**What it does:**
1. Calls `spiProcessRx()`
2. Inside `spiProcessRx()`:
   - Clears the DMA transfer-complete flag
   - Pulls CS high (ends the SPI transaction)
   - Calls `rxCallBack()` — which is `mpu6500ReadData()`
   - Sets `spiStatus = SPI_IDLE` so the next transaction can be accepted
3. Inside `mpu6500ReadData()` (the callback):
   - Copies `spiBufferRx` into `mpu->rxbuffer`
   - Combines the high and low bytes for each axis into 16-bit signed integers
   - Calls `MPU6500_Update()` which feeds each raw value into the averaging filter
   - Increments the global sample counter `count`
   - When `count` reaches 5, it calls `softwareInterruptTrigger()` and resets `count` to 0

This IRQ does the actual data parsing and filtering.

---

### 3. EXTI1_IRQHandler — Software Triggered, Averaged Data Ready

**Trigger:** Not a hardware pin. `softwareInterruptTrigger()` writes to the EXTI software interrupt register (`LL_EXTI_GenerateSWI_0_31`) which immediately pends EXTI1 in the NVIC. This is called from inside `mpu6500ReadData()` once every 5 samples.

**What it does:**
1. Clears the EXTI1 pending flag
2. Sets `flag = 1` in main
3. The main loop checks `flag`, prints the latest averaged accelerometer, gyroscope, and temperature values via `printf`, then clears `flag`

This IRQ is the bridge between the ISR context (where data is parsed) and the main loop (where data is printed). It fires at 1/5th the rate of EXTI0.

---

<svg width="820" height="220" viewBox="0 0 820 220" xmlns="http://www.w3.org/2000/svg" font-family="ui-monospace,SFMono-Regular,Menlo,monospace">
<defs>
  <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
    <path d="M2 1L8 5L2 9" fill="none" stroke="#888" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
  </marker>
</defs>

<!-- MPU-6500 -->
<rect x="20" y="88" width="110" height="44" rx="8" fill="#f1f0e8" stroke="#5f5e5a" stroke-width="0.5"/>
<text x="75" y="114" text-anchor="middle" font-size="13" font-weight="500" fill="#2c2c2a">MPU-6500</text>

<!-- Arrow 1 -->
<line x1="130" y1="110" x2="238" y2="110" stroke="#888" stroke-width="1.5" marker-end="url(#arrow)"/>
<text x="184" y="101" text-anchor="middle" font-size="11" fill="#666">INT / PA0</text>

<!-- EXTI0 -->
<rect x="240" y="68" width="156" height="84" rx="8" fill="#e6f1fb" stroke="#185fa5" stroke-width="0.5"/>
<text x="318" y="96" text-anchor="middle" font-size="13" font-weight="500" fill="#0c447c">EXTI0_IRQHandler</text>
<text x="318" y="114" text-anchor="middle" font-size="11" fill="#185fa5">Pull CS low</text>
<text x="318" y="130" text-anchor="middle" font-size="11" fill="#185fa5">Start SPI + DMA</text>

<!-- Arrow 2 -->
<line x1="396" y1="110" x2="494" y2="110" stroke="#888" stroke-width="1.5" marker-end="url(#arrow)"/>
<text x="445" y="101" text-anchor="middle" font-size="11" fill="#666">RX done</text>

<!-- DMA2_Stream0 -->
<rect x="496" y="68" width="168" height="84" rx="8" fill="#e1f5ee" stroke="#0f6e56" stroke-width="0.5"/>
<text x="580" y="91" text-anchor="middle" font-size="11" font-weight="500" fill="#085041">DMA2_Stream0_IRQHandler</text>
<text x="580" y="109" text-anchor="middle" font-size="11" fill="#0f6e56">Parse · filter</text>
<text x="580" y="126" text-anchor="middle" font-size="11" fill="#0f6e56">count++ → SWI @ 5</text>

<!-- Arrow 3 -->
<line x1="664" y1="110" x2="718" y2="110" stroke="#888" stroke-width="1.5" marker-end="url(#arrow)"/>
<text x="691" y="101" text-anchor="middle" font-size="11" fill="#666">SWI</text>

<!-- EXTI1 -->
<rect x="720" y="68" width="76" height="84" rx="8" fill="#faece7" stroke="#993c1d" stroke-width="0.5"/>
<text x="758" y="95" text-anchor="middle" font-size="13" font-weight="500" fill="#712b13">EXTI1</text>
<text x="758" y="113" text-anchor="middle" font-size="11" fill="#993c1d">flag</text>
<text x="758" y="129" text-anchor="middle" font-size="11" fill="#993c1d">= 1</text>

<!-- Arrow down to main -->
<line x1="758" y1="152" x2="758" y2="183" stroke="#888" stroke-width="1.5" marker-end="url(#arrow)"/>

<!-- main loop — two lines to fit comfortably -->
<rect x="692" y="183" width="112" height="32" rx="6" fill="#f1f0e8" stroke="#5f5e5a" stroke-width="0.5"/>
<text x="748" y="196" text-anchor="middle" font-size="10" fill="#2c2c2a">main: printf</text>
<text x="748" y="209" text-anchor="middle" font-size="10" fill="#2c2c2a">averaged data</text>

<!-- Rate labels -->
<text x="318" y="178" text-anchor="middle" font-size="10" fill="#aaa">every sample (1 kHz)</text>
<text x="580" y="178" text-anchor="middle" font-size="10" fill="#aaa">every sample</text>
<text x="726" y="172" text-anchor="middle" font-size="10" fill="#aaa">every 5th</text>
</svg>

##  MPU-6500 Setup Flow
,
### Initialization

```
mpu6500Init()
    │
    ├── WHO_AM_I check
    ├── Device RESET  (PWR_MGMT_1)
    ├── Signal path RESET
    ├── SET_CLOCK_SOURCE  → auto-select best clock
    ├── Set SMPLRT_DIV
    ├── Configure DLPF  (CONFIG register)
    ├── Set GYRO full-scale range  (GYRO_CONFIG)
    ├── Set ACCEL full-scale range  (ACCEL_CONFIG)
    ├── Set gyroScale / accelScale factors
    ├── Disable I2C interface  (USER_CTRL)
    └── Enable DATA_READY interrupt  (INT_ENABLE)
```

---

## Averaging Filter

```c
initSWA(&swa, windowSize);
// Zero-initialises array[], sum, avg

// Called on every new raw sample:
ArrayUpdate(&swa, rawValue);
// ├── array[count] = rawValue
// ├── sum += rawValue
// └── When count reaches MAX_WINDOW_SIZE (5):
//         avg  = sum / 5
//         sum  = 0
//         softwareInterruptTrigger()
```

Each axis has its own `SWA_t` instance inside `MPU6500_t`:

```c
typedef struct {
    SWA_t ax, ay, az;   // Accelerometer axes
    SWA_t gx, gy, gz;   // Gyroscope axes
    SWA_t temp;         // Temperature
} MPU6500_t;
```


---

## Clock Configuration

```
HSE (25 MHz)
    │  /M = 25
    ▼
  1 MHz  (PLL input)
    │  ×N = 336
    ▼
336 MHz  (VCO)
    ├──  /P = 4  →  84 MHz  SYSCLK
    └──  /Q = 7  →  48 MHz  USB clock

AHB  prescaler /1  →  84 MHz
APB2 prescaler /1  →  84 MHz   (SPI1, SYSCFG)
APB1 prescaler /2  →  42 MHz   (TIM2 timer clock = 84 MHz after ×2)
```

