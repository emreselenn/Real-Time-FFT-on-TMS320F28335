# Real-Time FFT on TMS320F28335

This project performs real-time FFT on a PWM signal using Texas Instruments' TMS320F28335 DSP. It samples a 2 kHz PWM waveform via ADC at 50 kHz, buffers 32 or 128 samples, and computes their FFT. The magnitude spectrum is calculated and can be visualized using Code Composer Studio's (CCS) Graph tool.

---

## 🔧 What This Project Actually Does

- Generates a **2 kHz PWM signal** on `ePWM1A` (25% duty cycle)
- Samples that PWM signal via **ADCINA2** at **50 kHz** (Timer0-triggered)
- Buffers **32 or 128 samples**, depending on a compile-time `#define`
- Applies **FFT** (via `FFT32()` or `FFT128()` functions)
- Computes the **magnitude spectrum**
- Stores results in a buffer (`x1[]`) that can be:
  - Observed in real-time using **CCS Graph Tool**
  - Optionally extended to UART or external display (not implemented here)
- Uses **LEDs (GPIO9, GPIO11, GPIO34)** as visual indicators for sampling and processing

---

## 🧠 Technical Details

- CPU Frequency: 150 MHz
- ADC Sample Rate: 50 kHz
- FFT Size: 32 or 128 points (configured via `#define PTS 32`)
- Data Type: Fixed-point `COMPLEX` struct with real & imag components
- Magnitude: `sqrt(Re² + Im²)` computed after FFT
- Visualization: CCS → Tools → Graph → Single Time → Linked to `x1[]` buffer

