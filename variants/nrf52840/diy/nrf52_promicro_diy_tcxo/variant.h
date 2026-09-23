#ifndef _VARIANT_PROMICRO_DIY_
#define _VARIANT_PROMICRO_DIY_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

// #define USE_LFXO // Board uses 32khz crystal for LF
#define USE_LFRC // Board uses RC for LF

#define PROMICRO_DIY_TCXO

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
NRF52 PRO MICRO PIN ASSIGNMENT

| Pin   | Function    |     | Pin      | Function     | RF95  |
| ----- | ----------- | --- | -------- | ------------ | ----- |
| Gnd   |             |     | vbat     |              |       |
| P0.06 | Serial2 RX  |     | vbat     |              |       |
| P0.08 | Serial2 TX  |     | Gnd      |              |       |
| Gnd   |             |     | reset    |              |       |
| Gnd   |             |     | ext_vcc  | *see 0.13    |       |
| P0.17 | RXEN        |     | P0.31    | BATTERY_PIN  |       |
| P0.20 | GPS_TX      |     | P0.29    | BUSY         | DIO0  |
| P0.22 | GPS_RX      |     | P0.02    | MISO         | MISO  |
| P0.24 | GPS_EN      |     | P1.15    | MOSI         | MOSI  |
| P1.00 | BUTTON_PIN  |     | P1.13    | CS           | CS    |
| P0.11 | SCL         |     | P1.11    | SCK          | SCK   |
| P1.04 | SDA         |     | P0.10    | DIO1/IRQ     | DIO1  |
| P1.06 | Free pin    |     | P0.09    | RESET        | RST   |
|       |             |     |          |              |       |
|       | Mid board   |     |          | Internal     |       |
| P1.01 | Free pin    |     | 0.15     | LED          |       |
| P1.02 | Free pin    |     | 0.13     | 3V3_EN       |       |
| P1.07 | Free pin    |     |          |              |       |
*/

/*
Alternative pin assignement for Easy E22 Promicro build
https://github.com/brad112358/easy_E22

| Pin   | Function    |     | Pin      | Function     |
| ----- | ----------- | --- | -------- | ------------ |
| Gnd   |             |     | vbat     |              |
| P0.06 | BUTTON_PIN  |     | vbat     |              |
| P0.08 | PIN_BUZZER  |     | Gnd      |              |
| Gnd   |             |     | reset    |              |
| Gnd   |             |     | ext_vcc  | *see 0.13    |
| P0.17 | Serial2 TX  |     | P0.31    | BATTERY_PIN  |
| P0.20 | Rotary B    |     | P0.29    | DIO1         |
| P0.22 | Rotary A    |     | P0.02    | BUSY         |
| P0.24 | Rotary press|     | P1.15    | NRST         |
| P1.00 | TXEN        |     | P1.13    | MISO         |
| P0.11 | RXEN        |     | P1.11    | MOSI         |
| P1.04 | SDA         |     | P0.10    | SCK          |
| P1.06 | SCL         |     | P0.09    | CS           |
|       |             |     |          |              |
|       | Mid board   |     |          | Internal     |
| P1.01 | Serial2 RX  |     | 0.15     | LED          |
| P1.02 | TX to GPS   |     | 0.13     | 3V3_EN       |
| P1.07 | RX from GPS |     |          |              |
*/

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (1)
#define NUM_ANALOG_OUTPUTS (0)

// Pin 13 enables 3.3V periphery. If the Lora module is on this pin, then it should stay enabled at all times.
#define PIN_3V3_EN (0 + 13) // P0.13

// Analog pins
#define BATTERY_PIN (0 + 31) // P0.31 Battery ADC
#define ADC_CHANNEL ADC1_GPIO4_CHANNEL
#define ADC_RESOLUTION 14
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
// Definition of milliVolt per LSB => 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_MV_PER_LSB (0.73242188F)
// NAVARICO-V6 (23/09): ESTA PLACA LLEVA DIVISOR 1M/1M (factor 0.5), NO el 1.5M+1M (0.6) que trae de
// fabrica la variante de 2.8 para OTRO montaje. Con el valor de fabrica (1.73) la tension se leia un
// 13,5% POR DEBAJO de la real y el porcentaje de bateria se caia a 0.
// El valor correcto es el que usa NavaTastic V5.3 en esta misma placa: 2.0. Verificado linea a linea
// contra su variante: el resto de campos del ADC (canal, AREF 3.0, 12 bits, OCV) ya coincidian.
#define VBAT_DIVIDER (0.5F)
// Compensation factor for the VBAT divider (1 / 0.5 = 2.0)
#define VBAT_DIVIDER_COMP 2.0
// Fixed calculation of milliVolt from compensation value
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER VBAT_DIVIDER_COMP // REAL_VBAT_MV_PER_LSB
#define VBAT_RAW_TO_SCALED(x) (REAL_VBAT_MV_PER_LSB * x)

// WIRE IC AND IIC PINS
#define WIRE_INTERFACES_COUNT 1

#ifndef EASYPROMICRO
#define PIN_WIRE_SDA (32 + 4) // P1.04
#define PIN_WIRE_SCL (0 + 11) // P0.11
#else                         // Easy E22 Promicro arrangement
#define PIN_WIRE_SDA (32 + 4) // P1.04
#define PIN_WIRE_SCL (32 + 6) // P1.06
#endif

// LED
#define PIN_LED1 (0 + 15) // P0.15
// Actually red
#define LED_BLUE PIN_LED1
#define LED_STATE_ON 1 // State when LED is lit

// Button
#ifndef EASYPROMICRO
#define BUTTON_PIN (32 + 0) // P1.00
#else                       // Easy E22 Promicro arrangement
#define BUTTON_PIN (0 + 6)  // P0.06
#endif

// GPS
#ifndef EASYPROMICRO
#define GPS_TX_PIN (0 + 20) // P0.20 - This is data from the MCU
#define GPS_RX_PIN (0 + 22) // P0.22 - This is data from the GNSS
#define PIN_GPS_EN (0 + 24) // P0.24
#else                       // Easy E22 Promicro arrangement
#define GPS_TX_PIN (32 + 2) // P1.02 - This is data from the MCU
#define GPS_RX_PIN (32 + 7) // P1.07 - This is data from the GNSS
#define PIN_GPS_EN (0 + 13) // P0.13 - Use power control for GPS enable
#endif

#define GPS_UBLOX
// #define GPS_DEBUG 1

// UART interfaces
#define PIN_SERIAL1_TX GPS_TX_PIN
#define PIN_SERIAL1_RX GPS_RX_PIN

#ifndef EASYPROMICRO
#define PIN_SERIAL2_RX (0 + 6)  // P0.06
#define PIN_SERIAL2_TX (0 + 8)  // P0.08
#else                           // Easy E22 Promicro arrangement
#define PIN_SERIAL2_RX (32 + 1) // P1.01
#define PIN_SERIAL2_TX (0 + 17) // P0.17
// Buzzer - PWM
#define PIN_BUZZER (0 + 8)      // p0.08
#endif

// Serial interfaces
#define SPI_INTERFACES_COUNT 1

#ifndef EASYPROMICRO
#define PIN_SPI_MISO (0 + 2)   // P0.02
#define PIN_SPI_MOSI (32 + 15) // P1.15
#define PIN_SPI_SCK (32 + 11)  // P1.11

#define LORA_MISO PIN_SPI_MISO
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_SCK PIN_SPI_SCK
#define LORA_CS (32 + 13) // P1.13

// LORA MODULES
// NAVARICO-V6: las 4 radios del proyecto (HT-RA62, E22, E80, E22P) son todas SX126x. RF95 (SX127x),
// LR1121 y LR2021 son herencia de la plantilla upstream y solo engordan el binario (esta es la
// imagen mas grande de nRF52 y va justa contra el tope de 0xEA000). Se eligen POR ENV con
// -DNAVARICO_SOLO_SX126X (envs navarrico_*); quien suelde un LR1121/RF95 compila sin el flag.
#define USE_LLCC68
#define USE_SX1262
#define USE_SX1268
#ifndef NAVARICO_SOLO_SX126X
#define USE_RF95
#define USE_LR1121
#define USE_LR2021
#endif

// RF95 CONFIG
#define LORA_DIO0 (0 + 29) // P0.29 BUSY
#define LORA_DIO1 (0 + 10) // P0.10 IRQ
#define LORA_RESET (0 + 9) // P0.09 NRST

// RX/TX for RFM95/SX127x
#define RF95_RXEN (0 + 17)    // P0.17
#define RF95_TXEN RADIOLIB_NC // Assuming that DIO2 is connected to TXEN pin. If not, TXEN must be connected.

// SX126X CONFIG
#define SX126X_CS (32 + 13)      // P1.13 FIXME - we really should define LORA_CS instead
#define SX126X_DIO1 (0 + 10)     // P0.10 IRQ
#define SX126X_DIO2_AS_RF_SWITCH // Note for E22 modules: DIO2 is not attached internally to TXEN for automatic TX/RX switching,
                                 // so it needs connecting externally if it is used in this way
#define SX126X_BUSY (0 + 29)     // P0.29
#define SX126X_RESET (0 + 9)     // P0.09
#define SX126X_RXEN (0 + 17)     // P0.17
#define SX126X_TXEN RADIOLIB_NC  // Assuming that DIO2 is connected to TXEN pin. If not, TXEN must be connected.

// LR1121
#ifdef USE_LR1121
#define LR1121_IRQ_PIN (0 + 10)      // P0.10 IRQ
#define LR1121_NRESET_PIN LORA_RESET // P0.09 NRST
#define LR1121_BUSY_PIN (0 + 29)     // P0.29 BUSY
#define LR1121_SPI_NSS_PIN LORA_CS   // P1.13
#define LR1121_SPI_SCK_PIN LORA_SCK
#define LR1121_SPI_MOSI_PIN LORA_MOSI
#define LR1121_SPI_MISO_PIN LORA_MISO
#define LR11X0_DIO3_TCXO_VOLTAGE 1.8
#define LR11X0_DIO_AS_RF_SWITCH
#endif

// LR2021
#ifdef USE_LR2021
#define LR2021_IRQ_PIN (0 + 10)      // P0.10 IRQ
#define LR2021_NRESET_PIN LORA_RESET // P0.09 NRST
#define LR2021_BUSY_PIN (0 + 29)     // P0.29 BUSY
#define LR2021_SPI_NSS_PIN LORA_CS   // P1.13
#define LR2021_DIO3_TCXO_VOLTAGE 1.8
#define LR2021_DIO_AS_RF_SWITCH
#define LR2021_IRQ_DIO_NUM 9 // DIO9 → P0.10
#endif

// NAVARICO-V6: guarda de seguridad. Sin ninguna familia declarada el firmware compila pero no tiene
// driver de radio: arranca mudo. Fallar el build es preferible a un nodo que no se ve en la malla.
#if !defined(USE_SX1262) && !defined(USE_SX1268) && !defined(USE_LLCC68) && !defined(USE_RF95) &&        \
    !defined(USE_LR1121) && !defined(USE_LR2021)
#error "NAVARICO-V6: no hay ninguna familia de radio declarada en este variant.h"
#endif

#else // Easy E22 Promicro arrangement
#define USE_SSD1306
// #define USE_SH1106

#define USE_SX1262
#define PIN_SPI_MISO (32 + 13) // P1.13
#define PIN_SPI_MOSI (32 + 11) // P1.11
#define PIN_SPI_SCK (0 + 10)   // P0.10

#define LORA_MISO PIN_SPI_MISO
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_SCK PIN_SPI_SCK
#define LORA_CS (0 + 9)      // P0.09 NSS
#define LORA_DIO0 (0 + 2)    // P0.02 BUSY
#define LORA_DIO1 (0 + 29)   // P0.29 IRQ
#define LORA_RESET (32 + 15) // P1.15 NRST

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_DIO0
#define SX126X_RESET LORA_RESET
#define SX126X_RXEN (0 + 11) // P0.11
#define SX126X_TXEN (32 + 0) // P1.00
#define SX126X_MAX_POWER 8   // Default to prevent damage to E22_900M33S; Comment out for others
#undef TX_GAIN_LORA
#define TX_GAIN_LORA 22 // 8 for E22 900M30S, 25 for 900M33S, 22 for 3.7V battery powered 900M33S,  0 for 900M22S
#endif

// NAVARICO-V6 (bloque 2 del portaje V5.3): TOPE DE MANDO de la potencia, el que anuncia y acepta
// `set_txpower`. NO es el limite fisico de la radio (ese lo pone SX126X_MAX_POWER).
// OJO: esta linea esta en la rama #ifndef EASYPROMICRO, que es la que usan NUESTROS envs. La rama
// EasyProMicro (env nrf52_promicro_diy-easypromicro, que no compilamos) es la que lleva
// SX126X_MAX_POWER 8 "para no dañar el E22_900M33S": no aplica a nuestra placa.
// Valor por radio, igual que en NavaTastic V5.1 (E22P 12 / HT-RA62 SX1262 22), para que los dos
// firmwares acepten exactamente los mismos valores.
#ifdef NAVARICO_RADIO_E22P
#define NAVA_MAX_TX_POWER_DBM 12 // E22P: el maximo que el usuario puede configurar
#else
#define NAVA_MAX_TX_POWER_DBM 22 // HT-RA62 / SX1262
#endif

// #define SX126X_MAX_POWER 8 // set this if using a high-power board!

/*
On the SX1262, DIO3 sets the voltage for an external TCXO, if one is present. If one is not present, use TCXO_OPTIONAL to try both
settings.

| Mfr          | Module           | TCXO | RF Switch | Notes                                 |
| ------------ | ---------------- | ---- | --------- | ------------------------------------- |
| Ebyte        | E22-900M22S      | Yes  | Ext       |                                       |
| Ebyte        | E22-900MM22S     | No   | Ext       |                                       |
| Ebyte        | E22-900M30S      | Yes  | Ext       |                                       |
| Ebyte        | E22-900M33S      | Yes  | Ext       | MAX_POWER must be set to 8 for this   |
| Ebyte        | E220-900M22S     | No   | Ext       | LLCC68, looks like DIO3 not connected |
| AI-Thinker   | RA-01SH          | No   | Int       | SX1262                                |
| Heltec       | HT-RA62          | Yes  | Int       |                                       |
| NiceRF       | Lora1262         | yes  | Int       |                                       |
| Waveshare    | Core1262-HF      | yes  | Ext       |                                       |
| Waveshare    | LoRa Node Module | yes  | Int       |                                       |
| Seeed        | Wio-SX1262       | yes  | Ext       | Cute! DIO2/TXEN are not exposed       |
| Seeed        | Wio-LR1121       | yes  | Int       | LR1121, build -D LR1121_MODULE_WIO    |
| AI-Thinker   | RA-02            | No   | Int       | SX1278 **433mhz band only**           |
| RF Solutions | RFM95            | No   | Int       | Untested                              |
| Ebyte        | E80-900M2213S    | Yes  | Int       | LR1121 radio                          |

*/

#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#define TCXO_OPTIONAL // make it so that the firmware can try both TCXO and XTAL

// E-Ink DIY
#define PIN_EINK_CS (32 + 7)
#define PIN_EINK_DC (32 + 2)
#define PIN_EINK_RES (32 + 1)
#define PIN_EINK_BUSY (32 + 6)

// NAVARICO-V6: DESPERTAR POR BATERIA (LPCOMP). Meshtastic 2.8 lo tiene desactivado en esta placa;
// NavaTastic V5.1 lo activaba y es el mecanismo que hace que un nodo solar que se apago por bateria
// baja VUELVA a arrancar cuando el sol recarga. Sin esto, el nodo se queda apagado hasta que alguien
// lo encienda a mano (en la montana, sin boton: mudo). Decision del operador (13/09), coherente con
// el diseño de resiliencia del proyecto.
// Umbral: AIN7 con el divisor 0.5 del Promicro (1M/1M) -> 9/16 de VDD = ~3.71V reales de bateria.
// El nivel efectivo lo puede ajustar el operador en caliente con /nava set_vwake (niveles 1-5);
// este define es solo el valor por defecto del arranque.
#define BATTERY_LPCOMP_INPUT NRF_LPCOMP_INPUT_7
#define BATTERY_LPCOMP_THRESHOLD NRF_LPCOMP_REF_SUPPLY_9_16

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
