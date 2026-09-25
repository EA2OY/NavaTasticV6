#include "UptimeClock.h"
#include "configuration.h"
#include "mesh/Throttle.h"
#include <Adafruit_TinyUSB.h>
#include <Adafruit_nRFCrypto.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Wire.h>

#define APP_WATCHDOG_SECS 90
#define NRFX_WDT_ENABLED 1
#define NRFX_WDT0_ENABLED 1

// NAVARICO-V6: variables que el motor /nava necesita y que en NavaTastic V5.1 vivian en este mismo
// fichero. rawResetReason = causa del ultimo reinicio (se rellena en setup(), ver mas abajo).
uint32_t rawResetReason = 0;

// NAVARICO-V6: el apagado REAL del BLE lo hace el motor poniendo config.bluetooth.enabled = false
// (mecanismo oficial de Meshtastic, y la 2.8 lo respeta). Esta marca solo deja constancia del
// estado pedido: en NavaTastic V5.1 se escribia pero NUNCA se leia (nadie la consultaba), asi que
// se conserva por compatibilidad de interfaz con el motor, no porque gobierne nada.
bool bleForceDisabled = false;
void setBleForceDisabled(bool on)
{
    bleForceDisabled = on;
}

// NAVARICO-V6 - MODO TORMENTA (hibernacion con RTC2): la funcion timedSystemSleepSeconds() vive mas
// abajo, DEBAJO de todos los #include, porque necesita declaraciones que el framework nordic aporta
// mas abajo en este fichero (nrf_rtc_*, screen, notifyDeepSleep, variant_shutdown). Ponerla aqui
// arriba NO compila: fue el primer intento y fallo.
#define NRFX_WDT_CONFIG_NO_IRQ 1
#include "nrfx_power.h"
#include <assert.h>
#include <ble_gap.h>
#include <memory.h>
#include <nrfx_wdt.c>
#include <nrfx_wdt.h>
#include <stdio.h>
// #include <Adafruit_USBD_Device.h>
#include "HardwareRNG.h"
#include "NodeDB.h"
#include "Power.h"
#include "PowerMon.h"
#include "error.h"
#include "main.h"
#include "meshUtils.h"
#include <power/PowerHAL.h>

#include "Nrf52SaadcLock.h"
#include "concurrency/LockGuard.h"
#include <hal/nrf_lpcomp.h>

#ifdef BQ25703A_ADDR
#include "BQ25713.h"
#endif

// NAVARICO-V6 (storm): includes que V5.3 declaraba explicitamente para la hibernacion RTC2.
// (2.8 no tenia NINGUNA linea de nrf_rtc_*: el RTC se porta completo con su manejador de IRQ.)
#include <hal/nrf_rtc.h> // nrf_rtc_*, NRF_RTC_TASK_*, RTC_CHANNEL_INT_MASK
#include "sleep.h"       // notifyDeepSleep, cpuDeepSleep
#include "graphics/Screen.h" // screen->doDeepSleep() (el objeto `screen` lo declara este header)
// Declaracion forward al estilo V5.3: variant_shutdown() se usa mas abajo en este fichero antes de su
// declaracion weak (mas abajo aun), y cada placa pone la suya en su variant.cpp.
// setBluetoothEnable() se DEFINE en este mismo fichero (L404) y cpuDeepSleep() la usa desde la L709,
// pero la hibernacion del storm esta MAS ARRIBA: necesita verla antes.
void variant_shutdown();
void setBluetoothEnable(bool enable);

// ---------------------------------------------------------------------------
// NAVARICO-V6 - MODO TORMENTA: hibernacion temporizada (RTC2 + LOWPWR).
// PORTADO de NavaTastic V5.3 (que lo tenia probado en campo) el 24/09/2026.
//
// Por que ASI y no con sd_power_system_off() (la razon esta escrita en el original y es la clave de
// que esto sea seguro): System OFF solo despierta por GPIO/LPCOMP/NFC/reset fisico, NO por
// temporizador; y con la bateria por encima del umbral del LPCOMP no habria flanco de subida, asi que
// el nodo quedaria DORMIDO PARA SIEMPRE. Aqui se usa System ON en bajo consumo + RTC2 COMPARE.
//
// El contador del RTC es de 24 bits a 32768 Hz (~512 s maximo por comparacion), asi que el tiempo
// pedido se cubre en bloques de 500 s re-armando el COMPARE en cada despertar.
// Al cumplirse el total se hace NVIC_SystemReset(): el arranque restaura radio y perifericos.
//
// OJO (2.8): el cpuDeepSleep() de 2.8 NO tiene despertar por temporizador (su propio comentario dice
// "FIXME, configure RTC or button press to wake us") y en todo el arbol de 2.8 no habia NI UNA linea
// de nrf_rtc_*. Por eso el RTC2 se porta completo, con su manejador de interrupcion.
// ---------------------------------------------------------------------------
#define STORM_BLOCK_SECS 500u
#define RTC_FREQ_HZ 32768u
#define RTC_CC_MAX 0xFFFFFFu

static volatile bool rtc2StormWake = false;

extern "C" void RTC2_IRQHandler(void)
{
    if (nrf_rtc_event_check(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0)) {
        nrf_rtc_event_clear(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0);
        rtc2StormWake = true;
    }
}

void timedSystemSleepSeconds(uint32_t seconds)
{
    if (seconds == 0)
        return;

    LOG_INFO("Storm: entrando en hibernacion RTC2 por %lu segundos", (unsigned long)seconds);

    // 1. Dormir la RADIO de verdad: notifyDeepSleep dispara RadioInterface::sleep() (setStandby +
    //    lora.sleep(keepConfig) en el SX1262). TIENE que ir ANTES de SPI.end(), porque el comando
    //    SLEEP viaja por SPI. Sin esto la radio se queda en RX consumiendo ~10 mA.
    notifyDeepSleep.notifyObservers(NULL);

#ifdef HAS_WIRE
    Wire.end();
#endif
    SPI.end();
    if (Serial)
        Serial.end();
#ifdef PIN_SERIAL1_RX
    if (Serial1)
        Serial1.end();
#endif
    setBluetoothEnable(false);

#ifdef RADIO_POWER_ENABLE_PIN
    // Apagar el modulo de radio E22P entero (~40 mA durante toda la hibernacion). Solo lo tienen las
    // placas con ese pin; el arranque posterior lo vuelve a levantar (nrf52Setup).
    pinMode(RADIO_POWER_ENABLE_PIN, OUTPUT);
    digitalWrite(RADIO_POWER_ENABLE_PIN, LOW);
#endif

    // Apagar pantalla (10 uA en sueno profundo)
    if (screen) {
        screen->doDeepSleep();
    }

    // Apagado especifico de la placa, igual que hace el cpuDeepSleep() de 2.8: cada variante puede
    // tener perifericos que consuman y solo ella sabe como dejarlos.
    variant_shutdown();

    // 2. Configurar el RTC2 (LFCLK 32768 Hz, prescaler 0).
    nrf_rtc_prescaler_set(NRF_RTC2, 0);
    nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);
    nrf_rtc_event_enable(NRF_RTC2, RTC_CHANNEL_INT_MASK(0));
    nrf_rtc_int_enable(NRF_RTC2, RTC_CHANNEL_INT_MASK(0));
    NVIC_ClearPendingIRQ(RTC2_IRQn);
    NVIC_EnableIRQ(RTC2_IRQn);

    uint32_t remaining = seconds;
    while (remaining > 0) {
        uint32_t block = (remaining > STORM_BLOCK_SECS) ? STORM_BLOCK_SECS : remaining;
        uint32_t target = (nrf_rtc_counter_get(NRF_RTC2) + block * RTC_FREQ_HZ) & RTC_CC_MAX;

        nrf_rtc_cc_set(NRF_RTC2, 0, target);
        nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);
        nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);

        rtc2StormWake = false;

        // Dormir la CPU hasta el COMPARE del RTC2. sd_app_evt_wait() es la API correcta con
        // SoftDevice: deja que el stack gestione el sueno real. __WFE() despertaba la CPU en bucle
        // por eventos internos del SoftDevice y anulaba el ahorro (documentado en el V5.3 original).
        while (!rtc2StormWake) {
            sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
            sd_app_evt_wait();
        }

        remaining -= block;
    }

    LOG_INFO("Storm: tiempo cumplido, reiniciando para restaurar la radio");

    // 3. Apagar el RTC2 y reiniciar: el arranque restaura radio y perifericos.
    nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_STOP);
    NVIC_DisableIRQ(RTC2_IRQn);
    LOG_ERROR("RESET SREQ: storm cumplido (uptime %lus) -- reinicio para restaurar la radio",
              (unsigned long)(millis() / 1000));
    NVIC_SystemReset();
    while (1) {
        delay(1000);
    }
}

// WARNING! THRESHOLD + HYSTERESIS should be less than regulated VDD voltage - which depends on board
// and is 3.0 or 3.3V. Also VDD likes to read values like 2.9999 so make sure you account for that
// otherwise board will not boot at all. Before you modify this part - please triple read NRF52840 power design
// section in datasheet and you understand how REG0 and REG1 regulators work together.
#ifndef SAFE_VDD_VOLTAGE_THRESHOLD
#define SAFE_VDD_VOLTAGE_THRESHOLD 2.7
#endif

// hysteresis value
#ifndef SAFE_VDD_VOLTAGE_THRESHOLD_HYST
#define SAFE_VDD_VOLTAGE_THRESHOLD_HYST 0.2
#endif

uint16_t getVDDVoltage();

// Weak empty variant shutdown prep function.
// May be redefined by variant files.
// noinline: same reason as variant_enableBatteryLpcompWake() below -- weak default and call
// site are in this file, so LTO would inline the empty body and drop the variant's override.
__attribute__((noinline)) void variant_shutdown() __attribute__((weak));
__attribute__((noinline)) void variant_shutdown() {}

// Optional variant hook called each nrf52Loop(); e.g. for low-VDD System OFF.
__attribute__((noinline)) void variant_nrf52LoopHook(void) __attribute__((weak));
__attribute__((noinline)) void variant_nrf52LoopHook(void) {}

// Return false to skip LPCOMP wake when entering System OFF (e.g. user CLI shutdown).
// noinline: weak default and call site are in this file; without it GCC may inline the
// weak body and never link the strong override from variant.cpp.
__attribute__((noinline)) bool variant_enableBatteryLpcompWake() __attribute__((weak));
__attribute__((noinline)) bool variant_enableBatteryLpcompWake()
{
    return true;
}

// NAVARICO-V6: UMBRAL LPCOMP POR PLACA.
// Por que existe: el LPCOMP compara la tension del pin (bateria x divisor de la placa) contra una
// fraccion de VDD. Los niveles de despertar 1-5 estan calibrados con el divisor del Promicro
// (0.5, 1M/1M), asi que en placas con OTRO divisor esos niveles serian inalcanzables (por ejemplo
// en la T114 el 9_16 equivaldria a 9.1V reales). Cada placa con divisor distinto usa su umbral de
// fabrica; el Promicro conserva los 5 niveles ajustables con /nava set_vwake.
// Es una LINEA ROJA del proyecto: no se toca sin orden expresa del operador.
//
// *** AVISO IMPORTANTE (13/09) ***: esta funcion solo se compila si la placa declara
// BATTERY_LPCOMP_INPUT, y Meshtastic 2.8 lo tiene DESACTIVADO en TODAS las placas de este proyecto
// (en la T114 esta comentado con el motivo: "power leakage of 2.9mA in shutdown state, issue #8801").
// NavaTastic V5.1 SI lo activaba en las 4 familias, porque sin despertar por LPCOMP un nodo solar
// que se apaga por bateria baja NO VUELVE a arrancar cuando el sol recarga (se queda mudo en el
// monte sin boton). Es una decision de diseño del operador, NO automatica: para tenerlo hay que
// descomentar BATTERY_LPCOMP_INPUT y BATTERY_LPCOMP_THRESHOLD en el variant.h de cada placa.
// Con la macro sin definir, este bloque NO se compila y el comportamiento es el de la 2.8.
#ifdef BATTERY_LPCOMP_INPUT
nrf_lpcomp_ref_t getActiveLpcompThreshold()
{
#ifdef SEEED_SOLAR_NODE
    // Seed Solar Node P1: divisor ~0.303 (no 0.5) -> umbral de fabrica (~3.67V real)
    return BATTERY_LPCOMP_THRESHOLD; // NRF_LPCOMP_REF_SUPPLY_3_8
#elif defined(SEEED_XIAO_NRF52840_KIT)
    // Xiao Kit i2c / Xiao E22P: divisor de fabrica 1M/510k (~0.3377) -> umbral de fabrica
    return BATTERY_LPCOMP_THRESHOLD; // NRF_LPCOMP_REF_SUPPLY_3_8
#elif defined(HELTEC_T114)
    // Heltec T114: divisor 100/490 (~0.204) -> umbral de fabrica (~4.04V real).
    // Ojo: Meshtastic lo desactiva por una fuga de 2.9 mA en System OFF (issue #8801); aqui se
    // mantiene ACTIVO a proposito (coste aceptado por diseno, para poder despertar con solar).
    return BATTERY_LPCOMP_THRESHOLD; // NRF_LPCOMP_REF_SUPPLY_2_8
#else
    // Promicro / Faketec: divisor 0.5 -> los 5 niveles ajustables con /nava set_vwake
    switch (currentWakeLevel) {
    case 1:
        return NRF_LPCOMP_REF_SUPPLY_5_16; //  ~2.06V real
    case 2:
        return NRF_LPCOMP_REF_SUPPLY_3_8; //  ~2.48V real
    case 3:
        return NRF_LPCOMP_REF_SUPPLY_9_16; //  ~3.71V real (por defecto)
    case 4:
        return NRF_LPCOMP_REF_SUPPLY_11_16; //  ~4.54V real (solo con bateria alta)
    case 5:
        return NRF_LPCOMP_REF_SUPPLY_4_8; //  ~3.30V real (LiFePO4)
    default:
        return (nrf_lpcomp_ref_t)BATTERY_LPCOMP_THRESHOLD;
    }
#endif
}
#endif // BATTERY_LPCOMP_INPUT

static nrfx_wdt_t nrfx_wdt = NRFX_WDT_INSTANCE(0);
static nrfx_wdt_channel_id nrfx_wdt_channel_id_nrf52_main;

// This is a public global so that the debugger can set it to false automatically from our gdbinit
// @phaseloop comment: most part of codebase, including filesystem flash driver depend on softdevice
// methods so disabling it may actually crash thing. Proceed with caution.

bool useSoftDevice = true; // Set to false for easier debugging

static inline void debugger_break(void)
{
    __asm volatile("bkpt #0x01\n\t"
                   "mov pc, lr\n\t");
}

// PowerHAL NRF52 specific function implementations
bool powerHAL_isVBUSConnected()
{
    return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

bool powerHAL_isPowerLevelSafe()
{
    static bool powerLevelSafe = true;

#ifdef SAFE_VDD_VOLTAGE_THRESHOLD_MV
    uint16_t threshold = SAFE_VDD_VOLTAGE_THRESHOLD_MV;
#else
    uint16_t threshold = (uint16_t)(SAFE_VDD_VOLTAGE_THRESHOLD * 1000.0f + 0.5f); // convert V to mV
#endif
#ifdef SAFE_VDD_VOLTAGE_THRESHOLD_HYST_MV
    uint16_t hysteresis = SAFE_VDD_VOLTAGE_THRESHOLD_HYST_MV;
#else
    uint16_t hysteresis = (uint16_t)(SAFE_VDD_VOLTAGE_THRESHOLD_HYST * 1000.0f + 0.5f);
#endif

    if (powerLevelSafe) {
        if (getVDDVoltage() < threshold) {
            powerLevelSafe = false;
        }
    } else {
        // power level is only safe again when it raises above threshold + hysteresis
        if (getVDDVoltage() >= (threshold + hysteresis)) {
            powerLevelSafe = true;
        }
    }

    return powerLevelSafe;
}

void powerHAL_platformInit()
{

    // Enable POF power failure comparator. It will prevent writing to NVMC flash when supply voltage is too low.
    // Set to some low value as last resort - powerHAL_isPowerLevelSafe uses different method and should manage proper node
    // behaviour on its own.

    // POFWARN is pretty useless for node power management because it triggers only once and clearing this event will not
    // re-trigger it again until voltage rises to safe level and drops again. So we will use SAADC routed to VDD to read safely
    // voltage.

    // @phaseloop: I disable POFCON for now because it seems to be unreliable or buggy. Even when set at 2.0V it
    // triggers below 2.8V and corrupts data when pairing bluetooth - because it prevents filesystem writes and
    // adafruit BLE library triggers lfs_assert which reboots node and formats filesystem.
    // I did experiments with bench power supply and no matter what is set to POFCON, it always triggers right below
    // 2.8V. I compared raw registry values with datasheet.

    NRF_POWER->POFCON =
        ((POWER_POFCON_THRESHOLD_V22 << POWER_POFCON_THRESHOLD_Pos) | (POWER_POFCON_POF_Enabled << POWER_POFCON_POF_Pos));

    // remember to always match VBAT_AR_INTERNAL with AREF_VALUE in variant definition file
#ifdef VBAT_AR_INTERNAL
    analogReference(VBAT_AR_INTERNAL);
#else
    analogReference(AR_INTERNAL); // 3.6V
#endif
}

// get VDD voltage (in millivolts)
uint16_t getVDDVoltage()
{
    concurrency::LockGuard guard(concurrency::nrf52SaadcLock);

    // Match battery read resolution; SAADC is shared with AnalogBatteryLevel in Power.cpp.
    analogReadResolution(BATTERY_SENSE_RESOLUTION_BITS);

    // VDD range on NRF52840 is 1.8-3.3V so we need to remap analog reference to 3.6V
    analogReference(AR_INTERNAL);

    uint16_t vddADCRead = analogReadVDD();
    float voltage = ((1000 * 3.6) / pow(2, BATTERY_SENSE_RESOLUTION_BITS)) * vddADCRead;

// restore default battery reading reference
#ifdef VBAT_AR_INTERNAL
    analogReference(VBAT_AR_INTERNAL);
#endif

    return voltage;
}

bool loopCanSleep()
{
    // turn off sleep only while connected via USB
    // return true;
    return !Serial; // the bool operator on the nrf52 serial class returns true if connected to a PC currently
    // return !(TinyUSBDevice.mounted() && !TinyUSBDevice.suspended());
}

// handle standard gcc assert failures
void __attribute__((noreturn)) __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
    LOG_ERROR("assert failed %s: %d, %s, test=%s", file, line, func, failedexpr);
    // debugger_break(); FIXME doesn't work, possibly not for segger
    // Reboot cpu
    NVIC_SystemReset();
}

void getMacAddr(uint8_t *dmac)
{
    const uint8_t *src = (const uint8_t *)NRF_FICR->DEVICEADDR;
    dmac[5] = src[0];
    dmac[4] = src[1];
    dmac[3] = src[2];
    dmac[2] = src[3];
    dmac[1] = src[4];
    dmac[0] = src[5] | 0xc0; // MSB high two bits get set elsewhere in the bluetooth stack
}

bool getDeviceId(uint8_t *deviceId)
{
    // Nordic burns a FIPS-compliant random id into each chip at the factory. We concatenate
    // the device address to that random id to form the 16-byte hardware identifier.
    uint64_t device_id_start = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
    uint64_t device_id_end = ((uint64_t)NRF_FICR->DEVICEADDR[1] << 32) | NRF_FICR->DEVICEADDR[0];
    memcpy(deviceId, &device_id_start, sizeof(device_id_start));
    memcpy(deviceId + sizeof(device_id_start), &device_id_end, sizeof(device_id_end));
    return true;
}

#if !MESHTASTIC_EXCLUDE_BLUETOOTH
void setBluetoothEnable(bool enable)
{
    // For debugging use: don't use bluetooth
    if (!useSoftDevice) {
        if (enable)
            LOG_INFO("Disable NRF52 BLUETOOTH WHILE DEBUGGING");
        return;
    }

    // If user disabled bluetooth: init then disable advertising & reduce power
    // Workaround. Avoid issue where device hangs several days after boot..
    // Allegedly, no significant increase in power consumption
    if (!config.bluetooth.enabled) {
        static bool initialized = false;
        if (!initialized) {
            nrf52Bluetooth = new NRF52Bluetooth();
            nrf52Bluetooth->startDisabled();
            initialized = true;
        }
        return;
    }

    if (enable) {
        powerMon->setState(meshtastic_PowerMon_State_BT_On);

        // If not yet set-up
        if (!nrf52Bluetooth) {
            LOG_DEBUG("Init NRF52 Bluetooth");
            nrf52Bluetooth = new NRF52Bluetooth();
            nrf52Bluetooth->setup();
        }
        // Already setup, apparently
        else
            nrf52Bluetooth->resumeAdvertising();
    }
    // Disable (if previously set-up)
    else if (nrf52Bluetooth) {
        powerMon->clearState(meshtastic_PowerMon_State_BT_On);
        nrf52Bluetooth->shutdown();
    }
}
#else
#warning NRF52 "Bluetooth disable" workaround does not apply to builds with MESHTASTIC_EXCLUDE_BLUETOOTH
void setBluetoothEnable(bool enable) {}
#endif
/**
 * Override printf to use the SEGGER output library (note - this does not effect the printf method on the debug console)
 */
int printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    auto res = SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    return res;
}

namespace
{
constexpr uint8_t NRF52_MAGIC_LFS_IS_CORRUPT = 0xF5;
constexpr uint32_t MULTIPLE_CORRUPTION_DELAY_MILLIS = 20 * 60 * 1000;
// When the last format happened, not when the next one is due: measuring forward from the event
// bounds the pause below by the constant, where a stored deadline could hand delay() any value.
// Armed separately because preFSBegin() runs in the first millisecond of boot, so a zero timestamp
// is a legitimate value here, not an "unset" marker.
static uint32_t last_format_ms = 0;
static bool formatted_this_boot = false;

// Report the critical error from loop(), giving a chance for the screen to be initialized first.
inline void reportLittleFSCorruptionOnce()
{
    static bool report_corruption = formatted_this_boot;
    if (report_corruption) {
        report_corruption = false;
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_FLASH_CORRUPTION_UNRECOVERABLE);
    }
}
} // namespace

void preFSBegin()
{
    // The GPREGRET register keeps its value across warm boots. Check that this is a warm boot and, if GPREGRET
    // is set to NRF52_MAGIC_LFS_IS_CORRUPT, format LittleFS.
    if (!(NRF_POWER->RESETREAS == 0 && NRF_POWER->GPREGRET == NRF52_MAGIC_LFS_IS_CORRUPT))
        return;
    NRF_POWER->GPREGRET = 0;
    last_format_ms = Time::getMillis();
    formatted_this_boot = true;
    InternalFS.format();
    LOG_INFO("LittleFS format complete; restoring default settings");
}

extern "C" void lfs_assert(const char *reason)
{
    LOG_ERROR("LittleFS corruption detected: %s", reason);
    // Test the armed flag first, since elapsed-since-0 is inside the backoff for the first 20
    // minutes after each wrap.
    if (formatted_this_boot && Throttle::isWithinTimespanMs(last_format_ms, MULTIPLE_CORRUPTION_DELAY_MILLIS)) {
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_FLASH_CORRUPTION_UNRECOVERABLE);
        // Same clock Throttle just read, and clamped: the check above and a second, later read
        // can straddle the backoff, which would wrap the remainder into a ~50-day delay().
        const uint32_t elapsed = Time::getMillis() - last_format_ms;
        const uint32_t millis_remain =
            elapsed < MULTIPLE_CORRUPTION_DELAY_MILLIS ? MULTIPLE_CORRUPTION_DELAY_MILLIS - elapsed : 0;
        LOG_WARN("Pausing %u seconds to avoid wear on flash storage", millis_remain / 1000);
        delay(millis_remain);
    }
    LOG_INFO("Rebooting to format LittleFS");
    delay(500); // Give the serial port a bit of time to output that last message.
    // Try setting GPREGRET with the SoftDevice first. If that fails (perhaps because the SD hasn't been initialize yet) then set
    // NRF_POWER->GPREGRET directly.

    // TODO: this will/can crash CPU if bluetooth stack is not compiled in or bluetooth is not initialized
    // (regardless if enabled or disabled) - as there is no live SoftDevice stack
    // implement "safe" functions detecting softdevice stack state and using proper method to set registers

    // do not set GPREGRET if POFWARN is triggered because it means lfs_assert reports flash undervoltage protection
    // and not data corruption. Reboot is fine as boot procedure will wait until power level is safe again

    if (!NRF_POWER->EVENTS_POFWARN) {
        if (!(sd_power_gpregret_clr(0, 0xFF) == NRF_SUCCESS &&
              sd_power_gpregret_set(0, NRF52_MAGIC_LFS_IS_CORRUPT) == NRF_SUCCESS)) {
            NRF_POWER->GPREGRET = NRF52_MAGIC_LFS_IS_CORRUPT;
        }
    }

    // TODO: this should not be done when SoftDevice is enabled as device will not boot back on soft reset
    // as some data is retained in RAM which will prevent re-enabling bluetooth stack
    // Google what Nordic has to say about NVIC_* + SoftDevice
    // NAVARICO-V6 (25/09/2026): log explicito para poder atribuir un Reset reason 0x4 (SREQ) a ESTE
    // camino. Sin esto, un reinicio por lfs_assert era indistinguible de cualquier otro reset pedido
    // por software, y ademas deja la pista de que la LittleFS se corrompio.
    LOG_ERROR("RESET SREQ: lfs_assert (LittleFS corrupta) -> reinicio para formatear (uptime %lus)",
              (unsigned long)(millis() / 1000));
    NVIC_SystemReset();
}

void checkSDEvents()
{
    if (useSoftDevice) {
        uint32_t evt;
        while (NRF_SUCCESS == sd_evt_get(&evt)) {
            switch (evt) {
            case NRF_EVT_POWER_FAILURE_WARNING:
                RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_BROWNOUT);
                break;

            default:
                LOG_DEBUG("Unexpected SDevt %d", evt);
                break;
            }
        }
    } else {
        if (NRF_POWER->EVENTS_POFWARN)
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_BROWNOUT);
    }
}

void nrf52Loop()
{
    {
        static bool watchdog_running = false;
        if (!watchdog_running) {
            nrfx_wdt_enable(&nrfx_wdt);
            watchdog_running = true;
        }
    }
    nrfx_wdt_channel_feed(&nrfx_wdt, nrfx_wdt_channel_id_nrf52_main);

    checkSDEvents();
    reportLittleFSCorruptionOnce();

    variant_nrf52LoopHook(); // Optional variant hook called each nrf52Loop();
}

#ifdef USE_SEMIHOSTING
#include <SemihostingStream.h>
#include <meshUtils.h>

/**
 * Note: this variable is in BSS and therfore false by default.  But the gdbinit
 * file will be installing a temporary breakpoint that changes wantSemihost to true.
 */
bool wantSemihost;

/**
 * Turn on semihosting if the ICE debugger wants it.
 */
void nrf52InitSemiHosting()
{
    if (wantSemihost) {
        static SemihostingStream semiStream;
        // We must dynamically alloc because the constructor does semihost operations which
        // would crash any load not talking to a debugger
        semiStream.open();
        semiStream.println("Semihosting starts!");
        // Redirect our serial output to instead go via the ICE port
        console->setDestination(&semiStream);
    }
}
#endif

void nrf52Setup()
{
#ifdef ADC_V
    pinMode(ADC_V, INPUT);
#endif

    // NAVARICO-V6 (23/09, portado de NavaTastic V5.3): ALIMENTACION DE LA RADIO.
    // En las placas con modulo E22P, este pin NO es un RXEN del SX1262: es el pin que DA DE COMER al
    // modulo (Promicro E22P: P0.17 | Xiao E22P: D5). Si no se pone en ALTO aqui, la radio no se
    // enciende bien y el nodo arranca MUDO o SIN PODER TRANSMITIR (compila bien y falla en hardware).
    // Es lo primero que se hace en setup() a proposito: la radio tiene que tener alimentacion antes
    // de inicializarse.
    // Solo se define en las variantes que lo necesitan (RADIO_POWER_ENABLE_PIN), asi que en el resto
    // de placas este bloque NO existe y no toca nada.
    // ⚠️ 25/09/2026: al Promicro E22P le FALTABA el define en su variant.h (se perdio en el portaje),
    // asi que este bloque no se compilaba en la placa del operador y la radio se quedaba sin su pin de
    // alimentacion: RECIBIA pero NO TRANSMITIA. Ver el comentario del variant.h.
#ifdef RADIO_POWER_ENABLE_PIN
    pinMode(RADIO_POWER_ENABLE_PIN, OUTPUT);
    digitalWrite(RADIO_POWER_ENABLE_PIN, HIGH);
#endif

    // The Adafruit core's init() (cores/nRF5/wiring.c) caches RESETREAS into a static and then
    // W1C-clears the hardware register before setup() ever runs, so a raw NRF_POWER->RESETREAS
    // read here is ALWAYS 0. Use the core's cached copy so this log line is actually meaningful
    // (0x1 pin reset, 0x2 watchdog, 0x4 soft reset/SREQ, 0x8 CPU lockup, 0x10000 System OFF wake).
    uint32_t why = readResetReason();
    // per
    // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fpower.html
    LOG_DEBUG("Reset reason: 0x%x", why);
    // NAVARICO-V6 (portado de NavaTastic V5.1): se guarda para que el motor /nava pueda contarlo
    // como causa del reinicio en el aviso [Boot] (WDT / pin / software / lockup / LPCOMP / VBUS).
    rawResetReason = why;

#ifdef USE_SEMIHOSTING
    nrf52InitSemiHosting();
#endif

    // Per
    // https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/monitor-mode-debugging-with-j-link-and-gdbeclipse
    // This is the recommended setting for Monitor Mode Debugging
    NVIC_SetPriority(DebugMonitor_IRQn, 6UL);

#ifdef BQ25703A_ADDR
    auto *bq = new BQ25713();
    if (!bq->setup())
        LOG_ERROR("Charge controller init failed");
#endif

    // Init random seed
    uint32_t seed = 0;
    if (!HardwareRNG::seed(seed)) {
        LOG_WARN("Hardware RNG seed unavailable, using PRNG fallback");
        // Use a hardware timer value as a fallback seed for better entropy
        seed = micros();
    }
    LOG_DEBUG("Set random seed %u", seed);
    randomSeed(seed);

    // Set up nrfx watchdog. Do not enable the watchdog yet (we do that
    // the first time through the main loop), so that other threads can
    // allocate their own wdt channel to protect themselves from hangs.
    //
    // NAVARICO-V6 (decision del operador, 15/09): RUN_SLEEP en vez de PAUSE_SLEEP_HALT. Con
    // PAUSE_SLEEP_HALT el contador se PAUSA mientras el nodo duerme, asi que un cuelgue ocurrido
    // durante el sueno no lo reiniciaba nadie (solo se recuperaba con el boton fisico). El motivo
    // es el mismo que perseguia Guillermo: hay rutas que toman spiLock con xSemaphoreTake(...,
    // portMAX_DELAY), SIN timeout, asi que si la radio se queda el candado el hilo de loop() se
    // bloquea para siempre.
    //
    // POR QUE ES SEGURO EN NUESTRAS PLACAS (verificado antes de cambiarlo): la unica rama del
    // firmware que hace un delay() largo SIN alimentar el watchdog es el "sleepy tracker" de
    // cpuDeepSleep(), y exige a la vez rol TRACKER/TAK_TRACKER/SENSOR **y** config.power.
    // is_power_saving == true, que solo se activa bajo #if defined(USE_POWERSAVE). Ninguna de
    // nuestras placas define USE_POWERSAVE, asi que esa rama es inalcanzable. El camino normal
    // (msecToWake == portMAX_DELAY) llama a sd_power_system_off(), y en System OFF el watchdog
    // esta apagado porque lo esta el chip entero: no hay reinicios falsos posibles al dormir.
    nrfx_wdt_config_t wdt0_config = {
        .behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP, .reload_value = APP_WATCHDOG_SECS * 1000,
        // Note: Not using wdt interrupts.
        // .interrupt_priority = NRFX_WDT_DEFAULT_CONFIG_IRQ_PRIORITY
    };
    nrfx_err_t r = nrfx_wdt_init(&nrfx_wdt, &wdt0_config,
                                 nullptr // Watchdog event handler, not used, we just reset.
    );
    assert(r == NRFX_SUCCESS);

    r = nrfx_wdt_channel_alloc(&nrfx_wdt, &nrfx_wdt_channel_id_nrf52_main);
    assert(r == NRFX_SUCCESS);
}

void cpuDeepSleep(uint32_t msecToWake)
{
    // FIXME, configure RTC or button press to wake us
    // FIXME, power down SPI, I2C, RAMs
#if HAS_WIRE
    Wire.end();
#endif
    SPI.end();
#if SPI_INTERFACES_COUNT > 1
    SPI1.end();
#endif
    if (Serial)       // Another check in case of disabled default serial, does nothing bad
        Serial.end(); // This may cause crashes as debug messages continue to flow.

        // This causes troubles with waking up on nrf52 (on pro-micro in particular):
        // we have no Serial1 in use on nrf52, check Serial and GPS modules.
#ifdef PIN_SERIAL1_RX
    if (Serial1) // A straightforward solution to the wake from deepsleep problem
        Serial1.end();
#endif

    setBluetoothEnable(false);

#ifdef RAK4630
#ifdef PIN_3V3_EN
    digitalWrite(PIN_3V3_EN, LOW);
#endif
#ifdef AQ_SET_PIN
    // RAK-12039 set pin for Air quality sensor
    digitalWrite(AQ_SET_PIN, LOW);
#endif
#endif
    // Run shutdown code if specified in variant.cpp
    variant_shutdown();

    // Sleepy trackers or sensors can low power "sleep"
    // Don't enter this if we're sleeping portMAX_DELAY, since that's a shutdown event
    if (msecToWake != portMAX_DELAY &&
        (IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_TRACKER,
                   meshtastic_Config_DeviceConfig_Role_TAK_TRACKER, meshtastic_Config_DeviceConfig_Role_SENSOR) &&
         config.power.is_power_saving == true)) {
        sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        delay(msecToWake);
        // NAVARICO-V6 (25/09/2026): log explicito para atribuir un Reset reason 0x4 (SREQ) a ESTE
        // camino (sueno temporizado de tracker/sensor, que acaba en reset y no en System OFF).
        LOG_ERROR("RESET SREQ: sueno temporizado de tracker/sensor cumplido (rol %d, %lums)",
                  (int)config.device.role, (unsigned long)msecToWake);
        NVIC_SystemReset();
    } else {
        // Resume on user button press
        // https://github.com/lyusupov/SoftRF/blob/81c519ca75693b696752235d559e881f2e0511ee/software/firmware/source/SoftRF/src/platform/nRF52.cpp#L1738
        constexpr uint32_t DFU_MAGIC_SKIP = 0x6d;
        sd_power_gpregret_clr(0, 0xFF);           // Clear the register before setting a new values in it for stability reasons
        sd_power_gpregret_set(0, DFU_MAGIC_SKIP); // Equivalent NRF_POWER->GPREGRET = DFU_MAGIC_SKIP

        // FIXME, use system off mode with ram retention for key state?
        // FIXME, use non-init RAM per
        // https://devzone.nordicsemi.com/f/nordic-q-a/48919/ram-retention-settings-with-softdevice-enabled

#ifdef BATTERY_LPCOMP_INPUT
        // Only enable LPCOMP wake if the variant allows it
        if (variant_enableBatteryLpcompWake()) {
#ifdef ADC_CTRL
            // NAVARICO-V6: mantener el divisor de tension de la bateria activo durante el sueno,
            // para que el comparador lea la tension dividida (sin esto la lectura no es valida).
            pinMode(ADC_CTRL, OUTPUT);
            digitalWrite(ADC_CTRL, ADC_CTRL_ENABLED);
#endif
            // NAVARICO-V6: re-armado limpio (deshabilitar antes de configurar)
            NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;

            nrf_lpcomp_input_select(NRF_LPCOMP, BATTERY_LPCOMP_INPUT);

            nrf_lpcomp_config_t c;
            // NAVARICO-V6: UMBRAL POR PLACA (getActiveLpcompThreshold) en vez del valor fijo del
            // variant. Es el diseno de NavaTastic: cada placa tiene su divisor de tension real.
            c.reference = (nrf_lpcomp_ref_t)getActiveLpcompThreshold();            c.detection = NRF_LPCOMP_DETECT_UP; // flank up
            // NAVARICO-V6: histéresis ACTIVA (~50 mV) para evitar el traqueteo al cargar con solar.
            c.hyst = NRF_LPCOMP_HYST_ENABLED;
            nrf_lpcomp_configure(NRF_LPCOMP, &c);

            // NAVARICO-V6: limpiar EXPLICITAMENTE todos los latches de evento (despertares "calientes")
            NRF_LPCOMP->EVENTS_READY = 0;
            NRF_LPCOMP->EVENTS_DOWN = 0;
            NRF_LPCOMP->EVENTS_UP = 0;
            NRF_LPCOMP->EVENTS_CROSS = 0;

            NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Enabled;
            NRF_LPCOMP->TASKS_START = 1;

            battery_adcEnable();

            // NAVARICO-V6 (endurecimiento, portado de NavaTastic V5.1): espera ACOTADA en vez de un
            // bucle infinito. El READY del LPCOMP tarda microsegundos en condiciones validas; si no
            // llegara, NUNCA se entra en System OFF sin fuente de despertar (el nodo quedaria mudo
            // en el monte): se reintenta el re-armado una vez y, si sigue sin estar listo, se
            // reinicia solo. En el caso normal (READY inmediato) el comportamiento NO cambia.
            // OJO: este bucle sin tope es el bug real de la 2.8 (auditoria A2); no copiar el
            // bloque de docs/guia_integracion_navarrico.md, que lo tiene sin corregir.
            bool lpcompReady = false;
            for (uint8_t intento = 0; intento < 2 && !lpcompReady; intento++) {
                uint32_t t0 = micros();
                while (NRF_LPCOMP->EVENTS_READY == 0 && (uint32_t)(micros() - t0) < 1000) {
                    ;
                }
                lpcompReady = (NRF_LPCOMP->EVENTS_READY != 0);
                if (!lpcompReady) {
                    NRF_LPCOMP->TASKS_STOP = 1;
                    NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;
                    NRF_LPCOMP->EVENTS_READY = 0;
                    NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Enabled;
                    NRF_LPCOMP->TASKS_START = 1;
                }
            }
            if (!lpcompReady) {
                LOG_ERROR("NAVARICO: LPCOMP sin READY tras reintento; reinicio autonomo en vez de dormir sin despertar");
                NVIC_SystemReset();
            }

            // NAVARICO-V6: limpiar otra vez y asentar, para evitar despertares inmediatos
            NRF_LPCOMP->EVENTS_READY = 0;
            NRF_LPCOMP->EVENTS_UP = 0;
            delay(10);
        }
#endif

        auto ok = sd_power_system_off();
        if (ok != NRF_SUCCESS) {
            LOG_ERROR("FIXME: Ignoring soft device (EasyDMA pending?) and forcing system-off");
            NRF_POWER->SYSTEMOFF = 1;
        }
    }

    // The following code should not be run, because we are off
    while (1) {
        delay(5000);
        LOG_DEBUG(".");
    }
}

void clearBonds()
{
    if (!nrf52Bluetooth) {
        nrf52Bluetooth = new NRF52Bluetooth();
        nrf52Bluetooth->setup();
    }
    nrf52Bluetooth->clearBonds();
}

void enterDfuMode()
{
// SDK kit does not have native USB like almost all other NRF52 boards
#ifdef NRF_USE_SERIAL_DFU
    enterSerialDfu();
#else
    enterUf2Dfu();
#endif
}

// NAVARICO-V6 (portado de NavaTastic V5.1): tension TEORICA de despertar por LPCOMP en mV, para los
// avisos solares ([Sueno]/[Vivo]/[Listo]). Solo informativo: no cambia el hardware.
// OJO: esta funcion se define SIEMPRE en nRF52 porque el motor /nava la llama sin condicion; si el
// despertar por LPCOMP estuviera desactivado en la placa, devuelve el valor por defecto (nivel 3,
// umbral del Promicro) en vez de la constante del variant, que podria no existir.
uint16_t navaGetLpcompWakeMv()
{
#ifdef BATTERY_LPCOMP_INPUT
#if defined(SEEED_SOLAR_NODE) || defined(SEEED_XIAO_NRF52840_KIT)
    return 3670; // 3_8 real (~3.67V) con el divisor de Seed y Xiao
#elif defined(HELTEC_T114)
    return 4040; // 2_8 real (~4.04V) con el divisor 100/490 de la T114
#endif
#endif
    switch (currentWakeLevel) {
    case 1:
        return 2060;
    case 2:
        return 2480;
    case 3:
        return 3710; // por defecto (LiPo/NiMH/Sodio)
    case 4:
        return 4540;
    case 5:
        return 3300; // LiFePO4
    default:
        return 3710;
    }
}
