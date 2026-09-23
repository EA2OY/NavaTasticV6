/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  /* App region ends at 0xED000 = el origen de LittleFS (NAVARICO-V6 23/09, "tramo 2 sin mudanza").
   * El anillo del warm-node-store se queda en 0xEC000-0xED000, EN MEDIO, y NO se solapa con el firmware
   * porque en RAM-only no lee ni escribe flash NUNCA (ver src/mesh/WarmNodeStore.h, que ademas lleva un
   * static_assert que falla el build si algun dia se activa la escritura sin recolocar el anillo).
   * Resultado: el firmware gana los 4 KB del tramo 2 y LITTLEFS NO SE MUEVE, asi que NO hay migracion de
   * ficheros del nodo (nada de Clean Slate). El guard de extra_scripts/nrf52_warm_region.py defiende
   * ahora el ORIGEN DE LITTLEFS, que es la frontera que de verdad importa.
   *
   * S140 v6.x app region starts at 0x26000 (152 KB SoftDevice); v7.x uses 0x27000. All other
   * boundaries are identical - see nrf52840_s140_v7.ld. */
  FLASH (rx)     : ORIGIN = 0x26000, LENGTH = 0xED000 - 0x26000

  /* SRAM required by Softdevice depend on
   * - Attribute Table Size (Number of Services and Characteristics)
   * - Vendor UUID count
   * - Max ATT MTU
   * - Concurrent connection peripheral + central + secure links
   * - Event Len, HVN queue, Write CMD queue
   *
   * With our fixed Bluefruit config (1 peripheral link, BANDWIDTH_MAX / MTU 247,
   * 0x1000 attr table) sd_ble_enable() reports a requirement well below the old
   * 0x20006000 ORIGIN; every byte of gap is unusable RAM. 0x20004000 keeps a
   * ~2+ KB margin over the reported base. NRF52Bluetooth::setup() now checks
   * Bluefruit.begin() and records a critical error if the SoftDevice ever
   * rejects this base (e.g. after an SD/Bluefruit config change) - re-measure
   * with CFG_DEBUG=1 (Bluefruit logs "SoftDevice's RAM requires: 0x...")
   * before raising bandwidth/MTU/link settings.
   */
  RAM (rwx) :  ORIGIN = 0x20004000, LENGTH = 0x20040000 - 0x20004000
}

SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM

  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf52_common.ld"
