/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  /* App region ends at 0xEC000: the warm-node-store record-ring is 1 x 4 KB page
   * (src/mesh/WarmNodeStore.h, NAVARICO-V6 13/09). Con el anillo en RAM-only, sus paginas de flash
   * no las usa nadie, asi que se devuelven al firmware. LittleFS (0xED000) NO se mueve: moverlo
   * exigiria mudar los ficheros del nodo y eso es trabajo aparte, pendiente y documentado. Boards on
   * the framework-default linker script are covered by the post-link guard in
   * extra_scripts/nrf52_warm_region.py instead.
   *
   * AVISO 23/09/2026 - LECCION CARA, NO REPETIR: se probo a subir este tope a 0xED000 ("tramo 2 sin
   * mudanza", aprovechando que el anillo del warm-store es RAM-only y no toca flash) y el nodo de banco
   * DEJO DE ARRANCAR. La cuenta de direcciones cuadraba (app hasta 0xEC600, LittleFS en 0xED000,
   * bootloader en 0xF4000) y el build pasaba el guard, pero 0xEC000-0xED000 NO es territorio libre.
   * REVERTIDO. NO VOLVER A SUBIR ESTE TOPE SIN BANCO DELANTE. */
  FLASH (rx)     : ORIGIN = 0x27000, LENGTH = 0xEC000 - 0x27000

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
