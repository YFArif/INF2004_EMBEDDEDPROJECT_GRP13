"""S5 static RAM and flash, read from the linker map (minimum-bit evidence).

Usage (from the repo root, after `make` in build_make):
    python testing/subsystem5/s5_ram.py build_make/mtk3pico_smp1_usb_cdc_wifi.map

Prints .bss + .data (static RAM) and .text + .rodata + .data (flash) for
every object file in app_program/subsystem5. Task stacks are not included:
the kernel allocates them (TASK_STACK_SIZE in app_main.c).
"""

import collections
import re
import sys

SECTION = re.compile(r"^ (\.(?:bss|data|text|rodata)\S*)\s*$")
ENTRY = re.compile(r"^ (\.(?:bss|data|text|rodata)\S*)?\s+0x[0-9a-f]+"
                   r"\s+0x([0-9a-f]+)\s+(\S+\.o\S*)")


def main(map_path):
    ram = collections.Counter()
    flash = collections.Counter()
    pending = None

    with open(map_path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.rstrip("\n")
            match = SECTION.match(line)

            if match:
                pending = match.group(1)
                continue

            match = ENTRY.match(line)

            if match and (match.group(1) or pending):
                section = match.group(1) or pending
                size = int(match.group(2), 16)
                obj = match.group(3)

                if "subsystem5" in obj:
                    name = re.sub(r".*/", "", obj).replace(")", "")

                    if section.startswith((".bss", ".data")):
                        ram[name] += size

                    if section.startswith((".text", ".rodata", ".data")):
                        flash[name] += size

            pending = None

    for name in sorted(ram):
        print(f"RAM   {name:28s} {ram[name]:6d} B")

    print(f"S5 static RAM {sum(ram.values())} B, "
          f"flash {sum(flash.values())} B")


if __name__ == "__main__":
    main(sys.argv[1])
