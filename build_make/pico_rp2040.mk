################################################################################
# micro T-Kernel 3.0 BSP makefile
#     Target Board: Raspberry Pi Pico
################################################################################

################################################################################
# Execution model
#
#   SMP=1  (default)  dual core.  TK_MAX_CORE is 2.
#   SMP=0             single core.  TK_MAX_CORE is 1, the per-core arrays
#                     collapse and the current-core index folds to a constant
#                     zero, so the generated code is equivalent to the
#                     pre-SMP port.
#
# Passed to the assembler as well: the dispatcher indexes the per-core arrays
# by SIO CPUID and needs the same profile.
################################################################################

SMP ?= 1

ifneq ($(SMP),0)
ifneq ($(SMP),1)
$(error Unknown SMP '$(SMP)'; use 0 or 1)
endif
endif

# Console default is needed here because the image name includes it.
#
# Defaults to usb_cdc so the console appears on the Pico's own USB cable.
# usb_cdc still mirrors every line to UART0 (GP0 TX / GP1 RX) for early boot
# and panic messages, so GP0/GP1 stay reserved either way.  CONSOLE=uart drops
# the USB console and its TinyUSB dependency.
CONSOLE ?= usb_cdc

# Pico W radio.  Defaults to cyw43: the radio is powered up and owned by a
# core-0 polling service task, using the Pico SDK's cyw43 driver.  WIFI=none
# leaves the radio powered down.
WIFI ?= cyw43
ifneq ($(filter $(WIFI),none cyw43),$(WIFI))
$(error Unknown WIFI '$(WIFI)'; use none or cyw43)
endif

ifeq ($(WIFI),cyw43)
WIFI_SUFFIX := _wifi
# SMP=0 is supported: the radio service task simply drops its TP_PRC1 affinity
# request when TK_SUPPORT_SMP is off (see cyw43_utk_kernel.c), the same way the
# USB console task already did. Everything touching the radio and lwIP still
# runs on that one task, which is what docs/RELEASE.md safety rule 6 actually
# requires -- single-core satisfies it trivially.
endif

# Optional lwIP network stack on top of WIFI=cyw43. NO_SYS=1 (raw API, no
# tcpip thread): everything that calls into lwIP -- cyw43_poll(),
# sys_check_timeouts(), and any application tcp_*/udp_*/dns_* call -- runs
# on the same processor-1-owned task that already services the radio, per
# docs/RELEASE.md safety rule 6. See lib/liblwip/sysdepend/pico_rp2040/.
NET ?= none
ifneq ($(filter $(NET),none lwip),$(NET))
$(error Unknown NET '$(NET)'; use none or lwip)
endif
ifeq ($(NET),lwip)
ifneq ($(WIFI),cyw43)
$(error NET=lwip requires WIFI=cyw43)
endif
NET_SUFFIX := _lwip
endif

# Optional MQTT client on top of NET=lwip. Connects and publishes from the
# same processor-1-owned task as everything else above -- see
# lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c. Broker address and topic
# come from config/wifi_secrets.h (gitignored; copy from
# config/wifi_secrets.example.h), same as the WiFi credentials.
MQTT ?= 0
ifneq ($(filter $(MQTT),0 1),$(MQTT))
$(error Unknown MQTT '$(MQTT)'; use 0 or 1)
endif
ifeq ($(MQTT),1)
ifneq ($(NET),lwip)
$(error MQTT=1 requires NET=lwip)
endif
MQTT_SUFFIX := _mqtt
endif

# TEST=1 builds the subsystem self-tests into the image (APP_TEST=1 in C).
# The image gets a _test suffix so it cannot be mistaken for the normal one.
TEST ?= 0
ifneq ($(filter $(TEST),0 1),$(TEST))
$(error Unknown TEST '$(TEST)'; use 0 or 1)
endif
ifeq ($(TEST),1)
TEST_SUFFIX := _test
endif

# CRASH_REPORT=1 also copies every console line to UART0 on GP0 (TX) /
# GP1 (RX), 115200 8N1.  UART output is sent immediately, so it still shows
# the last messages and the kernel's crash report after a crash, when the
# USB console can no longer send anything.  It needs a USB-serial adapter on
# GP0/GP1 and takes those two pins.
# CRASH_REPORT=0 (default): console on USB only; the boot code leaves GP0/GP1
# alone, so a subsystem may claim them in TEAM.md.
# CONSOLE=uart always uses UART0 as the console, so it forces CRASH_REPORT=1.
CRASH_REPORT ?= 0
ifneq ($(filter $(CRASH_REPORT),0 1),$(CRASH_REPORT))
$(error Unknown CRASH_REPORT '$(CRASH_REPORT)'; use 0 or 1)
endif
ifeq ($(CONSOLE),uart)
override CRASH_REPORT := 1
endif
ifeq ($(CONSOLE)$(CRASH_REPORT),usb_cdc1)
CRASH_SUFFIX := _crash
endif

#
# Images are named after the profile they were built with.  Every image is
# flashed by hand from a downloads folder, so identically-named artifacts are
# a live hazard: an SMP=1 build was once flashed in place of an SMP=0 one and
# only the banner in the log revealed it.
#
EXE_FILE := mtk3pico_smp$(SMP)_$(CONSOLE)$(WIFI_SUFFIX)$(NET_SUFFIX)$(MQTT_SUFFIX)$(CRASH_SUFFIX)$(TEST_SUFFIX)

GCC := arm-none-eabi-gcc
AS := arm-none-eabi-gcc
LINK := arm-none-eabi-gcc
SIZE := arm-none-eabi-size
E2U := tools/elf2uf2

# Fail early, with a readable message, when the cross compiler is not on PATH.
ifneq ($(MAKECMDGOALS),clean)
ifeq ($(shell command -v $(GCC) 2>/dev/null),)
$(error $(GCC) not found. Add your Arm GNU Toolchain 'bin' folder to PATH, e.g. export PATH="/path/to/arm-gnu-toolchain/bin:$$PATH")
endif
endif

# The USB console and the radio take sources from the Pico C SDK.  Its location
# must come from the environment (export PICO_SDK_PATH=...) or the command line.
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(CONSOLE)$(WIFI),uartnone)
ifeq ($(strip $(PICO_SDK_PATH)),)
$(error PICO_SDK_PATH is not set. export PICO_SDK_PATH=/path/to/pico-sdk, or build with CONSOLE=uart WIFI=none)
endif
ifeq ($(wildcard $(PICO_SDK_PATH)/pico_sdk_init.cmake),)
$(error PICO_SDK_PATH='$(PICO_SDK_PATH)' does not look like a Pico C SDK folder)
endif
endif
endif

# The ELF-to-UF2 converter is a small host program built from tools/elf2uf2
# with the host C++ compiler the first time it is needed.
$(E2U): ../tools/elf2uf2/main.cpp
	@echo 'Building host tool: $@'
	c++ -O2 -std=c++14 -I../tools/elf2uf2 -o $@ $<

$(EXE_FILE).elf: | $(E2U)

CFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding\
    -std=gnu11 \
    -O2 -g3 \
    -MMD -MP \
    -mfloat-abi=soft \

ASFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding\
    -x assembler-with-cpp \
    -O2 -g3 \
    -MMD -MP \

LFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding \
    -nostartfiles \
    -O2 -g3 \
    -mfloat-abi=soft \

LNKFILE := "../etc/linker/pico_rp2040/tkernel_map.ld"

# Applied here, after CFLAGS/ASFLAGS are assigned with ':=', which would
# otherwise discard anything appended above.
ifeq ($(SMP),1)
CFLAGS  += -DCNF_SMP=1
ASFLAGS += -DCNF_SMP=1
endif
CFLAGS  += -DAPP_TEST=$(TEST)
CFLAGS  += -DTM_CONSOLE_UART=$(CRASH_REPORT)
ifeq ($(CRASH_REPORT),0)
CFLAGS  += -DBOARD_UART0_FREE=1
endif

################################################################################
# Console selection
#
#   CONSOLE=usb_cdc             USB CDC-ACM console, with the UART0 mirror
#                               retained for early boot and panics.
#   CONSOLE=uart                UART0 only, no TinyUSB dependency.
#
# usb_cdc pulls seven TinyUSB sources and a set of pico-sdk headers from
# PICO_SDK_PATH.  Only headers and those sources are used; the port does not
# link the pico-sdk runtime, and lib/libtm/sysdepend/pico_rp2040/usb/
# usb_sdk_compat.c supplies the few SDK hooks TinyUSB calls, mapped onto
# micro T-Kernel's tk_def_int().
################################################################################

################################################################################
# Stale-object guard
#
# Every profile compiles into the same object tree, so switching profiles
# without cleaning silently relinks the previous profile's objects: SMP=0 and
# SMP=1 built back to back produced byte-identical images. Record the profile
# and discard the objects whenever it changes. Only *.o and *.d are removed:
# build_make/mtkernel_3/ also holds the tracked subdir.mk files, so removing
# the directory wholesale deletes source. Images are named after their
# profile, so both can coexist. This runs at parse time, so it completes
# before any recipe executes.
################################################################################

PROFILE_ID := smp$(SMP)-$(CONSOLE)-wifi$(WIFI)-net$(NET)-mqtt$(MQTT)-crash$(CRASH_REPORT)-test$(TEST)
PROFILE_FILE := .build_profile

ifneq ($(strip $(shell cat $(PROFILE_FILE) 2>/dev/null)),$(PROFILE_ID))
$(info Build profile is now $(PROFILE_ID); discarding objects built for another profile.)
$(shell find mtkernel_3 -name '*.o' -o -name '*.d' | xargs -r rm -f; \
	  rm -f $(EXE_FILE).elf $(EXE_FILE).map $(EXE_FILE).uf2; \
	  echo $(PROFILE_ID) > $(PROFILE_FILE))
endif


ifeq ($(CONSOLE),usb_cdc)

TINYUSB_PATH := $(PICO_SDK_PATH)/lib/tinyusb

ifeq ($(wildcard $(TINYUSB_PATH)/src/tusb.c),)
$(error TinyUSB not found at $(TINYUSB_PATH). Set PICO_SDK_PATH, or build with CONSOLE=uart)
endif

USBDIR := ../lib/libtm/sysdepend/pico_rp2040/usb

USB_SRCS := usb_console.c usb_descriptors.c usb_sdk_compat.c usb_tinyusb_glue.c
USB_OBJS := $(addprefix ./mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/,$(USB_SRCS:.c=.o))

TINYUSB_SRCS := tusb.c \
                common/tusb_fifo.c \
                device/usbd.c \
                device/usbd_control.c \
                class/cdc/cdc_device.c \
                portable/raspberrypi/rp2040/rp2040_usb.c \
                portable/raspberrypi/rp2040/dcd_rp2040.c
TINYUSB_OBJS := $(addprefix ./mtkernel_3/tinyusb/,$(TINYUSB_SRCS:.c=.o))

USB_INCLUDES := -I"$(USBDIR)" \
                -I"$(TINYUSB_PATH)/src" \
                -I"$(PICO_SDK_PATH)/src/common/pico_base_headers/include" \
                -I"$(PICO_SDK_PATH)/src/common/pico_binary_info/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_compiler/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_sections/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_panic/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_common/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_base/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_irq/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync_spin_lock/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_timer/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include"

CFLAGS += -DTM_CONSOLE_USB_CDC=1 -DNDEBUG $(USB_INCLUDES)

OBJS += $(USB_OBJS) $(TINYUSB_OBJS)
C_DEPS += $(USB_OBJS:.o=.d) $(TINYUSB_OBJS:.o=.d)

mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/%.o: $(USBDIR)/%.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/tinyusb/%.o: $(TINYUSB_PATH)/src/%.c
	@echo 'Building TinyUSB file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

else ifneq ($(CONSOLE),uart)
$(error Unknown CONSOLE '$(CONSOLE)'; use usb_cdc or uart)
endif

################################################################################
# CYW43439 radio (polling architecture). NET=lwip below adds lwIP on top,
# still within the same polling model -- see docs/RELEASE.md and
# lib/liblwip/sysdepend/pico_rp2040/.
################################################################################

ifeq ($(WIFI),cyw43)

CYW43_DRIVER_PATH := $(PICO_SDK_PATH)/lib/cyw43-driver
CYW43_PICO_PATH := $(PICO_SDK_PATH)/src/rp2_common/pico_cyw43_driver
WIFIDIR := ../lib/libwifi/sysdepend/pico_rp2040

ifeq ($(wildcard $(CYW43_DRIVER_PATH)/src/cyw43_ll.c),)
$(error CYW43 driver not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules)
endif

WIFI_LOCAL_SRCS := cyw43_utk.c cyw43_utk_kernel.c cyw43_utk_compat.c
WIFI_LOCAL_OBJS := $(addprefix ./mtkernel_3/lib/libwifi/sysdepend/pico_rp2040/,$(WIFI_LOCAL_SRCS:.c=.o))

CYW43_OBJS := ./mtkernel_3/cyw43/cyw43_ll.o \
              ./mtkernel_3/cyw43/cyw43_ctrl.o \
              ./mtkernel_3/cyw43/cyw43_stats.o \
              ./mtkernel_3/cyw43/cyw43_bus_pio_spi.o \
              ./mtkernel_3/cyw43/pio.o \
              ./mtkernel_3/cyw43/dma.o \
              ./mtkernel_3/cyw43/gpio.o

WIFI_INCLUDES := -I"$(WIFIDIR)" \
                 -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
                 -I"$(CYW43_DRIVER_PATH)/src" \
                 -I"$(CYW43_DRIVER_PATH)/firmware" \
                 -I"$(CYW43_PICO_PATH)/include" \
                 -I"$(PICO_SDK_PATH)/src/boards/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_base_headers/include" \
                 -I"$(PICO_SDK_PATH)/src/common/hardware_claim/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_binary_info/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_util/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_common/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_compiler/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_sections/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_panic/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_base/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_pio/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_dma/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_irq/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync_spin_lock/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_clocks/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_timer/include"

CFLAGS += -DTM_WIFI_CYW43=1 -DCYW43_LWIP=$(if $(filter lwip,$(NET)),1,0) -DCYW43_ENABLE_BLUETOOTH=0 \
          -DCYW43_USE_OTP_MAC=1 -DPICO_CYW43_LOGGING_ENABLED=0 \
          -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 -DPICO_BUILD=1 \
          -DPICO_NO_HARDWARE=0 -DRASPBERRYPI_PICO_W=1 \
          -DCYW43_DEFAULT_PIN_WL_REG_ON=23u \
          -DCYW43_DEFAULT_PIN_WL_DATA_OUT=24u \
          -DCYW43_DEFAULT_PIN_WL_DATA_IN=24u \
          -DCYW43_DEFAULT_PIN_WL_HOST_WAKE=24u \
          -DCYW43_DEFAULT_PIN_WL_CLOCK=29u \
          -DCYW43_DEFAULT_PIN_WL_CS=25u \
          -DNDEBUG -ffunction-sections -fdata-sections $(WIFI_INCLUDES)
LFLAGS += -Wl,--gc-sections

OBJS += $(WIFI_LOCAL_OBJS) $(CYW43_OBJS)
C_DEPS += $(WIFI_LOCAL_OBJS:.o=.d) $(CYW43_OBJS:.o=.d)

mtkernel_3/lib/libwifi/sysdepend/pico_rp2040/%.o: $(WIFIDIR)/%.c
	@echo 'Building Wi-Fi port file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_ll.o: $(CYW43_DRIVER_PATH)/src/cyw43_ll.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_ctrl.o: $(CYW43_DRIVER_PATH)/src/cyw43_ctrl.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_stats.o: $(CYW43_DRIVER_PATH)/src/cyw43_stats.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_bus_pio_spi.o: $(CYW43_PICO_PATH)/cyw43_bus_pio_spi.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/pio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_pio/pio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/dma.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_dma/dma.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/gpio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

ifeq ($(NET),lwip)

LWIPDIR := $(PICO_SDK_PATH)/lib/lwip/src
LIBLWIP_PORT := ../lib/liblwip/sysdepend/pico_rp2040

ifeq ($(wildcard $(LWIPDIR)/core/init.c),)
$(error lwIP not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules)
endif

# COREFILES + CORE4FILES (IPv4 only) + the one NETIFFILES entry we need, per
# $(PICO_SDK_PATH)/lib/lwip/src/Filelists.mk. altcp*.c is included because
# apps/mqtt/mqtt.c (MQTT=1, below) is written against the altcp API -- used
# here in its default plain-TCP passthrough mode (LWIP_ALTCP=0), not TLS.
LWIP_CORE_SRCS := core/init.c core/def.c core/dns.c core/inet_chksum.c \
                   core/ip.c core/mem.c core/memp.c core/netif.c core/pbuf.c \
                   core/raw.c core/stats.c core/sys.c \
                   core/altcp.c core/altcp_alloc.c core/altcp_tcp.c \
                   core/tcp.c core/tcp_in.c core/tcp_out.c core/timeouts.c \
                   core/udp.c \
                   core/ipv4/acd.c core/ipv4/autoip.c core/ipv4/dhcp.c \
                   core/ipv4/etharp.c core/ipv4/icmp.c core/ipv4/igmp.c \
                   core/ipv4/ip4_frag.c core/ipv4/ip4.c core/ipv4/ip4_addr.c \
                   netif/ethernet.c
LWIP_CORE_OBJS := $(addprefix ./mtkernel_3/lwip/,$(LWIP_CORE_SRCS:.c=.o))

LWIP_PORT_OBJS := ./mtkernel_3/lib/liblwip/sysdepend/pico_rp2040/sys_arch.o

CYW43_LWIP_OBJS := ./mtkernel_3/cyw43/cyw43_lwip.o

LWIP_INCLUDES := -I"$(LIBLWIP_PORT)/include" \
                 -I"$(LIBLWIP_PORT)" \
                 -I"$(LWIPDIR)/include"

CFLAGS += -DTM_NET_LWIP=1 $(LWIP_INCLUDES)

OBJS += $(LWIP_CORE_OBJS) $(LWIP_PORT_OBJS) $(CYW43_LWIP_OBJS)
C_DEPS += $(LWIP_CORE_OBJS:.o=.d) $(LWIP_PORT_OBJS:.o=.d) $(CYW43_LWIP_OBJS:.o=.d)

mtkernel_3/lwip/%.o: $(LWIPDIR)/%.c
	@echo 'Building lwIP file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/lib/liblwip/sysdepend/pico_rp2040/%.o: ../lib/liblwip/sysdepend/pico_rp2040/%.c
	@echo 'Building lwIP port file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_lwip.o: $(CYW43_DRIVER_PATH)/src/cyw43_lwip.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

ifeq ($(MQTT),1)

MQTT_CORE_OBJS := ./mtkernel_3/lwip/apps/mqtt/mqtt.o
MQTT_PORT_OBJS := ./mtkernel_3/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.o

CFLAGS += -DTM_MQTT=1

OBJS += $(MQTT_CORE_OBJS) $(MQTT_PORT_OBJS)
C_DEPS += $(MQTT_CORE_OBJS:.o=.d) $(MQTT_PORT_OBJS:.o=.d)

mtkernel_3/lwip/apps/mqtt/mqtt.o: $(LWIPDIR)/apps/mqtt/mqtt.c
	@echo 'Building lwIP file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

endif

endif

endif

include mtkernel_3/lib/libtm/sysdepend/pico_rp2040/subdir.mk
include mtkernel_3/lib/libtm/sysdepend/no_device/subdir.mk
include mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/subdir.mk
include mtkernel_3/lib/libbsp/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/pico_rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/cpu/core/armv6m/subdir.mk
