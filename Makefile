PROJECT = ari
BUILD_DIR = build

# 1. Kaynak Dosyalar (Main + FreeRTOS)
SRCS = $(wildcard src/*.c)
SRCS += freertos/tasks.c
SRCS += freertos/list.c
SRCS += freertos/queue.c
SRCS += freertos/timers.c
SRCS += freertos/ARM_CM4F/port.c
SRCS += freertos/MemMang/heap_4.c

# 2. Donanım ve Kütüphane Yolları
DEVICE = stm32f412ret6
OPENCM3_DIR = libopencm3

# 3. FPU AYARLARI (Hard Float - Değiştirme!)
ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

# 4. Include Yolları
INCLUDES = -Iinclude
INCLUDES += -Iconfig
INCLUDES += -Ifreertos/include
INCLUDES += -Ifreertos/ARM_CM4F
INCLUDES += -I$(OPENCM3_DIR)/include

# 5. Kütüphane Bağlantısı
LDLIBS += -L$(OPENCM3_DIR)/lib -lopencm3_stm32f4
LDLIBS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# 6. Linker Script (Kendi yazdığımız dosya)
LDSCRIPT = stm32f412.ld

# 7. Derleyici Komutları
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS += -Os -g -Wall -Wextra -std=c99 -fno-common
CFLAGS += $(ARCH_FLAGS) -DSTM32F4 $(INCLUDES)
LDFLAGS += $(ARCH_FLAGS) -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections

.PHONY: all clean flash

all: $(BUILD_DIR)/$(PROJECT).bin

$(BUILD_DIR)/$(PROJECT).elf: $(SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(PROJECT).bin
	 dfu-util -a 0 -s 0x08000000:leave -D $(BUILD_DIR)/$(PROJECT).bin