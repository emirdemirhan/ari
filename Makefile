PROJECT = ari
BUILD_DIR = build

# ------------------------------------------------
# 1. Kaynak Dosyalar (C ve C++ Ayrıldı)
# ------------------------------------------------

# C Dosyaları (.c)
C_SOURCES = $(wildcard src/*.c)
C_SOURCES += $(wildcard src/drivers/*.c)
C_SOURCES += $(wildcard src/flightcontrol/*.c)
C_SOURCES += $(wildcard src/communication/*.c)
C_SOURCES += freertos/tasks.c
C_SOURCES += freertos/list.c
C_SOURCES += freertos/queue.c
C_SOURCES += freertos/timers.c
C_SOURCES += freertos/ARM_CM4F/port.c
C_SOURCES += freertos/MemMang/heap_4.c

# C++ Dosyaları (.cpp) -> YENİ EKLENDİ
# ESKF.cpp ve TinyMatrix.cpp buraya dahil olacak
CPP_SOURCES = $(wildcard src/*.cpp)
CPP_SOURCES += $(wildcard src/flightcontrol/*.cpp)
CPP_SOURCES += $(wildcard src/ari_math/*.cpp)

# ------------------------------------------------
# 2. Araçlar ve Yollar
# ------------------------------------------------
DEVICE = stm32f412ret6
OPENCM3_DIR = libopencm3

# Derleyiciler
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++       # C++ Derleyicisi
OBJCOPY = arm-none-eabi-objcopy
SZ = arm-none-eabi-size

# ------------------------------------------------
# 3. Flags (Bayraklar)
# ------------------------------------------------

# Ortak Ayarlar (İşlemci, FPU, Debug)
ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
COMMON_FLAGS = $(ARCH_FLAGS) -O3 -g -Wall -Wextra -fno-common -DSTM32F4

# Include Yolları
INCLUDES = -Iinclude \
           -Iinclude/drivers \
           -Iinclude/flightcontrol \
           -Iinclude/ari_math \
           -Iinclude/communication \
           -Iconfig \
           -Ifreertos/include \
           -Ifreertos/ARM_CM4F \
           -I$(OPENCM3_DIR)/include

# C Bayrakları (Sadece .c dosyaları için)
CFLAGS = $(COMMON_FLAGS) -std=c99 $(INCLUDES)

# C++ Bayrakları (Sadece .cpp dosyaları için) -> YENİ EKLENDİ
# -fno-rtti -fno-exceptions: Gömülü sistemde RAM şişmesin diye kapattık.
CXXFLAGS = $(COMMON_FLAGS) -std=c++17 -fno-rtti -fno-exceptions $(INCLUDES)

# Linker Bayrakları
LDSCRIPT = stm32f412.ld
# Linker kütüphaneleri (g++ ile linkleyeceğimiz için -lc -lm yeterli, -lstdc++ otomatik gelir)
LDLIBS = -L$(OPENCM3_DIR)/lib -lopencm3_stm32f4
LDLIBS += -Wl,--start-group -lc -lm -lnosys -Wl,--end-group
LDFLAGS = $(ARCH_FLAGS) -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections

# ------------------------------------------------
# 4. Derleme Kuralları (Build Rules)
# ------------------------------------------------

# Object (.o) dosyalarını build klasörüne yönlendiriyoruz
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))

# Kaynak dosyalarını bulmak için VPATH (Derleyici nerede arayacağını bilsin)
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

.PHONY: all clean flash

all: $(BUILD_DIR)/$(PROJECT).bin

# 1. Adım: ELF Oluşturma (Linking)
# Dikkat: $(CXX) yani g++ kullanıyoruz ki C++ kütüphaneleri bağlansın.
$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS) | $(BUILD_DIR)
	$(CXX) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@
	$(SZ) $@

# 2. Adım: C Dosyalarını Derleme (.c -> .o)
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# 3. Adım: C++ Dosyalarını Derleme (.cpp -> .o) -> YENİ KURAL
$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

# 4. Adım: BIN Dosyası Oluşturma
$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

# Build klasörünü oluştur
$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(PROJECT).bin
	dfu-util -a 0 -s 0x08000000:leave -D $(BUILD_DIR)/$(PROJECT).bin