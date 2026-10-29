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
C_SOURCES += $(wildcard src/simulation/*.c)
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
# -O2: PIL ölçümünde kontrol adımı -Og ile ortalama 403 us / en fazla 849 us sürdü (2 ms bütçe).
# Adım adım hata ayıklamak için geçici olarak -Og kullanılabilir.
COMMON_FLAGS = $(ARCH_FLAGS) -O2 -g -Wall -Wextra -fno-common -DSTM32F4 -MMD -MP

# Include Yolları
INCLUDES = -Iinclude \
           -Iinclude/drivers \
           -Iinclude/flightcontrol \
           -Iinclude/ari_math \
           -Iinclude/communication \
		   -Iinclude/simulation \
           -Iconfig \
           -Ifreertos/include \
           -Ifreertos/ARM_CM4F \
           -I$(OPENCM3_DIR)/include



# HIL hattı seçenekleri (bayrak değişince farklı BUILD_DIR kullanın veya make clean):
#   make HIL_BAUD=115200        hat hızı (simülatörde --baud ile aynı olmalı)
#   make HIL_RX_DIAG=1          LED görevi saniyede bir USART1 alım sayaçlarını metin olarak yazar
ifdef HIL_BAUD
COMMON_FLAGS += -DHIL_BAUD=$(HIL_BAUD)
endif
ifdef HIL_RX_DIAG
COMMON_FLAGS += -DHIL_RX_DIAG=$(HIL_RX_DIAG)
endif

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

.PHONY: all clean flash host-test sil

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

# Host testi: Kalman ve EKF kestirimcilerini PC'de derleyip gerçek değerle karşılaştırır
HOST_CXX = g++
HOST_TEST = $(BUILD_DIR)/estimator_compare

host-test: | $(BUILD_DIR)
	$(HOST_CXX) -std=c++17 -O1 -Wall -Iinclude -Iinclude/flightcontrol -Iinclude/ari_math \
		tests/estimator_compare.cpp src/flightcontrol/stest_wrapper.cpp -o $(HOST_TEST)
	$(HOST_TEST)

# Header bağımlılıkları (-MMD -MP ile üretilen .d dosyaları):
# bir .h değişince onu include eden kaynaklar yeniden derlenir.
# SIL kütüphanesi: kontrol adımı (controller.c), kestirimciler ve HIL protokolü PC için
# derlenir; simülatör (sim/) bunu ctypes ile yükler. Firmware ile aynı kaynaklar.
HOST_CC = gcc
SIL_DIR = $(BUILD_DIR)/sil
SIL_LIB = $(SIL_DIR)/libari_sil.so
SIL_C_SOURCES = sil/sil_lib.c src/flightcontrol/controller.c src/simulation/hil_protocol.c
SIL_CPP_SOURCES = src/flightcontrol/stest_wrapper.cpp
SIL_OBJECTS = $(addprefix $(SIL_DIR)/,$(notdir $(SIL_C_SOURCES:.c=.o) $(SIL_CPP_SOURCES:.cpp=.o)))
SIL_FLAGS = -O2 -g -fPIC -Wall -Wextra -MMD -MP -Iinclude -Iinclude/flightcontrol -Iinclude/ari_math -Iinclude/simulation

sil: $(SIL_LIB)

$(SIL_LIB): $(SIL_OBJECTS)
	$(HOST_CXX) -shared $(SIL_OBJECTS) -o $@

$(SIL_DIR)/%.o: sil/%.c | $(SIL_DIR)
	$(HOST_CC) -c $(SIL_FLAGS) -std=c99 $< -o $@

$(SIL_DIR)/%.o: src/flightcontrol/%.c | $(SIL_DIR)
	$(HOST_CC) -c $(SIL_FLAGS) -std=c99 $< -o $@

$(SIL_DIR)/%.o: src/simulation/%.c | $(SIL_DIR)
	$(HOST_CC) -c $(SIL_FLAGS) -std=c99 $< -o $@

$(SIL_DIR)/%.o: src/flightcontrol/%.cpp | $(SIL_DIR)
	$(HOST_CXX) -c $(SIL_FLAGS) -std=c++17 -fno-rtti -fno-exceptions $< -o $@

$(SIL_DIR):
	mkdir -p $@

-include $(OBJECTS:.o=.d)
-include $(SIL_OBJECTS:.o=.d)
