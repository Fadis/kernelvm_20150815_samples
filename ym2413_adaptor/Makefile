TOOLCHAIN = /opt/gcc/4.9-linaro/gcc-arm-none-eabi-4_9-2015q2-20150609/install-native/
PROJECT=template
MCU=LPC1114
OPTS = -Os -fstrict-aliasing -funsigned-char -funsigned-bitfields -fshort-enums -ffunction-sections -fno-common -fdata-sections -fomit-frame-pointer
CXX_OPTS = -Os -fstrict-aliasing -funsigned-char -funsigned-bitfields -fshort-enums -ffunction-sections -fno-common -fdata-sections -fomit-frame-pointer -fno-threadsafe-statics -fno-exceptions -fno-rtti -finline-functions -finline-small-functions -findirect-inlining -finline -fpartial-inlining -freorder-blocks  -falign-loops -falign-jumps -funroll-loops -fpeel-loops -std=c++14
WARNINGS = -Wno-strict-aliasing -Wundef -Wall -Wextra -Wunreachable-code #-nostdlib
LDSCRIPT=core/$(MCU).ld

ASM_SOURCES=$(wildcard core/*.s)
C_SOURCES=$(wildcard core/*.c)
CXX_SOURCES=$(wildcard src/*.cpp) $(wildcard core/*.cpp)
ELF_NAME=$(PROJECT).elf
BIN_NAME=$(PROJECT).bin

OBJECTS=$(patsubst %,./%,$(C_SOURCES:.c=.o)) $(patsubst %,./%,$(CXX_SOURCES:.cpp=.o)) $(patsubst %,./%,$(ASM_SOURCES:.s=.o))
DEPS=$(patsubst %,./%,$(C_SOURCES:.c=.d)) $(patsubst %,./%,$(CXX_SOURCES:.cpp=.d)) $(patsubst %,./%,$(ASM_SOURCES:.s=.d)) 
LSTFILES=$(patsubst %,./%,$(C_SOURCES:.c=.lst)) $(patsubst %,./%,$(CXX_SOURCES:.cpp=.lst)) $(patsubst %,./%,$(ASM_SOURCES:.s=.lst))

AS = $(TOOLCHAIN)/bin/arm-none-eabi-as
CC = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
CXX = $(TOOLCHAIN)/bin/arm-none-eabi-g++
OBJCOPY = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
REMOVE = rm -f
SIZE = $(TOOLCHAIN)/bin/arm-none-eabi-size
CFLAGS = -std=gnu99 -mcpu=cortex-m0 -mthumb -I. -Icore -Iinclude $(OPTS) -Wstrict-prototypes $(WARNINGS) -Wa,-adhlns=./$(<:.c=.lst)
CXXFLAGS = -std=gnu++11 -mcpu=cortex-m0 -mthumb -I. -Icore -Iinclude $(CXX_OPTS) $(WARNINGS) -Wa,-adhlns=./$(<:.cpp=.lst)
	LDFLAGS = -T$(LDSCRIPT) -Wl,--gc-sections -specs=nosys.specs
	#LDFLAGS = -T$(LDSCRIPT) -Wl,--gc-sections

#########################################################################

all: .bin ./core $(BIN_NAME) stats

.bin:
	mkdir .bin

./core:
	mkdir ./core

$(BIN_NAME): ./$(ELF_NAME) Makefile
	@$(OBJCOPY) -R .stack -O binary ./$(ELF_NAME) $(BIN_NAME)

./$(ELF_NAME): $(OBJECTS) Makefile
	@echo "  LD ($(OBJECTS)) -> $(ELF_NAME)"
	@$(CXX) -o ./$(ELF_NAME) $(LDFLAGS) -lc $(OBJECTS) $(CFLAGS)

stats: ./$(ELF_NAME) 
	@$(SIZE) ./$(ELF_NAME)

clean:
	@echo "  Cleanup $<"
	@$(REMOVE) $(OBJECTS)
	@$(REMOVE) $(DEPS)
	@$(REMOVE) $(LSTFILES)
	@$(REMOVE) $(BIN_NAME)
	@$(REMOVE) $(ELF_NAME)

#########################################################################

./%.o: %.c Makefile 
	@echo "  CC $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

./%.o: %.cpp Makefile 
	@echo "  CXX $<"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

./%.o: %.s Makefile 
	@echo "  AS $<"
	@$(AS) -o $@ -c $<

.PHONY : clean all stats

