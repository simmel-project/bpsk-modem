CC         := $(CROSS_COMPILE)gcc
CXX        := $(CROSS_COMPILE)g++
OBJCOPY    := $(CROSS_COMPILE)objcopy

RM         := rm -rf
COPY       := cp -a
PATH_SEP   := /

ifeq ($(OS),Windows_NT)
# When SHELL=sh.exe and this actually exists, make will silently
# switch to using that instead of cmd.exe.  Unfortunately, there's
# no way to tell which environment we're running under without either
# (1) printing out an error message, or (2) finding something that
# works everywhere.
# As a result, we force the shell to be cmd.exe, so it works both
# under cygwin and normal Windows.
SHELL      = cmd.exe
COPY       = copy
RM         = del
PATH_SEP   = \\
endif

BASE_DIR   := .
ADD_CFLAGS := -I$(BASE_DIR)/include -I$(BASE_DIR)/CMSIS/include -DARM_MATH_CM0
ADD_LFLAGS :=
PACKAGE    := bpsktest

SRC_DIR    := $(BASE_DIR)/src
DBG_CFLAGS := -ggdb -g -DDEBUG -Wall -O3
DBG_LFLAGS := -ggdb -g -Wall
CFLAGS     := $(ADD_CFLAGS) \
			  -Wall -Wextra \
			  -std=gnu11
CXXFLAGS   := $(CFLAGS) -std=c++11
LFLAGS     := $(CFLAGS) $(ADD_LFLAGS) -L$(LD_DIR)
LIBS       := -lm

OBJ_DIR    := .obj

CSOURCES   := $(wildcard $(SRC_DIR)/*.c) $(wildcard CMSIS/source/*.c)
CPPSOURCES := $(wildcard $(SRC_DIR)/*.cpp) $(wildcard CMSIS/source/*.cpp)
ASOURCES   := $(wildcard $(SRC_DIR)/*.S) $(wildcard CMSIS/source/*.S)
COBJS      := $(addprefix $(OBJ_DIR)/, $(notdir $(CSOURCES:.c=.o)))
CXXOBJS    := $(addprefix $(OBJ_DIR)/, $(notdir $(CPPSOURCES:.cpp=.o)))
AOBJS      := $(addprefix $(OBJ_DIR)/, $(notdir $(ASOURCES:.S=.o)))
OBJECTS    := $(COBJS) $(CXXOBJS) $(AOBJS)
VPATH      := $(SRC_DIR) CMSIS/source

QUIET      := @

ALL        := all
TARGET     := $(PACKAGE)
CLEAN      := clean

$(ALL): $(TARGET)

$(OBJECTS): | $(OBJ_DIR)

$(TARGET): $(OBJECTS) $(LDSCRIPTS)
	$(QUIET) echo "  LD       $@"
	$(QUIET) $(CC) $(OBJECTS) $(LFLAGS) -o $@ $(LIBS)

$(DEBUG): CFLAGS += $(DBG_CFLAGS)
$(DEBUG): LFLAGS += $(DBG_LFLAGS)
CFLAGS += $(DBG_CFLAGS)
LFLAGS += $(DBG_LFLAGS)
$(DEBUG): $(TARGET)

ifneq ($(BAUD_RATE),)
CFLAGS += -DBAUD_RATE=$(BAUD_RATE)
endif
ifneq ($(F_LO),)
CFLAGS += -DF_LO=$(F_LO)
endif
ifneq ($(F_HI),)
CFLAGS += -DF_HI=$(F_HI)
endif
ifneq ($(SAMPLE_RATE),)
CFLAGS += -DSAMPLE_RATE=$(SAMPLE_RATE)
endif

$(OBJ_DIR):
	$(QUIET) mkdir $(OBJ_DIR)

$(COBJS) : $(OBJ_DIR)/%.o : %.c $(BASE_DIR)/Makefile
	$(QUIET) echo "  CC       $<	$(notdir $@)"
	$(QUIET) $(CC) -c $< $(CFLAGS) -o $@ -MMD

$(OBJ_DIR)/%.o: %.cpp
	$(QUIET) echo "  CXX      $<	$(notdir $@)"
	$(QUIET) $(CXX) -c $< $(CXXFLAGS) -o $@ -MMD

$(OBJ_DIR)/%.o: %.S
	$(QUIET) echo "  AS       $<	$(notdir $@)"
	$(QUIET) $(CC) -x assembler-with-cpp -c $< $(CFLAGS) -o $@ -MMD

.PHONY: clean

clean:
	$(QUIET) echo "  RM      $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.d))"
	-$(QUIET) $(RM) $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.d))
	$(QUIET) echo "  RM      $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.d))"
	-$(QUIET) $(RM) $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.o))
	$(QUIET) echo "  RM      $(TARGET) $(PACKAGE).bin $(PACKAGE).symbol $(PACKAGE).ihex $(PACKAGE).dfu"
	-$(QUIET) $(RM) $(TARGET) $(PACKAGE).bin $(PACKAGE).symbol $(PACKAGE).ihex $(PACKAGE).dfu

include $(wildcard $(OBJ_DIR)/*.d)
