#====================================================================
#
# Copyright (c) 2020 Juergen Liegner  All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#
# 3. Neither the name of the author(s) nor the names of any contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR(S) OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
#====================================================================/

# Projekt name 
PROJECT  = stm32f103c8-bionx

DEBUG=0

ifeq ($(DEBUG), 1)
  # Optimierung normal=size debug=keine
  OPT =     -Os
  OPT_DBG = -O0 
  OUTDIR=debug
  DEFS += -DDEBUG
#  LDDEBUGFLAGS= -L system -ldebugio_v7m_t_le_eabi -ldebugio_bkpt_v7m_t_le_eabi -ldebugio_mempoll_v7m_t_le_eabi
else
  # Optimierung normal=size debug=size
  OPT =     -Os
  OPT_DBG = -Os 
  OUTDIR=release
  # CROSSWORKSFLAG=-DSTARTUP_FROM_RESET
endif  

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPP  = $(TRGT)g++
SZ   = $(TRGT)size
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

MCU  = cortex-m3

# Alle Defines fuer C und C++ hier einfuegen
DEFS +=-D__USE_CMSIS -D__NEWLIB__ $(CROSSWORKSFLAG) -DSTM32F10X_MD -DSTARTUP_FROM_RESET
#-DUSE_STDPERIPH_DRIVER 

# Alle Defines fuer den Assembler hier einfuegen
ADEFS = $(CROSSWORKSFLAG)

# include Verzeichnisse (wird mit -I.. ersetzt)
DINCDIR = ./src ./system 

# lib verzeichnisse (wird mit -L.. ersetzt)
DLIBDIR = 

# alle zusaetzlichen libs 
LIBS = 

# Projekt name 
PROJECT  = stm32f103c8-bionx

# Linker script file
LDSCRIPT = ./system/gnu/stm32f103c8.ld

FULL_PRJ = $(OUTDIR)/$(PROJECT)

# alle Quelldateien
# SRC       = C Quellen 
# SRC_DBG   = C Quellen mit Debuginformationen 
# SRCPP     = C++ Quellen 
# SRCPP_DBG = C++ Quellen mit Debuginformationen 

SRCPP     += $(wildcard ./libs/*.cpp) 
SRC       += $(wildcard ./libs/*.c) 
SRCPP_DBG += ./src/main.cpp
SRCPP_DBG += ./src/printf.cpp
SRCPP_DBG += ./src/can.cpp
SRC       += ./system/gnu/startup_stm32f103.c

# List ASM source files here
ASRC      += 

# aus den Verzeichnissen in DINCDIR Compilerschalter in der Form -I... erzeugen
INCDIR  = $(patsubst %,-I%,$(DINCDIR))

# aus den Verzeichnissen in DINCDIR Compilerschalter in der Form -L... erzeugen
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR))

MCFLAGS = -mcpu=$(MCU)

# Compilerschalter fuer Assembler Dateien
ASFLAGS  = -mthumb -g -gdwarf-2 
ASFLAGS += $(ADEFS) $(MCFLAGS)

# Compilerschalter fuer C Dateien
CFLAGS   = -gdwarf-2 -g2 -mthumb -fno-strict-aliasing -fomit-frame-pointer -Wall -fverbose-asm  
CFLAGS   += -std=gnu99 
CFLAGS   += -ffunction-sections -fdata-sections 
CFLAGS   += $(DEFS) $(MCFLAGS)

# Compilerschalter fuer C++ Dateien
CPPFLAGS =  -gdwarf-2 -g2 -mthumb  -fno-strict-aliasing  -fomit-frame-pointer  -fverbose-asm  -Wall 
CPPFLAGS += -fno-rtti -fno-exceptions -std=gnu++11
CPPFLAGS += -ffunction-sections -fdata-sections 
CPPFLAGS += $(DEFS) $(MCFLAGS)

LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles $(LDDEBUGFLAGS) -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch,--gc-sections $(LIBDIR)
# 
#
# Templates für die einzelnen Dateitypen. Es wird am Ende für jede einzelen Datei eine
# eigene Regel erstellt. Achtung die Leerzeilen vor endef duerfen nicht entfernt werden

define ASMTEMPL
$(OUTDIR)/$(1).o: $(2)
	$(AS) -c $(ASFLAGS) -Wa,-ahlms=$(OUTDIR)/$(1).lst $(2) -o $(OUTDIR)/$(1).o
  
endef

define CTEMPL
$(OUTDIR)/$(1).o: $(2)
	$(CC) -c $(CFLAGS) $(OPT) $(INCDIR) -Wa,-ahlms=$(OUTDIR)/$1.lst  -MD -MP -MF .dep/$(1).d  $(2) -o $(OUTDIR)/$(1).o
  
endef

define CPPTEMPL
$(OUTDIR)/$(1).o: $(2)
	$(CPP) -c $(CPPFLAGS) $(OPT) $(INCDIR)  -Wa,-ahlms=$(OUTDIR)/$1.lst -MD -MP -MF .dep/$(1).d $(2) -o $(OUTDIR)/$(1).o
  
endef

define CTEMPL_DBG
$(OUTDIR)/$(1).o: $(2)
	$(CC) -c $(CFLAGS) $(OPT_DBG) $(INCDIR) -Wa,-ahlms=$(OUTDIR)/$1.lst  -MD -MP -MF .dep/$(1).d $(2) -o $(OUTDIR)/$(1).o
  
endef

define CPPTEMPL_DBG
$(OUTDIR)/$(1).o: $(2)
	$(CPP) -c $(CPPFLAGS) $(OPT_DBG) $(INCDIR) -Wa,-ahlms=$(OUTDIR)/$1.lst  -MD -MP -MF .dep/$(1).d $(2) -o $(OUTDIR)/$(1).o
  
endef


# macht aus z.B. aus src/main.cpp oder ./libs/i2c/i2c.c die Dateinamen fuer dei Objectdateien in der Form: release/main.o release/i2c.o
# das bedeutet, es duerfen keinen gleichen dateinamen in unterschiedlichen Verzeichnissen vorkommen
OBJS= $(patsubst %.o,$(OUTDIR)/%.o,$(notdir $(SRCPP:%.cpp=%.o) $(SRC:%.c=%.o) $(SRCPP_DBG:%.cpp=%.o) $(SRC_DBG:%.c=%.o) $(ASRC:%.s=%.o)))

# Hauptregel fuer das Bilden. $(OUTDIR) steht am Anfang um das Verzeichnis (release oder debug) anzulegen
# wenn es nooch nicht vorhanden ist. 
all: $(OUTDIR) .dep $(OBJS) $(FULL_PRJ).elf $(FULL_PRJ).hex $(FULL_PRJ).bin

# Verzeichnis anlegen wenn es nicht existiert
$(OUTDIR):
	mkdir  $(OUTDIR)  2>/dev/null

.dep:
	-mkdir .dep
	
  
# erzeugt fuer jede Datei eine eigene Regel. Als Vorlage dienen die Templates
# es entstehen Regeln in der Form:
# release/main.o: ./scr/main.cpp
#		$(CC) ....... ./scr/main.cpp -o release/main.o
# diese Regeln stehen erst mal nur im EVALSTR hintereinander
EVALSTR= $(foreach a,$(SRC),$(call CTEMPL,$(notdir $(a:%.c=%)),$(a)))
EVALSTR+=$(foreach a,$(SRCPP),$(call CPPTEMPL,$(notdir $(a:%.cpp=%)),$(a)))
EVALSTR+=$(foreach a,$(SRC_DBG),$(call CTEMPL_DBG,$(notdir $(a:%.c=%)),$(a)))
EVALSTR+=$(foreach a,$(SRCPP_DBG),$(call CPPTEMPL_DBG,$(notdir $(a:%.cpp=%)),$(a)))
EVALSTR+=$(foreach a,$(ASRC),$(call ASMTEMPL,$(notdir $(a:%.s=%)),$(a)))

# die Regeln werden in EVALSTR zusammengebaut, mit $(eval $(EVALSTR)) werden sie dann 
# in das makefile eingebaut. Wenn man hier $(info $(EVALSTR)) einfuegt
# werden die Regeln ausgegeben. Hilfreich um fehler zu finden
$(eval $(EVALSTR))

%elf: $(OBJS) 
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

%hex: %elf
	$(HEX) $< $@

%bin: %elf
	$(BIN) $< $@

clean:
	-rm -f $(OUTDIR)/*.elf
	-rm -f $(OUTDIR)/*.map
	-rm -f $(OUTDIR)/*.lst
	-rm -f $(OUTDIR)/*.hex
	-rm -f $(OUTDIR)/*.bin
	-rm -f $(OUTDIR)/*.o
	-rm -f $(OUTDIR)/*.prepatch
	-rm -fR .dep

# 
# Alle Abhaengigkeiten includen 
#
-include $(wildcard .dep/*)




