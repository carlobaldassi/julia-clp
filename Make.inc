# -*- mode: makefile-gmake -*-

USR = $(JULIACLPHOME)/usr
USRLIB = $(USR)/lib
USRBIN = $(USR)/bin
USRINC = $(USR)/include
LLVMROOT = $(USR)
BUILD = $(USR)

OS = $(shell uname)
ARCH = $(shell uname -m)

ifeq ($(OS), MINGW32_NT-6.1)
OS=WINNT
endif

ifeq ($(OS), WINNT)
fPIC = 
else
fPIC = -fPIC
endif

# Compiler specific stuff

FC = gfortran
JFFLAGS = -O2 $(fPIC)

USEGCC = 1
USECLANG = 0

ifeq ($(OS), Darwin)
OSVER = $(shell uname -r | cut -b 1-2)
ifeq ($(OSVER), 10)
USEGCC = 1
USECLANG = 0
else
USEGCC = 0
USECLANG = 1
endif
endif

ifeq ($(USEGCC),1)
CC = gcc
CXX = g++
JCFLAGS = -std=gnu99 -pipe $(fPIC) -fno-strict-aliasing -D_FILE_OFFSET_BITS=64
JCXXFLAGS = -pipe $(fPIC) -fno-rtti
DEBUGFLAGS = -ggdb3 -DDEBUG
SHIPFLAGS = -O3 -DNDEBUG -falign-functions
ifneq ($(ARCH), ppc64)
  SHIPFLAGS += -momit-leaf-frame-pointer
endif
endif

ifeq ($(USECLANG),1)
CC = clang
CXX = clang++
JCFLAGS = -pipe $(fPIC) -fno-strict-aliasing -D_FILE_OFFSET_BITS=64
JCXXFLAGS = -pipe $(fPIC) -fno-rtti
DEBUGFLAGS = -g -DDEBUG
SHIPFLAGS = -O3 -DNDEBUG
ifeq ($(OS), Darwin)
CC += -mmacosx-version-min=10.6
CXX += -mmacosx-version-min=10.6
JCFLAGS += -D_LARGEFILE_SOURCE -D_DARWIN_USE_64_BIT_INODE=1
endif
endif

# Use libraries available on the system instead of building them

USE_SYSTEM_CLP=0

ifeq ($(USE_DEBIAN), 1)
USE_SYSTEM_CLP=1
endif

# OS specific stuff

# must end with a / and have no trailing spaces
INSTALL_NAME_ID_DIR = @rpath/

RPATH = 
RPATH_ORIGIN = 

ifeq ($(OS), WINNT)
SHELL_EXT = bat
else
SHELL_EXT = sh
endif

ifeq ($(OS), Linux)
INSTALL_NAME_CMD = true -ignore
INSTALL_NAME_CHANGE_CMD = true -ignore
SHLIB_EXT = so
RPATH = -Wl,-rpath,'$$ORIGIN/../lib'
RPATH_ORIGIN = -Wl,-rpath,'$$ORIGIN'
OSLIBS += -ldl -lrt -Wl,--export-dynamic -Wl,--version-script=$(JULIACLPHOME)/src/julia.expmap -Wl,--no-whole-archive $(LIBUNWIND)
WHOLE_ARCHIVE = -Wl,--whole-archive
NO_WHOLE_ARCHIVE = -Wl,--no-whole-archive
JLDFLAGS = -Wl,-Bdynamic
endif

ifeq ($(OS), FreeBSD)
INSTALL_NAME_CMD = true -ignore
INSTALL_NAME_CHANGE_CMD = true -ignore
SHLIB_EXT = so
RPATH = -Wl,-rpath,'$$ORIGIN/../lib' -Wl,-z,origin
RPATH_ORIGIN = -Wl,-rpath,'$$ORIGIN' -Wl,-z,origin
WHOLE_ARCHIVE = -Wl,--whole-archive
NO_WHOLE_ARCHIVE = -Wl,--no-whole-archive
JLDFLAGS = -Wl,-Bdynamic
OSLIBS += -lkvm -lrt -Wl,--export-dynamic -Wl,--version-script=$(JULIACLPHOME)/src/julia.expmap $(NO_WHOLE_ARCHIVE) $(LIBUNWIND)
endif

ifeq ($(OS), Darwin)
INSTALL_NAME_CMD = install_name_tool -id $(INSTALL_NAME_ID_DIR)
INSTALL_NAME_CHANGE_CMD = install_name_tool -change
RPATH = -Wl,-rpath,@executable_path/../lib
SHLIB_EXT = dylib
OSLIBS += -ldl -Wl,-w -framework ApplicationServices
WHOLE_ARCHIVE = -Xlinker -all_load
NO_WHOLE_ARCHIVE =
JLDFLAGS =
endif

ifeq ($(OS), WINNT)
INSTALL_NAME_CMD = true -ignore
INSTALL_NAME_CHANGE_CMD = true -ignore
SHLIB_EXT = dll
WHOLE_ARCHIVE = -Wl,--whole-archive
NO_WHOLE_ARCHIVE = -Wl,--no-whole-archive
OSLIBS += -Wl,--export-all-symbols -Wl,--version-script=$(JULIACLPHOME)/src/julia.expmap -lpthread $(NO_WHOLE_ARCHIVE) -lPsapi -lkernel32 -lWs2_32 -lIphlpapi
endif

# Make tricks

define dir_target
$$(subst $$(abspath $(JULIACLPHOME))/,,$$(abspath $(1))): $$(abspath $(1))
$$(abspath $(1)): 
	@mkdir -p $$@
endef

ifeq ($(OS), WINNT)
define mingw_to_dos
$(subst /,\\,$(subst $(shell $(2) pwd),$(shell $(2) cmd //C cd),$(abspath $(1))))
endef
define symlink_target
CLEAN_TARGETS += clean-$(2)/$(1)
clean-$$(abspath $(2)/$(1)): 
	@-cmd //C rmdir $$(call mingw_to_dos,$(2)/$(1),cd $(2) &&)
$$(subst $$(abspath $(JULIACLPHOME))/,,$$(abspath $(2)/$(1))): $$(abspath $(2)/$(1))
$$(abspath $(2)/$(1)): | $$(abspath $(2))
	@cmd //C mklink //J $$(call mingw_to_dos,$(2)/$(1),cd $(2) &&) $$(call mingw_to_dos,$(1),) 
endef
else
define symlink_target
CLEAN_TARGETS += clean-$(2)/$(1)
clean-$$(abspath $(2)/$(1)): 
	@-rm $$(abspath $(2)/$(1))
$$(subst $$(abspath $(JULIACLPHOME))/,,$$(abspath $(2)/$(1))): $$(abspath $(2)/$(1))
$$(abspath $(2)/$(1)): | $$(abspath $(2))
	@ln -sf $$(abspath $(1)) $$@ 
endef
endif

# Colors for make
ifndef USE_QUIET
USE_QUIET = 1
endif
ifeq ($(USE_QUIET), 1)

CCCOLOR="\033[34m"
LINKCOLOR="\033[34;1m"
PERLCOLOR="\033[35m"

SRCCOLOR="\033[33m"
BINCOLOR="\033[37;1m"
JULCOLOR="\033[34;1m"
ENDCOLOR="\033[0m"

GOAL=$(subst $(abspath $(JULIACLPHOME))/,,$(abspath $@))

QUIET_CC = @printf '    %b %b\n' $(CCCOLOR)CC$(ENDCOLOR) $(SRCCOLOR)$(GOAL)$(ENDCOLOR);
QUIET_LINK = @printf '    %b %b\n' $(LINKCOLOR)LINK$(ENDCOLOR) $(BINCOLOR)$(GOAL)$(ENDCOLOR);
QUIET_PERL = @printf '    %b %b\n' $(PERLCOLOR)PERL$(ENDCOLOR) $(BINCOLOR)$(GOAL)$(ENDCOLOR);

else

QUIET_CC =
QUIET_LINK =
QUIET_PERL =

endif
