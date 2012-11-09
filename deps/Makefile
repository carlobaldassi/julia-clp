include Versions.make

## high-level setup ##

JULIACLPHOME = $(abspath ..)
include $(JULIACLPHOME)/Make.inc

# prevent installing libs into usr/lib64 on opensuse
unexport CONFIG_SITE

LIBS = clp

WGET = $(abspath .)/jldownload
WGET_DASH_O = $(WGET)

## Common build target prefixes

default: $(USR) install
compile: $(addprefix compile-, $(LIBS))
install: $(addprefix install-, $(LIBS))
cleanall: $(addprefix clean-, $(LIBS))
distclean: $(addprefix distclean-, $(LIBS))
	rm -rf $(USR)

## PATHS ##
DIRS = $(addprefix $(USR)/,lib include bin share etc)

$(foreach dir,$(DIRS),$(eval $(call dir_target,$(dir))))

$(USR): $(DIRS)

## CLP ##

ifeq ($(USE_SYSTEM_CLP), 1)
CLP_OBJ_TARGET =
else
CLP_OBJ_TARGET = $(USRLIB)/libClp.$(SHLIB_EXT)
endif

compile-clp: install-clp
install-clp: $(CLP_OBJ_TARGET)

Clp-$(CLP_VER).tgz:
	$(WGET) http://www.coin-or.org/download/source/Clp/$@

Clp-$(CLP_VER)/configure: Clp-$(CLP_VER).tgz
	tar zxf $<
	patch -p0 < Clp-$(CLP_VER).patch
	touch $@
Clp-$(CLP_VER)/config.status: Clp-$(CLP_VER)/configure
	cd Clp-$(CLP_VER) && \
	./configure --prefix=$(abspath $(USR)) CC="$(CC)" CXX="$(CXX)"
$(CLP_OBJ_TARGET): Clp-$(CLP_VER)/config.status
	$(MAKE) -C Clp-$(CLP_VER) install
	$(INSTALL_NAME_CMD)libClp.dylib $@
	touch $@

clean-clp:
	-$(MAKE) -C Clp-$(CLP_VER) uninstall || true
	-$(MAKE) -C Clp-$(CLP_VER) clean
distclean-clp: clean-clp
	rm -rf Clp-$(CLP_VER).tgz Clp-$(CLP_VER)

## phony targets ##

.PHONY: \
	default compile install cleanall distclean \
	compile-* install-* clean-* distclean-*
