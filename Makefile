JULIACLPHOME = $(abspath .)
include $(JULIACLPHOME)/Make.inc

all: default
default: deps

DIRS = $(BUILD)/bin $(BUILD)/etc $(BUILD)/lib/julia $(BUILD)/share/julia

$(foreach dir,$(DIRS),$(eval $(call dir_target,$(dir))))
$(foreach link,extras base ui test,$(eval $(call symlink_target,$(link),$(BUILD)/lib/julia)))
$(foreach link,doc examples,$(eval $(call symlink_target,$(link),$(BUILD)/share/julia)))

MAKEs = $(MAKE)
ifeq ($(USE_QUIET), 1)
MAKEs += -s
endif

deps:
	@$(MAKEs) -C deps

cleanall:
	@$(MAKE) -C deps cleanall

.PHONY: deps cleanall
