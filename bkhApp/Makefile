TOP = ..
include $(TOP)/configure/CONFIG
DIRS += mbusSrc
DIRS += bkhErrSrc
DIRS += bkhSrc
DIRS += motorSrc

DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *db*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *Db*))
include $(TOP)/configure/RULES_DIRS
