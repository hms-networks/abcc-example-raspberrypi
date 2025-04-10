# define the C compiler to use
CC = gcc

# define any compile-time flags
CFLAGS_RELEASE = -O2
CFLAGS_DEBUG = -g -Og
CFLAGS_CSTANDARD = c11
CFLAGS_WARN  = -Wall
CFLAGS_WARN += -Wpedantic
CFLAGS_WARN += -Wextra
CFLAGS_WARN += -Wdouble-promotion
CFLAGS_WARN += -Wmissing-include-dirs
CFLAGS_WARN += -Wswitch-default
CFLAGS_WARN += -Wswitch-enum
CFLAGS_WARN += -Wunused-parameter
CFLAGS_WARN += -Wfloat-equal
CFLAGS_WARN += -Wundef
CFLAGS_WARN += -Wshadow
CFLAGS_WARN += -Wunsafe-loop-optimizations
CFLAGS_WARN += -Wbad-function-cast
CFLAGS_WARN += -Wcast-qual
CFLAGS_WARN += -Wmissing-declarations
CFLAGS_WARN += -Wstrict-prototypes
CFLAGS_WARN += -Wold-style-definition
CFLAGS_WARN += -Waggregate-return
CFLAGS_WARN += -Wlogical-op
CFLAGS_WARN += -Wconversion

CFLAGS_DEFINE = -D _GNU_SOURCE

CFLAGS = $(CFLAGS_WARN) -std=$(CFLAGS_CSTANDARD) $(CFLAGS_DEFINE)

# define any directories containing header files other than /usr/include
#
#INCLUDES = -I/home/newhall/include -I../include
INCLUDES = -I./src/abcc_adaptation

# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
#LFLAGS = -L/home/newhall/lib -L../lib
LFLAGS =

# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname
#   option, something like (this will link in libmylib.so and libm.so:
#LIBS = -lmylib -lm
LIBS  = -lgpiod
LIBS += -lpthread

# define the C source files
SRCS  = ./src/main.c
SRCS += ./src/abcc_adaptation/abcc_hardware_abstraction.c
SRCS += ./src/example_application/abcc_network_data_parameters.c
SRCS += ./src/example_application/implemented_callback_functions.c
SRCS += ./src/example_application/Logprint/logprint.c

# store the path to the Anybus CompactCom Driver API directory
ABCC_API_DIR := ./lib/abcc-driver-api

# include the Anybus CompactCom Driver API
include $(ABCC_API_DIR)/abcc-driver-api.mk

# define the C object files
#
# This uses Suffix Replacement within a macro:
#   $(name:string1=string2)
#         For each word in 'name' replace 'string1' with 'string2'
# Below we are replacing the suffix .c of all words in the macro SRCS
# with the .o suffix
#
OBJS = $(SRCS:.c=.o)

# define the executable file
TARGET = raspberry_pi_example_project

#
# The following part of the makefile is generic; it can be used to
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

release: CFLAGS += $(CFLAGS_RELEASE)
release: $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET) $(OBJS) $(LFLAGS) $(LIBS)

debug: CFLAGS += $(CFLAGS_DEBUG)
debug: $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET) $(OBJS) $(LFLAGS) $(LIBS)

# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file)
# (see the gnu make manual section about automatic variables)
.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	$(RM) $(OBJS) *~ $(TARGET)

# DO NOT DELETE THIS LINE -- make depend needs it
