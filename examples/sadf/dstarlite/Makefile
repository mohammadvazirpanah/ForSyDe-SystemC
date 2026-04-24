# In ~/.bachrc --> export LD_LIBRARY_PATH=/home/mohammad/systemc-2.3.3/lib-linux64:$LD_LIBRARY_PATH 
EXTRACFLAGS = 
EXTRA_LIBS = 
MODULE = main
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
SYSTEMC = /home/mohammad/systemc-2.3.3
TARGET_ARCH = linux64
#SFF = ../../../src
SFF = /home/mohammad/ForSyDe-SystemC/src
CC     = g++  
#CC     = clang++  
# OPT    = -O0 -std=c++17
OPT    = -O0 
# DEBUG  = -g
#SYSDIR = -I $(SYSTEMC)/include
#LIBDIR = -L. -L$(SYSTEMC)/lib-$(TARGET_ARCH)
INCDIR = -I. -I.. $(SYSDIR) -I$(SFF)

## Build with maximum gcc warning level
#CFLAGS = -Wall -Wno-unknown-pragmas -pthread $(DEBUG) $(OPT) $(EXTRACFLAGS)
# CFLAGS = -Wall -pthread $(DEBUG) $(OPT) $(EXTRACFLAGS)
CFLAGS = -Wall $(DEBUG) $(OPT) $(EXTRACFLAGS) -std=c++17 -DFORSYDE_INTROSPECTION -DFORSYDE_SELF_REPORTING
#CFLAGS = -arch i386 -Wall -Wno-deprecated -Wno-return-type -Wno-char-subscripts $(DEBUG) $(OPT) $(EXTRACFLAGS)


# LIBS   =  -lstdc++ -lm $(EXTRA_LIBS) -lsystemc
LIBS   = -lm $(EXTRA_LIBS) -lsystemc

EXE    = $(MODULE).x

.PHONY: clean 

#$(EXE): $(OBJS) $(SYSTEMC)/lib-$(TARGET_ARCH)/libsystemc.a 
$(EXE): $(OBJS)
	$(CC) $(CFLAGS) $(INCDIR) $(LIBDIR) -o $@ $(OBJS) $(LIBS)  2>&1 | c++filt


%.o : %.cpp
	$(CC) $(CFLAGS) $(INCDIR) -c -MMD -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d


clean:
	-rm -f $(OBJS) *~ $(EXE) *.vcd *.wif *.isdb *.dmp *.P *.log

-include $(SRCS:.cpp=.P)
