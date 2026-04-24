MODULE     = main
SRCS       = $(wildcard *.cpp)
OBJS       = $(SRCS:.cpp=.o)

SYSTEMC    = /deps/systemc
TARGET_ARCH= linux64
SFF        = /forsyde/src

ROS_DISTRO ?= noetic
ROS_ROOT   = /opt/ros/$(ROS_DISTRO)

CC         = g++
OPT        = -O0
CFLAGS     = -Wall $(OPT) -std=c++17 \
             -DDISPLAY \
             -DFORSYDE_INTROSPECTION \
             -DFORSYDE_SELF_REPORTING

SYSDIR     = -I$(SYSTEMC)/include
LIBDIR     = -L$(SYSTEMC)/lib-$(TARGET_ARCH)

ROS_INCLUDES = -I$(ROS_ROOT)/include
ROS_LIBS     = -L$(ROS_ROOT)/lib \
               -lroscpp -lrosconsole -lrostime \
               -lroscpp_serialization -lxmlrpcpp \
               -lcpp_common \
               -lrosconsole_log4cxx -lrosconsole_backend_interface \
               -lpthread -lboost_system -lboost_thread

INCDIR     = -I. -I$(SFF) $(SYSDIR) $(ROS_INCLUDES)
LIBS       = -lm -lsystemc $(ROS_LIBS)

EXE        = $(MODULE).x

.PHONY: clean

$(EXE): $(OBJS)
	$(CC) $(CFLAGS) $(INCDIR) $(LIBDIR) -o $@ $(OBJS) $(LIBS) 2>&1 | c++filt

%.o : %.cpp
	$(CC) $(CFLAGS) $(INCDIR) -c -MMD -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

clean:
	-rm -f $(OBJS) *~ $(EXE) *.vcd *.P *.log gen/*.xml gen/*.dot

-include $(SRCS:.cpp=.P)
