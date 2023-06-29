
PLATFORM_PATH = ../../platform

XSENS_MTI680G_SDK_PATH = $(PLATFORM_PATH)/movella/xsens/MTi-680g/mtsdk-2022.0-xda_public_cpp/xspublic
XSENS_MTI680G_AARCH64_LIB_PATH = $(PLATFORM_PATH)/movella/xsens/MTi-680g/lib/arm_aarch64
XSENS_MTI680G_LIB_PATH = $(XSENS_MTI680G_AARCH64_LIB_PATH)

TARGETS:= di_gps_xsens
INCLUDE=-I. -I$(XSENS_MTI680G_SDK_PATH)
CFLAGS=-g $(INCLUDE)
CXXFLAGS=-std=c++11 $(CFLAGS)
LFLAGS=-L$(XSENS_MTI680G_LIB_PATH) -lxscontroller -lxscommon -lxstypes -lpthread -lrt -ldl

.PHONY: $(OBJLIBS)
all : $(TARGETS)

di_gps_xsens: di_gps_xsens.cpp.o

$(TARGETS):
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LFLAGS)

%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean :
	-$(RM) $(OBJECTS) $(TARGETS)
	-$(RM) *.o
