CFLAGS = -I../include -Wall -fPIC -static

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	EXT = so
else
	CFLAGS += -D__APPLE__
	EXT = dylib
endif

all: 
	@rm -f lib/*.$(EXT)
	@rm -f *.o 
	g++ $(CFLAGS) -c v_repExtMtb.cpp -o v_repExtMtb.o
	g++ $(CFLAGS) -c ../common/socketOutConnection.cpp -o outConnection.o
	g++ $(CFLAGS) -c ../common/luaFunctionData.cpp -o functionData.o
	g++ $(CFLAGS) -c ../common/luaFunctionDataItem.cpp -o functionDataItem.o
	g++ $(CFLAGS) -c ../common/v_repLib.cpp -o v_repLib.o
	@mkdir -p lib
	g++ v_repExtMtb.o socketOutConnection.o luaFunctionData.o luaFunctionDataItem.o v_repLib.o -o lib/libv_repExtMtb.$(EXT) -lpthread -ldl -shared 

