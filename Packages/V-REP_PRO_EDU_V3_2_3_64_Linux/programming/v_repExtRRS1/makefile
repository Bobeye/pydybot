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
	g++ $(CFLAGS) -c v_repExtRRS1.cpp -o v_repExtRRS1.o
	g++ $(CFLAGS) -c inputOutputBlock.cpp -o inputOutputBlock.o
	g++ $(CFLAGS) -c ../common/socketOutConnection.cpp -o socketOutConnection.o
	g++ $(CFLAGS) -c ../common/luaFunctionData.cpp -o luaFunctionData.o
	g++ $(CFLAGS) -c ../common/luaFunctionDataItem.cpp -o luaFunctionDataItem.o
	g++ $(CFLAGS) -c ../common/v_repLib.cpp -o v_repLib.o
	@mkdir -p lib
	g++ v_repExtRRS1.o socketOutConnection.o luaFunctionData.o luaFunctionDataItem.o inputOutputBlock.o v_repLib.o -o lib/libv_repExtRRS1.$(EXT) -lpthread -ldl -shared 

