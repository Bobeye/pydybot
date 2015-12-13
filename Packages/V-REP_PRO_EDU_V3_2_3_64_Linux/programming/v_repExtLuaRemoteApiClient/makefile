CFLAGS = -I../include -I../remoteApi -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wall -fPIC -static

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
	g++ $(CFLAGS) -c v_repExtLuaRemoteApiClient.cpp -o v_repExtLuaRemoteApiClient.o
	g++ $(CFLAGS) -c remoteApiLink.cpp -o remoteApiLink.o
	gcc $(CFLAGS) -c ../remoteApi/extApi.c -o extApi.o
	gcc $(CFLAGS) -c ../remoteApi/extApiPlatform.c -o extApiPlatform.o
	g++ $(CFLAGS) -c ../common/luaFunctionData.cpp -o luaFunctionData.o
	g++ $(CFLAGS) -c ../common/luaFunctionDataItem.cpp -o luaFunctionDataItem.o
	g++ $(CFLAGS) -c ../common/v_repLib.cpp -o v_repLib.o
	@mkdir -p lib
	g++ extApi.o extApiPlatform.o luaFunctionData.o luaFunctionDataItem.o v_repExtLuaRemoteApiClient.o remoteApiLink.o v_repLib.o -o lib/libv_repExtLuaRemoteApiClient.$(EXT) -lpthread -ldl -shared
