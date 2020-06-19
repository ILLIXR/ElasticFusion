CXX=clang++
CXXFLAGS=-std=c++2a -Wall -fPIC -I../../Core/src/ -I./Tools/ -I/home/hyjale/Pangolin/build/src/include/

LD_LIBS=-lpthread -pthread -lpangolin -L/user/local/lib/libefusion.so

DBG_FLAGS=-g
OPT_FLAGS=-O3

DBG_SO_NAME=plugin.dbg.so
OPT_SO_NAME=plugin.opt.so

SRCFILES=MainController.cpp Tools/LiveLogReader.cpp

$(DBG_SO_NAME): CXXFLAGS += $(DBG_FLAGS)
$(DBG_SO_NAME): $(SRCFILES)
	$(CXX) $(CXXFLAGS) $^ -shared -o $@  $(LD_LIBS)

$(OPT_SO_NAME): CXXFLAGS += $(OPT_FLAGS)
$(OPT_SO_NAME): $(SRCFILES)
	$(CXX) $(CXXFLAGS) $^ -shared -o $@  $(LD_LIBS)

clean:
	rm -rf *.o plugin.dbg.so plugin.opt.so
