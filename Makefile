SRC=i2c_master.cpp

all:
	g++ -std=c++11 -static -I $(SYSTEMC_HOME)/include $(SRC) -L $(SYSTEMC_HOME)/lib-linux64 -lsystemc
# g++ -std=c++11 -static -I $(SYSTEMC_HOME)/include $(sr) -L $(SYSTEMC_HOME)/lib-linux64 -lsystemc
