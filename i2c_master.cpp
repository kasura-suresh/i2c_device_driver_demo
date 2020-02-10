#include <systemc.h>
#include <iostream>
using namespace std;

typedef uint8_t byte;
 
// APB or Host bus interface
struct apb_interface : public sc_interface {
  virtual bool apb_read(unsigned addr, unsigned& data) = 0;
  virtual bool apb_write(unsigned addr, unsigned data) = 0;
};


// I2C interface (for communication between I2C Master and I2C Slave)
struct i2c_interface : public sc_interface {
  virtual void send_start_bit() = 0;
  virtual bool send_addr(unsigned addr) = 0;
  virtual bool send_rd_wr(unsigned rd_wr) = 0;
  virtual bool send_data(byte data) = 0;
  virtual bool recv_data(byte& data) = 0;
  virtual void send_stop_bit() = 0;
};


// I2C Master module : Implements register read/write calls from Host using apb_interface
// And calls i2c if functions to send/receive data to/from I2C Slave
SC_MODULE(I2C_Master), public apb_interface {
  // Export to handle the incoming register read/write from CPU
  sc_export<apb_interface>  host_if;

  // Port to handle the sending/receiving of data to/from I2C_Slave
  sc_port<i2c_interface>    i2c_if;

  // I2C Master Component Registers Address/locations: Address map
  //     0x0 : baud_rate_reg
  //     0x4 : tx_data_reg
  //     0x8 : rx_data_reg
  //     0xc : i2c_slave_addr
  //     0x10 : rd_wr register
  enum RegAddr {
    BAUD_RATE_REG_ADDR = 0x0,
    TX_DATA_REG_ADDR = 0x4,
    RX_DATA_REG_ADDR = 0x8,
    I2C_SLAVE_ADDR_REG_ADDR = 0xc,
    RD_WR_REG_ADDR = 0x10
  };

  // Registers bank 
  sc_signal<unsigned>   baud_rate_reg{"baud_rate_reg", 16};  // default baud rate = 16 bits/sec
  sc_fifo<byte>         tx_data_reg{"tx_data_reg", 2048};      // Transmit fifo to send data
  sc_fifo<byte>         rx_data_reg{"rx_data_reg", 64};      // Receive fifo to receive data
  sc_signal<unsigned>   i2c_slave_addr{"i2c_slave_addr", 0x100};  // default slave addr = 0x100
  sc_signal<bool>       rd_wr{"rd_wr", 0};  // 0 => read operation, 1 => write operation

  unsigned db_no{0}; // Number of the Data byte being transferred

  // Function to read the registers inside I2C Master
  virtual bool apb_read(unsigned addr, unsigned& data) {
    if (addr == BAUD_RATE_REG_ADDR) {    // Read baud rate register
     SC_REPORT_ERROR("I2C_Read", "Reading Baud rate = ");
     cout << baud_rate_reg << endl;
     data = baud_rate_reg;
    } else if (addr == TX_DATA_REG_ADDR) {   // Error to read tx_data_reg fifo
     SC_REPORT_ERROR("I2C_Read", "cannot read from transmit fifo (write only)");
     return false;
    } else if (addr == RX_DATA_REG_ADDR) {  // Read the rx_data_reg fifo contents
     data = rx_data_reg.read();
    } else if (addr == I2C_SLAVE_ADDR_REG_ADDR) {  // Read the i2c_slave_addr register
     data = i2c_slave_addr;
    } else if (addr == RD_WR_REG_ADDR) {  // Read the rd_wr register
     data = rd_wr & 0x1;
    } else {   // Error : Out of bounds of all register addresses
     SC_REPORT_ERROR("I2C_Read", "Address out of range ");
     return false;
    }
    return true;
  }

  // Function to write the registers inside I2C Master
  virtual bool apb_write(unsigned addr, unsigned data) {
    if (addr == BAUD_RATE_REG_ADDR) {    // Read baud rate register
     SC_REPORT_INFO(name(), "Writing Baud rate = ");
     cout << hex << data << endl;
     baud_rate_reg = data;
    } else if (addr == TX_DATA_REG_ADDR) {   // Error to read tx_data_reg fifo
     SC_REPORT_INFO(name(), "Pushing data into tx fifo = ");
     cout << hex << data << endl;
     tx_data_reg.write(data);
    } else if (addr == RX_DATA_REG_ADDR) {  // Read the rx_data_reg fifo contents
     SC_REPORT_ERROR(name(), "cannot write to receive fifo (read only)");
     return false;
    } else if (addr == I2C_SLAVE_ADDR_REG_ADDR) {  // Read the i2c_slave_addr register
     SC_REPORT_INFO(name(), "Writing to SLAVE_ADDR_REG = ");
     cout << hex << data << endl;
     i2c_slave_addr = data;
    } else if (addr == RD_WR_REG_ADDR) {  // Read the rd_wr register
     SC_REPORT_INFO(name(), "Writing to RD_WR_REG = ");
     cout << hex << (data & 0x1) << endl;
     rd_wr = data & 0x1;
    } else {   // Error : Out of bounds of all register addresses
     SC_REPORT_ERROR(name(), "Address out of range ");
     return false;
    }
    return true;
  }


  SC_CTOR(I2C_Master) : host_if("host_if") {
    host_if.bind(*this);
    SC_THREAD(send_data);
    sensitive << tx_data_reg.data_written_event();
  }

  // Sends the data in tx_data_reg fifo using I2C port to the connected slave
  void send_data() {
    while (true) {
      byte data = tx_data_reg.read();  // If data is written to fifo, this is triggered..
      send_i2c_data(data);  // Send the byte 'data' using I2C protocol
    }
  }


  // I2C transfer
  // (a) I2C phase
  enum I2C_State { START_BIT, ADDR_PHASE, OP_PHASE, DATA_PHASE, STOP_BIT };
  I2C_State m_i2c_state{STOP_BIT};  // By default, in STOP state

  // Send a byte of data using I2C protocol : START_BIT -> ADDR -> DATA BYTES -> STOP_BIT
  void send_i2c_data(byte data) {
    SC_REPORT_INFO(__func__, "Sending by I2C data = ");
    cout << hex << (unsigned)data << endl;
    if (m_i2c_state == STOP_BIT) {
      m_i2c_state = START_BIT;
    }

    // TODO: Clock/baud rate based delay between phases

    if (m_i2c_state == START_BIT) {
      i2c_if->send_start_bit();
      m_i2c_state = ADDR_PHASE;
    }

    if (m_i2c_state == ADDR_PHASE) {
      if (i2c_if->send_addr(i2c_slave_addr)) {
        cout << "Addr : " << i2c_slave_addr << " sent on I2C if/port.." << endl;
        m_i2c_state = OP_PHASE;
      } else {
        cerr << "Addr : " << i2c_slave_addr << " sent on I2C if/port was not acknowledged by any slave .." << endl;
        m_i2c_state = STOP_BIT;  // FIXME: Check if this is correct
      }
    }

    if (m_i2c_state == OP_PHASE) {
      if (i2c_if->send_rd_wr(rd_wr)) {  // Write operation
        cout << "Operation : " << rd_wr << " sent on I2C if/port.." << endl;
        m_i2c_state = DATA_PHASE;
      } else {
        cerr << "Operation " << rd_wr << " send failed on I2C if/port .." << endl;
        m_i2c_state = STOP_BIT;  // FIXME: Check if this is correct
      }
    }

    if (m_i2c_state == DATA_PHASE) {
      db_no++;
      if (!i2c_if->send_data(data)) {
        cerr << "Transmit of data byte number " << db_no << " failed.. more data to send.. hence sending stop bit" << endl; // FIXME: Is this correct?
        m_i2c_state = STOP_BIT;
      } else
        cout << "Transmit of data byte number " << db_no << ", data = " << (unsigned)data << " was successful.." << endl;
      if (tx_data_reg.num_available() == 0) {
        cout << "No more data to send.. hence sending stop bit" << endl; // FIXME: Is this correct?
        m_i2c_state = STOP_BIT;
      }
    }

    if (m_i2c_state == STOP_BIT) {
      i2c_if->send_stop_bit();
    }
  }
};


// A dummy I2C Slave
SC_MODULE(I2C_Slave), public i2c_interface {
  sc_export<i2c_interface> i2c_if;

  SC_CTOR(I2C_Slave) : i2c_if("i2c_if") {
    i2c_if.bind(*this);
  }

  // Functions that implement i2c_interface protocol
  virtual void send_start_bit() { }
  bool send_addr(unsigned addr) { 
    // if (addr == 0x123) return true;
    return true; 
  }
  bool send_rd_wr(unsigned rd_wr_op) { return true; }
  virtual bool send_data(byte data) { SC_REPORT_INFO(name(), "received data = "); cout << hex << (unsigned)data << endl; return true; }
  // Send all 0xFF bytes to I2C Master
  virtual bool recv_data(byte& data) { SC_REPORT_INFO(name(), "sending data = 0xFF"); data = 0xFF; return true; }
  virtual void send_stop_bit() { SC_REPORT_INFO(name(), "Received STOP bit"); }
};



// A testbench/dummy CPU/host_if Master
SC_MODULE(Host) {
  sc_port<apb_interface>  bus_if;

  SC_CTOR(Host) {
    SC_THREAD(test_i2c_master);
  }

  void test_i2c_master() {
    byte test_data[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd };

    // Configure the I2C Master
    bus_if->apb_write(0x0000, 512); // Set I2C Master baud rate as 512bps
    bus_if->apb_write(0x000c, 0x123);   // Set I2C Slave address = 0x123
    bus_if->apb_write(0x0010, 0x1);   // Set I2C operation = Write (1) (Read = 0)

    // Send the data to I2C Master to be transferred to I2C Slave
    for (int i = 0; i < sizeof(test_data)/sizeof(test_data[0]); ++i) {
      bus_if->apb_write(0x0004, test_data[i]);  // Writing to tx_fifo reg of I2C Master
    }
  }
};


int sc_main(int argc, char* argv[])
{
  I2C_Master    i2c_master{"i2c_master"};
  I2C_Slave     i2c_slave{"i2c_slave"};
  Host          cpu{"cpu"};

  cpu.bus_if(i2c_master.host_if);
  i2c_master.i2c_if(i2c_slave.i2c_if);

  cout << "Kasura did this..." << endl;

  sc_start();

  return 0;
}

 
