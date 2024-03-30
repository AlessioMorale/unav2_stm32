#ifndef INTERFACE_H
#define INTERFACE_H
#pragma once

#include "comminterface.h"

namespace unav::comm {
constexpr size_t RX_BUFFER_SIZE = 512;
constexpr size_t TX_BUFFER_SIZE = 512;

template <class T_Comm> class Interface {
public:
  Interface(T_Comm *comm_interface) {
    static_cast<CommInterface *>(comm_interface);
    comm_interface_ = comm_interface;
    comm_interface_->init();
  }
  ~Interface();

  void spin(){

  }

private:
  T_Comm *comm_interface_;
  etl::vector<uint8_t, RX_BUFFER_SIZE> rx_buffer_;
  etl::vector<uint8_t, TX_BUFFER_SIZE> tx_buffer_;

  /* data */
};
} // namespace unav::comm

#endif /* INTERFACE_H */
