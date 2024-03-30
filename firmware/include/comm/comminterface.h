#ifndef COMM_INTERFACE_H
#define COMM_INTERFACE_H
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <etl/vector.h>
namespace unav::comm {
struct CommInterface {
  virtual void init() = 0;
  virtual void read(etl::ivector<uint8_t> &buffer, size_t &read_count) = 0;
  virtual void write(etl::ivector<uint8_t> &buffer, size_t length) = 0;
  virtual bool is_ready() = 0;
  virtual ~CommInterface() {
  }
};
} // namespace unav::comm
#endif COMM_INTERFACE_H