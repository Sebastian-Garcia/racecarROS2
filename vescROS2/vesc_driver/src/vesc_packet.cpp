// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_packet.hpp"

#include <cassert>
#include <iterator>
#include <memory>
#include <string>

#include "vesc_driver/datatypes.hpp"
#include "vesc_driver/vesc_packet_factory.hpp"

namespace vesc_driver
{

constexpr CRC::Parameters<crcpp_uint16, 16> VescFrame::CRC_TYPE;

VescFrame::VescFrame(int payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256) {
    // single byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + payload_size));
    *frame_->begin() = 2;
    *(frame_->begin() + 1) = payload_size;
    payload_.first = frame_->begin() + 2;
  } else {
    // two byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + 1 + payload_size));
    *frame_->begin() = 3;
    *(frame_->begin() + 1) = payload_size >> 8;
    *(frame_->begin() + 2) = payload_size & 0xFF;
    payload_.first = frame_->begin() + 3;
  }

  payload_.second = payload_.first + payload_size;
  *(frame_->end() - 1) = 3;
}

VescFrame::VescFrame(const BufferRangeConst & frame, const BufferRangeConst & payload)
{
  /* VescPacketFactory::createPacket() should make sure that the input is valid, but run a few cheap
     checks anyway */
  assert(std::distance(frame.first, frame.second) >= VESC_MIN_FRAME_SIZE);
  assert(std::distance(frame.first, frame.second) <= VESC_MAX_FRAME_SIZE);
  assert(std::distance(payload.first, payload.second) <= VESC_MAX_PAYLOAD_SIZE);
  assert(
    std::distance(frame.first, payload.first) > 0 &&
    std::distance(payload.second, frame.second) > 0);

  frame_.reset(new Buffer(frame.first, frame.second));
  payload_.first = frame_->begin() + std::distance(frame.first, payload.first);
  payload_.second = frame_->begin() + std::distance(frame.first, payload.second);
}

VescPacket::VescPacket(const std::string & name, int payload_size, int payload_id)
: VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(std::distance(payload_.first, payload_.second) > 0);
  *payload_.first = payload_id;
}

VescPacket::VescPacket(const std::string & name, std::shared_ptr<VescFrame> raw)
: VescFrame(*raw), name_(name)
{
}

/*------------------------------------------------------------------------------------------------*/

VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw)
: VescPacket("FWVersion", raw)
{
}

int VescPacketFWVersion::fwMajor() const
{
  return *(payload_.first + 1);
}

int VescPacketFWVersion::fwMinor() const
{
  return *(payload_.first + 2);
}

REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)

VescPacketRequestFWVersion::VescPacketRequestFWVersion()
: VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw)
: VescPacket("Values", raw)
{
}

double VescPacketValues::temp_mos1() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 1)) << 8) +
    static_cast<uint16_t>(*(payload_.first + 2)));
  return static_cast<double>(v) / 10.0;
}

double VescPacketValues::temp_mos2() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 3)) << 8) +
    static_cast<uint16_t>(*(payload_.first + 4)));
  return static_cast<double>(v) / 10.0;
}

double VescPacketValues::current_motor() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 5)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 6)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 7)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 8)));
  return static_cast<double>(v) / 100.0;
}

double VescPacketValues::current_in() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 9)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 10)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 11)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 12)));
  return static_cast<double>(v) / 100.0;
}


double VescPacketValues::duty_now() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 21)) << 8) +
    static_cast<uint16_t>(*(payload_.first + 22)));
  return static_cast<double>(v) / 1000.0;
}

double VescPacketValues::rpm() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 23)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 24)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 25)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 26)));
  return static_cast<double>(-1 * v);
}

double VescPacketValues::amp_hours() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 27)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 28)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 29)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 30)));
  return static_cast<double>(v);
}

double VescPacketValues::amp_hours_charged() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 31)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 32)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 33)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 34)));
  return static_cast<double>(v);
}

double VescPacketValues::tachometer() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 35)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 36)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 37)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 38)));
  return static_cast<double>(v);
}

double VescPacketValues::tachometer_abs() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 39)) << 24) +
    (static_cast<uint32_t>(*(payload_.first + 40)) << 16) +
    (static_cast<uint32_t>(*(payload_.first + 41)) << 8) +
    static_cast<uint32_t>(*(payload_.first + 42)));
  return static_cast<double>(v);
}

int VescPacketValues::fault_code() const
{
  return static_cast<int32_t>(*(payload_.first + 56));
}

double VescPacketValues::v_in() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

double VescPacketValues::temp_pcb() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

double VescPacketValues::watt_hours() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

double VescPacketValues::watt_hours_charged() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

REGISTER_PACKET_TYPE(COMM_GET_VALUES, VescPacketValues)

VescPacketRequestValues::VescPacketRequestValues()
: VescPacket("RequestValues", 1, COMM_GET_VALUES)
{
  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/


VescPacketSetDuty::VescPacketSetDuty(double duty)
: VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  /** @todo range check duty */

  int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrent::VescPacketSetCurrent(double current)
: VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake)
: VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetRPM::VescPacketSetRPM(double rpm)
: VescPacket("SetRPM", 5, COMM_SET_RPM)
{
  int32_t v = static_cast<int32_t>(rpm);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetPos::VescPacketSetPos(double pos)
: VescPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos)
: VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  int16_t v = static_cast<int16_t>(servo_pos * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

}  // namespace vesc_driver