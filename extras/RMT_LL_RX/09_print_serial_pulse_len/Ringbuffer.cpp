/*=============================================================================
MIT License

Copyright (c) 2024 qqqlab https://github.com/qqqlab/ESP32-Serial-RMT-UART

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================*/

//modified version

/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/



#include "Ringbuffer.h"

#include <assert.h>
#include <string.h>

#include <esp_attr.h> //IRAM_ATTR

void Ringbuffer::begin(uint8_t* buffer, uint16_t buffer_size)
{
	_size = buffer_size;
	_ringbuffer = buffer;
  _isExternalBuffer = true;
}

Ringbuffer::~Ringbuffer()
{
	deallocate();
}

bool Ringbuffer::allocate(uint16_t buffer_size)
{
	assert(_ringbuffer == nullptr);

	_size = buffer_size;
	_ringbuffer = new uint8_t[_size];
  _isExternalBuffer = false;
	return _ringbuffer != nullptr;
}

void Ringbuffer::deallocate()
{
  if(!_isExternalBuffer) delete[] _ringbuffer;
	_ringbuffer = nullptr;
	_size = 0;
  _isExternalBuffer = false;
}


IRAM_ATTR uint16_t Ringbuffer::available() const
{
	if (_start > _end) {
		return _start - _end - 1;

	} else {
		return _start - _end - 1 + _size;
	}
}

IRAM_ATTR uint16_t Ringbuffer::used() const
{
	if (_start <= _end) {
		return _end - _start;
	} else {
		return _end - _start + _size; // Wrap around
	}
}


IRAM_ATTR bool Ringbuffer::push(const uint8_t *buf, uint16_t buf_len)
{
	if (buf_len == 0 || buf == nullptr) {
		// Nothing to add, we better don't try.
		return false;
	}

	if (_start > _end) {
		// Add after end up to start, no wrap around.

		// Leave one byte free so that start don't end up the same
		// which signals empty.
		const uint16_t available = _start - _end - 1;

		if (available < buf_len) {
			return false;
		}

		memcpy(&_ringbuffer[_end], buf, buf_len);
		_end += buf_len;

	} else {
		// Add after end, maybe wrap around.
		const uint16_t available = _start - _end - 1 + _size;

		if (available < buf_len) {
			return false;
		}

		const uint16_t remaining_packet_len = _size - _end;

		if (buf_len > remaining_packet_len) {
			memcpy(&_ringbuffer[_end], buf, remaining_packet_len);
			memcpy(&_ringbuffer[0], buf + remaining_packet_len, buf_len - remaining_packet_len);
			_end = buf_len - remaining_packet_len;
		} else {
			memcpy(&_ringbuffer[_end], buf, buf_len);
			_end += buf_len;
		}
	}

	return true;
}

IRAM_ATTR uint16_t Ringbuffer::pop(uint8_t *buf, uint16_t buf_max_len)
{
	if (buf == nullptr) return 0; // User needs to supply a valid pointer.

	if (_start == _end) return 0; // Empty

	if (_start < _end) {
		// No wrap around.
		uint16_t to_copy_len = _end - _start;
    if(to_copy_len > buf_max_len) to_copy_len = buf_max_len;

		memcpy(buf, &_ringbuffer[_start], to_copy_len);
		_start += to_copy_len;

		return to_copy_len;
	} else {
		// Potential wrap around.
		uint16_t to_copy_len = _end - _start + _size;

		if (to_copy_len > buf_max_len) {
			to_copy_len = buf_max_len;
		}

		const uint16_t remaining_buf_len = _size - _start;
		if (to_copy_len > remaining_buf_len) {
			memcpy(buf, &_ringbuffer[_start], remaining_buf_len);
			memcpy(buf + remaining_buf_len, &_ringbuffer[0], to_copy_len - remaining_buf_len);
			_start = to_copy_len - remaining_buf_len;
		} else {
			memcpy(buf, &_ringbuffer[_start], to_copy_len);
			_start += to_copy_len;
		}
		return to_copy_len;
	}
}

uint8_t Ringbuffer::peek(uint16_t offset) {
  uint32_t pos = _start + offset;
  if(pos > _size) pos -= _size;
  return _ringbuffer[pos];
}

/*
//get a pointer to internal buffer and max linear read length
IRAM_ATTR uint8_t* Ringbuffer::get_linear_read_buffer(uint16_t *linear_len) {
	if (_start <= _end) {
		*linear_len = _end - _start; //no wrap
	} else {
		*linear_len = _size - _start; // Wrap around
	}
  return &_ringbuffer[_start];
}

//get a pointer to internal buffer and max linear write length
IRAM_ATTR uint8_t* Ringbuffer::get_linear_write_buffer(uint16_t *linear_len) {
	if (_start <= _end) {
		*linear_len = _size - _end; //write until end of buffer
	} else {
		*linear_len = _start - _end - 1; //write until byte before start
	}
  return &_ringbuffer[_end];
}

//pop len bytes without reading them
IRAM_ATTR uint16_t Ringbuffer::pop_skip(uint16_t len)
{
	if (_start == _end) return 0; // Empty

  //limit len to used size
  uint16_t used_len = used();
  if(len > used_len) len = used_len;

  uint32_t newstart = (uint32_t)_start + len;
  if(newstart > _size) newstart -= _size; //wrap around
  _start = newstart;

  return len;
}

//push len bytes without writing them
IRAM_ATTR uint16_t Ringbuffer::push_skip(uint16_t len)
{
  //limit len to available size
  uint16_t avail_len = available();
  if(len > avail_len) len = avail_len;

  uint32_t newend = (uint32_t)_end + len;
  if(newend > _size) newend -= _size; //wrap around
  _end = newend;

  return len;
}
*/