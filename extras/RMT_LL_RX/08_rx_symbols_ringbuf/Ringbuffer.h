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

#pragma once

#include <stdint.h>


// FIFO ringbuffer implementation.
//
// The ringbuffer can store up 1 byte less than allocated as
// start and end marker need to be one byte apart when the buffer
// is full, otherwise it would suddenly be empty.
//
// The buffer is not thread-safe.

class Ringbuffer
{
public:

  //begin with externally allocated buffer
  void begin(uint8_t* buffer, uint16_t buffer_size);

  uint8_t peek(uint16_t offset);

/*
  //get a pointer to internal buffer and max linear read length
  uint8_t* get_linear_read_buffer(uint16_t *linear_len);

  //get a pointer to internal buffer and max linear write length
  uint8_t* get_linear_write_buffer(uint16_t *linear_len);

  //pop len bytes without reading them
  uint16_t pop_skip(uint16_t len);

  //push len bytes without writing them
  uint16_t push_skip(uint16_t len);
*/
	/* @brief Constructor
	 *
	 * @note Does not allocate automatically.
	 */
	Ringbuffer() = default;

	/*
	 * @brief Destructor
	 *
	 * Automatically calls deallocate.
	 */
	~Ringbuffer();

	/* @brief Allocate ringbuffer
	 *
	 * @param buffer_size Number of bytes to allocate on heap.
	 *
	 * @returns false if allocation fails.
	 */
	bool allocate(uint16_t buffer_size);

	/*
	 * @brief Deallocate ringbuffer
	 *
	 * @note only required to deallocate and reallocate again.
	 */
	void deallocate();

	/*
	 * @brief Space available to copy bytes into
	 *
	 * @returns number of free bytes.
	 */
	uint16_t available() const;

	/*
	 * @brief Space used to copy data from
	 *
	 * @returns number of used bytes.
	 */
	uint16_t used() const;

	/*
	 * @brief Copy data into ringbuffer
	 *
	 * @param buf Pointer to buffer to copy from.
	 * @param buf_len Number of bytes to copy.
	 *
	 * @returns true if packet could be copied into buffer.
	 */
	bool push(const uint8_t *buf, uint16_t buf_len);

	/*
	 * @brief Get data from ringbuffer
	 *
	 * @param buf Pointer to buffer where data can be copied into.
	 * @param max_buf_len Max number of bytes to copy.
	 *
	 * @returns 0 if buffer is empty.
	 */
	uint16_t pop(uint8_t *buf, uint16_t max_buf_len);

private:
	uint16_t _size {0};
	uint8_t *_ringbuffer {nullptr};
	uint16_t _start{0};
	uint16_t _end{0};
  bool _isExternalBuffer {false};
};