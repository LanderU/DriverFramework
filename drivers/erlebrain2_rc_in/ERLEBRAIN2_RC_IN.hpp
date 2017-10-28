/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "BaroSensor.hpp"
#include "../common/common.hpp"


#define PAGE_SIZE                (4*1024)
#define LINUX_RC_INPUT_NUM_CHANNELS 16
//Parametres
#define RCIN_RPI_BUFFER_LENGTH   8
#define RCIN_RPI_SAMPLE_FREQ     500
#define RCIN_RPI_DMA_CHANNEL     0
#define RCIN_RPI_MAX_COUNTER     1300
#define PPM_INPUT_RPI            4 //RPI_GPIO_4
#define RCIN_RPI_MAX_SIZE_LINE   50

//Memory Addresses
#define RCIN_RPI_RPI1_DMA_BASE 0x20007000
#define RCIN_RPI_RPI1_CLK_BASE 0x20101000
#define RCIN_RPI_RPI1_PCM_BASE 0x20203000

#define RCIN_RPI_RPI2_DMA_BASE 0x3F007000
#define RCIN_RPI_RPI2_CLK_BASE 0x3F101000
#define RCIN_RPI_RPI2_PCM_BASE 0x3F203000

#define RCIN_RPI_GPIO_LEV0_ADDR  0x7e200034
#define RCIN_RPI_DMA_LEN         0x1000
#define RCIN_RPI_CLK_LEN         0xA8
#define RCIN_RPI_PCM_LEN         0x24
#define RCIN_RPI_TIMER_BASE      0x7e003004

#define RCIN_RPI_DMA_SRC_INC     (1<<8)
#define RCIN_RPI_DMA_DEST_INC    (1<<4)
#define RCIN_RPI_DMA_NO_WIDE_BURSTS  (1<<26)
#define RCIN_RPI_DMA_WAIT_RESP   (1<<3)
#define RCIN_RPI_DMA_D_DREQ      (1<<6)
#define RCIN_RPI_DMA_PER_MAP(x)  ((x)<<16)
#define RCIN_RPI_DMA_END         (1<<1)
#define RCIN_RPI_DMA_RESET       (1<<31)
#define RCIN_RPI_DMA_INT         (1<<2)

#define RCIN_RPI_DMA_CS          (0x00/4)
#define RCIN_RPI_DMA_CONBLK_AD   (0x04/4)
#define RCIN_RPI_DMA_DEBUG       (0x20/4)

#define RCIN_RPI_PCM_CS_A        (0x00/4)
#define RCIN_RPI_PCM_FIFO_A      (0x04/4)
#define RCIN_RPI_PCM_MODE_A      (0x08/4)
#define RCIN_RPI_PCM_RXC_A       (0x0c/4)
#define RCIN_RPI_PCM_TXC_A       (0x10/4)
#define RCIN_RPI_PCM_DREQ_A      (0x14/4)
#define RCIN_RPI_PCM_INTEN_A     (0x18/4)
#define RCIN_RPI_PCM_INT_STC_A   (0x1c/4)
#define RCIN_RPI_PCM_GRAY        (0x20/4)

#define RCIN_RPI_PCMCLK_CNTL     38
#define RCIN_RPI_PCMCLK_DIV      39

namespace DriverFramework
{

struct rc_channel_data {
    uint64_t timestamp; // required for logger
    uint64_t timestamp_publication;
    uint64_t timestamp_last_signal;
    int32_t  channel_count;
    int32_t  rssi;
    uint16_t rc_lost_frame_count;
    uint16_t rc_total_frame_count;
    uint16_t rc_ppm_frame_length;
    uint16_t values[18];
    bool rc_failsafe;
    bool rc_lost;
    uint8_t input_source;
};

enum state_t{
    RCIN_RPI_INITIAL_STATE = -1,
    RCIN_RPI_ZERO_STATE = 0,
    RCIN_RPI_ONE_STATE = 1
};

//Memory table structure
typedef struct {
    void **virt_pages;
    void **phys_pages;
    uint32_t page_count;
} memory_table_t;

//DMA control block structure
typedef struct {
  uint32_t info, src, dst, length, stride, next, pad[2];
} dma_cb_t;

class Memory_table {
// Allow RCInput_RPI access to private members of Memory_table
friend class ErleBrain2RcInput;

private:
    void** _virt_pages;
    void** _phys_pages;
    uint32_t _page_count;

public:
    Memory_table();
    Memory_table(uint32_t, int);
    ~Memory_table();

    //Get virtual address from the corresponding physical address from memory_table.
    void* get_virt_addr(const uint32_t phys_addr) const;

    // This function returns physical address with help of pointer, which is offset from the beginning of the buffer.
    void* get_page(void **pages, const uint32_t addr) const;

    // This function returns offset from the beginning of the buffer using (virtual) address in 'pages' and memory_table.
    uint32_t get_offset(void **pages, const uint32_t addr) const;

    //How many bytes are available for reading in circle buffer?
    uint32_t bytes_available(const uint32_t read_addr, const uint32_t write_addr) const;

    uint32_t get_page_count() const;
};


class ErleBrain2RcInput : public DevObj
{
public:
    ErleBrain2RcInput(const char *device_path) :
        DevObj("ErleBrain2RcInput", device_path, "/dev/input_rc", DeviceBusType_UNKNOWN, 1000000/RCIN_RPI_SAMPLE_FREQ),
        m_prev_tick(0),
        m_delta_time(0),
        m_curr_tick_inc(1000/RCIN_RPI_SAMPLE_FREQ),
        m_curr_pointer(0),
        m_curr_channel(0),
        m_width_s0(0),
        m_curr_signal(0),
        m_last_signal(228),
        m_state(RCIN_RPI_INITIAL_STATE),
        m_rc_channel_data{},
        m_enableGPIO(new GPIO(PPM_INPUT_RPI))
	{
        int version = 2; //Lander

        set_physical_addresses(version);

        //Init memory for buffer and for DMA control blocks. See comments in "init_ctrl_data()" to understand values "2" and "113"
        m_circle_buffer = new Memory_table(RCIN_RPI_BUFFER_LENGTH * 2, version);
        m_con_blocks = new Memory_table(RCIN_RPI_BUFFER_LENGTH * 113, version);

        m_rc_channel_data.channel_count = -1;
	}

    virtual ~ErleBrain2RcInput() {
        stop_dma();
        m_enableGPIO->disable();
        delete m_enableGPIO;
        delete m_circle_buffer;
        delete m_con_blocks;
    }

    virtual int init();

protected:
	virtual void _measure();
    virtual int _publish(struct rc_channel_data &data) { return 0; }

private:
    void init_registers();
    void* map_peripheral(uint32_t base, uint32_t len);
    void set_sigaction();

    static void termination_handler(int signum);
    static void stop_dma();

    void init_ctrl_data();
    void init_dma_cb(dma_cb_t** cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb);
    void init_PCM();
    void init_DMA();

    void set_physical_addresses(int version);
    void process_ppmsum_pulse(uint16_t width_usec);


    uint32_t m_dma_base;
    uint32_t m_clk_base;
    uint32_t m_pcm_base;

    uint64_t m_curr_tick;
    uint64_t m_prev_tick;
    uint64_t m_delta_time;

    uint32_t m_curr_tick_inc;
    uint32_t m_curr_pointer;
    uint32_t m_curr_channel;
    uint32_t m_counter;

    uint16_t m_width_s0;
    uint16_t m_width_s1;

    uint8_t m_curr_signal;
    uint8_t m_last_signal;

    state_t m_state;

    //registers
    static volatile uint32_t *m_pcm_reg;
    static volatile uint32_t *m_clk_reg;
    static volatile uint32_t *m_dma_reg;

    Memory_table *m_circle_buffer;
    Memory_table *m_con_blocks;

    struct rc_channel_data    m_rc_channel_data;
    GPIO         *m_enableGPIO;
};

} // namespace DriverFramework
