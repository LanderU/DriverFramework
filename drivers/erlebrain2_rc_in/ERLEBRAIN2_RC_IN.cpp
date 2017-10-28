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

#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>
#include "DriverFramework.hpp"
#include "ERLEBRAIN2_RC_IN.hpp"

using namespace DriverFramework;

volatile uint32_t *ErleBrain2RcInput::m_pcm_reg;
volatile uint32_t *ErleBrain2RcInput::m_clk_reg;
volatile uint32_t *ErleBrain2RcInput::m_dma_reg;

Memory_table::Memory_table()
{
    _page_count = 0;
}

//Init Memory table
Memory_table::Memory_table(uint32_t page_count, int version)
{
    uint32_t i;
    int fdMem, file;
    //Cache coherent adresses depends on RPI's version
    uint32_t bus = version == 1 ? 0x40000000 : 0xC0000000;
    uint64_t pageInfo;
    void* offset;

    _virt_pages = (void**)malloc(page_count * sizeof(void*));
    _phys_pages = (void**)malloc(page_count * sizeof(void*));
    _page_count = page_count;

    if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        DF_LOG_ERR("Failed to open /dev/mem");
        exit(-1);
    }

    if ((file = open("/proc/self/pagemap", O_RDWR | O_SYNC)) < 0) {
        DF_LOG_ERR("Failed to open /proc/self/pagemap");
        exit(-1);
    }

    //Magic to determine the physical address for this page:
    offset = mmap(0, _page_count*PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
    lseek(file, ((uintptr_t)offset)/PAGE_SIZE*8, SEEK_SET);

    //Get list of available cache coherent physical addresses
    for (i = 0; i < _page_count; i++) {
        _virt_pages[i]  =  mmap(0, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
        if (MAP_FAILED == _virt_pages[i]) {
            DF_LOG_ERR("error: mmap failed!");
            exit(-1);
        }
        ssize_t r = ::read(file, &pageInfo, 8);
	if(r==0) continue;
        _phys_pages[i] = (void*)((uintptr_t)(pageInfo*PAGE_SIZE) | bus);
    }

    //Map physical addresses to virtual memory
    for (i = 0; i < _page_count; i++) {
        munmap(_virt_pages[i], PAGE_SIZE);
        _virt_pages[i]  = mmap(_virt_pages[i], PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED, fdMem, ((uintptr_t)_phys_pages[i] & (version == 1 ? 0xFFFFFFFF : ~bus)));
        if (MAP_FAILED == _virt_pages[i]) {
            DF_LOG_ERR("error: mmap failed!");
            exit(-1);
        }
        memset(_virt_pages[i], 0xee, PAGE_SIZE);
    }
    close(file);
    close(fdMem);
}

Memory_table::~Memory_table()
{
    free(_virt_pages);
    free(_phys_pages);
}

// This function returns physical address with help of pointer, which is offset from the beginning of the buffer.
void* Memory_table::get_page(void** const pages, uint32_t addr) const
{
    if (addr >= PAGE_SIZE * _page_count) {
        DF_LOG_ERR("error: out of memory. %" PRIu32, addr);
        return NULL;
    }
    return (uint8_t*)pages[(uint32_t) addr / PAGE_SIZE] + addr % PAGE_SIZE;
}

//Get virtual address from the corresponding physical address from memory_table.
void* Memory_table::get_virt_addr(const uint32_t phys_addr) const
{
    // FIXME: Can't the address be calculated directly?
    // FIXME: if the address room  in _phys_pages is not fragmented one may avoid a complete loop ..
    uint32_t i = 0;
    for (; i < _page_count; i++) {
        if ((uintptr_t) _phys_pages[i] == (((uintptr_t) phys_addr) & 0xFFFFF000)) {
            return (void*) ((uintptr_t) _virt_pages[i] + (phys_addr & 0xFFF));
        }
    }
    return NULL;
}

// FIXME: in-congruent function style see above
// This function returns offset from the beginning of the buffer using virtual address and memory_table.
uint32_t Memory_table::get_offset(void ** const pages, const uint32_t addr) const
{
    uint32_t i = 0;
    for (; i < _page_count; i++) {
        if ((uintptr_t) pages[i] == (addr & 0xFFFFF000) ) {
            return (i*PAGE_SIZE + (addr & 0xFFF));
        }
    }
    return -1;
}

//How many bytes are available for reading in circle buffer?
uint32_t Memory_table::bytes_available(const uint32_t read_addr, const uint32_t write_addr) const
{
    if (write_addr > read_addr) {
        return (write_addr - read_addr);
    }
    else {
        return _page_count * PAGE_SIZE - (read_addr - write_addr);
    }
}

uint32_t Memory_table::get_page_count() const
{
    return _page_count;
}

int ErleBrain2RcInput::init() {
    DevObj::init();
    DF_LOG_INFO("ErleBrain2RcInput::init");
    init_registers();
    //Enable PPM input
    m_enableGPIO->disable();
    m_enableGPIO->enable();
    m_enableGPIO->setDirection(GPIO::INPUT);

    //Configuration
    set_sigaction();
    init_ctrl_data();
    init_PCM();
    init_DMA();

    //wait a bit to let DMA fill queues and come to stable sampling
    usleep(300000);

    //Reading first sample
    m_curr_tick = *((uint64_t*) m_circle_buffer->get_page(m_circle_buffer->_virt_pages, m_curr_pointer));
    m_prev_tick = m_curr_tick;
    m_curr_pointer += 8;
    m_curr_signal = *((uint8_t*) m_circle_buffer->get_page(m_circle_buffer->_virt_pages, m_curr_pointer)) & 0x10 ? 1 : 0;
    m_last_signal = m_curr_signal;
    m_curr_pointer++;

    return 0;
}

//Processing signal
void ErleBrain2RcInput::_measure()
{
    int j;
    void* x;

    //Now we are getting address in which DMAC is writing at current moment
    dma_cb_t* ad = (dma_cb_t*) m_con_blocks->get_virt_addr(m_dma_reg[RCIN_RPI_DMA_CONBLK_AD | RCIN_RPI_DMA_CHANNEL << 8]);
    for(j = 1; j >= -1; j--){
        x = m_circle_buffer->get_virt_addr((ad + j)->dst);
        if(x != NULL) {
            break;
        }
    }

    //How many bytes have DMA transfered (and we can process)?
    m_counter = m_circle_buffer->bytes_available(m_curr_pointer, m_circle_buffer->get_offset(m_circle_buffer->_virt_pages, (uintptr_t)x));
    //We can't stay in method for a long time, because it may lead to delays
    if (m_counter > RCIN_RPI_MAX_COUNTER) {
        m_counter = RCIN_RPI_MAX_COUNTER;
    }

    //Processing ready bytes
    for (;m_counter > 0x40;m_counter--) {
        //Is it timer sample?
        if (m_curr_pointer %  (64) == 0) {
            m_curr_tick = *((uint64_t*) m_circle_buffer->get_page(m_circle_buffer->_virt_pages, m_curr_pointer));
            m_curr_pointer+=8;
            m_counter-=8;
        }
        //Reading required bit
        //DF_LOG_INFO("m_curr_signal1 %d, %d, %d, %d", m_curr_pointer, m_circle_buffer->get_page_count(), (uint32_t) m_curr_pointer / 4096, m_curr_pointer % 4096);
        //DF_LOG_INFO("m_circle_buffer->get_page = %p", m_circle_buffer->get_page(m_circle_buffer->_virt_pages, m_curr_pointer));
        m_curr_signal = *((uint8_t*) m_circle_buffer->get_page(m_circle_buffer->_virt_pages, m_curr_pointer)) & 0x10 ? 1 : 0;
        //DF_LOG_INFO("m_curr_signal2 %d", m_curr_pointer);
        //If the signal changed
        if (m_curr_signal != m_last_signal) {
            m_delta_time = m_curr_tick - m_prev_tick;
            m_prev_tick = m_curr_tick;
            switch (m_state) {
            case RCIN_RPI_INITIAL_STATE:
                m_state = RCIN_RPI_ZERO_STATE;
                break;
            case RCIN_RPI_ZERO_STATE:
                if (m_curr_signal == 0) {
                    m_width_s0 = (uint16_t) m_delta_time;
                    m_state = RCIN_RPI_ONE_STATE;
                }
                break;
            case RCIN_RPI_ONE_STATE:
                if (m_curr_signal == 1) {
                    m_width_s1 = (uint16_t) m_delta_time;
                    m_state = RCIN_RPI_ZERO_STATE;
                    process_ppmsum_pulse(m_width_s0 + m_width_s1);
                }
                break;
            }
        }
        m_last_signal = m_curr_signal;
        m_curr_pointer++;
        if (m_curr_pointer >= m_circle_buffer->get_page_count()*PAGE_SIZE) {
            m_curr_pointer = 0;
        }
        m_curr_tick+=m_curr_tick_inc;
    }
}

void ErleBrain2RcInput::init_registers()
{
    m_dma_reg = (uint32_t*)map_peripheral(m_dma_base, RCIN_RPI_DMA_LEN);
    m_pcm_reg = (uint32_t*)map_peripheral(m_pcm_base, RCIN_RPI_PCM_LEN);
    m_clk_reg = (uint32_t*)map_peripheral(m_clk_base, RCIN_RPI_CLK_LEN);
}

//Map peripheral to virtual memory
void* ErleBrain2RcInput::map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR);
    void * vaddr;

    if (fd < 0) {
        DF_LOG_ERR("Failed to open /dev/mem");
        return NULL;
    }
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED) {
        DF_LOG_ERR("rpio-pwm: Failed to map peripheral at 0x%08x", base);
    }

    close(fd);
    return vaddr;
}

//We must stop DMA when the process is killed
void ErleBrain2RcInput::set_sigaction()
{
    for (int i = 0; i < 64; i++) {
        //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled

        //if (i == 11) {  //Segmentation fault
        //    continue;
        //}

        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = ErleBrain2RcInput::termination_handler;
        sigaction(i, &sa, NULL);
    }
}

/* We need to be sure that the DMA is stopped upon termination */
void ErleBrain2RcInput::termination_handler(int signum)
{
    if (signum != 18 && signum != 28) {
        stop_dma();
        DF_LOG_ERR("error: DMA Interrupted %s(%d)", strsignal(signum), signum);
        exit(-1);
    }
}

void ErleBrain2RcInput::stop_dma()
{
    m_dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = 0;
}

//This function is used to init DMA control blocks (setting sampling GPIO register, destination adresses, synchronization)
void ErleBrain2RcInput::init_ctrl_data()
{
    uint32_t phys_fifo_addr;
    uint32_t dest = 0;
    uint32_t cbp = 0;
    dma_cb_t* cbp_curr;
    //Set fifo addr (for delay)
    phys_fifo_addr = ((m_pcm_base + 0x04) & 0x00FFFFFF) | 0x7e000000;

    //Init dma control blocks.
    /*We are transferring 1 byte of GPIO register. Every 56th iteration we are
      sampling TIMER register, which length is 8 bytes. So, for every 56 samples of GPIO we need
      56 * 1 + 8 = 64 bytes of buffer. Value 56 was selected specially to have a 64-byte "block"
      TIMER - GPIO. So, we have integer count of such "blocks" at one virtual page. (4096 / 64 = 64
      "blocks" per page. As minimum, we must have 2 virtual pages of buffer (to have integer count of
      vitual pages for control blocks): for every 56 iterations (64 bytes of buffer) we need 56 control blocks for GPIO
      sampling, 56 control blocks for setting frequency and 1 control block for sampling timer, so,
      we need 56 + 56 + 1 = 113 control blocks. For integer value, we need 113 pages of control blocks.
      Each control block length is 32 bytes. In 113 pages we will have (113 * 4096 / 32) = 113 * 128 control
      blocks. 113 * 128 control blocks = 64 * 128 bytes of buffer = 2 pages of buffer.
      So, for 56 * 64 * 2 iteration we init DMA for sampling GPIO
      and timer to (64 * 64 * 2) = 8192 bytes = 2 pages of buffer.
    */
    //    fprintf(stderr, "ERROR SEARCH1\n");

    uint32_t i = 0;
    for (i = 0; i < 56 * 128 * RCIN_RPI_BUFFER_LENGTH; i++) // 8 * 56 * 128 == 57344
    {
        //Transfer timer every 56th sample
        if(i % 56 == 0) {
            cbp_curr = (dma_cb_t*)m_con_blocks->get_page(m_con_blocks->_virt_pages, cbp);

            init_dma_cb(&cbp_curr, RCIN_RPI_DMA_NO_WIDE_BURSTS | RCIN_RPI_DMA_WAIT_RESP | RCIN_RPI_DMA_DEST_INC | RCIN_RPI_DMA_SRC_INC,
              RCIN_RPI_TIMER_BASE,
              (uintptr_t) m_circle_buffer->get_page(m_circle_buffer->_phys_pages, dest),
              8,
              0,
              (uintptr_t) m_con_blocks->get_page(m_con_blocks->_phys_pages, cbp + sizeof(dma_cb_t) ) );

            dest += 8;
            cbp += sizeof(dma_cb_t);
        }

        // Transfer GPIO (1 byte)
        cbp_curr = (dma_cb_t*)m_con_blocks->get_page(m_con_blocks->_virt_pages, cbp);
        init_dma_cb(&cbp_curr, RCIN_RPI_DMA_NO_WIDE_BURSTS | RCIN_RPI_DMA_WAIT_RESP,
            RCIN_RPI_GPIO_LEV0_ADDR,
            (uintptr_t) m_circle_buffer->get_page(m_circle_buffer->_phys_pages, dest),
            1,
            0,
            (uintptr_t) m_con_blocks->get_page(m_con_blocks->_phys_pages, cbp + sizeof(dma_cb_t) ) );

        dest += 1;
        cbp += sizeof(dma_cb_t);

        // Delay (for setting sampling frequency)
        /* DMA is waiting data request signal (DREQ) from PCM. PCM is set for 1 MhZ freqency, so,
           each sample of GPIO is limited by writing to PCA queue.
        */
        cbp_curr = (dma_cb_t*)m_con_blocks->get_page(m_con_blocks->_virt_pages, cbp);
        init_dma_cb(&cbp_curr, RCIN_RPI_DMA_NO_WIDE_BURSTS | RCIN_RPI_DMA_WAIT_RESP | RCIN_RPI_DMA_D_DREQ | RCIN_RPI_DMA_PER_MAP(2),
            RCIN_RPI_TIMER_BASE,
            phys_fifo_addr,
            4,
            0,
            (uintptr_t)m_con_blocks->get_page(m_con_blocks->_phys_pages, cbp + sizeof(dma_cb_t) ) );

        cbp += sizeof(dma_cb_t);
    }
    //Make last control block point to the first (to make circle)
    cbp -= sizeof(dma_cb_t);
    ((dma_cb_t*)m_con_blocks->get_page(m_con_blocks->_virt_pages, cbp))->next = (uintptr_t) m_con_blocks->get_page(m_con_blocks->_phys_pages, 0);
}

//Method to init DMA control block
void ErleBrain2RcInput::init_dma_cb(dma_cb_t** cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb)
{
    (*cbp)->info = mode;
    (*cbp)->src = source;
    (*cbp)->dst = dest;
    (*cbp)->length = length;
    (*cbp)->next = next_cb;
    (*cbp)->stride = stride;
}

/*Initialise PCM
  See BCM2835 documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
*/
void ErleBrain2RcInput::init_PCM()
{
    m_pcm_reg[RCIN_RPI_PCM_CS_A] = 1;                                          // Disable Rx+Tx, Enable PCM block
    usleep(100);
    m_clk_reg[RCIN_RPI_PCMCLK_CNTL] = 0x5A000006;                              // Source=PLLD (500MHz)
    usleep(100);
    m_clk_reg[RCIN_RPI_PCMCLK_DIV] = 0x5A000000 | ((50000/RCIN_RPI_SAMPLE_FREQ)<<12);   // Set pcm div. If we need to configure DMA frequency.
    usleep(100);
    m_clk_reg[RCIN_RPI_PCMCLK_CNTL] = 0x5A000016;                              // Source=PLLD and enable
    usleep(100);
    m_pcm_reg[RCIN_RPI_PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16;             // 1 channel, 8 bits
    usleep(100);
    m_pcm_reg[RCIN_RPI_PCM_MODE_A] = (10 - 1) << 10;                           //PCM mode
    usleep(100);
    m_pcm_reg[RCIN_RPI_PCM_CS_A] |= 1<<4 | 1<<3;                               // Clear FIFOs
    usleep(100);
    m_pcm_reg[RCIN_RPI_PCM_DREQ_A] = 64<<24 | 64<<8;                           // DMA Req when one slot is free?
    usleep(100);
    m_pcm_reg[RCIN_RPI_PCM_CS_A] |= 1<<9;                                      // Enable DMA
    usleep(100);
    m_pcm_reg[RCIN_RPI_PCM_CS_A] |= 1<<2;                                      // Enable Tx
    usleep(100);
}

/*Initialise DMA
  See BCM2835 documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
*/
void ErleBrain2RcInput::init_DMA()
{
    m_dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = RCIN_RPI_DMA_RESET;                 //Reset DMA
    usleep(100);
    m_dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = RCIN_RPI_DMA_INT | RCIN_RPI_DMA_END;
    m_dma_reg[RCIN_RPI_DMA_CONBLK_AD | RCIN_RPI_DMA_CHANNEL << 8] = reinterpret_cast<uintptr_t>(m_con_blocks->get_page(m_con_blocks->_phys_pages, 0));//Set first control block address
    m_dma_reg[RCIN_RPI_DMA_DEBUG | RCIN_RPI_DMA_CHANNEL << 8] = 7;                      // clear debug error flags
    m_dma_reg[RCIN_RPI_DMA_CS | RCIN_RPI_DMA_CHANNEL << 8] = 0x10880001;                // go, mid priority, wait for outstanding writes
}

//Physical addresses of peripheral depends on Raspberry Pi's version
void ErleBrain2RcInput::set_physical_addresses(int version)
{
    if (version == 1) {
        m_dma_base = RCIN_RPI_RPI1_DMA_BASE;
        m_clk_base = RCIN_RPI_RPI1_CLK_BASE;
        m_pcm_base = RCIN_RPI_RPI1_PCM_BASE;
    }
    else if (version == 2) {
        m_dma_base = RCIN_RPI_RPI2_DMA_BASE;
        m_clk_base = RCIN_RPI_RPI2_CLK_BASE;
        m_pcm_base = RCIN_RPI_RPI2_PCM_BASE;
    }
}

void ErleBrain2RcInput::process_ppmsum_pulse(uint16_t width_usec)
{
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (m_rc_channel_data.channel_count >= 5) {
            uint64_t ts = offsetTime();

            m_rc_channel_data.timestamp_publication = ts;
            m_rc_channel_data.timestamp_last_signal = ts;
            //m_rc_channel_data.channel_count = _channels;
            m_rc_channel_data.rssi = 100;
            m_rc_channel_data.rc_lost_frame_count = 0;
            m_rc_channel_data.rc_total_frame_count = 1;
            m_rc_channel_data.rc_ppm_frame_length = 100;
            m_rc_channel_data.rc_failsafe = false;
            m_rc_channel_data.rc_lost = false;
            m_rc_channel_data.input_source = 2; //RC_INPUT_SOURCE_PX4IO_PPM;

            _publish(m_rc_channel_data);

            //DF_LOG_INFO("got channel data");
        }
        m_rc_channel_data.channel_count = 0;
        return;
    }

    if (m_rc_channel_data.channel_count == -1) {
        // we are not synchronised
        return;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        m_rc_channel_data.values[m_rc_channel_data.channel_count] = width_usec;

        // move to next channel
        m_rc_channel_data.channel_count++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, s %s", strsignal(signum));o we wait for a wide pulse
    if (m_rc_channel_data.channel_count >= LINUX_RC_INPUT_NUM_CHANNELS) {
        m_rc_channel_data.channel_count = -1;
    }
}
