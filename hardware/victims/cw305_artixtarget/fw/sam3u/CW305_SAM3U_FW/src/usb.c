/*
   Copyright (c) 2014-2016 NewAE Technology Inc. All rights reserved.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   */

#include <asf.h>
#include "conf_usb.h"
#include "stdio_serial.h"
#include "ui.h"
#include "genclk.h"
#include "fpga_program.h"
#include "pdi/XPROGNewAE.h"
#include "pdi/XPROGTimeout.h"
#include "pdi/XPROGTarget.h"
#include "usart_driver.h"
#include "usb.h"
#include "fpga_xmem.h"
#include "cdce906.h"
#include "tps56520.h"
#include <string.h>
#include "aes.h"

#define FW_VER_MAJOR 0
#define FW_VER_MINOR 20
#define FW_VER_DEBUG 0

volatile bool g_captureinprogress = true;

static volatile bool main_b_vendor_enable = true;

COMPILER_WORD_ALIGNED
static uint8_t main_buf_loopback[MAIN_LOOPBACK_SIZE];

void main_vendor_bulk_in_received(udd_ep_status_t status,
        iram_size_t nb_transfered, udd_ep_id_t ep);
void main_vendor_bulk_out_received(udd_ep_status_t status,
        iram_size_t nb_transfered, udd_ep_id_t ep);

void main_suspend_action(void)
{
    ui_powerdown();
}

void main_resume_action(void)
{
    ui_wakeup();
}

void main_sof_action(void)
{
    if (!main_b_vendor_enable)
        return;
    ui_process(udd_get_frame_number());
}

bool main_vendor_enable(void)
{
    main_b_vendor_enable = true;
    // Start data reception on OUT endpoints
#if UDI_VENDOR_EPS_SIZE_BULK_FS
    //main_vendor_bulk_in_received(UDD_EP_TRANSFER_OK, 0, 0);
    udi_vendor_bulk_out_run(
            main_buf_loopback,
            sizeof(main_buf_loopback),
            main_vendor_bulk_out_received);
#endif
    return true;
}

void main_vendor_disable(void)
{
    main_b_vendor_enable = false;
}

/* Read/write into FPGA memory-mapped space */
#define REQ_MEMREAD_BULK 0x10
#define REQ_MEMWRITE_BULK 0x11
#define REQ_MEMREAD_CTRL 0x12
#define REQ_MEMWRITE_CTRL 0x13
#define REQ_MEMWRITE_CTRL_SAMU3 0x15

/* Get status of INITB and PROG lines */
#define REQ_FPGA_STATUS 0x15

/* Enter FPGA Programming mode */
#define REQ_FPGA_PROGRAM 0x16

/* Get SAM3U Firmware Version */
#define REQ_FW_VERSION 0x17

/* Program XMEGA (DMM volt-meter) */
#define REQ_XMEGA_PROGRAM 0x20

/* Various Settings */
#define REQ_SAM3U_CFG 0x22

/* Send data to PLL chip */
#define REQ_CDCE906 0x30

/* Set VCC-INT Voltage */
#define REQ_VCCINT 0x31

/* Size in byte for the batch run */
#define BUFLEN_BYTES 4
#define ADDR_BYTES 4
#define BATCH_CONFIG_HEADER_BYTES 8 

#define MAX_STATES_AM 16
#define MAX_PT_BYTES 256
#define MAX_K_BYTES 256
#define MAX_FPT_BYTES 32
#define MAX_FKEY_BYTES 32
#define MAX_STATE_BYTES (MAX_PT_BYTES + MAX_K_BYTES + MAX_FPT_BYTES + MAX_FKEY_BYTES)

#define MAX_REFRESH 256
#define REFRESH_BYTES 4
#define ORDER 0
#define MIN_BUFFER_BYTES (BUFLEN_BYTES + ADDR_BYTES + BATCH_CONFIG_HEADER_BYTES + MAX_REFRESH*REFRESH_BYTES + MAX_STATES_AM*MAX_STATE_BYTES)

COMPILER_WORD_ALIGNED static uint8_t ctrlbuffer[MIN_BUFFER_BYTES]; //
#define CTRLBUFFER_WORDPTR ((uint32_t *) ((void *)ctrlbuffer))

typedef enum {
    bep_emem=0,
    bep_fpgabitstream=10
} blockep_usage_t;

static blockep_usage_t blockendpoint_usage = bep_emem;

static uint8_t * ctrlmemread_buf;
static unsigned int ctrlmemread_size;

void ctrl_readmem_bulk(void);
void ctrl_readmem_ctrl(void);
void ctrl_writemem_bulk(void);
void ctrl_writemem_ctrl(void);
void ctrl_writemem_ctrl_sam3u(void);
void ctrl_progfpga_bulk(void);
bool ctrl_xmega_program(void);
void ctrl_xmega_program_void(void);

uint32_t sam3u_mem[256];

void ctrl_xmega_program_void(void)
{
    XPROGProtocol_Command();
}

void ctrl_readmem_bulk(void){
    uint32_t buflen = *(CTRLBUFFER_WORDPTR);	
    uint32_t address = *(CTRLBUFFER_WORDPTR + 1);

    FPGA_setlock(fpga_blockin);

    /* Do memory read */	
    udi_vendor_bulk_in_run(
            (uint8_t *) PSRAM_BASE_ADDRESS + address,
            buflen,
            main_vendor_bulk_in_received
            );	
}

void ctrl_readmem_ctrl(void){
    uint32_t buflen = *(CTRLBUFFER_WORDPTR);
    uint32_t address = *(CTRLBUFFER_WORDPTR + 1);

    FPGA_setlock(fpga_ctrlmem);

    /* Do memory read */
    ctrlmemread_buf = (uint8_t *) PSRAM_BASE_ADDRESS + address;

    /* Set size to read */
    ctrlmemread_size = buflen;

    /* Start Transaction */
}


/* Used to generate a random byte */
#define MAX_PRNG_IDX 16
static uint8_t aes_rnd_byte(uint8_t * buf_idx, uint8_t * prng_state, struct AES_ctx * aesctx){
    if (*buf_idx==MAX_PRNG_IDX){
        // buffer empty, need to refresh and reset idx 
        AES_ECB_encrypt(aesctx,prng_state);
        *buf_idx=0;
    }
    // Inc index
    *buf_idx += 1;
    // Return randomn byte
    return prng_state[(*buf_idx)-1];
}

/* Used for configurable batch runs*/
void ctrl_writemem_ctrl_sam3u(void){
    uint32_t buflen = *(CTRLBUFFER_WORDPTR);
    uint32_t address = *(CTRLBUFFER_WORDPTR + 1);

    uint32_t config_glb = *(CTRLBUFFER_WORDPTR + 2);  // 32'h  nrefresh(8bits) | nstate(8bits)  |        nbatch(16bits) 
    uint32_t config_size = *(CTRLBUFFER_WORDPTR + 3); // 32'h  pt_fsize(8bits) | pt_size(8bits) | key_fsize(8bits) | key_size(8bits)
    uint32_t status_loop_delay = *(CTRLBUFFER_WORDPTR + 4); // 32'h  pt_fsize(8bits) | pt_size(8bits) | key_fsize(8bits) | key_size(8bits)
    uint32_t prng_seed = *(CTRLBUFFER_WORDPTR + 5) ;

    // first order
    // o0: norefresh, 168; o1: 232, o2: 368
    #if ORDER == 0
    uint8_t dpay [168] = {
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
        };
    #elif ORDER == 1
    uint8_t dpay [232] = {
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff,
        0xff, 0xff, 0xff, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3, 0x0, 0x0, 0x10, 0x0, 0x1, 0x0, 0x11, 0x0,
        0x2, 0x0, 0x12, 0x0, 0x3, 0x0, 0x13, 0x0, 0x4, 0x0, 0x14, 0x0, 0x5, 0x0, 0x15, 0x0,
        0x6, 0x0, 0x16, 0x0, 0x7, 0x0, 0x17, 0x0, 0x8, 0x0, 0x18, 0x0, 0x9, 0x0, 0x19, 0x0,
        0xa, 0x0, 0x1a, 0x0, 0xb, 0x0, 0x1b, 0x0, 0xc, 0x0, 0x1c, 0x0, 0xd, 0x0, 0x1d, 0x0,
        0xe, 0x0, 0x1e, 0x0, 0xf, 0x0, 0x1f, 0x0
        };
    #else
    uint8_t dpay [368] = {
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3,
        0x0, 0x0, 0x20, 0x0, 0x1, 0x0, 0x21, 0x0, 0x2, 0x0, 0x22, 0x0, 0x3, 0x0, 0x23, 0x0,
        0x4, 0x0, 0x24, 0x0, 0x5, 0x0, 0x25, 0x0, 0x6, 0x0, 0x26, 0x0, 0x7, 0x0, 0x27, 0x0,
        0x8, 0x0, 0x28, 0x0, 0x9, 0x0, 0x29, 0x0, 0xa, 0x0, 0x2a, 0x0, 0xb, 0x0, 0x2b, 0x0,
        0xc, 0x0, 0x2c, 0x0, 0xd, 0x0, 0x2d, 0x0, 0xe, 0x0, 0x2e, 0x0, 0xf, 0x0, 0x2f, 0x0,
        0x10, 0x0, 0x20, 0x0, 0x11, 0x0, 0x21, 0x0, 0x12, 0x0, 0x22, 0x0, 0x13, 0x0, 0x23, 0x0,
        0x14, 0x0, 0x24, 0x0, 0x15, 0x0, 0x25, 0x0, 0x16, 0x0, 0x26, 0x0, 0x17, 0x0, 0x27, 0x0,
        0x18, 0x0, 0x28, 0x0, 0x19, 0x0, 0x29, 0x0, 0x1a, 0x0, 0x2a, 0x0, 0x1b, 0x0, 0x2b, 0x0,
        0x1c, 0x0, 0x2c, 0x0, 0x1d, 0x0, 0x2d, 0x0, 0x1e, 0x0, 0x2e, 0x0, 0x1f, 0x0, 0x2f, 0x0
        };
    #endif
    uint8_t * ctrlbuf_payload = dpay; // (uint8_t *)(CTRLBUFFER_WORDPTR + 6);

    // Fetch control
    uint16_t nbatch = config_glb & 0xFFFF;
    uint8_t nstate = (config_glb >> 16 ) & 0xFF;
    uint8_t nrefresh = (config_glb >> 24 ) & 0xFF;

    uint8_t key_size = config_size & 0xFF;
    uint8_t key_fsize = (config_size >> 8) & 0xFF;
    uint8_t pt_size = (config_size >> 16) & 0xFF;
    uint8_t pt_fsize = (config_size >> 24) & 0xFF;

    // Initialise the AES PRNG
    struct AES_ctx aesctx;
    uint8_t prng_key[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    prng_key[0] = prng_seed & 0xff;
    prng_key[1] = (prng_seed >> 8) & 0xff;
    prng_key[2] = (prng_seed >> 16) & 0xff;
    prng_key[3] = (prng_seed >> 24) & 0xff;
    AES_init_ctx(&aesctx,prng_key);
    uint8_t prng_state[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t rng_index = 0;

    // Could be removed, used to ease readibility
    uint32_t global_state_size = key_size + key_fsize + pt_size + pt_fsize;

    // Temporary RAM for refresh
    uint8_t refresh_RAM_state[key_size+pt_size]; 
    uint16_t radd0, radd1;

    // Perform routine for all cases
    uint8_t rndbyte;
    uint8_t mask;
    uint8_t state_idx;

    // Threshold for rejection sampling of state_idx
    uint16_t rej_threshold = 256 - (256 % ((uint16_t) nstate));
    // Let us use reciprocal multiplication, to ensure constant time modulo
    uint32_t reciprocal = (0x10000 + (uint32_t) nstate - 1) / ((uint32_t) nstate);

    for(uint16_t c=0;c<nbatch;c++) {
        // Reset bufid to have constant time between runs
        rng_index = MAX_PRNG_IDX;

        // Generate randomness byte to choose which 
        // state to use and update random state
        do {
            state_idx = aes_rnd_byte(&rng_index, prng_state, &aesctx);
        } while (state_idx >= rej_threshold);
        // q = state_idx / nstate
        uint8_t q = (((uint32_t) state_idx) * reciprocal) >> 16;
        // state_idx = state_idx % nstate
        state_idx = state_idx - nstate*q;

        // Lock FPGA
        FPGA_setlock(fpga_generic);

        // Generate key configuration.
        for (uint8_t fb=0;fb<key_fsize;fb++){
            // Load byte where flag are encoded
            uint8_t fbyte = ctrlbuf_payload[(state_idx*global_state_size)+key_size+pt_size+fb];
            // FIXME: optimsize remain shit
            uint16_t remain = key_size - 8*fb;
            uint8_t max = 0;
            if (remain >= 8) {
                max = 8;
            } else {
                max = remain;
            }
            // Read the flags
            for (uint8_t i=0;i<max;i++) {
                // Check if randomness should be generated
                rndbyte = aes_rnd_byte(&rng_index, prng_state, &aesctx);
                mask = 0 - ((fbyte>>i) & 0x1);
                refresh_RAM_state[8*fb+i] = ctrlbuf_payload[(state_idx*global_state_size)+8*fb+i] ^ (mask & rndbyte);
            }
        }

        // Generate pt configuration.
        for (uint8_t fb=0;fb<pt_fsize;fb++){
            // Load byte where flag are encoded
            uint8_t fbyte = ctrlbuf_payload[(state_idx*global_state_size)+key_size+pt_size+key_fsize+fb];
            // FIXME: optimsize remain shit
            uint16_t remain = pt_size - 8*fb;
            uint8_t max = 0;
            if (remain >= 8) {
                max = 8;
            } else {
                max = remain;
            }
            // Read the flags
            for (uint8_t i=0;i<max;i++) {
                rndbyte = aes_rnd_byte(&rng_index, prng_state, &aesctx);
                mask = 0 - ((fbyte>>i) & 0x1);
                refresh_RAM_state[key_size+8*fb+i] = ctrlbuf_payload[(state_idx*global_state_size)+key_size+8*fb+i] ^ (mask & rndbyte);
            }
        }

        // Refresh mechanism
        for (uint8_t i=0;i<nrefresh;i++) { 
            // Fetch refreshes addresses
            radd0 = ctrlbuf_payload[(nstate*global_state_size)+4*i] | (ctrlbuf_payload[(nstate*global_state_size)+4*i+1] << 8); 
            radd1 = ctrlbuf_payload[(nstate*global_state_size)+4*i+2] | (ctrlbuf_payload[(nstate*global_state_size)+4*i+3] << 8); 
            // Generate random byte and update random state
            rndbyte = aes_rnd_byte(&rng_index, prng_state, &aesctx);
            // Refresh 
            refresh_RAM_state[radd0] ^= rndbyte;
            refresh_RAM_state[radd1] ^= rndbyte;
        }

        // Write run data to FPGA
        for (uint8_t i=0;i<key_size;i++) {
            xram[0x400+0x100+i]=refresh_RAM_state[i];
        }
        for (uint8_t i=0;i<pt_size;i++) {
            xram[0x400+0x200+i]=refresh_RAM_state[key_size+i];
        }

        // Start one FPGA run
        FPGA_setlock(fpga_unlocked); 


        gpio_set_pin_high(FPGA_TRIGGER_GPIO);
        delay_cycles(50);
        gpio_set_pin_low(FPGA_TRIGGER_GPIO);
        uint8_t done_status = 0;
        while(done_status==0) {
            done_status = gpio_pin_is_high(PIN_EBI_DATA_BUS_D0);
        }
    }

}

void ctrl_writemem_ctrl(void){
    uint32_t buflen = *(CTRLBUFFER_WORDPTR);
    uint32_t address = *(CTRLBUFFER_WORDPTR + 1);

    uint8_t * ctrlbuf_payload = (uint8_t *)(CTRLBUFFER_WORDPTR + 2);

    //printf("Writing to %x, %d\n", address, buflen);

    FPGA_setlock(fpga_generic);

    /* Start Transaction */

    /* Do memory write */
    for(unsigned int i = 0; i < buflen; i++){
        xram[i+address] = ctrlbuf_payload[i];
    }

    FPGA_setlock(fpga_unlocked);
}

void ctrl_writemem_bulk(void){
    //uint32_t buflen = *(CTRLBUFFER_WORDPTR);
    //uint32_t address = *(CTRLBUFFER_WORDPTR + 1);

    FPGA_setlock(fpga_blockout);

    /* Set address */
    //Not required - this is done automatically via the XMEM interface
    //instead of using a "cheater" port.

    /* Transaction done in generic callback */
}

static void ctrl_sam3ucfg_cb(void)
{
    switch(udd_g_ctrlreq.req.wValue & 0xFF)
    {
        /* Turn on slow clock */
        case 0x01:
            osc_enable(OSC_MAINCK_XTAL);
            osc_wait_ready(OSC_MAINCK_XTAL);
            pmc_switch_mck_to_mainck(CONFIG_SYSCLK_PRES);
            break;

            /* Turn off slow clock */
        case 0x02:
            pmc_switch_mck_to_pllack(CONFIG_SYSCLK_PRES);
            break;

            /* Jump to ROM-resident bootloader */
        case 0x03:
            /* Turn off connected stuff */
            board_power(0);

            /* Clear ROM-mapping bit. */
            efc_perform_command(EFC0, EFC_FCMD_CGPB, 1);

            /* Disconnect USB (will kill connection) */
            udc_detach();

            /* With knowledge that I will rise again, I lay down my life. */
            while (RSTC->RSTC_SR & RSTC_SR_SRCMP);
            RSTC->RSTC_CR |= RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
            while(1);
            break;

            /* Turn off FPGA Clock */
        case 0x04:
            gpio_configure_pin(PIN_PCK0, PIO_OUTPUT_0);
            break;

            /* Turn on FPGA Clock */
        case 0x05:
            gpio_configure_pin(PIN_PCK0, PIN_PCK0_FLAGS);
            break;

            /* Toggle trigger pin */
        case 0x06:
            gpio_set_pin_high(FPGA_TRIGGER_GPIO);
            delay_cycles(250);
            gpio_set_pin_low(FPGA_TRIGGER_GPIO);
            break;

            /* Oh well, sucks to be you */
        default:
            break;
    }
}

static uint8_t cdce906_status;
static uint8_t cdce906_data;

#define USB_STATUS_NONE       0
#define USB_STATUS_PARAMWRONG 1
#define USB_STATUS_OK         2
#define USB_STATUS_COMMERR    3
#define USB_STATUS_CSFAIL     4

static void ctrl_cdce906_cb(void)
{
    //Catch heartbleed-style error
    if (udd_g_ctrlreq.req.wLength > udd_g_ctrlreq.payload_size){
        return;
    }

    cdce906_status = USB_STATUS_NONE;

    if (udd_g_ctrlreq.req.wLength < 3){
        cdce906_status = USB_STATUS_PARAMWRONG;
        return;
    }

    cdce906_status = USB_STATUS_COMMERR;
    if (udd_g_ctrlreq.payload[0] == 0x00){
        if (cdce906_read(udd_g_ctrlreq.payload[1], &cdce906_data)){
            cdce906_status = USB_STATUS_OK;
        }

    } else if (udd_g_ctrlreq.payload[0] == 0x01){
        if (cdce906_write(udd_g_ctrlreq.payload[1], udd_g_ctrlreq.payload[2])){
            cdce906_status = USB_STATUS_OK;
        }
    } else {
        cdce906_status = USB_STATUS_PARAMWRONG;
        return;
    }
}

static uint8_t vccint_status = 0;
static uint16_t vccint_setting = 1000;

static void ctrl_vccint_cb(void)
{
    //Catch heartbleed-style error
    if (udd_g_ctrlreq.req.wLength > udd_g_ctrlreq.payload_size){
        return;
    }

    vccint_status = USB_STATUS_NONE;

    if ((udd_g_ctrlreq.payload[0] ^ udd_g_ctrlreq.payload[1] ^ 0xAE) != (udd_g_ctrlreq.payload[2])){
        vccint_status = USB_STATUS_PARAMWRONG;
        return;
    }

    if (udd_g_ctrlreq.req.wLength < 3){
        vccint_status = USB_STATUS_CSFAIL;
        return;
    }

    uint16_t vcctemp = (udd_g_ctrlreq.payload[0]) | (udd_g_ctrlreq.payload[1] << 8);
    if ((vcctemp < 600) || (vcctemp > 1200)){
        vccint_status = USB_STATUS_PARAMWRONG;
        return;
    }

    vccint_status = USB_STATUS_COMMERR;

    if (tps56520_set(vcctemp)){
        vccint_setting = vcctemp;
        vccint_status = USB_STATUS_OK;
    }

    return;
}


bool main_setup_out_received(void)
{
    //Add buffer if used
    udd_g_ctrlreq.payload = ctrlbuffer;
    udd_g_ctrlreq.payload_size = min(udd_g_ctrlreq.req.wLength,	sizeof(ctrlbuffer));

    blockendpoint_usage = bep_emem;

    switch(udd_g_ctrlreq.req.bRequest){
        /* Memory Read */
        case REQ_MEMREAD_BULK:
            udd_g_ctrlreq.callback = ctrl_readmem_bulk;
            return true;
        case REQ_MEMREAD_CTRL:
            udd_g_ctrlreq.callback = ctrl_readmem_ctrl;
            return true;	

            /* Memory Write */
        case REQ_MEMWRITE_BULK:
            udd_g_ctrlreq.callback = ctrl_writemem_bulk;
            return true;

        case REQ_MEMWRITE_CTRL:
            udd_g_ctrlreq.callback = ctrl_writemem_ctrl;
            return true;		

        case REQ_MEMWRITE_CTRL_SAMU3:
            udd_g_ctrlreq.callback = ctrl_writemem_ctrl_sam3u;
            return true;		

            /* Send bitstream to FPGA */
        case REQ_FPGA_PROGRAM:
            udd_g_ctrlreq.callback = ctrl_progfpga_bulk;
            return true;

            /* XMEGA Programming */
        case REQ_XMEGA_PROGRAM:
            /*
               udd_g_ctrlreq.payload = xmegabuffer;
               udd_g_ctrlreq.payload_size = min(udd_g_ctrlreq.req.wLength,	sizeof(xmegabuffer));
               */
            udd_g_ctrlreq.callback = ctrl_xmega_program_void;
            return true;

            /* Misc hardware setup */
        case REQ_SAM3U_CFG:
            udd_g_ctrlreq.callback = ctrl_sam3ucfg_cb;
            return true;

            /* CDCE906 read/write */
        case REQ_CDCE906:
            udd_g_ctrlreq.callback = ctrl_cdce906_cb;
            return true;

            /* VCC-INT Setting */
        case REQ_VCCINT:
            udd_g_ctrlreq.callback = ctrl_vccint_cb;
            return true;

        default:
            return false;
    }					
}


void ctrl_progfpga_bulk(void){

    switch(udd_g_ctrlreq.req.wValue){
        case 0xA0:
            fpga_program_setup1();			
            break;

        case 0xA1:
            /* Waiting on data... */
            fpga_program_setup2();
            blockendpoint_usage = bep_fpgabitstream;
            break;

        case 0xA2:
            /* Done */
            blockendpoint_usage = bep_emem;
            break;

        default:
            break;
    }
}


bool main_setup_in_received(void)
{
    /*
       udd_g_ctrlreq.payload = main_buf_loopback;
       udd_g_ctrlreq.payload_size =
       min( udd_g_ctrlreq.req.wLength,
       sizeof(main_buf_loopback) );
       */

    static uint8_t  respbuf[64];
    //unsigned int cnt;

    switch(udd_g_ctrlreq.req.bRequest){
        case REQ_MEMREAD_CTRL:
            udd_g_ctrlreq.payload = ctrlmemread_buf;
            udd_g_ctrlreq.payload_size = ctrlmemread_size;
            ctrlmemread_size = 0;

            if (FPGA_lockstatus() == fpga_ctrlmem){
                FPGA_setlock(fpga_unlocked);
            }

            return true;
            break;

        case REQ_FPGA_STATUS:
            respbuf[0] = FPGA_ISDONE();
            respbuf[1] = FPGA_INITB_STATUS();
            respbuf[2] = 0;
            respbuf[3] = 0;
            udd_g_ctrlreq.payload = respbuf;
            udd_g_ctrlreq.payload_size = 4;
            return true;
            break;

        case REQ_XMEGA_PROGRAM:
            return XPROGProtocol_Command();
            break;

        case REQ_FW_VERSION:
            respbuf[0] = FW_VER_MAJOR;
            respbuf[1] = FW_VER_MINOR;
            respbuf[2] = FW_VER_DEBUG;
            udd_g_ctrlreq.payload = respbuf;
            udd_g_ctrlreq.payload_size = 3;
            return true;
            break;

        case REQ_CDCE906:
            respbuf[0] = cdce906_status;
            respbuf[1] = cdce906_data;
            udd_g_ctrlreq.payload = respbuf;
            udd_g_ctrlreq.payload_size = 2;
            return true;
            break;

        case REQ_VCCINT:
            respbuf[0] = vccint_status;
            respbuf[1] = (uint8_t)vccint_setting;
            respbuf[2] = (uint8_t)(vccint_setting >> 8);
            udd_g_ctrlreq.payload = respbuf;
            udd_g_ctrlreq.payload_size = 3;
            return true;
            break;	

        default:
            return false;
    }
    return false;
}

void main_vendor_bulk_in_received(udd_ep_status_t status,
        iram_size_t nb_transfered, udd_ep_id_t ep)
{
    UNUSED(nb_transfered);
    UNUSED(ep);
    if (UDD_EP_TRANSFER_OK != status) {
        return; // Transfer aborted/error
    }	

    if (FPGA_lockstatus() == fpga_blockin){		
        FPGA_setlock(fpga_unlocked);
    }
}

void main_vendor_bulk_out_received(udd_ep_status_t status,
        iram_size_t nb_transfered, udd_ep_id_t ep)
{
    UNUSED(ep);
    if (UDD_EP_TRANSFER_OK != status) {
        // Transfer aborted

        //restart
        udi_vendor_bulk_out_run(
                main_buf_loopback,
                sizeof(main_buf_loopback),
                main_vendor_bulk_out_received);

        return;
    }

    if (blockendpoint_usage == bep_emem){
        for(unsigned int i = 0; i < nb_transfered; i++){
            xram[i] = main_buf_loopback[i];
        }

        if (FPGA_lockstatus() == fpga_blockout){
            FPGA_setlock(fpga_unlocked);
        }
    } else if (blockendpoint_usage == bep_fpgabitstream){

        /* Send byte to FPGA - this could eventually be done via SPI */		
        for(unsigned int i = 0; i < nb_transfered; i++){
            fpga_program_sendbyte(main_buf_loopback[i]);
        }

        FPGA_CCLK_LOW();
    }

    //printf("BULKOUT: %d bytes\n", (int)nb_transfered);

    udi_vendor_bulk_out_run(
            main_buf_loopback,
            sizeof(main_buf_loopback),
            main_vendor_bulk_out_received);
}
