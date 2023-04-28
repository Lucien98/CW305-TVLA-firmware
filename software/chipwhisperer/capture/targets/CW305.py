#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2015-2019, NewAE Technology Inc
# All rights reserved.
#
# Find this and more at newae.com - this file is part of the chipwhisperer
# project, http://www.chipwhisperer.com . ChipWhisperer is a registered
# trademark of NewAE Technology Inc in the US & Europe.
#
#    This file is part of chipwhisperer.
#
#    chipwhisperer is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    chipwhisperer is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with chipwhisperer.  If not, see <http://www.gnu.org/licenses/>.
#=================================================
import logging
import time
import random
from datetime import datetime
import os.path
from ._base import TargetTemplate
from chipwhisperer.hardware.naeusb.naeusb import NAEUSB,packuint32
from chipwhisperer.hardware.naeusb.pll_cdce906 import PLLCDCE906
from chipwhisperer.hardware.naeusb.fpga import FPGA
from chipwhisperer.common.utils import util
from chipwhisperer.common.utils.util import camel_case_deprecated

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

import numpy

class CW305_USB(object):
    REQ_SYSCFG = 0x22
    REQ_VCCINT = 0x31
    SYSCFG_CLKOFF = 0x04
    SYSCFG_CLKON = 0x05
    SYSCFG_TOGGLE = 0x06
    VCCINT_XORKEY = 0xAE


class CW305(TargetTemplate):

    """CW305 target object.

    This class contains the public API for the CW305 hardware.
    To connect to the CW305, the easiest method is::

        import chipwhisperer as cw
        scope = cw.scope()
        target = cw.target(scope,
                targets.CW305, bsfile=<valid FPGA bitstream file>)

    If you're using the reference designs, typical configuration
    requires you to set the FPGA VCC-INT voltage and enable and 
    set the clock via the PLL. You'll probably also want to
    disable the USB clock during encryption to reduce noise::

        target.vccint_set(1.0) #set VCC-INT to 1V
        
        target.pll.pll_enable_set(True) #Enable PLL chip
        target.pll.pll_outenable_set(False, 0) # Disable unused PLL0
        target.pll.pll_outenable_set(True, 1)  # Enable PLL 
        target.pll.pll_outenable_set(False, 2) # Disable unused PLL2

        # optional, but reduces power trace noise
        target.clkusbautooff = True
        target.clksleeptime = 1 # 1ms typically good for sleep


    Don't forget to clock the ChipWhisperer ADC off the FPGA instead
    of the internal clock::

        scope.clock.adc_src = "extclk_x4"
        scope.clock.reset_adc() # make sure the DCM is locked

    Note that connecting to the CW305 includes programming the CW305 FPGA.
    For more help about CW305 settings, try help() on this CW305 submodule:

       * target.pll
    """


    _name = "ChipWhisperer CW305 (Artix-7)"
    BATCHRUN_START = 0x1
    BATCHRUN_RANDOM_KEY = 0x2
    BATCHRUN_RANDOM_PT = 0x4

    def __init__(self):
        TargetTemplate.__init__(self)
        self._naeusb = NAEUSB()
        self.pll = PLLCDCE906(self._naeusb, ref_freq = 12.0E6)
        self.fpga = FPGA(self._naeusb)

        self.hw = None
        self.oa = None

        self._woffset = 0x400
        self._woffset_sam3U = 0x000
        self._sam3U_max_nstates = 5
        self._sam3U_max_nrefresh = 255

        self._clksleeptime = 1
        self._clkusbautooff = True
        self.last_key = bytearray([0]*16)

    def _getNAEUSB(self):
        return self._naeusb


    def fpga_write(self, addr, data):
        """Write to an address on the FPGA

        Args:
            addr (int): Address to write to
            data (list): Data to write to addr

        Raises:
            IOError: User attempted to write to a read-only location
        """
        if addr < self._woffset:
            raise IOError("Write to read-only location: 0x%04x"%addr)

        return self._naeusb.cmdWriteMem(addr, data)

    def fpga_read(self, addr, readlen):
        """Read from an address on the FPGA

        Args:
            addr (int): Address to read from
            readlen (int): Length of data to read

        Returns:
            Requested data as a list
        """
        if addr > self._woffset:
            logging.info('Read from write address, confirm this is not an error')

        data = self._naeusb.cmdReadMem(addr, readlen)
        return data

    def usb_clk_setenabled(self, status):
        """ Turn on or off the Data Clock to the FPGA """
        if status:
            self._naeusb.sendCtrl(CW305_USB.REQ_SYSCFG, CW305_USB.SYSCFG_CLKON)
        else:
            self._naeusb.sendCtrl(CW305_USB.REQ_SYSCFG, CW305_USB.SYSCFG_CLKOFF)

    def usb_trigger_toggle(self, _=None):
        """ Toggle the trigger line high then low """
        self._naeusb.sendCtrl(CW305_USB.REQ_SYSCFG, CW305_USB.SYSCFG_TOGGLE)

    def vccint_set(self, vccint=1.0):
        """ Set the VCC-INT for the FPGA """

        # print "vccint = " + str(vccint)

        if (vccint < 0.6) or (vccint > 1.15):
            raise ValueError("VCC-Int out of range 0.6V-1.1V")

        # Convert to mV
        vccint = int(vccint * 1000)
        vccsetting = [vccint & 0xff, (vccint >> 8) & 0xff, 0]

        # calculate checksum
        vccsetting[2] = vccsetting[0] ^ vccsetting[1] ^ CW305_USB.VCCINT_XORKEY

        self._naeusb.sendCtrl(CW305_USB.REQ_VCCINT, 0, vccsetting)

        resp = self._naeusb.readCtrl(CW305_USB.REQ_VCCINT, dlen=3)
        if resp[0] != 2:
            raise IOError("VCC-INT Write Error, response = %d" % resp[0])

    def vccint_get(self):
        """ Get the last set value for VCC-INT """

        resp = self._naeusb.readCtrl(CW305_USB.REQ_VCCINT, dlen=3)
        return float(resp[1] | (resp[2] << 8)) / 1000.0

    def _con(self, scope=None, bsfile=None, force=False):
        """Connect to CW305 board, and download bitstream.

        If the target has already been programmed it skips reprogramming
        unless forced.

        Args:
            scope (ScopeTemplate): An instance of a scope object.
            bsfile (path): The path to the bitstream file to program the FPGA with.
            force (bool): Whether or not to force reprogramming.
            force (bool): Whether or not to force reprogramming.
        """

        self._naeusb.con(idProduct=[0xC305])
        if self.fpga.isFPGAProgrammed() == False or force:
            if bsfile is None:
                print("No FPGA Bitstream file specified.")
            elif not os.path.isfile(bsfile):
                print(("FPGA Bitstream not configured or '%s' not a file." % str(bsfile)))
            else:
                from datetime import datetime
                starttime = datetime.now()
                status = self.fpga.FPGAProgram(open(bsfile, "rb"), exceptOnDoneFailure=False)
                stoptime = datetime.now()
                if status:
                    logging.info('FPGA Config OK, time: %s' % str(stoptime - starttime))
                else:
                    logging.warning('FPGA Done pin failed to go high, check bitstream is for target device.')
        self.usb_clk_setenabled(True)
        self.fpga_write(0x100+self._woffset, [0])
        self.pll.cdce906init()

    def _dis(self):
        if self._naeusb:
            self._naeusb.close()

    def checkEncryptionKey(self, key):
        """Validate encryption key"""
        return key

    def loadEncryptionKey(self, key):
        """Write encryption key to FPGA"""
        self.key = key
        key = key[::-1]
        self.fpga_write(0x100+self._woffset, key)

    def loadInput(self, inputtext):
        """Write input to FPGA"""
        self.input = inputtext
        text = inputtext[::-1]
        self.fpga_write(0x200+self._woffset, text)

    def is_done(self):
        """Check if FPGA is done"""
        result = self.fpga_read(0x50, 1)[0]

        if result == 0x00:
            return False
        else:
            # Clear trigger
            self.fpga_write(0x40+self._woffset, [0])
            # LED Off
            self.fpga_write(0x10+self._woffset, [0])
            return True

    isDone = camel_case_deprecated(is_done)

    def readOutput(self):
        """"Read output from FPGA"""
        data = self.fpga_read(0x200, 16)
        data = data[::-1]
        #self.newInputData.emit(util.list2hexstr(data))
        return data

    @property
    def clkusbautooff(self):
        """ If set, the USB clock is automatically disabled on capture.

        The USB clock is re-enabled after self.clksleeptime milliseconds.

        Reads/Writes to the FPGA will not be possible until after
        the USB clock is reenabled, meaning :code:`usb_trigger_toggle()`
        must be used to trigger the FPGA to perform an encryption.

        :Getter: Gets whether to turn off the USB clock on capture

        :Setter: Sets whether to turn off the USB clock on capture
        """
        return self._clkusbautooff

    @clkusbautooff.setter
    def clkusbautooff(self, state):
        self._clkusbautooff = state

    @property
    def clksleeptime(self):
        """ Time (in milliseconds) that the USB clock is disabled for upon
        capture, if self.clkusbautooff is set.
        """
        return self._clksleeptime

    @clksleeptime.setter
    def clksleeptime(self, value):
        self._clksleeptime = value

    def go(self):
        """Disable USB clock (if requested), perform encryption, re-enable clock"""
        if self.clkusbautooff:
            self.usb_clk_setenabled(False)

        #LED On
        self.fpga_write(0x10+self._woffset, [0x01])

        time.sleep(0.001)
        self.usb_trigger_toggle()
        # self.FPGAWrite(0x100, [1])
        # self.FPGAWrite(0x100, [0])

        if self.clkusbautooff:
            time.sleep(self.clksleeptime/1000.0)
            self.usb_clk_setenabled(True)

    def simpleserial_read(self, cmd, pay_len, end='\n', timeout=250, ack=True):
        """Read data from target

        Mimics simpleserial protocol of serial based targets

        Args:
            cmd (str): Command to ues. Only accepts 'r' for now.
            pay_len: Unused
            end: Unused
            timeout: Unused
            ack: Unused

        Returns: Value from Crypto output register

        .. versionadded:: 5.1
            Added simpleserial_read to CW305
        """
        if cmd == "r":
            return self.readOutput()
        else:
            raise ValueError("Unknown command {}".format(cmd))

    def simpleserial_write(self, cmd, data, end=None):
        """Write data to target.

        Mimics simpleserial protocol of serial based targets.

        Args:
            cmd (str): Command to use. Target supports 'p' (write plaintext),
                and 'k' (write key).
            data (bytearray): Data to write to target
            end: Unused

        Raises:
            ValueError: Unknown command

        .. versionadded:: 5.1
            Added simpleserial_write to CW305
        """
        if cmd == 'p':
            self.loadInput(data)
            self.go()
        elif cmd == 'k':
            self.loadEncryptionKey(data)
        else:
            raise ValueError("Unknown command {}".format(cmd))

    def set_key(self, key, ack=False, timeout=250):
        """Checks if key is different from the last one sent. If so, send it.

        Args:
            key (bytearray):  key to send
            ack: Unused
            timeout: Unused

        .. versionadded:: 5.1
            Added set_key to CW305
        """
        if self.last_key != key:
            self.last_key = key
            self.simpleserial_write('k', key)

    def batchRun(
            self,
            nbatch=1024,
            nstate=1, 
            init_key=numpy.zeros([1,16],dtype=numpy.uint8),
            init_pt=numpy.zeros([1,16],dtype=numpy.uint8),
            flags_key=numpy.zeros([1,16],dtype=numpy.uint8),
            flags_pt=numpy.zeros([1,16],dtype=numpy.uint8),
            refreshes=[],
            seed=None,
            delay_status_loop=50
            ):
        """
            CAUTION: this method require a sufficient buffer size on the ASM3U
            to work properly. The theoritical bound for the parameter may not
            be reachable depending on the user setting (memory overflow
            possible if not used properly).

            Run multiple encryptions on random data, with configurable runs.
            Together with default value, each run can be configured to use
            default (i.e., fixed) or randomnly generated values. In adition,
            this configuration can be performed at hte byte level, allowing a
            user to use randomn generation of states for some byte while
            keeping some data constant accross the runs.  The status flags are
            used in this matter: each byte is associated with a flag. If the
            flag is 0, the provided default byte value is used. If the flag is
            1, the value of the byte will be randomly generated. 

            User can configure 'nstate' different configurations that will be
            run in parallel during the runs (i.e., during the nbatch runs, the
            configuration to use for a single run is randomly selected accross
            the 'nstate' provided) . The only constraint is that the size of
            the key and text should be similar for each configuration.

            In addition to the configuration of each run, it is possible to use
            the "refresh" mecanism. This mecanism occurs just before starting
            the run. The latter allows the bitwise XOR a randomly generated
            byte to two different memory word of the used configuration. In
            particular, 'refreshes' contain a list of pair of tuple (a pair in
            in fact a list containing 2 tuple of two elements). Each element of
            the pair points to a state byte. The first element of each tuple is
            the byte type (either 'k' for the key or 't' for the text), while
            the second is the byte index. For example, consider the following
            configuration:

                refreshes = [[('k',0),('k',8)],[('k',1),('t',2)]]

                will proceed as follows:
                1) for the first pair [('k',0),('k',8)]
                    - bitwise XOR the randomly generated byte b0 to
                        - the byte at index '0' of the key
                        - the byte at index '8' of the key
                2) for the second pair [('k',1),('t',2)]
                    - bitwise XOR the randomly generated byte b1 to 
                        - the byte at index '1' of the key
                        - the byte at index '2' of the text

            CAUTION: if used, the refresh mecanism will be used for each run,
            regardless of the state configuration used. 

            Args:
                nbatch (int): The number of encryption to run (default 1024).
                nstate (int): The number of configurations to be run in parallel (default 1, up to 255 included).
                key_size (int): Size of the key word in bytes (default 16, up to 255 included).
                pt_size (int): Size of the text word in bytes (default 16, up to 255 included).
                init_key (numpy uin8 array of shape [nbatch,key_size]): The default values of the key for each configuration. (key_size up to 255 included)
                init_pt (numpy uint8 array of shape [nbatch,pt_size]): The default values of the test for each configuration. (pt_size up to 255 included)
                flags_key (numpy uint8 array of shape [nbatch,key_size]): The configuration flag of each key byte. 
                flags_pt (numpy uint8 array of shape [nbatch,pt_size]): The configuration flag of each pt byte. 
                seed (int): random int32 seed for the PRG.
        """
        ######## Some useful functions
        # Function to create payload with length multiple of 32bits. Takes
        # list of byte, and append 0-bytes as required to match 32-bits multiple
        def paduint32(bl):
            l = len(bl)
            lm4 = l % 4
            if lm4 != 0:
                topad = 4-lm4
                bl.extend(topad*[0])

        # Return the amount of bytes required to encode the flag as one-hot 
        # encoding. (0: fixed to the init value, 1: random byte) 
        def amflagB(size):
            sdiv8 = size // 8
            smod8 = size % 8
            if smod8==0:
                return sdiv8
            else:
                return sdiv8 + 1
        
        # Generate bytes flag encoding
        def encode_flags(flags):
            return numpy.packbits(flags,bitorder='little').tolist()

        # Encode a full state
        def encode_state(s_ikey,s_ipt,s_fkey,s_fpt):
            spay = []
            spay.extend(s_ikey)
            spay.extend(s_ipt)
            spay.extend(encode_flags(s_fkey))
            spay.extend(encode_flags(s_fpt))
            return spay

        # PRNG function for LCG prng (first version, outdated now)
        def prng_byte_lcg(state):
            rnd_byte = (state >> 24) & 0xff
            new_state = state + (((state*state) & 0xffffffff) | 0x5)
            new_state &= 0xffffffff
            return [new_state,rnd_byte]
        
        # PRNG function for aes
        def prng_byte_aes(buf_idx,state_bytes,aesobj):
            if buf_idx==16:
                cipher = aesobj.update(state_bytes)
                nbuf = 0
            else:
                cipher = state_bytes
                nbuf = buf_idx
            # Upd
            nbuf += 1
            return [cipher,cipher[nbuf-1],nbuf]

        # Function to encode the refresh pairs. 
        def encode_refreshes(rpairs,offset_text):
            rf_enc = []
            for p in rpairs:
                (t0,i0) = p[0]
                (t1,i1) = p[1]
                add0 = i0 if t0=='k' else i0+offset_text 
                add1 = i1 if t1=='k' else i1+offset_text 
                rf_enc.extend([add0 & 0xff, add0 >> 8, add1 & 0xff, add1 >> 8])
            return rf_enc
            
        ###############################

        # Generate seed if none provided
        if seed is None:
            seed = random.randint(0,2**32)
        
        # Initialise AES PRNG
        prng_key = 16*[0]
        prng_key[0] = seed & 0xFF
        prng_key[1] = (seed >> 8) & 0xFF
        prng_key[2] = (seed >> 16) & 0xFF
        prng_key[3] = (seed >> 24) & 0xFF
        aes_obj = Cipher(algorithms.AES(bytes(prng_key)), modes.ECB())
        aes_obj = aes_obj.encryptor()
        prng_state = bytes(16*[0])
        rng_buf_idx = 16

        # Fetch values
        key_size = init_key.shape[1]
        pt_size = init_pt.shape[1]

        # Format config
        nbatch_2B = nbatch & 0XFFFF
        nstate_1B = nstate & 0XFF
        nrefresh_1B = len(refreshes) & 0xFF
        key_size_1B = key_size & 0XFF
        key_fsize_1B = amflagB(key_size_1B) & 0xFF
        pt_size_1B = pt_size & 0XFF
        pt_fsize_1B = amflagB(pt_size_1B) & 0xFF
        delay_status_loop_4B = delay_status_loop & 0xFFFFFFFF

        # Some verification
        assert key_size <= 255
        assert pt_size <= 255

        assert init_key.shape[0]==nstate and flags_key.shape[0]==nstate
        assert flags_key.shape[1]==key_size
        assert init_pt.shape[0]==nstate and flags_pt.shape[0]==nstate
        assert flags_pt.shape[1]==pt_size

        assert nbatch_2B==nbatch
        assert nstate_1B==nstate
        assert nrefresh_1B==len(refreshes)
        assert key_size_1B==key_size
        assert pt_size_1B==pt_size
        
        assert nstate<=self._sam3U_max_nstates
        assert nrefresh_1B<=self._sam3U_max_nrefresh

        # Encode states payload
        dpay = []
        for i in range(nstate):
            dpay.extend(encode_state(
                init_key[i,:].astype(numpy.uint8),
                init_pt[i,:].astype(numpy.uint8),
                flags_key[i,:].astype(bool),
                flags_pt[i,:].astype(bool)
                ))

        # Add the refresh encoding
        ref_encoding = encode_refreshes(refreshes,key_size)
        dpay.extend(ref_encoding)

        # Ensure proper length
        paduint32(dpay)

        # Create packet
        data = []
        data.extend(packuint32(nbatch_2B | (nstate_1B << 16) | (nrefresh_1B << 24)))
        data.extend(packuint32(
            key_size_1B | 
            (key_fsize_1B << 8) |
            (pt_size_1B << 16) |
            (pt_fsize_1B << 24)
            ))
        data.extend(packuint32(delay_status_loop_4B))
        data.extend(packuint32(seed))
        data.extend(dpay)

        # Write to controller
        self.sam3u_write(0,data)

        #### Predict values
        state_used = numpy.zeros([1,nbatch],dtype=numpy.uint8)
        key_used = numpy.zeros([nbatch,key_size],dtype=numpy.uint8) 
        pt_used = numpy.zeros([nbatch,pt_size],dtype=numpy.uint8) 
        full_state_size = 2*key_size + 2*pt_size
        for c in range(nbatch):
            # Reset buffer rnd
            rng_buf_idx = 16

            # Generate values for each cases
            # Generate byte to select state used
            rej_threshold = 256 - (256 % nstate)
            while True:
                [prng_state,rndb,rng_buf_idx] = prng_byte_aes(rng_buf_idx,prng_state,aes_obj)
                if rndb < rej_threshold:
                    break
            state_idx = rndb % nstate
            state_used[0,c] = state_idx
            
            # Generate KEY
            for kbi in range(key_size):
                [prng_state,rndb,rng_buf_idx] = prng_byte_aes(rng_buf_idx,prng_state,aes_obj)
                if flags_key[state_idx,kbi]:
                    # Need to generate random byte
                    key_used[c,kbi] = init_key[state_idx,kbi] ^ rndb
                else:
                    key_used[c,kbi] = init_key[state_idx,kbi]
            # Generate PT
            for pbi in range(pt_size):
                [prng_state,rndb,rng_buf_idx] = prng_byte_aes(rng_buf_idx,prng_state,aes_obj)
                if flags_pt[state_idx,pbi]:
                    # Need to generate random byte
                    pt_used[c,pbi] = init_pt[state_idx,pbi] ^ rndb
                else:
                    pt_used[c,pbi] = init_pt[state_idx,pbi]

            # Apply refreshes
            for raddrp in refreshes:
                # Generate random byte
                [prng_state,rndb,rng_buf_idx] = prng_byte_aes(rng_buf_idx,prng_state,aes_obj)
                for pe in raddrp:
                    if pe[0]=='t':
                        pt_used[c,pe[1]] = pt_used[c,pe[1]] ^ rndb
                    elif pe[0]=='k':
                        key_used[c,pe[1]] = key_used[c,pe[1]] ^ rndb
                    else:
                        raise ValueError("Unknown refresh data type")

        return key_used,pt_used,state_used

    def sam3u_write(self, addr, data):
        """Write to an address on the FPGA

        Args:
            addr (int): Address to write to
            data (list): Data to write to addr

        Raises:
            IOError: User attempted to write to a read-only location
        """
        if addr < self._woffset_sam3U:
            raise IOError("Write to read-only location: 0x%04x"%addr)
        #if len(data) > (256+addr):
        #    raise IOError("Write will overflow at location: 0x%04x"%(256))

        return self._naeusb.cmdWriteSam3U(addr, data)

