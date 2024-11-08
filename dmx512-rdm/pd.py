##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2016 Fabian J. Stumpf <sigrok@fabianstumpf.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from .lists import *

class Decoder(srd.Decoder):
    api_version = 3
    id = 'dmx512 RDM'
    name = 'DMX512 RDM'
    longname = 'Digital MultipleX 512 with RDM'
    desc = 'Digital MultipleX 512 (DMX512) lighting protocol.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Embedded/industrial', 'Lighting']
    channels = (
        {'id': 'dmx', 'name': 'DMX data', 'desc': 'Any DMX data line'},
    )
    options = (
        {'id': 'invert', 'desc': 'Invert Signal?', 'default': 'no',
            'values': ('yes', 'no')},
    )

    pid_part_one = "0"
    uuidsend1 = "0"
    uuidsend2 = "0"
    uuidsend3 = "0"
    uuidsend4 = "0"
    uuidsend5 = "0"
    uuidsend6 = "0"
    uuidrcv1 = "0"
    uuidrcv2 = "0"
    uuidrcv3 = "0"
    uuidrcv4 = "0"
    uuidrcv5 = "0"
    uuidrcv6 = "0"
    isrdm = "0"
    mesglentgh = 0

    annotations = (
        # Type of info
        ('bit', 'Bit'), # 0
        ('break', 'Break'), # 1
        ('mab', 'Mark after break'), # 2
        ('startbit', 'Start bit'), # 3
        ('stopbits', 'Stop bit'), # 4
        ('startcode', 'Start code'), # 5
        ('channel', 'Channel'), # 6
        ('interframe', 'Interframe'), # 7
        ('interpacket', 'Interpacket'), # 8
        ('data', 'Data'), # 9
        ('error', 'Error'), # 10
        ('rdm', 'RDM Data'), # 11
        ('rdmst', 'RDM Start Code'), # 12
        ('rdmsubst', 'RDM sub Start Code'), # 13
        ('rdmmsgl', 'RDM Message length'), # 14
        ('rdmdestuuid', 'RDM Destination UUID'), # 15
        ('rdmtransnum', 'RDM Transaction Number'), # 16
        ('rdmportid', 'RDM Port ID'), # 17
        ('rdmmsgcnt', 'RDM Message Count'), # 18
        ('rdmsubdev', 'RDM Sub-Device'), # 19
        ('rdmcmdclass', 'RDM Command Class'), # 20
        ('rdmpid', 'RDM Parameter ID'), # 21
        ('rdmpdl', 'RDM Parameter Data Length'), # 22
        ('rdmpidval', 'RDM PID value'), # 23
        ('manufracturer', 'Manufracturer'), # 24
        ('cdmcont', 'RDM content'), # 25
    )
    annotation_rows = (
        # Type of info displayed on each line
        ('name', 'Logical', (1, 2, 5, 6, 7, 8)),
        ('data', 'Data', (9,)),
        ('bits', 'Bits', (0, 3, 4)),
        ('errors', 'Errors', (10,)),
        ('rdm', 'RDM', (11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, )),
        ('rdmpidval', 'RDM value', (23, 24)),
        ('rdmcontent', 'RDM content', (25,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.sample_usec = None
        self.run_start = -1
        self.state = 'FIND BREAK'

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            self.sample_usec = 1 / value * 1000000
            self.skip_per_bit = int(4 / self.sample_usec)

    def putr(self, data):
        self.put(self.run_start, self.samplenum, self.out_ann, data)

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        inv = self.options['invert'] == 'yes'

        (dmx,) = self.wait({0: 'h' if inv else 'l'})
        self.run_start = self.samplenum

        while True:
            # Seek for an interval with no state change with a length between
            # 88 and 1000000 us (BREAK).
            if self.state == 'FIND BREAK':
                (dmx,) = self.wait({0: 'f' if inv else 'r'})
                runlen = (self.samplenum - self.run_start) * self.sample_usec
                if runlen > 88 and runlen < 1000000:
                    self.putr([1, ['Break']])
                    self.state = 'MARK MAB'
                    self.channel = 0
                elif runlen >= 1000000:
                    # Error condition.
                    self.putr([10, ['Invalid break length']])
                else:
                    (dmx,) = self.wait({0: 'h' if inv else 'l'})
                    self.run_start = self.samplenum
            # Directly following the BREAK is the MARK AFTER BREAK.
            elif self.state == 'MARK MAB':
                self.run_start = self.samplenum
                (dmx,) = self.wait({0: 'r' if inv else 'f'})
                self.putr([2, ['MAB']])
                self.state = 'READ BYTE'
                self.channel = 0
                self.bit = 0
                self.aggreg = dmx
                self.run_start = self.samplenum
            # Mark and read a single transmitted byte
            # (start bit, 8 data bits, 2 stop bits).
            elif self.state == 'READ BYTE':
                bit_start = self.samplenum
                # uuid_end = self.run_start + 312 * self.sample_usec
                # UUID is 312us long
                bit_end = self.run_start + (self.bit + 1) * self.skip_per_bit
                (dmx,) = self.wait({'skip': round(self.skip_per_bit/2)})
                bit_value = not dmx if inv else dmx

                if self.bit == 0:
                    self.byte = 0
                    self.put(bit_start, bit_end,
                             self.out_ann, [3, ['Start bit']])
                    if bit_value != 0:
                        # (Possibly) invalid start bit, mark but don't fail.
                        self.put(bit_start, bit_end,
                                 self.out_ann, [10, ['Invalid start bit']])
                elif self.bit >= 9:
                    self.put(bit_start, bit_end,
                             self.out_ann, [4, ['Stop bit']])
                    if bit_value != 1:
                        # Invalid stop bit, mark.
                        self.put(bit_start, bit_end,
                            self.out_ann, [10, ['Invalid stop bit']])
                        if self.bit == 10:
                            # On invalid 2nd stop bit, search for new break.
                            self.state = 'FIND BREAK'
                else:
                    # Label and process one bit.
                    self.put(bit_start, bit_end,
                             self.out_ann, [0, [str(bit_value)]])
                    self.byte |= bit_value << (self.bit - 1)


                # Label a complete byte.
                if self.state == 'READ BYTE' and self.bit == 10:
                    

                    if self.channel == 0: # If channel is 0 then it's a start channel 
                        if self.byte == 204: # If byte is 254 (0xcc) then it's an RDM start channel 
                            self.put(self.run_start, bit_end, self.out_ann, [12, ['RDM Start Code']]) # Print that it's an RDM start code channel
                            isrdm = 1
                        else:
                            self.put(self.run_start, bit_end, self.out_ann, [5, ['Start code']]) # Print that it's a DMX start code channel
                            isrdm = 0
                    
                    elif self.channel == 1 and isrdm == 1:
                        self.put(self.run_start, bit_end, self.out_ann, [13, ['RDM sub Start Code']]) 
                    
                    elif self.channel == 2 and isrdm == 1:
                        mesglentgh = self.byte
                        self.put(self.run_start, bit_end, self.out_ann, [14, ['RDM Message length : ' + str(self.byte)]])

                        self.put(self.run_start - 26 * self.skip_per_bit, bit_end + 13 * self.byte * self.skip_per_bit, self.out_ann, [6, ['RDM Packet']])
                    
                    elif self.channel == 3 and isrdm == 1:

                        # RDM Destination UUID
                        self.put(self.run_start, bit_end + 65 * self.skip_per_bit , self.out_ann, [15, ['RDM Destination UUID']])

                        uuidrcv1 = str("0x{:02x}".format(self.byte))[2:]

                    elif self.channel == 4 and isrdm == 1:

                        uuidrcv2 = str("0x{:02x}".format(self.byte))[2:]

                    elif self.channel == 5 and isrdm == 1:

                        uuidrcv3 = str("0x{:02x}".format(self.byte))[2:]
                    
                    elif self.channel == 6 and isrdm == 1:

                        uuidrcv4 = str("0x{:02x}".format(self.byte))[2:]
                    
                    elif self.channel == 7 and isrdm == 1:

                        uuidrcv5 = str("0x{:02x}".format(self.byte))[2:]
                    
                    elif self.channel == 8 and isrdm == 1:

                        uuidrcv6 = str("0x{:02x}".format(self.byte))[2:]

                        fulluuid = uuidrcv1 + uuidrcv2 + uuidrcv3 + uuidrcv4 + uuidrcv5 + uuidrcv6

                        uuid = uuidrcv1 + uuidrcv2
                        uuidmanu = str(uuid)
                        manufracturer = esta_manufracturers_id.get(uuidmanu, "NOT FOUND")

                        # Decodes and checks the UID library in lists
                        self.put(self.run_start - 65 * self.skip_per_bit, bit_end + 0 * self.skip_per_bit , self.out_ann, [24, [ str(manufracturer) + ' / ' + str(fulluuid) ]])


                        # RDM Source UUID
                    elif self.channel == 9 and isrdm == 1: # If channel is 0 then it's a start channel 
                        self.put(self.run_start, bit_end + 65 * self.skip_per_bit , self.out_ann, [15, ['RDM Source UUID']])

                        uuidsend1 =str("0x{:02x}".format(self.byte))[2:]
                    
                    elif self.channel == 10 and isrdm == 1: # If channel is 0 then it's a start channel 

                        uuidsend2 =str("0x{:02x}".format(self.byte))[2:]

                        # uuid2 = uuidsend1 + uuidsend2
                        # uuidmanu2 = str(uuid2)
                        # manufracturer2 = esta_manufracturers_id.get(uuidmanu2, "NOT FOUND")

                        # # Decodes and checks the UID library in lists 
                        # self.put(self.run_start - 13 * self.skip_per_bit, bit_end + 52 * self.skip_per_bit , self.out_ann, [24, [ str(manufracturer2) ]])

                    elif self.channel == 11 and isrdm == 1:

                        uuidsend3 = str("0x{:02x}".format(self.byte))[2:]

                    elif self.channel == 12 and isrdm == 1:

                        uuidsend4 = str("0x{:02x}".format(self.byte))[2:]
                    
                    elif self.channel == 13 and isrdm == 1:

                        uuidsend5 = str("0x{:02x}".format(self.byte))[2:]
                    
                    elif self.channel == 14 and isrdm == 1:

                        uuidsend6 = str("0x{:02x}".format(self.byte))[2:]

                        fulluuid2 = uuidsend1 + uuidsend2 + uuidsend3 + uuidsend4 + uuidsend5 + uuidsend6

                        uuid2 = uuidsend1 + uuidsend2
                        uuidmanu2 = str(uuid2)
                        manufracturer2 = esta_manufracturers_id.get(uuidmanu2, "NOT FOUND")

                        # Decodes and checks the UID library in lists
                        self.put(self.run_start - 65 * self.skip_per_bit, bit_end + 0 * self.skip_per_bit , self.out_ann, [24, [ str(manufracturer2) + ' / ' + str(fulluuid2) ]])


                    # RDM Transaction Number
                    elif self.channel == 15 and isrdm == 1:  
                        self.put(self.run_start, bit_end, self.out_ann, [16, ['RDM Transaction Number']])
                    
                    # RDM Port ID
                    elif self.channel == 16 and isrdm == 1: 
                        self.put(self.run_start, bit_end, self.out_ann, [17, ['RDM Port ID']])
                    
                    # RDM Message Count
                    elif self.channel == 17 and isrdm == 1: 
                        self.put(self.run_start, bit_end, self.out_ann, [18, ['RDM Message Count']])

                    # RDM Sub-Device
                    elif self.channel == 18 and isrdm == 1: 
                        self.put(self.run_start, bit_end + 13 * self.skip_per_bit , self.out_ann, [19, ['RDM Sub-Device']])
                    
                    # RDM Command Class
                    elif self.channel == 20 and isrdm == 1: 
                        self.put(self.run_start, bit_end, self.out_ann, [20, ['RDM Command Class']])
                    
                    # RDM Parameter ID
                    elif self.channel == 21 and isrdm == 1: 
                        self.put(self.run_start, bit_end + 13 * self.skip_per_bit , self.out_ann, [21, ['RDM Parameter ID']])
                        pid_part_one = self.byte

                    # PID
                    elif self.channel == 22 and isrdm == 1: # PID
                        pid_part_one_str = str("0x{:02x}".format(pid_part_one))
                        pid_part_two_str = str("0x{:02x}".format(self.byte))
                        pid = pid_part_one_str + pid_part_two_str[2:]
                        pid = str(pid)
                        rdm_pid = rdm_pid_list.get(pid, "PID NOT FOUND")
                        # Checks PID in dict
                        self.put(self.run_start - 13 * self.skip_per_bit, bit_end + 80 * self.skip_per_bit , self.out_ann, [23, [ str(pid) + ' / ' + str(rdm_pid) ]])
                        #
                    # if it's DMX
                    else:
                        self.put(self.run_start, bit_end, self.out_ann, [6, ['Channel ' + str(self.channel)]]) # else, print the channel walue
                    # self.put(self.run_start, bit_end, self.out_ann, d)

                    # Annotate the byte value as CAPITAL HEX
                    self.put(self.run_start + self.skip_per_bit,
                        bit_end - 2 * self.skip_per_bit,
                        self.out_ann, [9, [ "{:02X}".format(self.byte) ]])


                    # Continue by scanning the IFT.
                    self.channel += 1
                    self.run_start = self.samplenum
                    self.state = 'MARK IFT'

                self.bit += 1
                (dmx,) = self.wait({'skip': round(bit_end - self.samplenum)})
            # Mark the INTERFRAME-TIME between bytes / INTERPACKET-TIME between packets.
            elif self.state == 'MARK IFT':
                self.run_start = self.samplenum
                if self.channel > 512:
                    (dmx,) = self.wait({0: 'h' if inv else 'l'})
                    self.putr([8, ['Interpacket']])
                    self.state = 'FIND BREAK'
                    self.run_start = self.samplenum
                else:
                    if (not dmx if inv else dmx):
                        (dmx,) = self.wait({0: 'h' if inv else 'l'})
                        self.putr([7, ['Interframe']])
                    self.state = 'READ BYTE'
                    self.bit = 0
                    self.run_start = self.samplenum
