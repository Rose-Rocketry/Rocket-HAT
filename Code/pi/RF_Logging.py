#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: reciverfilesink2.3
# Author: pi
# GNU Radio version: 3.8.2.0

import time
from gnuradio import blocks
from gnuradio import digital
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio.fft import logpwrfft
import fcdproplus


class RF_Logging(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "reciverfilesink2.3")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 192000
        self.bpak = bpak = digital.constellation_bpsk().base()

        ##################################################
        # Blocks
        ##################################################
        self.logpwrfft_x_0 = logpwrfft.logpwrfft_c(
            sample_rate=samp_rate,
            fft_size=1024,
            ref_scale=2,
            frame_rate=30,
            avg_alpha=1.0,
            average=False)
        self.fcdproplus_fcdproplus_0 = fcdproplus.fcdproplus('',1)
        self.fcdproplus_fcdproplus_0.set_lna(1)
        self.fcdproplus_fcdproplus_0.set_mixer_gain(1)
        self.fcdproplus_fcdproplus_0.set_if_gain(1)
        self.fcdproplus_fcdproplus_0.set_freq_corr(0)
        self.fcdproplus_fcdproplus_0.set_freq(426000000)
        self.digital_diff_decoder_bb_0 = digital.diff_decoder_bb(2)
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(12000, 2, True)
        self.digital_constellation_decoder_cb_0 = digital.constellation_decoder_cb(bpak)
        self.blocks_uchar_to_float_0 = blocks.uchar_to_float()
        self.blocks_moving_average_xx_0 = blocks.moving_average_ff(1000, 1, 4000, 1)
        self.blocks_max_xx_0 = blocks.max_ff(1024, 1)
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_float*1, '/home/pi/datafile-11.dat', False)
        self.blocks_file_sink_1.set_unbuffered(False)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_float*1, '/home/pi/datafile-1.dat', False)
        self.blocks_file_sink_0.set_unbuffered(False)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_max_xx_0, 0), (self.blocks_moving_average_xx_0, 0))
        self.connect((self.blocks_moving_average_xx_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.blocks_uchar_to_float_0, 0), (self.blocks_file_sink_1, 0))
        self.connect((self.digital_constellation_decoder_cb_0, 0), (self.digital_diff_decoder_bb_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.digital_constellation_decoder_cb_0, 0))
        self.connect((self.digital_diff_decoder_bb_0, 0), (self.blocks_uchar_to_float_0, 0))
        self.connect((self.fcdproplus_fcdproplus_0, 0), (self.digital_costas_loop_cc_0, 0))
        self.connect((self.fcdproplus_fcdproplus_0, 0), (self.logpwrfft_x_0, 0))
        self.connect((self.logpwrfft_x_0, 0), (self.blocks_max_xx_0, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.logpwrfft_x_0.set_sample_rate(self.samp_rate)

    def get_bpak(self):
        return self.bpak

    def set_bpak(self, bpak):
        self.bpak = bpak





def main(top_block_cls=RF_Logging, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    end = time.time()
    tb.stop()
    tb.wait()


if __name__ == '__main__':
   start = time.time() 
   main()
