#!/bin/bash

avrdude -cusbtiny -pattiny85 -U lfuse:w:0xe2:m -U hfuse:w:0xd7:m -U efuse:w:0xfe:m
avrdude -cusbtiny -pattiny85 -U flash:w:urboot_t85_2s_i8m0_57k6_swio_rxb1_txb0_no-led.hex