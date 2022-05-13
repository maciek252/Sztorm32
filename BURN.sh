openocd -f bluepill.cfg -c 'init_reset halt; program build/ch.bin 0x8000000 verify;'
