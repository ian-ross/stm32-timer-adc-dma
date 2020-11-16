# STM32 timer+ADC+DMA examples

All examples built from a single `Makefile`: set the
`GCC_INSTALL_ROOT` variable to point to the `bin` directory in an
`arm-none-eabi` GCC installation.

Examples are:

 - `ex1.c`: simple polling ADC;
 - `ex2.c`: manually triggered single sample ADC with DMA;
 - `ex3.c`: manually triggered multiple sample ADC with DMA;
 - `ex4.c`: timer-driven multiple sample ADC with DMA;
 - `ex5.c` and `ex5-view.py`: bare-bones USB oscilloscope.

All written to run on a Nucleo-144 board with an STM32F767ZI.

## Python setup

The USB oscilloscope example uses Python (you'll need Python 3), and
relies on a couple of Python libraries, listed in the usual
`requirements.txt` file. The best way to deal with this is to create a
virtual environment:

```
python -m venv .venv
. ./.venv/bin/activate
pip install -r requirements.txt
```

Then you'll be able to run the `ex5-view.py` script.

**Note** The serial port used by the Python script is hard-coded.
Depending on your Linux distribution (or if you're using Windows),
you may need to change it. Search for the string "`/dev/ttyACM0`" and
replace it with whatever your operating system uses for the USB
virtual serial port from the ST-Link on the Nucleo board.
