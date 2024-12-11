enum {
    GPIO_BASE = 0xFE200000,
    GPFSEL0_OFFSET = 0x00,
    GPFSEL1_OFFSET = 0x04,
    GPFSEL2_OFFSET = 0x08,
    GPFSEL3_OFFSET = 0x0c,
    GPFSEL4_OFFSET = 0x10,
    GPFSEL5_OFFSET = 0x14,
    GPSET0_OFFSET = 0x1c,
    GPSET1_OFFSET = 0x20,
    GPCLR0_OFFSET = 0x28,
    GPCLR1_OFFSET = 0x2c,
    GPUPPDN0_OFFSET = 0xe4,
    GPUPPDN1_OFFSET = 0xe8,
    GPLEV0_OFFSET = 0x34,
    TIMER_BASE = 0xFE00B000,
    TIMER_CTRL_OFFSET = 0x408,
    TIMER_LOAD_OFFSET = 0x400,
    TIMER_IRQCNTL_OFFSET = 0x40c,
    TIMER_RAWIRQ_OFFSET = 0x410,
    TIMER_VALUE_OFFSET = 0x404
};

long SET_BASE = (long)(GPIO_BASE + GPSET0_OFFSET);
long SET_BASE1 = (long)(GPIO_BASE + GPSET1_OFFSET);
long CLEAR_BASE = (long)(GPIO_BASE + GPCLR0_OFFSET);
long CLEAR_BASE1 = (long)(GPIO_BASE + GPCLR1_OFFSET);
long SEL2_BASE = (long)(GPIO_BASE + GPFSEL2_OFFSET);
long LEV0_BASE = (long)(GPIO_BASE + GPLEV0_OFFSET);
long GPUPDN1_BASE = (long)(GPIO_BASE + GPUPPDN1_OFFSET);

long CLK_CTRL = (long)(TIMER_BASE + TIMER_CTRL_OFFSET);

long TIMER_LOAD = (long)(TIMER_BASE + TIMER_LOAD_OFFSET);
long TIMER_IRQCNTL = (long)(TIMER_BASE + TIMER_IRQCNTL_OFFSET);
long TIMER_RAWIRQ = (long)(TIMER_BASE + TIMER_RAWIRQ_OFFSET);
long TIMER_VALUE = (long)(TIMER_BASE + TIMER_VALUE_OFFSET);

long clock_speed = 500000000;
unsigned int time_scale = 1;

void clear_register(long reg) {

    *(unsigned int*)reg = (unsigned int)0;

}

void gpio_write(long reg, unsigned int val) { *(volatile unsigned int*)reg = val; }
unsigned int gpio_read(long reg) { return *(volatile unsigned int*)reg; }

int pin_function(unsigned int pin_number, unsigned int function) {
    long reg = GPIO_BASE + ((pin_number / 10) * 4);

    unsigned int bit_mask = (1 << 3) - 1;
    
    unsigned int curval = *(unsigned int*)reg;
    curval &= ~(bit_mask << ((pin_number % 10) * 3));
    curval |= function << ((pin_number % 10) * 3);

    gpio_write(reg, curval);

    return 1;
}

int gpio_basic_call(long reg, unsigned int value, unsigned int field_size, unsigned int ls_bit) {

    gpio_write(reg, value << ls_bit);

    return 1;

}

int get_pin_value(unsigned int pin_number) {
    long reg = LEV0_BASE + ((pin_number / 32) * 4);

    unsigned int mask = (1 << (pin_number % 32));

    if(mask & *(volatile unsigned int*)reg) {
        return 1;
    } else {
        return 0;
    }

}

int pause(unsigned int count) {

    long lower_32 = 0xFE003000 + 0x04;

    unsigned int start_time = *(volatile unsigned int*)lower_32;
    unsigned int current_time = *(volatile unsigned int*)lower_32;

    while(current_time - start_time < count) {
        current_time = *(volatile unsigned int*)lower_32;
    }
    
    return 1;

}

void load_clock(unsigned int countdown) {
    *(unsigned int*)TIMER_LOAD = countdown;
}

void enable_clock() {
    gpio_basic_call(CLK_CTRL, 1, 1, 7);
}
void pause_clock() {
    gpio_basic_call(CLK_CTRL, 0, 1, 7);
}

void reset_clock_irq() {

    *(unsigned int*)TIMER_IRQCNTL = (unsigned int)1;
    *(unsigned int*)TIMER_IRQCNTL = (unsigned int)0;
}

unsigned int has_clock_finished() {
    return((unsigned int)1 & *(volatile unsigned int*)TIMER_RAWIRQ);
}

void ir_send0(unsigned int pin_number) {
    unsigned int ticks_per_us = 500;
    unsigned int swap = 0;

    load_clock(562.5 * ticks_per_us * time_scale);

    while(!has_clock_finished()) {
        if(swap == 0) {
            gpio_basic_call(SET_BASE, 1, 1, pin_number);
            swap = 1;
        } else if(swap == 1) {
            gpio_basic_call(CLEAR_BASE, 1, 1, pin_number);
            swap = 0;
        }
        pause((unsigned int)(13 * time_scale)); // Carrier frequency. 1 MHz clock
    }
    reset_clock_irq();

    load_clock(562.5 * ticks_per_us * time_scale);
    gpio_basic_call(CLEAR_BASE, 1, 1, pin_number);

    while(!has_clock_finished()) {}
    reset_clock_irq();

}

void ir_send1(unsigned int pin_number) {
    unsigned int ticks_per_us = 500;
    unsigned int swap = 0;

    load_clock((unsigned int)(562 * ticks_per_us * time_scale));

    while(!has_clock_finished()) {
        if(swap == 0) {
            gpio_basic_call(SET_BASE, 1, 1, pin_number);
            swap = 1;
        } else if(swap == 1) {
            gpio_basic_call(CLEAR_BASE, 1, 1, pin_number);
            swap = 0;
        }
        pause((unsigned int)(13 * time_scale)); // Carrier frequency. 1 MHz clock
    }
    reset_clock_irq();

    load_clock((unsigned int)(1687 * ticks_per_us * time_scale));
    gpio_basic_call(CLEAR_BASE, 1, 1, pin_number);

    while(!has_clock_finished()) {}
    reset_clock_irq();

}

void send_leading_pulse(unsigned int pin_number) {

    unsigned int ticks_per_us = 500;
    unsigned int swap = 0;

    reset_clock_irq();

    load_clock((unsigned int)(9000 * ticks_per_us * time_scale));

    gpio_basic_call(SET_BASE, 1, 1, pin_number);

    while(!has_clock_finished()) {
        if(swap == 0) {
            gpio_basic_call(SET_BASE, 1, 1, pin_number);
            swap = 1;
        } else if(swap == 1) {
            gpio_basic_call(CLEAR_BASE, 1, 1, pin_number);
            swap = 0;
        }
        pause((unsigned int)(13 * time_scale)); // Carrier frequency. 1 MHz clock
    }
    reset_clock_irq();
    swap = 1;

    gpio_basic_call(CLEAR_BASE, 1, 1, pin_number);
    pause((unsigned int)(4500 * time_scale));

}

void ir_send_data(unsigned int data, unsigned int data_length, unsigned int pin_number) {

    reset_clock_irq();
    send_leading_pulse(pin_number);

    for(int i = 0; i < data_length; i++) {

        if(data & (1 << i)) {
            ir_send1(pin_number);
            reset_clock_irq();
        } else {
            ir_send0(pin_number);
            reset_clock_irq();
        }

    }

    reset_clock_irq();
    ir_send0(pin_number);
    reset_clock_irq();

}

// UART

void gpio_useAsAlt5(unsigned int pin_number) {
    *(unsigned int*)(GPIO_BASE + GPUPPDN0_OFFSET) = 0;
    pin_function(pin_number, 2);
}

enum {
    AUX_BASE        = 0xFE000000 + 0x215000,
    AUX_ENABLES     = AUX_BASE + 4,
    AUX_MU_IO_REG   = AUX_BASE + 64,
    AUX_MU_IER_REG  = AUX_BASE + 68,
    AUX_MU_IIR_REG  = AUX_BASE + 72,
    AUX_MU_LCR_REG  = AUX_BASE + 76,
    AUX_MU_MCR_REG  = AUX_BASE + 80,
    AUX_MU_LSR_REG  = AUX_BASE + 84,
    AUX_MU_CNTL_REG = AUX_BASE + 96,
    AUX_MU_BAUD_REG = AUX_BASE + 104,
    AUX_UART_CLOCK  = 500000000,
    UART_MAX_QUEUE  = 16 * 1024
};

#define AUX_MU_BAUD(baud) ((AUX_UART_CLOCK/(baud*8))-1)

void uart_init() {
    gpio_write(AUX_ENABLES, 1); //enable UART1
    gpio_write(AUX_MU_IER_REG, 0);
    gpio_write(AUX_MU_CNTL_REG, 0);
    gpio_write(AUX_MU_LCR_REG, 3); //8 bits
    gpio_write(AUX_MU_MCR_REG, 0);
    gpio_write(AUX_MU_IER_REG, 0);
    gpio_write(AUX_MU_IIR_REG, 0xC6); //disable interrupts
    gpio_write(AUX_MU_BAUD_REG, AUX_MU_BAUD(115200));
    gpio_useAsAlt5(14);
    gpio_useAsAlt5(15);
    gpio_write(AUX_MU_CNTL_REG, 3); //enable RX/TX
}

unsigned int uart_isWriteByteReady() { return gpio_read(AUX_MU_LSR_REG) & 0x20; }

void uart_writeByteBlockingActual(unsigned char ch) {
    while (!uart_isWriteByteReady()); 
    gpio_write(AUX_MU_IO_REG, (unsigned int)ch);
}

void uart_writeText(char *buffer) {
    while (*buffer) {
       if (*buffer == '\n') uart_writeByteBlockingActual('\r');
       uart_writeByteBlockingActual(*buffer++);
    }
}

void main() {

    unsigned int last_value = 0;

    pin_function(20, 1);
    pin_function(21, 1);
    pin_function(22, 1);
    pin_function(23, 0);
    pin_function(24, 0);

    *(unsigned int*)CLK_CTRL = (1 << 7); // Enable clock
    *(unsigned int*)CLK_CTRL |= (1 << 1); // 32 bit counter
    *(unsigned int*)CLK_CTRL |= (1 << 9); // Free running counter

    long reg = (long)(TIMER_BASE + 0x41c); // Reset divider to 0
    *(unsigned int*)reg = (unsigned int)0;

    last_value = get_pin_value(24);

    uart_init();

    uart_writeText("Initializing...\n");

    long lower_32 = 0xFE003000 + 0x04;

    while(1) {

        unsigned int ticks_per_us = clock_speed / 1000000;

        while(1) {

            if(get_pin_value(24) == 1) {
                pause(20000);
                ir_send_data((unsigned int)4244768519, 32, 20);
            }

        }

        

        //LED power
            // 11111111 00000000 11111101 00000010 -- desired output
            
            // 10111111010000001111111100000000 -- inverted?
            // 01000000 10111111 00000000 11111111 -- reversed

        //LED blue
            // 00 11111111 00000000 01011101 10100010

        // TV power
            // 00 00011111 00011111 10111111 01000000
            // 522174272
            // 00000010111111011111100011111000
            // 11111101000000100000011100000111


        if(get_pin_value(24) == 0) {

            unsigned int start_time = *(volatile unsigned int*)lower_32;

            pause(600); // a little longer than first pulse for data bit
            while(get_pin_value(24) == 0) {} // in case of stray long signal

            if(*(volatile unsigned int*)lower_32 - start_time > 10000) {
                uart_writeText("Initial pulse");
            }
            
            reset_clock_irq();
            load_clock(1000 * ticks_per_us);

            start_time = *(volatile unsigned int*)lower_32;

            while(get_pin_value(24) == 1) {}

            if(has_clock_finished()) {
                uart_writeText("0");
                if(*(volatile unsigned int*)lower_32 - start_time > 5000) {
                    uart_writeText("AGC");
                }
            } else {
                uart_writeText("1");
            }

            reset_clock_irq();

        }

        ///
    }

}