#include "osal.h"
#include "mb_slave.h"
#include "mb_rtu.h"
#include "mb_tcp.h"

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>


#define RTU_ADDRESS_SMS40    0x16
#define RTU_ADDRESS_RMU40    0x19
#define RTU_ADDRESS_MODBUS40 0x20

static volatile sig_atomic_t keep_running = 1;

static void sig_handler(int _)
{
    (void)_;
    keep_running = 0;
}

static struct opt
{
    int verbose;
    struct
    {
        mb_tcp_cfg_t cfg;
    } tcp_details;
    struct
    {
        char device[80];
        mb_slave_cfg_t      slave;
        mb_rtu_serial_cfg_t cfg;
    } rtu_details;
} opt;

static os_timer_t *tmr1p5;
static void (*t1p5_callback)(void *arg);

static os_timer_t *tmr3p5;
static void (*t3p5_callback)(void *arg);

static void *tmr_arg;

static void tmr1p5_expired(os_timer_t *tmr, void *arg)
{
    if (t1p5_callback)
        t1p5_callback(tmr_arg);
}

static void tmr3p5_expired(os_timer_t *tmr, void *arg)
{
    if (t3p5_callback)
        t3p5_callback(tmr_arg);
}

static void mb_tx_enable(int level)
{
    /* This function controls the transceiver enabled state, if
      possible */
}

static void mb_tmr_init(uint32_t t1p5, uint32_t t3p5)
{
    os_timer_set(tmr1p5, t1p5);
    os_timer_set(tmr3p5, t3p5);
}

static void mb_tmr_start(
    void (*t1p5_expired)(void *arg),
    void (*t3p5_expired)(void *arg),
    void *arg)
{
    tmr_arg = arg;

    if (t1p5_expired)
    {
        t1p5_callback = t1p5_expired;
        os_timer_start(tmr1p5);
    }

    if (t3p5_expired)
    {
        t3p5_callback = t3p5_expired;
        os_timer_start(tmr3p5);
    }
}

mb_transport_t *mb_rtu_create(
    const char *device,
    mb_rtu_serial_cfg_t *serial_cfg)
{
    mb_transport_t *rtu;
    mb_rtu_cfg_t rtu_cfg;

    rtu_cfg.serial = device;
    rtu_cfg.serial_cfg = serial_cfg;
    rtu_cfg.tx_enable = mb_tx_enable;
    rtu_cfg.tmr_init = mb_tmr_init;
    rtu_cfg.tmr_start = mb_tmr_start;

    tmr1p5 = os_timer_create(0, tmr1p5_expired, NULL, true);
    tmr3p5 = os_timer_create(0, tmr3p5_expired, NULL, true);

    rtu = mb_rtu_init(&rtu_cfg);
    return rtu;
}

static uint8_t coils[2] = {0x55, 0xAA};
static uint16_t hold[4] = {0x1234, 0x5678, 0x55AA, 0xAA55};

static int coil_get(uint16_t address, uint8_t *data, size_t quantity)
{
    uint16_t offset;

    for (offset = 0; offset < quantity; offset++)
    {
        uint32_t bit = address + offset;
        int value;

        value = mb_slave_bit_get(coils, bit);
        mb_slave_bit_set(data, offset, value);
    }
    return 0;
}

static int coil_set(uint16_t address, uint8_t *data, size_t quantity)
{
    uint16_t offset;

    for (offset = 0; offset < quantity; offset++)
    {
        uint32_t bit = address + offset;
        int value;

        value = mb_slave_bit_get(data, offset);
        mb_slave_bit_set(coils, bit, value);
    }
    return 0;
}

static int input_get(uint16_t address, uint8_t *data, size_t quantity)
{
    uint16_t offset;

    for (offset = 0; offset < quantity; offset++)
    {
        mb_slave_bit_set(data, offset, 1);
    }
    return 0;
}

static int hold_get(uint16_t address, uint8_t *data, size_t quantity)
{
    uint16_t offset;

    for (offset = 0; offset < quantity; offset++)
    {
        uint32_t reg = address + offset;

        mb_slave_reg_set(data, offset, hold[reg]);
    }
    return 0;
}

static int hold_set(uint16_t address, uint8_t *data, size_t quantity)
{
    uint16_t offset;

    for (offset = 0; offset < quantity; offset++)
    {
        uint32_t reg = address + offset;

        hold[reg] = mb_slave_reg_get(data, offset);
    }
    return 0;
}

static int reg_get(uint16_t address, uint8_t *data, size_t quantity)
{
    uint16_t offset;

    for (offset = 0; offset < quantity; offset++)
    {
        mb_slave_reg_set(data, offset, 0x1100 | (offset & 0xFF));
    }
    return 0;
}

const mb_iomap_t mb_slave_iomap = {
    .coils = {
        .size = 16,
        .get = coil_get,
        .set = coil_set
    },
    .inputs = {
        .size = 2,
        .get = input_get,
        .set = NULL
    },
    .holding_registers = {
        .size = 4,
        .get = hold_get,
        .set = hold_set,
    },
    .input_registers = {
        .size = 5,
        .get = reg_get,
        .set = NULL
    },
    .num_vendor_funcs = 0,
    .vendor_funcs = NULL,
};

static void help(const char *name)
{
    printf("NIBE Modbus rtu to tcp gateway\n");
    exit(EXIT_SUCCESS);
}

static void parse_opt(int argc, char *argv[], struct opt* opt)
{
    static struct option options[] = {
        {"help", no_argument, NULL, 'h'},
        {"verbose", no_argument, NULL, 'v'},
        {"unit", optional_argument, NULL, 'd'},
        {"device", required_argument, NULL, 'd'},
        {NULL, 0, NULL, 0}
    };

    while (1)
    {
        int c;
        int option_index = 0;

        c = getopt_long(argc, argv, "hvu:d:", options, &option_index);
        if (c == -1)
            break;

        switch (c)
        {
        case 'h':
            help(argv[0]);
            break;
        case 'v':
            opt->verbose = 1;
            break;
        case 'u':
            opt->rtu_details.slave.id = atoi(optarg);
            break;
        case 'd':
            strcpy(opt->rtu_details.device, optarg);
            break;
        default:
            exit(EXIT_FAILURE);
        }
    }
}

int main(int argc, char *argv[])
{
    mb_slave_t *rtu_slave;
    mb_transport_t *rtu_transport;
    struct opt opt = {
        .rtu_details.cfg.baudrate = 9600,
        .rtu_details.cfg.parity = NONE,
        .rtu_details.slave = {
            .id = RTU_ADDRESS_MODBUS40,
            .priority = 15,
            .stack_size = 2048,
            .iomap = &mb_slave_iomap,
        },
    };

    parse_opt(argc, argv, &opt);

    rtu_transport = mb_rtu_create(opt.rtu_details.device, &opt.rtu_details.cfg);
    rtu_slave = mb_slave_init(&opt.rtu_details.slave, rtu_transport);


    signal(SIGINT, sig_handler);
    while(keep_running) {
        os_usleep(1000);
    }

    mb_slave_shutdown(rtu_slave);
}
