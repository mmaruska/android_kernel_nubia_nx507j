// device name/id/address/counts
#define TAOS_DEVICE_NAME                "taos"
#define TAOS_DEVICE_ID                  "tritonFN"
#define TAOS_ID_NAME_SIZE               10
#define TAOS_TRITON_CHIPIDVAL           0x00
#define TAOS_TRITON_MAXREGS             32
#define TAOS_DEVICE_ADDR1               0x29
#define TAOS_DEVICE_ADDR2               0x39
#define TAOS_DEVICE_ADDR3               0x49
#define TAOS_MAX_NUM_DEVICES            3
#define TAOS_MAX_DEVICE_REGS            32
#define I2C_MAX_ADAPTERS                12

// TRITON register offsets
// According to ~/trm/TMD2772_TMD2772WA_Datasheet_EN_v2.pdf  figure 31
// 00 = ENABLE = Enables states and interrupts
#define TAOS_TRITON_CNTRL               0x00
/* integration time: */
#define TAOS_TRITON_ALS_TIME            0X01
/* proximity time  -- never used??? */
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
// threshold for generating IRQ:
#if 0
// mmc: so these are not used, tuned!
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
// low 2 bytes ^^
//
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#endif

// rate of IRQs:
#define TAOS_TRITON_INTERRUPT           0x0C
// drive level
#define TAOS_TRITON_PRX_CFG             0x0D
// # of pulses
#define TAOS_TRITON_PRX_COUNT           0x0E
// mmc: this is used! |xx pdrive yy pdiode| zz pgain| ww again
#define TAOS_TRITON_GAIN                0x0F

#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
// proximity: ADC low & high data.
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_PRX_OFFSET          0x1E
#define TAOS_TRITON_TEST_STATUS         0x1F

// Triton cmd reg masks
//0x by clli2
#define TAOS_TRITON_CMD_REG             0X80
#define TAOS_TRITON_CMD_AUTO            0x20
#define TAOS_TRITON_CMD_BYTE_RW         0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60
#define TAOS_TRITON_CMD_PROX_INTCLR     0X05
#define TAOS_TRITON_CMD_ALS_INTCLR      0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR  0X07
#define TAOS_TRITON_CMD_TST_REG         0X08
#define TAOS_TRITON_CMD_USER_REG        0X09

// Triton cntrl reg masks
// figure 33: PEN 04
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL   0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0X04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01
#define TAOS_TRITON_STATUS_PRXVALID     0x02
#define TAOS_TRITON_STATUS_ADCINTR      0x10
#define TAOS_TRITON_STATUS_PRXINTR      0x20

// lux constants
#define TAOS_MAX_LUX                    10000
#define TAOS_SCALE_MILLILUX             3
#define TAOS_FILTER_DEPTH               3
#define CHIP_ID                         0x3d

#define TAOS_INPUT_NAME                 "lightsensor"
#define POLL_DELAY	                    msecs_to_jiffies(5)
#define TAOS_ALS_ADC_TIME_WHEN_PROX_ON	0xF0//0XF5//0XEB
#define TAOS_ALS_GAIN_DIVIDE            1000
#define TAOS_ALS_GAIN_1X                0
#define TAOS_ALS_GAIN_8X                1
#define TAOS_ALS_GAIN_16X               2
#define TAOS_ALS_GAIN_120X              3

#define CAL_THRESHOLD                      "/persist/proxdata/threshold"
#define PATH_PROX_OFFSET                   "/persist/sensors/proximity/offset/proximity_offset"
#define PATH_PROX_UNCOVER_DATA             "/persist/sensors/proximity/uncover_data"

#define PROX_LED_PULSE_CNT                  12
#define PROX_THRESHOLD_DISTANCE             100
#define PROX_DATA_TARGET                    150
#define PROX_DATA_MAX                       1023
#define PROX_OFFSET_CAL_BUFFER_SIZE         30
#define PROX_OFFSET_CAL_THRESHOLD           800
#define PROX_OFFSET_CAL_ABILITY_MAX         72 // -9*72
#define PROX_DATA_SAFE_RANGE_MIN            (PROX_DATA_TARGET - 50)
#define PROX_DATA_SAFE_RANGE_MAX            (PROX_DATA_TARGET + 250)
#define PROX_OFFSET_CAL_GETDATA_DELAY       10
#define PROX_DEFAULT_THRESHOLD_HIGH         800
#define PROX_DEFAULT_THRESHOLD_LOW          700
#define PROX_THRESHOLD_HIGH_MAX             800
#define PROX_THRESHOLD_HIGH_MIN             500
#define PROX_DEFAULT_OFFSET_CNT              0
#define PROX_THRESHOLD_SAFE_DISTANCE        300



