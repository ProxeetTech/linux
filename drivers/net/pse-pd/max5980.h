/*
 * MAX5980 IEEE 802.3at/af Quad Port Power-over-Ethernet PSE Controller
 */

#ifndef __MAX5980_H__
#define __MAX5980_H__

#define MAX5980_PORTS_NUM	4
#define ASCII_TO_DIGIT(ascii_digit) (ascii_digit - 0x30) 

enum max5980_port_id {
    MAX5980_PORT_1 = 0,
    MAX5980_PORT_2,
    MAX5980_PORT_3,
    MAX5980_PORT_4
};
/* Power enable register */
#define MAX5980_P_ON			1
#define MAX5980_P_OFF			0

/* Interrupt registers */
#define MAX5980_INT_REG                     0x00
#define MAX5980_PE_INT                      (1 << 0)
#define MAX5980_PG_INT                      (1 << 1)
#define MAX5980_DIS_INT                     (1 << 2)
#define MAX5980_DET_INT                     (1 << 3)
#define MAX5980_CLS_INT                     (1 << 4)
#define MAX5980_TCUT_INT                    (1 << 5)
#define MAX5980_TST_INT                     (1 << 6)
#define MAX5980_SUP_INT                     (1 << 7)

#define MAX5980_INT_MASK_REG                0x01
#define MAX5980_PE_MASK                     (1 << 0)
#define MAX5980_PG_MASK                     (1 << 1)
#define MAX5980_DIS_MASK                    (1 << 2)
#define MAX5980_DET_MASK                    (1 << 3)
#define MAX5980_CLS_MASK                    (1 << 4)
#define MAX5980_TCUT_MASK                   (1 << 5)
#define MAX5980_TST_MASK                    (1 << 6)
#define MAX5980_SUP_MASK                    (1 << 7)

/* Events registers */
#define MAX5980_PWR_EVENT_REG               0x02
#define MAX5980_PWR_EVENT_COR_REG           0x03
#define MAX5980_PE_CHG1                     (1 << 0)
#define MAX5980_PE_CHG2                     (1 << 1)
#define MAX5980_PE_CHG3                     (1 << 2)
#define MAX5980_PE_CHG4                     (1 << 3)
#define MAX5980_PG_CHG1                     (1 << 4)
#define MAX5980_PG_CHG2                     (1 << 5)
#define MAX5980_PG_CHG3                     (1 << 6)
#define MAX5980_PG_CHG4                     (1 << 7)

#define MAX5980_DET_EVENT_REG               0x04
#define MAX5980_DET_EVENT_COR_REG           0x05
#define MAX5980_DET1                        (1 << 0)
#define MAX5980_DET2                        (1 << 1)
#define MAX5980_DET3                        (1 << 2)
#define MAX5980_DET4                        (1 << 3)
#define MAX5980_CLS1                        (1 << 4)
#define MAX5980_CLS2                        (1 << 5)
#define MAX5980_CLS3                        (1 << 6)
#define MAX5980_CLS4                        (1 << 7)

#define MAX5980_FAULT_EVENT_REG             0x06
#define MAX5980_FAULT_EVENT_COR_REG         0x07
#define MAX5980_TCUT1                       (1 << 0)
#define MAX5980_TCUT2                       (1 << 1)
#define MAX5980_TCUT3                       (1 << 2)
#define MAX5980_TCUT4                       (1 << 3)
#define MAX5980_DIS1                        (1 << 4)
#define MAX5980_DIS2                        (1 << 5)
#define MAX5980_DIS3                        (1 << 6)
#define MAX5980_DIS4                        (1 << 7)

#define MAX5980_STARTUP_EVENT_REG           0x08
#define MAX5980_STARTUP_EVENT_COR_REG       0x09
#define MAX5980_TSTART1                     (1 << 0)
#define MAX5980_TSTART2                     (1 << 1)
#define MAX5980_TSTART3                     (1 << 2)
#define MAX5980_TSTART4                     (1 << 3)
#define MAX5980_ICV1                        (1 << 4)
#define MAX5980_ICV2                        (1 << 5)
#define MAX5980_ICV3                        (1 << 6)
#define MAX5980_ICV4                        (1 << 7)

#define MAX5980_SUPPLY_EVENT_REG            0x0A
#define MAX5980_SUPPLY_EVENT_COR_REG        0x0B
#define MAX5980_VEE_UVLO                    (1 << 4)
#define MAX5980_VDD_UVLO                    (1 << 5)
#define MAX5980_FETBAD                      (1 << 6)
#define MAX5980_TSD                         (1 << 7)

/* Status registers */
#define MAX5980_PORT1_STATUS_REG            0x0C
#define MAX5980_PORT2_STATUS_REG            0x0D
#define MAX5980_PORT3_STATUS_REG            0x0E
#define MAX5980_PORT4_STATUS_REG            0x0F
#define MAX5980_DET_ST_0                    (1 << 0)
#define MAX5980_DET_ST_1                    (1 << 1)
#define MAX5980_DET_ST_2                    (1 << 2)
#define MAX5980_CLASS_0                     (1 << 4)
#define MAX5980_CLASS_1                     (1 << 5)
#define MAX5980_CLASS_2                     (1 << 6)
#define MAX5980_DET_RES_NONE                0x0
#define MAX5980_DET_RES_DCP                 0x1
#define MAX5980_DET_RES_HIGH_CAP            0x2
#define MAX5980_DET_RES_RLOW                0x3
#define MAX5980_DET_RES_DET_OK              0x4
#define MAX5980_DET_RES_RHIGH               0x5
#define MAX5980_DET_RES_OPEN                0x6
#define MAX5980_DET_RES_DCN                 0x7
#define MAX5980_CLASS_RES_UNKNOWN           0x0
#define MAX5980_CLASS_RES_1                 0x1
#define MAX5980_CLASS_RES_2                 0x2
#define MAX5980_CLASS_RES_3                 0x3
#define MAX5980_CLASS_RES_4                 0x4
#define MAX5980_CLASS_RES_5                 0x5
#define MAX5980_CLASS_RES_0                 0x6
#define MAX5980_CLASS_RES_CUR_LIM           0x7
#define MAX5980_GET_DET_RES(reg)            (reg & 0x7)
#define MAX5980_GET_CLASS_RES(reg)          ((reg >> 4) & 0x7)

#define MAX5980_PWR_STATUS_REG              0x10
#define MAX5980_PWR_EN1                     (1 << 0)
#define MAX5980_PWR_EN2                     (1 << 1)
#define MAX5980_PWR_EN3                     (1 << 2)
#define MAX5980_PWR_EN4                     (1 << 3)
#define MAX5980_PGOOD1                      (1 << 4)
#define MAX5980_PGOOD2                      (1 << 5)
#define MAX5980_PGOOD3                      (1 << 6)
#define MAX5980_PGOOD4                      (1 << 7)

#define MAX5980_PIN_STATUS_REG              0x11
#define MAX5980_AUTO                        (1 << 0)
#define MAX5980_ID0                         (1 << 2)
#define MAX5980_ID1                         (1 << 3)
#define MAX5980_SLAVE0                      (1 << 4)
#define MAX5980_SLAVE1                      (1 << 5)

/* Configuration registers */
#define MAX5980_OPER_MODE_REG               0x12
#define MAX5980_P1_M0                       (1 << 0)
#define MAX5980_P1_M1                       (1 << 1)
#define MAX5980_P2_M0                       (1 << 2)
#define MAX5980_P2_M1                       (1 << 3)
#define MAX5980_P3_M0                       (1 << 4)
#define MAX5980_P3_M1                       (1 << 5)
#define MAX5980_P4_M0                       (1 << 6)
#define MAX5980_P4_M1                       (1 << 7)

#define MAX5980_PT_MODE_FIELD_WIDTH	2
#define MAX5980_PT_MODE_MASK		0x3

#define MAX5980_OPER_MODE_SHUTDOWN          0x0
#define MAX5980_OPER_MODE_MANUAL            0x1
#define MAX5980_OPER_MODE_SEMIAUTO          0x2
#define MAX5980_OPER_MODE_AUTO              0x3
#define MAX5980_GET_PORT_OPER_MODE(reg, port)     ((reg >> (port * 2) - 2) & 0x3)

#define MAX5980_DISCONN_EN_REG              0x13
#define MAX5980_DCD_EN1                     (1 << 0)
#define MAX5980_DCD_EN2                     (1 << 1)
#define MAX5980_DCD_EN3                     (1 << 2)
#define MAX5980_DCD_EN4                     (1 << 3)
#define MAX5980_ACD_EN1                     (1 << 4)
#define MAX5980_ACD_EN2                     (1 << 5)
#define MAX5980_ACD_EN3                     (1 << 6)
#define MAX5980_ACD_EN4                     (1 << 7)

#define MAX5980_DET_AND_CLASS_EN_REG        0x14
#define MAX5980_DET_EN1                     (1 << 0)
#define MAX5980_DET_EN2                     (1 << 1)
#define MAX5980_DET_EN3                     (1 << 2)
#define MAX5980_DET_EN4                     (1 << 3)
#define MAX5980_CLASS_EN1                   (1 << 4)
#define MAX5980_CLASS_EN2                   (1 << 5)
#define MAX5980_CLASS_EN3                   (1 << 6)
#define MAX5980_CLASS_EN4                   (1 << 7)

#define MAX5980_DIS_EN			            0x1
#define MAX5980_DET_CLAS_EN		            0x11

#define MAX5980_MIDSPAN_EN_REG              0x15
#define MAX5980_MIDSPAN1                    (1 << 0)
#define MAX5980_MIDSPAN2                    (1 << 1)
#define MAX5980_MIDSPAN3                    (1 << 2)
#define MAX5980_MIDSPAN4                    (1 << 3)

#define MAX5980_MISC_CONF1_REG              0x17
#define MAX5980_DET_CHG                     (1 << 6)
#define MAX5980_INT_EN                      (1 << 7)

/* Push buttons registers */
#define MAX5980_DET_CLASS_PUSHBTN_REG       0x18
#define MAX5980_DET_PB1                     (1 << 0)
#define MAX5980_DET_PB2                     (1 << 1)
#define MAX5980_DET_PB3                     (1 << 2)
#define MAX5980_DET_PB4                     (1 << 3)
#define MAX5980_CLS_PB1                     (1 << 4)
#define MAX5980_CLS_PB2                     (1 << 5)
#define MAX5980_CLS_PB3                     (1 << 6)
#define MAX5980_CLS_PB4                     (1 << 7)

#define MAX5980_PWR_EN_PUSHBTN_REG          0x19
#define MAX5980_PWR_ON1                     (1 << 0)
#define MAX5980_PWR_ON2                     (1 << 1)
#define MAX5980_PWR_ON3                     (1 << 2)
#define MAX5980_PWR_ON4                     (1 << 3)
#define MAX5980_PWR_OFF1                    (1 << 4)
#define MAX5980_PWR_OFF2                    (1 << 5)
#define MAX5980_PWR_OFF3                    (1 << 6)
#define MAX5980_PWR_OFF4                    (1 << 7)
#define MAX5980_PWR_ON_SHIFT				0
#define MAX5980_PWR_OFF_SHIFT				4
#define MAX5980_PORT_SELECT(port_i) 		(0x1 << (port_i))

#define MAX5980_GLOBAL_PUSHBTN_REG          0x1A
#define MAX5980_RESET_P1                    (1 << 0)
#define MAX5980_RESET_P2                    (1 << 1)
#define MAX5980_RESET_P3                    (1 << 2)
#define MAX5980_RESET_P4                    (1 << 3)
#define MAX5980_RESET_IC                    (1 << 4)
#define MAX5980_PIN_CLR                     (1 << 6)
#define MAX5980_INT_CLR                     (1 << 7)

/* General registers */
#define MAX5980_ID_REG                      0x1B
#define MAX5980_CLASS5_EN_REG               0x1C
#define MAX5980_TLIM12_PROG_REG             0x1E
#define MAX5980_TLIM34_PROG_REG             0x1F

/* Reserved registers */
#define MAX5980_MISC_CONF2_REG              0x29
#define MAX5980_LSC_EN                      (1 << 4)
#define MAX5980_VEE_R4                      (1 << 3)
#define MAX5980_VEE_R3                      (1 << 2)
#define MAX5980_VEE_R2                      (1 << 1)
#define MAX5980_VEE_R1                      (1 << 0)

/* Port current and voltage info */
#define MAX5980_DC_REGS			0x30
#define MAX5980_DC_INFO_SZ		16

/* Current/Voltage registers */
#define MAX5980_PORT1_CURRENT0              0x30
#define MAX5980_PORT1_CURRENT1              0x31
#define MAX5980_PORT1_VOLT0                 0x32
#define MAX5980_PORT1_VOLT1                 0x33
#define MAX5980_PORT2_CURRENT0              0x34
#define MAX5980_PORT2_CURRENT1              0x35
#define MAX5980_PORT2_VOLT0                 0x36
#define MAX5980_PORT2_VOLT1                 0x37
#define MAX5980_PORT3_CURRENT0              0x38
#define MAX5980_PORT3_CURRENT1              0x39
#define MAX5980_PORT3_VOLT0                 0x3A
#define MAX5980_PORT3_VOLT1                 0x3B
#define MAX5980_PORT4_CURRENT0              0x3C
#define MAX5980_PORT4_CURRENT1              0x3D
#define MAX5980_PORT4_VOLT0                 0x3E
#define MAX5980_PORT4_VOLT1                 0x3F

/* Other functions registers */
#define MAX5980_FW_REG                      0x41
#define MAX5980_WDG_REG                     0x42
#define MAX5980_VEE_OV                      (1 << 6)
#define MAX5980_VEE_UV                      (1 << 5)
#define MAX5980_WDG_OFF_VALUE               0xB
#define MAX5980_WD_STAT                     (1 << 0)
#define MAX5980_WDG_VAL_OFFSET              1

#define MAX5980_DEV_ID_REV_NUM_REG          0x43

#define MAX5980_HIGH_POWER_EN_REG           0x44
#define MAX5980_HP_EN4                      (1 << 3)
#define MAX5980_HP_EN3                      (1 << 2)
#define MAX5980_HP_EN2                      (1 << 1)
#define MAX5980_HP_EN1                      (1 << 0)

#define MAX5980_PORT1_GPMD                  0x46
#define MAX5980_PORT1_ICUT                  0x47
#define MAX5980_PORT1_ILIM                  0x48
#define MAX5980_PORT1_HIGH_POWER_STAT_REG   0x49
#define MAX5980_PORT2_GPMD                  0x4B
#define MAX5980_PORT2_ICUT                  0x4C
#define MAX5980_PORT2_ILIM                  0x4D
#define MAX5980_PORT2_HIGH_POWER_STAT_REG   0x4E
#define MAX5980_PORT3_GPMD                  0x50
#define MAX5980_PORT3_ICUT                  0x51
#define MAX5980_PORT3_ILIM                  0x52
#define MAX5980_PORT3_HIGH_POWER_STAT_REG   0x53
#define MAX5980_PORT4_GPMD                  0x55
#define MAX5980_PORT4_ICUT                  0x56
#define MAX5980_PORT4_ILIM                  0x57
#define MAX5980_PORT4_HIGH_POWER_STAT_REG   0x58
#define MAX5980_LEG_EN                      (1 << 1)
#define MAX5980_PONG_EN                     (1 << 0)
#define MAX5980_RDIS                        (1 << 7)
#define MAX5980_CUT_RNG                     (1 << 6)
#define MAX5980_ILIM                        (1 << 6)
#define MAX5980_FET_BAD                     (1 << 1)
#define MAX5980_PONG_PD                     (1 << 0)

#define TLIM			1710
#define VOLT_LSB		5835
#define CURR_LSB		122070

struct pt_dc_parm {
	s16 i;
	s16 v;
};

struct pt_dflt {
	int enable;
	int mode;
	int pwr;
	const char *name;
};

struct max5980_platform_data {
	u32 irq;
	struct pt_dflt pt_df[MAX5980_PORTS_NUM];
};

struct max5980_data {
	struct i2c_client *client;
	struct max5980_platform_data pdata;
	struct pt_dc_parm pt_dc[MAX5980_PORTS_NUM];
};

#endif /* __MAX5980_H__ */
