/*
 * pycoproc.h - Copyright (c) 2022-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ########################################### Macros ##############################################

#define	pycoprocI2C_LOGIC			1					// 0 = delay, 1= stretch, 2= stages
#define	pycoprocADDR				0x08
#define	PYCOPROC_T_SNS				10000
#define	EXP_RTC_PERIOD				7000

// ######################################## Enumerations ###########################################

enum {								// commands
	pycoprocCMD_PEEK			= 0x00,
	pycoprocCMD_POKE			= 0x01,
	pycoprocCMD_MAGIC			= 0x02,
	pycoprocCMD_HW_VER			= 0x10,
	pycoprocCMD_FW_VER			= 0x11,
	pycoprocCMD_PROD_ID			= 0x12,

	pycoprocCMD_SETUP_SLEEP		= 0x20,
	pycoprocCMD_GO_SLEEP		= 0x21,
	pycoprocCMD_CALIBRATE		= 0x22,
	pycoprocCMD_BAUD_CHANGE		= 0x30,
	pycoprocCMD_DFU				= 0x31,
};

enum {								// register addresses
	pycoprocADDR_INTCON			= 0x0B,
	pycoprocADDR_PORTA			= 0x0C,
	pycoprocADDR_PORTC			= 0x0E,

	pycoprocADDR_STATUS			= 0x83,
	pycoprocADDR_TRISC			= 0x8E,

	pycoprocADDR_OPTION_REG		= 0x95,
	pycoprocADDR_PCON			= 0x96,

	pycoprocADDR_ADRESL			= 0x9B,					// A/D result
	pycoprocADDR_ADRESH			= 0x9C,

	pycoprocADDR_ADCON0			= 0x9D,					// A/D Control
	pycoprocADDR_ADCON1			= 0x9E,

	pycoprocADDR_ANSELA			= 0x018C,
	pycoprocADDR_ANSELB			= 0x018D,
	pycoprocADDR_ANSELC			= 0x018E,

	pycoprocADDR_WPUA			= 0x020C,

	pycoprocADDR_IOCAP			= 0x0391,
	pycoprocADDR_IOCAN			= 0x0392,

	pycoprocADDR_MEMORY_BANK	= 0x0620,
	pycoprocADDR_WAKE_REASON	= 0x064C,
};

enum {
	pycoproc_ADCON0_CHS_POSN	= 0x02,
	pycoprocADCON0_ADON_MASK	= 0x01,
	pycoprocADCON1_ADCS_POSN	= 0x04,
	pycoprocADCON0_GO_nDONE_MASK = 0x02,
};

enum {
	pycoprocMAGIC_OP_PEEK		= 0x00,
	pycoprocMAGIC_OP_POKE		= 0x01,
	pycoprocMAGIC_OP_CLR_BITS	= 0x02,
	pycoprocMAGIC_OP_SET_BITS	= 0x03,
	pycoprocMAGIC_OP_TGL_BITS	= 0x04,
};

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) {				// MAGIC command structure
	union {
		u8_t CMD;
		u8_t _RESULT;
	};
	u8_t ADDRL;
	u8_t ADDRH;
	union {
		u8_t _DATA;
		u8_t _AND;
	};
	u8_t _OR;
	u8_t _XOR;
} pycoproc_cmd_t;
DUMB_STATIC_ASSERT(sizeof(pycoproc_cmd_t) == 6);

typedef struct __attribute__((packed)) {
	union {
		pycoproc_cmd_t sCmd;
		u8_t u8Cmd[sizeof(pycoproc_cmd_t)];
	};
	union {							// HW_VER
		u16_t u16HW_VER;
		u8_t u8HW_VER[2];
	};
	union {							// FW_VER
		u16_t u16FW_VER;
		u8_t u8FW_VER[2];
	};
	union {							// PROD_ID
		u16_t u16PROD_ID;
		u8_t u8PROD_ID[2];
	};
} pycoproc_reg_t;
DUMB_STATIC_ASSERT(sizeof(pycoproc_reg_t) == 12);

struct i2c_di_t;
typedef struct {					// SI70006/13/14/20/xx TMP & RH sensors
	struct i2c_di_t * psI2C;		// 4 bytes
	SemaphoreHandle_t mux;
	#if (pycoprocI2C_LOGIC == 3)
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
	union {
		pycoproc_reg_t sReg;
		u8_t u8Reg[sizeof(pycoproc_reg_t)];
	};
} pycoproc_t;
#if (pycoprocI2C_LOGIC == 1)
	DUMB_STATIC_ASSERT(sizeof(pycoproc_t) == 20);
#elif (pycoprocI2C_LOGIC == 3)
	DUMB_STATIC_ASSERT(sizeof(pycoproc_t) == 64);
#endif

// ###################################### Public variables #########################################

extern pycoproc_t sPCP;

// ###################################### Public functions #########################################

int pycoprocMagic(u8_t Oper, u16_t Addr, int Data);

int	pycoprocIdentify(struct i2c_di_t * psI2C);
int	pycoprocConfig(struct i2c_di_t * psI2C);
int	pycoprocReConfig(struct i2c_di_t * psI2C);
int	pycoprocDiags(struct i2c_di_t * psI2C);
struct report_t;
int pycoprocReportAll(struct report_t * psR);

#ifdef __cplusplus
}
#endif
