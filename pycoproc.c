/*
 * pycoproc.c - Copyright (c) 2022-24 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halHAS_PYCOPROC > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "printfx.h"
#include "pycoproc.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"
#include "x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugDEVICE					(debugFLAG & 0x0001)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################ Macros #############################################


// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################


// ###################################### Local variables ##########################################

pycoproc_t sPCP = { 0 };

// #################################### Local ONLY functions #######################################

u8_t pycoprocWait(void) {
	int Count = 0;
	u8_t Status;
	i64TaskDelayUsec(10);
	while(1) {
		xRtosSemaphoreTake(&sPCP.mux, portMAX_DELAY);
		IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
		halI2C_Queue(sPCP.psI2C, i2cR_B, NULL, 0, &Status, 1, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
		IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
		xRtosSemaphoreGive(&sPCP.mux);
		if (Status == 0xFF) break;
		if (++Count > 500) { SL_ERR("Timeout"); break; }
		i64TaskDelayUsec(100);
	}
	return Status;
}

int pycoprocRead16(u8_t Reg, u8_t * pRxBuf) {
	xRtosSemaphoreTake(&sPCP.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	int iRV = halI2C_Queue(sPCP.psI2C, i2cWR_B, &Reg, 1, pRxBuf, 2, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPCP.mux);
	return iRV;
}

/*
int pycoprocWriteMemory(u16_t Addr, u8_t Data) {
	u8_t u8Buf[4] ;
	u8Buf[0] = pycoprocCMD_POKE;
	u8Buf[1] = (Addr & 0xFF);
	u8Buf[2] = (Addr >> 8) & 0xFF;
	u8Buf[3] = Data;
	xRtosSemaphoreTake(&sPCP.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	int iRV = halI2C_Queue(sPCP.psI2C, i2cW_B, u8Buf, sizeof(u8Buf),
			NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPCP.mux);
	return iRV;
}
*/

int pycoprocMagic(u8_t Oper, u16_t Addr, int Data) {
	size_t TxLen;
	Data &= 0xFF;
	sPCP.sReg.sCmd.ADDRL = (Addr & 0xFF);
	sPCP.sReg.sCmd.ADDRH = (Addr >> 8) & 0xFF;
	if (Oper == pycoprocMAGIC_OP_PEEK) {
		sPCP.sReg.sCmd.CMD = pycoprocCMD_PEEK;
		TxLen = 3;
	} else if (Oper == pycoprocMAGIC_OP_POKE) {
		sPCP.sReg.sCmd.CMD = pycoprocCMD_POKE;
		sPCP.sReg.sCmd._DATA = Data;
		TxLen = 4;
	} else {
		sPCP.sReg.sCmd.CMD = pycoprocCMD_MAGIC;
		sPCP.sReg.sCmd._AND = (Oper == pycoprocMAGIC_OP_CLR_BITS) ? (Data & 0xFF) : 0xFF;
		sPCP.sReg.sCmd._OR = (Oper == pycoprocMAGIC_OP_SET_BITS) ? (Data & 0xFF) : 0x00;
		sPCP.sReg.sCmd._XOR = (Oper == pycoprocMAGIC_OP_TGL_BITS) ? (Data & 0xFF) : 0x00;
		TxLen = 6;
	}
	xRtosSemaphoreTake(&sPCP.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	int iRV = halI2C_Queue(sPCP.psI2C, i2cW_B, sPCP.sReg.u8Cmd, TxLen, NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPCP.mux);
	if (iRV < erSUCCESS) goto exit;

	iRV = pycoprocWait() == 0xFF ? erSUCCESS : erFAILURE;
	if (iRV > erFAILURE && Oper == pycoprocMAGIC_OP_PEEK) {
		u8_t u8Buf[2];
		iRV = halI2C_Queue(sPCP.psI2C, i2cR_B, NULL, 0, u8Buf, 2, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
		if (iRV < erSUCCESS) goto exit;
		return sPCP.sReg.sCmd._RESULT = u8Buf[1];
	}
exit:
	return iRV;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	pycoprocIdentify(i2c_di_t * psI2C) {
	sPCP.psI2C = psI2C;
	psI2C->Type	= i2cDEV_PYCOPROC;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 25;
	psI2C->Test = 1;
	int iRV = pycoprocRead16(pycoprocCMD_FW_VER, sPCP.sReg.u8FW_VER);
	if (iRV < erSUCCESS) goto exit;

	sPCP.sReg.u16FW_VER = (sPCP.sReg.u8FW_VER[1] << 8) | sPCP.sReg.u8FW_VER[0];
	if (sPCP.sReg.u16FW_VER < 6) goto err_version;

	iRV = pycoprocRead16(pycoprocCMD_HW_VER, sPCP.sReg.u8HW_VER);
	if (iRV < erSUCCESS) goto exit;

	sPCP.sReg.u16HW_VER = (sPCP.sReg.u8HW_VER[1] << 8) | sPCP.sReg.u8HW_VER[0];
	iRV = pycoprocRead16(pycoprocCMD_PROD_ID, sPCP.sReg.u8PROD_ID);
	if (iRV < erSUCCESS) goto exit;

	sPCP.sReg.u16PROD_ID = (sPCP.sReg.u8PROD_ID[1] << 8) | sPCP.sReg.u8PROD_ID[0];
	sPCP.psI2C->IDok = 1;
	psI2C->Test = 0;
	SL_DBG("FW_VER=%d  HW_VER=%d  PROD_ID=%d", sPCP.sReg.u16FW_VER, sPCP.sReg.u16HW_VER, sPCP.sReg.u16PROD_ID);
	goto exit;
err_version:
	iRV = erINV_VERSION;
exit:
	return iRV;
}

int	pycoprocConfig(i2c_di_t * psI2C) {
	if (!psI2C->IDok) return erINV_STATE;

	psI2C->CFGok = 0;
	// init ADC for battery measurement
	int iRV = pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_ANSELC, 1 << 2);
	if (iRV < erSUCCESS) goto exit;

	iRV = pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_ADCON0, (0x06 << pycoproc_ADCON0_CHS_POSN) | pycoprocADCON0_ADON_MASK);
	if (iRV < erSUCCESS) goto exit;

	iRV = pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_ADCON1, (0x06 << pycoproc_ADCON0_CHS_POSN));
	if (iRV < erSUCCESS) goto exit;

	// enable pull-up on RA3
	iRV = pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_WPUA, 1 << 3);
	if (iRV < erSUCCESS) goto exit;

	// make RC5 an input
	iRV = pycoprocMagic(pycoprocMAGIC_OP_SET_BITS, pycoprocADDR_TRISC, 1 << 5);
	if (iRV < erSUCCESS) goto exit;

	// set RC6 and RC7 as outputs and enable power to the sensors and the GPS
	iRV = pycoprocMagic(pycoprocMAGIC_OP_CLR_BITS, pycoprocADDR_TRISC, ~(1 << 6));
	if (iRV < erSUCCESS) goto exit;

	iRV = pycoprocMagic(pycoprocMAGIC_OP_CLR_BITS, pycoprocADDR_TRISC, ~(1 << 7));
	if (iRV < erSUCCESS) goto exit;

	psI2C->CFGok = 1;
	// once off init....
	if (!psI2C->CFGerr) {
		IF_SYSTIMER_INIT(debugTIMING, stPYCOPROC, stMICROS, "PyCoProc", 100, 5000);
		#if (pycoprocI2C_LOGIC == 3)
		sPCP.th = xTimerCreateStatic("pycoproc", pdMS_TO_TICKS(5), pdFALSE, NULL, pycoprocTimerHdlr, &sPCP.ts );
		#endif
	}
exit:
	return erSUCCESS;
}

int	pycoprocDiags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

int pycoprocReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sPCP.psI2C);
	#if (pycoprocI2C_LOGIC == 3)
	iRV += xRtosReportTimer(psR, sPCP.th);
	#endif
	return iRV;
}

#endif
