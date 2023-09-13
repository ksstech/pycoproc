/*
 * pycoproc.c - Copyright (c) 2022-23 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_variables.h"

#if (halHAS_PYCOPROC > 0)
#include "hal_i2c_common.h"
#include "printfx.h"
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

pycoproc_t sPYCOPROC = { 0 };

// #################################### Local ONLY functions #######################################

u8_t pycoprocWait(void) {
	int Count = 0;
	u8_t Status;
	i64TaskDelayUsec(10);
	while(1) {
		halI2C_Queue(sPYCOPROC.psI2C, i2cR_B, NULL, 0, &Status, 1, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
		if (Status == 0xFF)
			break;
		if (++Count > 500) {
			SL_ERR("Timeout");
			break;
		}
		i64TaskDelayUsec(100);
	}
	return Status;
}

int pycoprocRead16(u8_t Reg, u8_t * pRxBuf) {
	xRtosSemaphoreTake(&sPYCOPROC.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	int iRV = halI2C_Queue(sPYCOPROC.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, sizeof(uint16_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPYCOPROC.mux);
	return iRV;
}

/*
int pycoprocWriteMemory(uint16_t Addr, u8_t Data) {
	u8_t u8Buf[4] ;
	u8Buf[0] = pycoprocCMD_POKE;
	u8Buf[1] = (Addr & 0xFF);
	u8Buf[2] = (Addr >> 8) & 0xFF;
	u8Buf[3] = Data;
	xRtosSemaphoreTake(&sPYCOPROC.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	int iRV = halI2C_Queue(sPYCOPROC.psI2C, i2cW_B, u8Buf, sizeof(u8Buf),
			NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPYCOPROC.mux);
	return iRV;
}
*/

int pycoprocMagic(u8_t Oper, uint16_t Addr, int Data) {
	size_t TxLen;
	Data &= 0xFF;
	sPYCOPROC.sReg.sCmd.ADDRL = (Addr & 0xFF);
	sPYCOPROC.sReg.sCmd.ADDRH = (Addr >> 8) & 0xFF;
	if (Oper == pycoprocMAGIC_OP_PEEK) {
		sPYCOPROC.sReg.sCmd.CMD = pycoprocCMD_PEEK;
		TxLen = 3;
	} else if (Oper == pycoprocMAGIC_OP_POKE) {
		sPYCOPROC.sReg.sCmd.CMD = pycoprocCMD_POKE;
		sPYCOPROC.sReg.sCmd._DATA = Data;
		TxLen = 4;
	} else {
		sPYCOPROC.sReg.sCmd.CMD = pycoprocCMD_MAGIC;
		sPYCOPROC.sReg.sCmd._AND = (Oper == pycoprocMAGIC_OP_CLR_BITS) ? (Data & 0xFF) : 0xFF;
		sPYCOPROC.sReg.sCmd._OR = (Oper == pycoprocMAGIC_OP_SET_BITS) ? (Data & 0xFF) : 0x00;
		sPYCOPROC.sReg.sCmd._XOR = (Oper == pycoprocMAGIC_OP_TGL_BITS) ? (Data & 0xFF) : 0x00;
		TxLen = 6;
	}
	IF_PT(debugDEVICE, "MAGIC: C=%d  A=0x%04X  D=0x%02X  [%-B]  ", Oper, Addr, Data, TxLen, sPYCOPROC.sReg.u8Cmd);
	halI2C_Queue(sPYCOPROC.psI2C, i2cW_B, sPYCOPROC.sReg.u8Cmd, TxLen, NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	pycoprocWait();
	if (Oper == pycoprocMAGIC_OP_PEEK) {
		u8_t u8Buf[2];
		halI2C_Queue(sPYCOPROC.psI2C, i2cR_B, NULL, 0, u8Buf, 2, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
		IF_PX(debugDEVICE, "R=[%-B]\r\n", 2, u8Buf);
		sPYCOPROC.sReg.sCmd._RESULT = u8Buf[1];
	} else {
		IF_P(debugDEVICE, strCRLF);
	}
	return sPYCOPROC.sReg.sCmd._RESULT;
}

#if (pycoprocI2C_LOGIC == 1)		// read and convert in 1 go...

int	pycoprocSense(epw_t * psEWP) {
	xRtosSemaphoreTake(&sPYCOPROC.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	pycoprocMagic(pycoprocMAGIC_OP_SET_BITS, pycoprocADDR_ADCON0, pycoprocADCON0_GO_nDONE_MASK);
	i64TaskDelayUsec(50);
	while (1) {
		pycoprocMagic(pycoprocCMD_PEEK, pycoprocADDR_ADCON0, 0);
		if ((sPYCOPROC.sReg.sCmd._RESULT & pycoprocADCON0_GO_nDONE_MASK) == 0) {
			break;
		}
		i64TaskDelayUsec(100);
	}
	x64_t X64;
	pycoprocMagic(pycoprocCMD_PEEK, pycoprocADDR_ADRESH, 0);
	X64.x32[1].u32 = sPYCOPROC.sReg.sCmd._RESULT << 2;
	pycoprocMagic(pycoprocCMD_PEEK, pycoprocADDR_ADRESL, 0);
	X64.x32[1].u32 |= sPYCOPROC.sReg.sCmd._RESULT >> 6;
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPYCOPROC.mux);
	// calculate actual voltage measured
	X64.x32[0].f32 = ((((float) X64.x32[1].u32 * 3.3 * 280.0) / 1023) / 180.0) + 0.01;
	vCV_SetValueRaw(&psEWP->var, X64);
	IF_PTL(debugDEVICE, " Raw=%d  Norm=%f\r\n", X64.x32[1].u32, X64.x32[0].f32);
	return erSUCCESS;
}

#elif (pycoprocI2C_LOGIC == 2)		// clock stretching

	#error "Not supported"

#elif (pycoprocI2C_LOGIC == 3)		// 3 step read -> wait -> convert

	#error "Not supported"

#endif

// ################################ Rules configuration support ####################################

int	pycoprocConfigMode (struct rule_t * psR, int Xcur, int Xmax) {
	// mode /pycoproc idx gain time rate
	u8_t	AI = psR->ActIdx;
	int gain = psR->para.x32[AI][0].i32;
	int time = psR->para.x32[AI][1].i32;
	int rate = psR->para.x32[AI][2].i32;
	IF_P(debugTRACK && ioB1GET(dbgMode), "mode 'PYCOPROC' Xcur=%d Xmax=%d gain=%d time=%d rate=%d\r\n", Xcur, Xmax, gain, time, rate);

	if (OUTSIDE(0, gain, 7) || OUTSIDE(0, time, 7) || OUTSIDE(0, rate, 7) || gain==4 || gain==5)
		RETURN_MX("Invalid gain / time / rate specified", erINV_PARA);
	// Add actual MODE support here
	return erSUCCESS;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	pycoprocIdentify(i2c_di_t * psI2C) {
	psI2C->TRXmS = 50;
	psI2C->CLKuS = 400;
	psI2C->Test = 1;
	sPYCOPROC.psI2C = psI2C;

	int iRV = pycoprocRead16(pycoprocCMD_FW_VER, sPYCOPROC.sReg.u8FW_VER);
	sPYCOPROC.sReg.u16FW_VER = (sPYCOPROC.sReg.u8FW_VER[1] << 8) | sPYCOPROC.sReg.u8FW_VER[0];
	IF_EXIT(iRV != erSUCCESS);
	IF_GOTO_L(sPYCOPROC.sReg.u16FW_VER < 6, exit_err);

	iRV = pycoprocRead16(pycoprocCMD_HW_VER, sPYCOPROC.sReg.u8HW_VER);
	sPYCOPROC.sReg.u16HW_VER = (sPYCOPROC.sReg.u8HW_VER[1] << 8) | sPYCOPROC.sReg.u8HW_VER[0];
	IF_EXIT(iRV != erSUCCESS);

	iRV = pycoprocRead16(pycoprocCMD_PROD_ID, sPYCOPROC.sReg.u8PROD_ID);
	sPYCOPROC.sReg.u16PROD_ID = (sPYCOPROC.sReg.u8PROD_ID[1] << 8) | sPYCOPROC.sReg.u8PROD_ID[0];
	IF_EXIT(iRV != erSUCCESS);

	IF_P(debugTRACK && ioB1GET(ioI2Cinit), "FW_VER=%d  HW_VER=%d  PROD_ID=%d\r\n", sPYCOPROC.sReg.u16FW_VER, sPYCOPROC.sReg.u16HW_VER, sPYCOPROC.sReg.u16PROD_ID);
	psI2C->Type	= i2cDEV_PYCOPROC;
	psI2C->Speed = i2cSPEED_400;
	psI2C->DevIdx = 0;
	goto exit;
exit_err:
	iRV = erFAILURE;
exit:
	psI2C->Test = 0;
	return iRV ;
}

int	pycoprocConfig(i2c_di_t * psI2C) {
	IF_SYSTIMER_INIT(debugTIMING, stPYCOPROC, stMICROS, "PyCoProc", 100, 5000);
	#if (pycoprocI2C_LOGIC == 3)
	sPYCOPROC.th = xTimerCreateStatic("pycoproc", pdMS_TO_TICKS(5), pdFALSE, NULL, pycoprocTimerHdlr, &sPYCOPROC.ts );
	#endif
	return pycoprocReConfig(psI2C);
}

int pycoprocReConfig(i2c_di_t * psI2C) {
	// init ADC for battery measurement
	pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_ANSELC, 1 << 2);
	pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_ADCON0, (0x06 << pycoproc_ADCON0_CHS_POSN) | pycoprocADCON0_ADON_MASK);
	pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_ADCON1, (0x06 << pycoproc_ADCON0_CHS_POSN));
	// enable pull-up on RA3
	pycoprocMagic(pycoprocMAGIC_OP_POKE, pycoprocADDR_WPUA, 1 << 3);
	// make RC5 an input
	pycoprocMagic(pycoprocMAGIC_OP_SET_BITS, pycoprocADDR_TRISC, 1 << 5);
	// set RC6 and RC7 as outputs and enable power to the sensors and the GPS
	pycoprocMagic(pycoprocMAGIC_OP_CLR_BITS, pycoprocADDR_TRISC, ~(1 << 6));
	pycoprocMagic(pycoprocMAGIC_OP_CLR_BITS, pycoprocADDR_TRISC, ~(1 << 7));

	epw_t * psEWP = &table_work[URI_PYCOPROC];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvF32, 1, 0);
	psEWP->Tsns = psEWP->Rsns = PYCOPROC_T_SNS;
	psEWP->uri = URI_PYCOPROC;
	xRtosSetDevice(devMASK_PYCOPROC);
	return erSUCCESS;
}

int	pycoprocDiags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

int pycoprocReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sPYCOPROC.psI2C);
	#if (pycoprocI2C_LOGIC == 3)
	iRV += xRtosReportTimer(psR, sPYCOPROC.th);
	#endif
	return iRV;
}
#endif
