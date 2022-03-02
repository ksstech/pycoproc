/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"pycoproc.h"
#include	<string.h>

#include	"hal_variables.h"
#include	"endpoints.h"
#include	"options.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"
#include	"x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugCONVERT				(debugFLAG & 0x0002)
#define	debugMAGIC					(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################ Macros #############################################

#define	pycoprocI2C_LOGIC			1					// 0 = delay, 1= stretch, 2= stages
#define	pycoprocADDR				0x08
#define	PYCOPROC_T_SNS				10000

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################


// ###################################### Local variables ##########################################

pycoproc_t sPYCOPROC = { 0 };

// #################################### Local ONLY functions #######################################

uint8_t pycoprocWait(void) {
	int Count = 0;
	uint8_t Status;
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

int pycoprocRead16(uint8_t Reg, uint8_t * pRxBuf) {
	xRtosSemaphoreTake(&sPYCOPROC.mux, portMAX_DELAY);
	IF_SYSTIMER_START(debugTIMING, stPYCOPROC);
	int iRV = halI2C_Queue(sPYCOPROC.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, sizeof(uint16_t), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPYCOPROC);
	xRtosSemaphoreGive(&sPYCOPROC.mux);
	return iRV;
}

/*
int pycoprocWriteMemory(uint16_t Addr, uint8_t Data) {
	uint8_t u8Buf[4] ;
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

int pycoprocMagic(uint8_t Oper, uint16_t Addr, int Data) {
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
	IF_PT(debugMAGIC, "MAGIC: C=%d  A=0x%04X  D=0x%02X  [%-B]  ", Oper, Addr, Data, TxLen, sPYCOPROC.sReg.u8Cmd);
	halI2C_Queue(sPYCOPROC.psI2C, i2cW_B, sPYCOPROC.sReg.u8Cmd, TxLen, NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	pycoprocWait();
	if (Oper == pycoprocMAGIC_OP_PEEK) {
		uint8_t u8Buf[2];
		halI2C_Queue(sPYCOPROC.psI2C, i2cR_B, NULL, 0, u8Buf, 2, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
		IF_P(debugMAGIC, "R=[%-B]\n", 2, u8Buf);
		sPYCOPROC.sReg.sCmd._RESULT = u8Buf[1];
	} else {
		IF_P(debugMAGIC, "\n");
	}
	return sPYCOPROC.sReg.sCmd._RESULT;
}

#if (pycoprocI2C_LOGIC == 1)		// read and convert in 1 go...

int	pycoprocReadHdlr(epw_t * psEWP) {
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
	vCV_SetValue(&psEWP->var, X64);
	IF_PTL(debugCONVERT, " Raw=%d  Norm=%f\n", X64.x32[1].u32, X64.x32[0].f32);
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
	uint8_t	AI = psR->ActIdx;
	int gain = psR->para.x32[AI][0].i32;
	int time = psR->para.x32[AI][1].i32;
	int rate = psR->para.x32[AI][2].i32;
	IF_P(debugCONFIG && ioB1GET(ioMode), "mode 'PYCOPROC' Xcur=%d Xmax=%d gain=%d time=%d rate=%d\n", Xcur, Xmax, gain, time, rate);

	if (OUTSIDE(0, gain, 7, int) || OUTSIDE(0, time, 7, int) || OUTSIDE(0, rate, 7, int) || gain==4 || gain==5) {
		ERR_RETURN("Invalid gain / time / rate specified", erINVALID_PARA);
	}
	int iRV = erSUCCESS;
	return iRV;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	pycoprocIdentify(i2c_di_t * psI2C_DI) {
	psI2C_DI->TRXmS	= 50;
	psI2C_DI->CLKuS = 400;
	psI2C_DI->Test = 1;
	sPYCOPROC.psI2C = psI2C_DI;

	int iRV = pycoprocRead16(pycoprocCMD_FW_VER, sPYCOPROC.sReg.u8FW_VER);
	sPYCOPROC.sReg.u16FW_VER = (sPYCOPROC.sReg.u8FW_VER[1] << 8) | sPYCOPROC.sReg.u8FW_VER[0];
	if (iRV != erSUCCESS)
		goto exit;
	if (sPYCOPROC.sReg.u16FW_VER < 6)
		goto exit_err;

	iRV = pycoprocRead16(pycoprocCMD_HW_VER, sPYCOPROC.sReg.u8HW_VER);
	sPYCOPROC.sReg.u16HW_VER = (sPYCOPROC.sReg.u8HW_VER[1] << 8) | sPYCOPROC.sReg.u8HW_VER[0];
	if (iRV != erSUCCESS)
		goto exit;

	iRV = pycoprocRead16(pycoprocCMD_PROD_ID, sPYCOPROC.sReg.u8PROD_ID);
	sPYCOPROC.sReg.u16PROD_ID = (sPYCOPROC.sReg.u8PROD_ID[1] << 8) | sPYCOPROC.sReg.u8PROD_ID[0];
	if (iRV != erSUCCESS)
		goto exit;

	IF_P(debugCONFIG, "FW_VER=%d  HW_VER=%d  PROD_ID=%d\n", sPYCOPROC.sReg.u16FW_VER, sPYCOPROC.sReg.u16HW_VER, sPYCOPROC.sReg.u16PROD_ID);
	psI2C_DI->Type		= i2cDEV_PYCOPROC;
	psI2C_DI->Speed		= i2cSPEED_400;
	psI2C_DI->DevIdx 	= 0;
	goto exit;
exit_err:
	iRV = erFAILURE;
exit:
	psI2C_DI->Test = 0;
	return iRV ;
}

int	pycoprocConfig(i2c_di_t * psI2C_DI) {
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
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = PYCOPROC_T_SNS;
	psEWP->uri = URI_PYCOPROC;

#if (pycoprocI2C_LOGIC == 3)
	sPYCOPROC.timer = xTimerCreate("pycoproc", pdMS_TO_TICKS(5), pdFALSE, NULL, pycoprocTimerHdlr);
#endif
	IF_SYSTIMER_INIT(debugTIMING, stPYCOPROC, stMICROS, "PyCoProc", 100, 5000);
	return erSUCCESS ;
}

int pycoprocReConfig(i2c_di_t * psI2C_DI) { return erSUCCESS; }

int	pycoprocDiags(i2c_di_t * psI2C_DI) { return erSUCCESS; }

// ######################################### Reporting #############################################

void pycoprocReportAll(void) {
	halI2C_DeviceReport(sPYCOPROC.psI2C);
}
