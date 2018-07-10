/**
 *   @file  loopbackDioIsr.c
 *
 *   @brief   
 *      This is an example application which shows how DIO transfer
 *      completion interrupts can be registered and serviced.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2010-2012 Texas Instruments, Inc.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  \par   Memory_alloc
*/
#include <xdc/std.h>
#include <string.h>
#include <c6x.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <xdc/runtime/Types.h>//hxl added it for test cpu fe=req

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
//#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h> 
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>

#include <xdc/cfg/global.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>
#include <ti/drv/srio/srio_osal.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
//#include <ti/drv/qmss/qmss_firmware.h> //2016-6-27 hxl deleted
/* Resource manager for QMSS, PA, CPPI */
#include <ti/platform/resource_mgr.h> //2016-6-27 hxl:added

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>

/* CSL Cache Functional Layer */
#include <ti/csl/csl_cacheAux.h>

/* PSC CSL Include Files */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>

/* CSL CPINTC Include Files. */
#include<ti/csl/csl_cpIntc.h>
#include<ti/csl/csl_cpintcAux.h>//hxl:hxl added it 2017-5-4
#include <ti/csl/src/intc/cslr_intc.h>//hxl:hxl added it 2017-5-5
#include <ti/csl/src/intc/csl_intc.h>//hxl:hxl added it 2017-5-11
/*----------------hxl added the following headers----------------------*/
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/family/c66/Cache.h>//hxl:2016-8-4
/* Platform utilities include */
#include <ti/platform/platform.h>
//#include <ti/sysbios/knl/Semaphore.h>////2016-2-29:hxl: added;//2016-6-30 hxl added it to client.h

#include "SRIO_loopbackDioIsr.h"
#include "resourcemgr_aided_hxl.h"//2016-2-29:hxl: added
#include "client.h"////2016-2-29:hxl: added
#include "device_srio_loopback.h"
#include "SRIO_MSM_init.h"
#include "srio_drv_hxl.h"//hxl:2017-6-29 added
#include "test.h"
#include "processTask.h"////2017-11-27:hxl: added
/*----end------------hxl added the following headers----------------------*/

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/
/*hxl moved to SRIO_loopbackDioIsr.h*/

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
//#define Num_line     800
//#define Point_image  333

#pragma DATA_SECTION(tx_rx_distance, ".shared")//将数据分配到MCM中，模拟FPGA中传来的回波数据
#pragma DATA_ALIGN (tx_rx_distance, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
short tx_rx_distance[Num_line][Point_image]={{0},{0},{0},{0}};//hxl:int 型4个字节

#pragma DATA_SECTION(testData, ".sharedVar")//将数据分配到MCM中，模拟FPGA中传来的回波数据
#pragma DATA_ALIGN (testData, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile short testData[7][4096]={{0},{0},{0},{0}};//hxl:int 型4个字节

#pragma DATA_SECTION(outData, ".sharedVar1")//将数据分配到MCM中，模拟FPGA中传来的回波数据
#pragma DATA_ALIGN (outData, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile short outData[7][4096]={{0},{0},{0},{0}};//hxl:int 型4个字节

#pragma DATA_SECTION(ready_go_x, ".sharedVar1")//将数据分配到MCM中，模拟FPGA中传来的回波数据
#pragma DATA_ALIGN (ready_go_x, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile short ready_go_x[Num_line*Point_image]={{0},{0},{0},{0}};//hxl:int 型4个字节

#pragma DATA_SECTION(ready_go_y, ".sharedVar1")//将数据分配到MCM中，模拟FPGA中传来的回波数据
#pragma DATA_ALIGN (ready_go_y, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile short ready_go_y[Num_line*Point_image]={{0},{0},{0},{0}};//hxl:int 型4个字节

/*-------hxl--------回波数据存放的数组-----*/
#pragma DATA_SECTION(fpgaData0, ".fpgaData0")//将数据分配到MCM中，模拟FPGA中传来的回波数据
#pragma DATA_ALIGN (fpgaData0, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
//volatile short  fpgaData0[TESTSIZE]={1,3,5,7,9};//hxl:int 型4个字节.volatile要求编译器直接从内存存取fpgaData
 short  fpgaData0[TOTALPOINTS]={1,3,5,7,9};//hxl:int 型4个字节.volatile要求编译器直接从内存存取fpgaData
#pragma DATA_SECTION(fpgaData1, ".fpgaData1")//将数据分配到MCM中，模拟FPGA传输而来的回波数据
#pragma DATA_ALIGN (fpgaData1, 8)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
//volatile short  fpgaData1[TESTSIZE]={2,4,6,8,10};//data0[1][16384+68]={0};//hxl:int 型4个字节
volatile short  fpgaData1[TOTALPOINTS]={2,4,6,8,10};//data0[1][16384+68]={0};//hxl:int 型4个字节

#pragma DATA_SECTION(beforeData, ".sharedVar1")//模拟读取FPGA中的回波数据用于信号处理的数组
#pragma DATA_ALIGN (beforeData, 128)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
//volatile short beforeData[LINENUM_BUF][TESTSIZE]={{1,2,3,},{4,5,6},{7,8,9}};//hxl:由SRIO接收的数据缓存
short beforeData[LINENUM_BUF][TOTALPOINTS]={{1,2,3,},{4,5,6},{7,8,9}};//hxl:由SRIO接收的数据缓存
/*---end----hxl----回波数据存放的数组-----*/


/*--hxl: vars added for function dioSockets_SW_DoorbellIsr_test1-----*/
#pragma DATA_SECTION(srcData, ".ParasPCMem")//模拟读取FPGA中的回波数据用于信号处理的数组
#pragma DATA_ALIGN (srcData, 2)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile unsigned short srcData[TOTALPOINTS*2]={0,1,2,3};//hxl:short占2 Bytes  for test,FPGA原始数据数组
//#pragma DATA_SECTION(dataToProcs, ".shareVar1")//模拟读取FPGA中的回波数据用于信号处理的数组
//#pragma DATA_ALIGN (dataToProcs, 2)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
//volatile unsigned short dataToProcs[TestSize*2]={0,1,2,3};//hxl:short占2 Bytes  for test,FPGA原始数据数组
#pragma DATA_SECTION(dataToProcs0, ".sharedVar1")//模拟读取FPGA中的回波数据用于信号处理的数组
#pragma DATA_ALIGN (dataToProcs0, 2)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile  short dataToProcs0[TOTALPOINTS]={0,1,2,3};//hxl:short占2 Bytes  for test,FPGA原始数据数组

#pragma DATA_SECTION(dataToProcs1, ".sharedVar1")//模拟读取FPGA中的回波数据用于信号处理的数组
#pragma DATA_ALIGN (dataToProcs1, 2)//hxl:指定data0变量的地址放在128字节对齐的位置上，即其地址的低7位为0；
volatile  short dataToProcs1[TOTALPOINTS]={0,1,2,3};//hxl:short占2 Bytes  for test,FPGA原始数据数组

#pragma DATA_SECTION(beforeData_test, ".sharedVar1")//将数据分配到DDR中，模拟FPGA传输而来的回波数据
#pragma DATA_ALIGN (beforeData_test, 8)//hxl:
volatile short beforeData_test[TOTALPOINTS]={0};//用于验证SRIO传输来的数据线的数据是否正确；for test
/*-end -hxl: vars added for function dioSockets_SW_DoorbellIsr_test1-----*/

/* Memory allocated for the descriptors. This is 16 bit aligned. */
//#pragma DATA_ALIGN (host_region, 16) //2016-6-27 hxl: deleted;
//Uint8   host_region[NUM_HOST_DESC * SIZE_HOST_DESC];//2016-6-27 hxl: deleted;

/* Memory used for the accumulator list. */
#pragma DATA_ALIGN (gHiPriAccumList_SRIO, 16)
UInt32              gHiPriAccumList_SRIO[64];

/* Global SRIO and QMSS Configuration */
Qmss_InitCfg   qmssInitConfig;

/* Global Varialble which keeps track of the core number executing the
 * application. */
UInt32          coreNum = 0xFFFF;

/* Shared Memory Variable to ensure synchronizing SRIO initialization
 * with all the other cores. */
/* Created an array to pad the cache line with SRIO_MAX_CACHE_ALIGN size */
#pragma DATA_ALIGN   (isSRIOInitialized, 128)
#pragma DATA_SECTION (isSRIOInitialized, ".srioSharedMem");
volatile Uint32     isSRIOInitialized[(SRIO_MAX_CACHE_ALIGN / sizeof(Uint32))] = { 0 };

Srio_DrvConfig  drvCfg;//hxl:added for SRIO driver
Srio_DrvHandle  hDrvManagedSrioDrv;
CSL_SrioHandle  hSrioCSL;

/* These are the device identifiers used in the Example Application */
const uint32_t DEVICE_ID1_16BIT    = 0x00AB;//0xBEEF;
const uint32_t DEVICE_ID1_8BIT     = 0xAB;//0xAB;hxl modified

const uint32_t DEVICE_ID2_16BIT    = 0x00FF;///hxl:For FPGA devive ID
const uint32_t DEVICE_ID2_8BIT     = 0xFF;//hxl:For FPGA device ID

const uint32_t DEVICE_ID3_16BIT    = 0x00FF;//hxl modified
const uint32_t DEVICE_ID3_8BIT     = 0xFF;//hxl modified
const uint32_t DEVICE_ID4_16BIT    = 0x00FF;//0x5678;hxl modified
const uint32_t DEVICE_ID4_8BIT     = 0xFF;//0x56;hxl modified
/*
const uint32_t DEVICE_ID2_16BIT    = 0x00AB;//hxl:For DSP devive ID
const uint32_t DEVICE_ID2_8BIT     = 0xAB;//hxl:For DSP device ID*/
/*----hxl added the following const-------------------*/



/* Source and Destination Data Buffers (payload buffers) */
UInt8* srcDataBuffer[2];

volatile short* ptr_DataBufferToFPGA;//hxl added this
volatile short * ptr_FPGAData=fpgaData1;
/* Global debug variable to track number of ISRs raised */
volatile UInt32 srioDioIsrCnt = 0;
/* Platform Information - we will read it form the Platform Library */
platform_info	gPlatformInfo;//

/* Global variable to indicate completion of ISR processing */
volatile UInt32 srioLsuIsrServiced = 0;


/*------------hxl added the following var-----------------------------------------*/
//----hxl added------------
Semaphore_Handle mysem1, mysem2;//定义一个semaphore handle
//ParasFromPC_Type  paras_PC_type=MainCtrl;//TransmitDely; //MainCtrl;//NoParas;// MainCtrl;
Uint8 intDstDoorbell[]={0};//doorBell 寄存器映射到DST0
Srio_SockHandle srioSocket_Doorbl, srioSocket_DIO_Sw;
volatile UInt32 srioDoorblIsrServiced=0;
volatile UInt32 srioDoorblIsrCnt=0; //hxl:added it for test; modified it

volatile UInt32 srioDioDoorbellGoodTransfers=0;
volatile UInt32 srioDioDoorbellBadTransfers  = 0;
UInt8* dstDoorbellBuffer[1]={0};
UInt8* srcDoorbellBuffer[1]={0};

volatile int Flag_EvenOdd=0;//hxl added this for test
volatile UInt16 WriteToIndex=0;
UInt16 flag_OverFlow=0;
UInt16 semCnt=0;
int jj=0;//hxl for test
CSL_SrioHandle      hSrio_test;//hxl:for test
unsigned long long int startTime1,t1,ts[]={0,0,0,0,0,0,0,0,0,0};//hxl modified for doorbell interval

volatile Int32 num_errLine=0;
volatile Int32 linth=0;//for FPGA data validation test
volatile  short* uipSrc0=dataToProcs0; //hxl added for validate data from FPGA
volatile  short* uipSrc1=dataToProcs1; //hxl added for validate data from FPGA
//volatile unsigned short* uipSrc=dataToProcs; //hxl added for validate data from FPGA
volatile short*uipDst=beforeData_test;//
Uint16 counter; //hxl added for validate data from FPGA;
Types_FreqHz cpuFreq;

/*------------hxl added the following function declare------------------------------*/
void myDoorblComeIsr(UArg argument);
/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/
extern UInt32 malloc_counter;
extern UInt32 free_counter;
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/* OSAL Data Buffer Memory Initialization. */
extern int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);


/**********************************************************************
 ************************ SRIO EXAMPLE FUNCTIONS **********************
 **********************************************************************/
/*hxl added the following functions*/
/**********************************************************************
 * the following function set fpgadata0 and fpgadata1 in the MSM to non cachable
 * 2016-8-4 not applicable now
*********************************************************************/
static void CACHE_init(void)/*-hxl--仅在该。c文件内部调用----*/
{
	//UInt32 MarRg;
	Cache_Mode mode_LDCache;
	//MarRg=Cache_getMar((void*)fpgaData0);
	mode_LDCache=Cache_getMode(Cache_Type_L2D);//hxl:normal
	mode_LDCache=Cache_getMode(Cache_Type_L1D);//hxl:normal
	//Cache_inv((void*)fpgaData0, 64, Cache_Type_ALLD, 1);//hxl;
	/*hxl:set fpgadata0 to non prefetchable; MSM's cachablility by L1D is not configurable
	 * except MSM is aliases configured with XMC's MPAX range*/

}

/**
 *  @b Description
 *  @n  
 *      Utility function which converts a local address to global.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Global Address
 */
 UInt32 l2_global_address (Uint32 addr)
{
	UInt32 corenum;

	/* Get the core number. */
	corenum = CSL_chipReadReg(CSL_CHIP_DNUM); 

	/* Compute the global address. */
	return (addr + (0x10000000 + (corenum*0x1000000)));
}

/**
 *  @b Description
 *  @n  
 *      Utility function that is required by the IPC module to set the proc Id.
 *      The proc Id is set via this function instead of hard coding it in the .cfg file
 *
 *  @retval
 *      Not Applicable.
 */
Void myStartupFxn (Void)
{
	MultiProc_setLocalId (CSL_chipReadReg (CSL_CHIP_DNUM));
}

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for SRIO. 
 *
 *  @retval
 *      Not Applicable.
 */
//static Int32 enable_srio (void) //hxl:使能SRIO的电源模块和时钟源
 Int32 enable_srio (void) //hxl:使能SRIO的电源模块和时钟源
{
#ifndef SIMULATOR_SUPPORT

	/****hxl added the following code*******************************************************************/
	 /* Return SRIO PSC status */
	if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_SRIO) == PSC_PDSTATE_ON) )
	{
		/* SRIO off.  */
		/* Set SRIO Power domain to Off */
		CSL_PSC_disablePowerDomain (CSL_PSC_PD_SRIO);
		/* Start the state transition */
		CSL_PSC_startStateTransition (CSL_PSC_PD_SRIO);
		/* Wait until the state transition process is completed. */
		while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_SRIO));
	}

	/*end***hxl added the following code*******************************************************************/
	 /* SRIO power domain is turned OFF by default. It needs to be turned on before doing any
     * SRIO device register access. This not required for the simulator. */

    /* Set SRIO Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_SRIO);

    /* Enable the clocks too for SRIO */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_SRIO, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_SRIO);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_SRIO));

    /* Return SRIO PSC status */
    if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_SRIO) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (CSL_PSC_LPSC_SRIO) == PSC_MODSTATE_ENABLE))
    {
        /* SRIO ON. Ready for use */            
        return 0;
    }
    else
    {
        /* SRIO Power on failed. Return error */            
        return -1;            
    }
#else
    /* PSC is not supported on simulator. Return success always */
    return 0;
#endif
}
 /******************************************************************************
  * 创建于2017-06-29
  * srio_soft_reset()
  * 对SRIO进行软件复位，用于SRIO数据传输异常时的恢复
  * 当前并不完整，仅对DIO类型的传输的崩溃恢复有效，不能对type9或type11的传输崩溃的恢复
  * ***************************************************************************/

 Int32 srio_soft_reset (void) //hxl:不使能SRIO的电源模块和时钟源
{

	 //Cppi_ChHnd          chHnd;
	 CSL_SrioHandle     hSrio;
	 int i,j,k;
	 #ifndef SIMULATOR_SUPPORT
	/*1.Set Teardown bit in RX/TX channel global configure registers.*/
	 /*  *chHnd=Cppi_getChannelNumber(gSRIODriverMCB.cppiHnd);
	 if(Cppi_channelTeardown(chHnd, Cppi_Wait_WAIT)!=CPPI_SOK)
	 {
		 return -1;
	 }*/
	 /*2. Flush all LSU transfer, and waiting for completion.*/
	 // Open the CSL SRIO Module 0
	 hSrio = CSL_SRIO_Open (0);
	 // Flush the LSU transaction on LSU 0 with SRC ID 0x0
	 /*flash all LSU transfers for all Source ID*/
	 for(i= 0; i<SRIO_MAX_LSU_NUM ; i++)
	 {
	 		/*flash LSU transfer for all Source ID*/
	 		for(j=0; j< SRIO_MAX_DEVICEID_NUM; j++)
	 		{
	 			CSL_SRIO_FlushLSUTransaction (hSrio, i, j);//hxl:SRC ID 0x0
	 			/*This can take more than one cycle to do the flush.
	 			wait for a while*/
	 			for(k=0; k< 100; k++)
	 				asm(" nop 5");
	 		}
	 }

	 /*3. Disable the PEREN bit of RIO_PCR register to stop all new logical layer transactions*/
	 // Disable the SRIO peripheral.
	 CSL_SRIO_DisablePeripheral (hSrio);
	 /*Wait one second to finish any current DMA transfer.*/
	 for(i=0; i< 100000000; i++)
	 asm(" nop 5");
	 /*4. Disable all the SRIO block with BLK_EN and GBL_EN;*/ //hxl:SrioDevice_init()中有这一步

	 /*5. Disable Serdes by writing 0 to the SRIO SERDES configuration registers;*/

	 /*6. Disable the SRIO through the PSC module;*/
	/* Set SRIO Power domain to Off */
	CSL_PSC_disablePowerDomain (CSL_PSC_PD_SRIO);
	/* Start the state transition */
	CSL_PSC_startStateTransition (CSL_PSC_PD_SRIO);

	/* Wait until the state transition process is completed. */
	while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_SRIO));

	/* Return SRIO PSC status */
	if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_SRIO) == PSC_PDSTATE_OFF) )
	{
		/* SRIO off.  */
		return 0;
	}
	else
	{
		/* SRIO Power off failed. Return error */
		return -1;
	}
	#else
	/* PSC is not supported on simulator. Return success always */
	return 0;
	#endif
	}

/**
 *  @b Description
 *  @n  
 *      System Initialization Code. This is added here only for illustrative
 *      purposes and needs to be invoked once during initialization at 
 *      system startup.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
/*
static Int32 system_init (Void)
{
    Int32               result;
    Qmss_MemRegInfo     memRegInfo;

    // Initialize the QMSS Configuration block. //
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));
    
    // Initialize the Host Region. //
    memset ((void *)&host_region, 0, sizeof(host_region));

    // Set up the linking RAM. Use the internal Linking RAM.
    // LLD will configure the internal linking RAM address and maximum internal linking RAM size if
    // a value of zero is specified. Linking RAM1 is not used //
    qmssInitConfig.linkingRAM0Base = 0;//
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC;   

#ifdef xdc_target__bigEndian
    // PDSP Configuration: Big Endian //
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
    // PDSP Configuration: Little Endian //
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif    

    // Initialize Queue Manager Sub System //
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }

    // Start the QMSS. //
    if (Qmss_start() != QMSS_SOK)
    {
        System_printf ("Error: Unable to start the QMSS\n");
        return -1;
    }

    // Memory Region 0 Configuration //
    memRegInfo.descBase         = (UInt32 *)l2_global_address((UInt32)host_region);
    memRegInfo.descSize         = SIZE_HOST_DESC;
    memRegInfo.descNum          = NUM_HOST_DESC;
    memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memRegInfo.memRegion        = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;    

    // Initialize and insert the memory region. //
    result = Qmss_insertMemoryRegion (&memRegInfo); 
    if (result < QMSS_SOK)
    {
        System_printf ("Error inserting memory region: %d\n", result);
        return -1;
    }

    // Initialize CPPI CPDMA //
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }

    // CPPI and Queue Manager are initialized. //
    System_printf ("Debug(Core %d): Queue Manager and CPPI are initialized.\n", coreNum);
    System_printf ("Debug(Core %d): Host Region 0x%x\n", coreNum, host_region);
    return 0;
}
*/
/*2016-6-29 hxl: this function is used for initialize QMSS driver,2016-6-30 deleted it*/
/*
int QMSS_SRIOprj_init(void)
{
    int				  i;
  //  CI_SERVICE_TELNET telnet; //hxl modified
  //  CI_SERVICE_HTTP   http;   //hxl modified
    QMSS_CFG_T      qmss_cfg;
    CPPI_CFG_T      cppi_cfg;

	// Get information about the platform so we can use it in various places //
	memset( (void *) &gPlatformInfo, 0, sizeof(platform_info));
	(void) platform_get_info(&gPlatformInfo);

	(void) platform_uart_init();
	(void) platform_uart_set_baudrate(115200);
	(void) platform_write_configure(PLATFORM_WRITE_ALL);

	// Clear the state of the User LEDs to OFF //
	for (i=0; i < gPlatformInfo.led[PLATFORM_USER_LED_CLASS].count; i++) {
		(void) platform_led(i, PLATFORM_LED_OFF, PLATFORM_USER_LED_CLASS);
	}

    // Initialize the components required to run this application:
     //  (1) QMSS
     //
    // Initialize QMSS //
    if (platform_get_coreid() == 0)
    {
        qmss_cfg.master_core        = 1;
    }
    else
    {
        qmss_cfg.master_core        = 0;
    }
    qmss_cfg.max_num_desc       = NUM_HOST_DESC_SRIO;//hxl: SRIO需要的描述符数量   当前：128
    qmss_cfg.desc_size          = SIZE_HOST_DESC_SRIO;//hxl:描述符大小为16的倍数，128=16*8
    qmss_cfg.mem_region         = Qmss_MemRegion_MEMORY_REGION0;//hxl:设置SRIO的memory region
    if (res_mgr_init_qmss (&qmss_cfg) != 0)
    {
        platform_write ("Failed to initialize the QMSS subsystem \n");
        return -1;
    }
    else
    {
    	platform_write ("QMSS successfully initialized \n");
    	return 0;
    }
}*/
/*2016-6-29 hxl: this function is used for insert memory region used by SRIO project*/
static int SRIOMemoryRegion_init(void)
{ /* Setup the descriptor memory regions.
     *
     * The Descriptor base addresses MUST be global addresses and
     * all memory regions MUST be setup in ascending order of the
     * descriptor base addresses.
     */
   Int32 result;
   Qmss_MemRegInfo             memCfg;
   /* Initialize and setup CPSW Host Descriptors required for example */
   // Initialize the Host Region. //
   //memset ((void *)&gHostDesc, 0, sizeof(gHostDesc));//hxl:程序合并时，这句的位置需要调整
   memCfg.descBase             =   (uint32_t *) Convert_CoreLocal2GlobalAddr ((uint32_t) gHostDesc)+128*128;
   memCfg.descSize             =   SIZE_HOST_DESC_SRIO;
   memCfg.descNum              =   NUM_HOST_DESC_SRIO;
   //memCfg.manageDescFlag       =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;//hxl:2016-7-1 hxl:modified
   //memCfg.memRegion            =  Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;//hxl:2016-7-1 hxl:modified
   memCfg.manageDescFlag       =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;
   memCfg.memRegion            =   Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;//Qmss_MemRegion_MEMORY_REGION1;
   memCfg.startIndex           =   0;//hxl: don’t care in static memory region configure.
 /* Insert Host Descriptor memory region */
   result = Qmss_insertMemoryRegion(&memCfg);//hxl:插入memory region1
   if (result == QMSS_MEMREGION_ALREADY_INITIALIZED)
  {
    platform_write ("Memory Region %d already Initialized \n", memCfg.memRegion);
  }
   if (result < QMSS_SOK)
   {
      System_printf ("Error inserting memory region: %d\n", result);
      return -1;
    }
   return 0;

}
/*
static Int32 system_init (Void)
{
    Int32               result;
    Qmss_MemRegInfo     memRegInfo;

    //if( QMSS_SRIOprj_init()<0)//2016-6-29 hxl:利用resource mgr 部分代码对QMSSdriver初始化，并调用LLD QMSS_start()
                                //2016-6-30 hxl:deleted it
    	//return -1;
    if(SRIOMemoryRegion_init()<0)
    	return -1;
    // Initialize CPPI CPDMA //
    //result = Cppi_init (&cppiGblCfgParams);//2016-6-30 hxl:deleted it
    //if (result != CPPI_SOK)
    //{
    //    System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
   //     return -1;
   // }

    System_printf ("Debug(Core %d): SRIO Host Region initialized\n", coreNum);
    return 0;
}*/
/*===================hxl added the following functions==============================================================*/
/*****************************************************************************************
 * -This doorbell ISR is used to Interrupt Core0 that FPGA has just finished Swrite a line
 * data to a dedicated address
 * -doorbell register 0, bit 0
 * write by hxl 2016-2-25
 * ***************************************************************************************/

void myDoorblComeIsr(UArg argument)
{
	    //startTime1 = CSL_tscRead ();
	    hSrio_test = CSL_SRIO_Open (0);
	    //Srio_dioCompletionIsr(hDrvManagedSrioDrv,intDstDoorbell);//doorbell0--> dst0
		CSL_SRIO_ClearDoorbellPendingInterrupt (hSrio_test, 0, 0xFFFF);//hxl:added
		if(0==Flag_EvenOdd)//hxl:int Flag_EvenOdd;注意类型提升之类的问题
		{
			Flag_EvenOdd=1;
			ptr_FPGAData=fpgaData0;

		}
		else
		{
			Flag_EvenOdd=0;
		    ptr_FPGAData=fpgaData1;
		}


		//if(srioDoorblIsrCnt<(Uint32)TOTALLINES)//hxl:实际应用程序不需要该条件
	   {

			//set fpgadata0 to non cacheable through L1D
			//Cache_inv((void*)fpgaData0, sizeof(fpgaData0), Cache_Type_ALLD, 1);
			//Cache_inv((void*)fpgaData1, sizeof(fpgaData1), Cache_Type_ALLD, 1);
			Cache_inv((void*)fpgaData0, sizeof(fpgaData0), Cache_Type_ALLD, TOTALPOINTS);
			Cache_inv((void*)fpgaData1, sizeof(fpgaData1), Cache_Type_ALLD, TOTALPOINTS);
			memcpy((void*)beforeData[WriteToIndex], (void*)ptr_FPGAData,(TOTALPOINTS-1)*sizeof(fpgaData0[0]));//hxl:读(TestSize-1)个数
			//memcpy((void*)beforeData[WriteToIndex], (void*)ptr_FPGAData,4096*sizeof(fpgaData0[0]));//hxl:读8192个数,仅用于测试。not applicable
			/*----hxl:实际应用程序中，线号需要由FPGA端发送过来，此处不需要自行添加*/
			//beforeData[WriteToIndex][0]=(short)srioDoorblIsrCnt;//hxl:for test,not applicable
			/*--end--hxl:实际应用程序中，线号需要由FPGA端发送过来，此处不需要自行添加*/
			semCnt=Semaphore_getCount(mysem1);
			if(semCnt==LINENUM_BUF)
			{
			 flag_OverFlow=1;
			}
			//semCnts[srioDoorblIsrCnt]=semCnt;
			WriteToIndex++;
			if(LINENUM_BUF==WriteToIndex)
			{
				 WriteToIndex=0;
			}
			// Debug: Increment the ISR count
			Semaphore_post(mysem1);//hxl:实际应用程序中需要保留此句代码
			srioDoorblIsrCnt++;
	   }
	    //t1=  CSL_tscRead();
	    //t1 =t1-startTime1;
	    srioDoorblIsrServiced=1;
	    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is application registered SRIO DIO LSU interrupt
 *      handler (ISR) which is used to process the pending DIO Interrupts.
 *      SRIO Driver users need to ensure that this ISR is plugged with
 *      their OS Interrupt Management API. The function expects the
 *      Interrupt Destination information to be passed along to the
 *      API because the DIO interrupt destination mapping is configurable
 *      during SRIO device initialization.
 *
 *  @param[in]  argument
 *      SRIO Driver Handle
 *
 *  @retval
 *      Not Applicable
 */
 void myDioTxCompletionIsr //hxl:发射完成中断服务函数
(
    UArg argument
)
{
    hSrioCSL = CSL_SRIO_Open (0);
	/* Pass the control to the driver DIO Tx Completion ISR handler */
    Srio_dioTxCompletionIsr ((Srio_DrvHandle)argument, hSrioCSL);
    CSL_SRIO_ClearLSUPendingInterrupt ( hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);//
    /* Wake up the pending task */
    srioLsuIsrServiced = 1;//hxl:全局变量。用于表明中断服务函数已完成

    /* Debug: Increment the ISR count */
    srioDioIsrCnt++;//hxl记录中断次数

    return;
}
/**
 *  @b Description
 *  @n
 *      DIO driver config and start
 *
 *  @retval
 *      Not Applicable.
 */
int dioDriverStart(Srio_DrvConfig* ptr_drvCfg,Srio_DrvHandle* ptr_hDrvManagedSrioDrv)
{
    UInt8           isAllocated;
    //uint16_t i;
    //Srio_DrvConfig  drvCfg;
    //Srio_DrvHandle  hDrvManagedSrioDrv;
    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)ptr_drvCfg, 0, sizeof(Srio_DrvConfig));
    /* Initialize the OSAL */
    if (Osal_dataBufferInitMemory(SRIO_MAX_MTU) < 0)
    {
	    System_printf ("Error: Unable to initialize the OSAL. \n");
	    return -1;
    }

    /********************************************************************************
     * The SRIO Driver Instance is going to be created with the following properties:
     * - Driver Managed
     * - Interrupt Support (Pass the Rx Completion Queue as NULL)
     ********************************************************************************/
    
    // Setup the SRIO Driver Managed Configuration. //
    ptr_drvCfg->bAppManagedConfig = FALSE;

    // Driver Managed: Receive Configuration //
    ptr_drvCfg->u.drvManagedCfg.bIsRxCfgValid             = 1;
    ptr_drvCfg->u.drvManagedCfg.rxCfg.rxMemRegion         = Qmss_MemRegion_MEMORY_REGION1;// Qmss_MemRegion_MEMORY_REGION1;//Qmss_MemRegion_MEMORY_REGION0;//2016-6-30 hxl:修改为memory region1
    ptr_drvCfg->u.drvManagedCfg.rxCfg.numRxBuffers        = 4;
    ptr_drvCfg->u.drvManagedCfg.rxCfg.rxMTU               = SRIO_MAX_MTU;
    
    // Accumulator Configuration. // 
   {
	    int32_t coreToQueueSelector[4];

      // This is the table which maps the core to a specific receive queue. //
	    coreToQueueSelector[0] = 704;
	    coreToQueueSelector[1] = 705;
	    coreToQueueSelector[2] = 706;
	    coreToQueueSelector[3] = 707;

	    // Since we are programming the accumulator we want this queue to be a HIGH PRIORITY Queue //
	    ptr_drvCfg->u.drvManagedCfg.rxCfg.rxCompletionQueue = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE,
	    															     coreToQueueSelector[coreNum], &isAllocated);//hxl:将队列设为高优先级队列类型
		if (ptr_drvCfg->u.drvManagedCfg.rxCfg.rxCompletionQueue < 0)
		{
			System_printf ("Error: Unable to open the SRIO Receive Completion Queue\n");
			return -1;
		}
       
	    //hxl 2016-7-6 deleted the following for test//
		// Accumulator Configuration is VALID. //
		ptr_drvCfg->u.drvManagedCfg.rxCfg.bIsAccumlatorCfgValid = 1;
		//drvCfg.u.drvManagedCfg.rxCfg.bIsAccumlatorCfgValid = 0;//2016-7-4 ,hxl:modified,accumulator is programmed in Qmss_init()

		// Accumulator Configuration. //
	    //drvCfg.u.drvManagedCfg.rxCfg.accCfg.channel             = coreNum;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.channel             = coreNum+4;//2016-7-5 hxl:modified for combining SRIO prj with net project
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.queueEnMask         = 0;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.queMgrIndex         = coreToQueueSelector[coreNum];
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.maxPageEntries      = 2;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.timerLoadCount      = 0;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
		ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

        // Initialize the accumulator list memory //
        memset ((Void *)&gHiPriAccumList_SRIO[0], 0, sizeof(gHiPriAccumList_SRIO));
        ptr_drvCfg->u.drvManagedCfg.rxCfg.accCfg.listAddress = l2_global_address((UInt32)&gHiPriAccumList_SRIO[0]);
    }

    /* Driver Managed: Transmit Configuration */
   ptr_drvCfg->u.drvManagedCfg.bIsTxCfgValid             = 1;
   ptr_drvCfg->u.drvManagedCfg.txCfg.txMemRegion         = Qmss_MemRegion_MEMORY_REGION1;//Qmss_MemRegion_MEMORY_REGION1;Qmss_MemRegion_MEMORY_REGION0;//2016-6-30 hxl:修改memory region 1
   ptr_drvCfg->u.drvManagedCfg.txCfg.numTxBuffers        = 4;
   ptr_drvCfg->u.drvManagedCfg.txCfg.txMTU               = SRIO_MAX_MTU;

    /* Start the Driver Managed SRIO Driver. */
    *ptr_hDrvManagedSrioDrv = Srio_start(ptr_drvCfg);
    if ((*ptr_hDrvManagedSrioDrv) == NULL)
    {
        System_printf ("Error(Core %d): SRIO Driver failed to start\n", coreNum);
        return -1;
    }   


   /*-- Enable Time Stamp Counter-- */
    //CSL_tscEnable();//hxl:used the time stamp counter //2016-7-6 hxl deleted it for test
	/* Check if there is a memory leak? Since we dont implement a 'deinit' API we need to
	     * be careful in these calculations
	     *  - For the Application Managed Driver Instance
	     *      There will be 'numRxBuffers' + 1 (Driver Instance)
	     *  - For the Driver Managed Driver Instance
	     *      There will be 'numRxBuffers' + 'numTxBuffers' + 1 (Driver Instance)
	     *  Take these into account while checking for memory leaks. */
	    if (free_counter +   //-------hxl deleted it for test----2016-7-6-----------------------------
	        (ptr_drvCfg->u.drvManagedCfg.rxCfg.numRxBuffers + ptr_drvCfg->u.drvManagedCfg.txCfg.numTxBuffers + 1) != malloc_counter)
	    {
	        System_printf ("Error: Memory Leak Detected\n");
	        Task_exit();
	    }
	platform_write("Debug(Core %d): SRIO driver is configured and started.\n", coreNum);//hxl:added this for print  information to uart

    // Run the loopback data transfers on the system initialization core. //
      if (coreNum == CORE_SYS_INIT)
      {
    	// DIO Write Operation //

      }
      return 0;

}

/**********************************************************************************************************************
 *SRIO 发送和接收buf的分配和初始化
 *by hxl
 * *******************************************************************************************************************/
Void SRIOSrcDstBufAllocI(UInt8* srcDataBuffer[SRIO_DIO_NUM_SRCBUF],int bufSizeInB,int bufNum)
{
	int i;
	// Allocate memory for the Source and Destination Buffers //hxl modified
	for (i = 0; i < (bufNum); i++)
	{
		srcDataBuffer[i] =  (uint8_t*)Osal_srioDataBufferMalloc( bufSizeInB);//hxl modified for test
		if (srcDataBuffer[i] == NULL)
		{
			System_printf ("Error: Source Buffer (%d) Memory Allocation Failed\n", i);
			return ;//hxl modified
		}
		// Initialize the data buffers //
		for (counter = 0; counter < bufSizeInB; counter++)
		{
			srcDataBuffer[i][counter] = 0xFF;////hxl:SWRITE to FPGA时的源buffer

		}
		//Debug Message:
		System_printf ("Debug(core0):  DIO Data Transfer buffer - Src(%d) 0x%p\n",
						 i, srcDataBuffer[i]); //hxl deleted it
	}

}
/**********************************************************************************************************************
 *SRIO 中断服务函数的分配
 *by hxl
 * *******************************************************************************************************************/
Void SRIOisrConfig(Srio_DrvHandle* ptr_hDrvManagedSrioDrv)
 {
	  int32_t                 eventId;
	  Srio_DrvHandle hDrvManagedSrioDrv=*ptr_hDrvManagedSrioDrv;
	/*----2016-7-6 hxl :the following part is conflict with net connect-------------------*/
	     // Hook up the SRIO interrupts with the core.
	     /*EventCombiner_dispatchPlug (48, (EventCombiner_FuncPtr)Srio_rxCompletionIsr, (UArg)hDrvManagedSrioDrv, TRUE);
	 	EventCombiner_enableEvent(48);*/
	     /*--end--2016-7-6 hxl :the following part is conflict with net connect-------------------*/
	 /*******************************************************************************************
	  * hxl:this is the Interrupt setting for function dioSockets_SW_DoorbellIsrTest1()
	  ******************************************************************************************/
	  /*--hxl added the following lines---SRIO interrupt setting-----------------------*/
	 //Get the CSL SRIO Handle. //
	 	hSrioCSL = CSL_SRIO_Open (0);
	     if (hSrioCSL == NULL)
	 	return;
	 	// SRIO DIO Interrupts need to be routed from the CPINTC0 to GEM Event.
	 	// - We have configured DIO Interrupts to get routed to Interrupt Destination 0
	     // - (Refer to the CSL_SRIO_RouteLSUInterrupts API configuration in the SRIO Initialization)
	 	// - We want this System Interrupt to mapped to Host Interrupt 8
	     CSL_SRIO_ClearDoorbellPendingInterrupt (hSrioCSL, 0, 0xFFFF);//hxl:added
	 	// Disable Interrupt Pacing for INTDST0 //
	 	CSL_SRIO_DisableInterruptPacing (hSrioCSL, 0);
	 	// Map the System Interrupt i.e. the Interrupt Destination 0 interrupt to the DIO ISR Handler. //
	 	CpIntc_dispatchPlug(CSL_INTC0_INTDST0, (CpIntc_FuncPtr)myDoorblComeIsr, (UArg)hDrvManagedSrioDrv, TRUE);//system event 112
	 	// The configuration is for CPINTC0. We map system interrupt 112 to Host Interrupt 8. //hxl:CPINTC0实际为CIC0。TI有些文档还未更新
	 	CpIntc_mapSysIntToHostInt(0, CSL_INTC0_INTDST0, 8);//二级中断事件INTC0_INTDST0映射到CIC0的host event 8,二级中断事件INTC0_INTDST0是二级中断事件号
	 	//Enable the Host Interrupt.
	 	CpIntc_enableHostInt(0, 8);//使能host中断8
	 	//Enable the System Interrupt
	 	CpIntc_enableSysInt(0, CSL_INTC0_INTDST0);
	 	//Get the event id associated with the host interrupt.
	 	eventId = CpIntc_getEventId(8);//hxl:返回host event对应的primary event事件号104(96~127)--eventGroup[3]
	 	// Plug the CPINTC Dispatcher.
	 	EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 8, TRUE);//hxl：必须要有
	 	CSL_SRIO_ClearDoorbellPendingInterrupt (hSrioCSL, 0, 0xFFFF);//hxl:added
	    /*-end -hxl added the following lines---SRIO interrupt setting-----------------------*/
	// SRIO DIO Interrupts need to be routed from the CPINTC0 to GEM Event.
		//  - We have configured DIO Interrupts to get routed to Interrupt Destination 1
		//    (Refer to the CSL_SRIO_RouteLSUInterrupts API configuration in the SRIO Initialization)
		//  - We want this System Interrupt to mapped to Host Interrupt 32
		// - hxl:用于通知DSP本次LSU发送完成//
		CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
		// Disable Interrupt Pacing for INTDST1
		CSL_SRIO_DisableInterruptPacing (hSrioCSL, 1);
		// Route LSU0 ICR0 to INTDST1
		CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 0, 1);//hxl:LSU:Load/Store Unit
		// Route LSU0 ICR1 to INTDST1
		CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 1, 1);
		// Route LSU0 ICR2 to INTDST1
		CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 2, 1);
		//Map the System Interrupt i.e. the Interrupt Destination 1 interrupt to the DIO ISR Handler.
		CpIntc_dispatchPlug(CSL_INTC0_INTDST1, (CpIntc_FuncPtr)myDioTxCompletionIsr, (UArg)hDrvManagedSrioDrv, TRUE);
		//The configuration is for CPINTC0. We map system interrupt 113 to Host Interrupt 2.  //hxl:CPINTC0实际为CIC0。TI有些文档还未更新
		CpIntc_mapSysIntToHostInt(0, CSL_INTC0_INTDST1,2);
				// Enable the Host Interrupt.
		CpIntc_enableHostInt(0, 2);//使能host中断2
		//Enable the System Interrupt
		CpIntc_enableSysInt(0, CSL_INTC0_INTDST1);
		//Get the event id associated with the host interrupt.
		eventId = CpIntc_getEventId(2);//hxl:返回host event 2对应的primary event事件号 62————对应system event 113;对应INT8
		//Plug the CPINTC Dispatcher.
		EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 2, TRUE);//hxl：必须要有
	   /* Enable Time Stamp Counter */
		CSL_tscEnable();//hxl:used the time stamp counter

 }

/**********************************************************************************************************************
 *SRIO 中断服务函数的分配
 *by hxl
 * *******************************************************************************************************************/
Void SRIOisrConfig_Dbl(Srio_DrvHandle* ptr_hDrvManagedSrioDrv,Uint8 isEnble)
 {
	  int32_t                 eventId;
	  CSL_CPINTC_Handle                          hnd;
	  Bool                      pendingStatus;
	  CSL_IntcParam vectId;
	  CSL_IntcObj intcObj104;
	  CSL_IntcHandle hIntc104;
	  Srio_DrvHandle hDrvManagedSrioDrv=*ptr_hDrvManagedSrioDrv;
	/*----2016-7-6 hxl :the following part is conflict with net connect-------------------*/
	     // Hook up the SRIO interrupts with the core.
	     /*EventCombiner_dispatchPlug (48, (EventCombiner_FuncPtr)Srio_rxCompletionIsr, (UArg)hDrvManagedSrioDrv, TRUE);
	 	EventCombiner_enableEvent(48);*/
	     /*--end--2016-7-6 hxl :the following part is conflict with net connect-------------------*/
	 /*******************************************************************************************
	  * hxl:this is the Interrupt setting for function dioSockets_SW_DoorbellIsrTest1()
	  ******************************************************************************************/
	  /*--hxl added the following lines---SRIO interrupt setting-----------------------*/
	 //Get the CSL SRIO Handle. //
	 	hSrioCSL = CSL_SRIO_Open (0);
	     if (hSrioCSL == NULL)
	    	 return;
	     eventId = CpIntc_getEventId(8);//hxl:返回
	     CSL_SRIO_ClearDoorbellPendingInterrupt (hSrioCSL, 0, 0xFFFF);//hxl:added
	     CpIntc_clearSysInt(0, CSL_INTC0_INTDST0); //hxl:清除system event CSL_INTC0_INTDST0。该事件属于属于CIC instance0
	     Hwi_clearInterrupt(10); //hxl:清除中断10
	     //Hwi_clearInterrupt(6); //hxl:清除中断6
	     hnd = CSL_CPINTC_open (0);  //hxl:获取CIC0 handle。
	     if(isEnble==1)
	     {
			// SRIO DIO Interrupts need to be routed from the CPINTC0 to GEM Event.
			// - We have configured DIO Interrupts to get routed to Interrupt Destination 0
			 // - (Refer to the CSL_SRIO_RouteLSUInterrupts API configuration in the SRIO Initialization)
			// - We want this System Interrupt to mapped to Host Interrupt 8

			// Disable Interrupt Pacing for INTDST0 //
			CSL_SRIO_DisableInterruptPacing (hSrioCSL, 0);
			// Map the System Interrupt i.e. the Interrupt Destination 0 interrupt to the DIO ISR Handler. //
			CpIntc_dispatchPlug(CSL_INTC0_INTDST0, (CpIntc_FuncPtr)myDoorblComeIsr, (UArg)hDrvManagedSrioDrv, TRUE);//system event 112
			// The configuration is for CPINTC0. We map system interrupt 112 to Host Interrupt 8. //hxl:CPINTC0实际为CIC0。TI有些文档还未更新
			CpIntc_mapSysIntToHostInt(0, CSL_INTC0_INTDST0, 8);//二级中断事件INTC0_INTDST0映射到CIC0的host event 8,二级中断事件INTC0_INTDST0是二级中断事件号
			//Enable the Host Interrupt.
			CpIntc_enableHostInt(0, 8);//使能host中断8
			//Enable the System Interrupt
			CpIntc_enableSysInt(0, CSL_INTC0_INTDST0);
			//Get the event id associated with the host interrupt.
			eventId = CpIntc_getEventId(8);//hxl:返回host event对应的primary event事件号104(96~127)--eventGroup[3]，
			// Plug the CPINTC Dispatcher.
			EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 8, TRUE);//hxl：必须要有
			CSL_SRIO_ClearDoorbellPendingInterrupt (hSrioCSL, 0, 0xFFFF);//hxl:added
			/*-end -hxl added the following lines---SRIO interrupt setting-----------------------*/
	     }
	     else
	     {
	    	 //Enable the Host Interrupt.
			CpIntc_disableHostInt(0, 8);//使能host中断8
			//Enable the System Interrupt
			CpIntc_disableSysInt(0, CSL_INTC0_INTDST0);
	     }

	     pendingStatus = CSL_CPINTC_isHostInterruptPending(hnd, 8);//hxl:判断host 8 system event的状态。调用前必须先调用CSL_CPINTC_open
		 //while (pendingStatus == TRUE)
		{
	    	 /*逐级清除中断或事件标志*/
	    	 CSL_SRIO_ClearDoorbellPendingInterrupt (hSrioCSL, 0, 0xFFFF);//hxl:added。
	    	 CpIntc_clearSysInt(0, CSL_INTC0_INTDST0); //hxl:清除system event CSL_INTC0_INTDST0。该事件属于属于CIC instance0。二级中断事件
			 vectId= CSL_INTC_VECTID_10 ;
			 hIntc104 = CSL_intcOpen (&intcObj104, eventId , &vectId , NULL);
			 CSL_intcHwControl(hIntc104,CSL_INTC_CMD_EVTCLEAR ,NULL);//hxl:清除初级事件
			 pendingStatus = CSL_CPINTC_isHostInterruptPending(hnd, 8);//hxl:判断host 8 system event的状态
			 Hwi_clearInterrupt(10); //hxl:清除中断10
			 //Hwi_clearInterrupt(6); //hxl:清除中断6
		}


 }
/**********************************************************************************
 * CSL_INTC0_INTDST1-->host2-->primary event 62-->ventGroupHwiNum[1]
 * ****************************************************************************/
/*
Void SRIOisrConfig_LSU(Srio_DrvHandle* ptr_hDrvManagedSrioDrv,Uint8 isEnble)
 {
	  int32_t                 eventId;
	  Srio_DrvHandle hDrvManagedSrioDrv=*ptr_hDrvManagedSrioDrv;
	  CSL_IntcParam vectId;
	  CSL_IntcObj intcObj62;
	  CSL_IntcHandle hIntc62;
	  //--hxl added the following lines---SRIO interrupt setting-----------------------//
	 //Get the CSL SRIO Handle. //
	 	hSrioCSL = CSL_SRIO_Open (0);
	     if (hSrioCSL == NULL)
	 	return;
	   eventId = CpIntc_getEventId(2);
	    CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
		CpIntc_clearSysInt(0, CSL_INTC0_INTDST1); //hxl:清除system event CSL_INTC0_INTDST0。该事件属于属于CIC instance0
		Hwi_clearInterrupt(8); //hxl:清除中断8
	    //-end -hxl added the following lines---SRIO interrupt setting-----------------------//
	    if(isEnble == 1)
	    {
			// SRIO DIO Interrupts need to be routed from the CPINTC0 to GEM Event.
			//  - We have configured DIO Interrupts to get routed to Interrupt Destination 1
			//    (Refer to the CSL_SRIO_RouteLSUInterrupts API configuration in the SRIO Initialization)
			//  - We want this System Interrupt to mapped to Host Interrupt 32
			// - hxl:用于通知DSP本次LSU发送完成//
			//CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
			// Disable Interrupt Pacing for INTDST1
			CSL_SRIO_DisableInterruptPacing (hSrioCSL, 1);
			// Route LSU0 ICR0 to INTDST1
			CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 0, 1);//hxl:LSU:Load/Store Unit
			// Route LSU0 ICR1 to INTDST1
			CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 1, 1);
			// Route LSU0 ICR2 to INTDST1
			CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 2, 1);
			//Map the System Interrupt i.e. the Interrupt Destination 1 interrupt to the DIO ISR Handler.
			CpIntc_dispatchPlug(CSL_INTC0_INTDST1, (CpIntc_FuncPtr)myDioTxCompletionIsr, (UArg)hDrvManagedSrioDrv, TRUE);
			//The configuration is for CPINTC0. We map system interrupt 113 to Host Interrupt 2.  //hxl:CPINTC0实际为CIC0。TI有些文档还未更新
			CpIntc_mapSysIntToHostInt(0, CSL_INTC0_INTDST1,2);
					// Enable the Host Interrupt.
			CpIntc_enableHostInt(0, 2);//使能host中断2
			//Enable the System Interrupt
			CpIntc_enableSysInt(0, CSL_INTC0_INTDST1);
			//Get the event id associated with the host interrupt.
			eventId = CpIntc_getEventId(2);//hxl:返回host event 2对应的primary event事件号 62;eventGroupHwiNum[2]----INT8
			//Plug the CPINTC Dispatcher.
			EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 2, TRUE);//hxl：必须要有

			 //逐级清除中断或事件标志//

			CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
			CpIntc_clearSysInt(0,  CSL_INTC0_INTDST1); //hxl:清除system event CSL_INTC0_INTDST0。该事件属于属于CIC instance0。二级中断事件
			vectId= CSL_INTC_VECTID_8 ;
			hIntc62 = CSL_intcOpen (&intcObj62, eventId , &vectId , NULL);
			CSL_intcHwControl(hIntc62,CSL_INTC_CMD_EVTCLEAR ,NULL);//hxl:清除初级事件
			Hwi_clearInterrupt(8); //hxl:清除中断8

	    }
	    else
	    {
	    	 //Disable the Host Interrupt.
	    	CpIntc_disableHostInt(0, 2);//使能host中断2
	    	//disable the System Interrupt
	    	CpIntc_disableSysInt(0, CSL_INTC0_INTDST1);
	    }


	   // Enable Time Stamp Counter //
		CSL_tscEnable();//hxl:used the time stamp counter

 }
 */
/**********************************************************************************
 * CSL_INTC0_INTDST1-->host4-->primary event 92-->ventGroupHwiNum[2]
 * ****************************************************************************/

Void SRIOisrConfig_LSU(Srio_DrvHandle* ptr_hDrvManagedSrioDrv,Uint8 isEnble)
 {
	  int32_t                 eventId;
	  Srio_DrvHandle hDrvManagedSrioDrv=*ptr_hDrvManagedSrioDrv;
	  CSL_IntcParam vectId;
	  CSL_IntcObj intcObj92;
	  CSL_IntcHandle hIntc92;
	  //--hxl added the following lines---SRIO interrupt setting-----------------------//
	 //Get the CSL SRIO Handle. //
	 	hSrioCSL = CSL_SRIO_Open (0);
	     if (hSrioCSL == NULL)
	 	return;
	   eventId = CpIntc_getEventId(4);
	    CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
		CpIntc_clearSysInt(0, CSL_INTC0_INTDST1); //hxl:清除system event CSL_INTC0_INTDST0。该事件属于属于CIC instance0
		Hwi_clearInterrupt(9); //hxl:清除中断8
	    //-end -hxl added the following lines---SRIO interrupt setting-----------------------//
	    if(isEnble == 1)
	    {
			// SRIO DIO Interrupts need to be routed from the CPINTC0 to GEM Event.
			//  - We have configured DIO Interrupts to get routed to Interrupt Destination 1
			//    (Refer to the CSL_SRIO_RouteLSUInterrupts API configuration in the SRIO Initialization)
			//  - We want this System Interrupt to mapped to Host Interrupt 32
			// - hxl:用于通知DSP本次LSU发送完成//
			//CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
			// Disable Interrupt Pacing for INTDST1
			CSL_SRIO_DisableInterruptPacing (hSrioCSL, 1);
			// Route LSU0 ICR0 to INTDST1
			CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 0, 1);//hxl:LSU:Load/Store Unit
			// Route LSU0 ICR1 to INTDST1
			CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 1, 1);
			// Route LSU0 ICR2 to INTDST1
			CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 2, 1);
			//Map the System Interrupt i.e. the Interrupt Destination 1 interrupt to the DIO ISR Handler.
			CpIntc_dispatchPlug(CSL_INTC0_INTDST1, (CpIntc_FuncPtr)myDioTxCompletionIsr, (UArg)hDrvManagedSrioDrv, TRUE);
			//The configuration is for CPINTC0. We map system interrupt 113 to Host Interrupt 2.  //hxl:CPINTC0实际为CIC0。TI有些文档还未更新
			CpIntc_mapSysIntToHostInt(0, CSL_INTC0_INTDST1,4);
					// Enable the Host Interrupt.
			CpIntc_enableHostInt(0, 4);//使能host中断2
			//Enable the System Interrupt
			CpIntc_enableSysInt(0, CSL_INTC0_INTDST1);
			//Get the event id associated with the host interrupt.
			eventId = CpIntc_getEventId(4);//hxl:返回host event 2对应的primary event事件号 62;eventGroupHwiNum[2]----INT8
			//Plug the CPINTC Dispatcher.
			EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 4, TRUE);//hxl：必须要有

			 //逐级清除中断或事件标志//

			CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
			CpIntc_clearSysInt(0,  CSL_INTC0_INTDST1); //hxl:清除system event CSL_INTC0_INTDST0。该事件属于属于CIC instance0。二级中断事件
			vectId= CSL_INTC_VECTID_9 ;
			hIntc92 = CSL_intcOpen (&intcObj92, eventId , &vectId , NULL);
			CSL_intcHwControl(hIntc92,CSL_INTC_CMD_EVTCLEAR ,NULL);//hxl:清除初级事件
			Hwi_clearInterrupt(9); //hxl:清除中断8

	    }
	    else
	    {
	    	 //Disable the Host Interrupt.
	    	CpIntc_disableHostInt(0, 4);//使能host中断2
	    	//disable the System Interrupt
	    	CpIntc_disableSysInt(0, CSL_INTC0_INTDST1);
	    }


	   // Enable Time Stamp Counter //
		//CSL_tscEnable();//hxl:used the time stamp counter

 }

#define INTERRUPT_LINE  0

/* Notify event number that the app uses */
#define EVENTID         10


UInt32 times = 0;
UInt16 recvnumes = 0;

#define masterProc 0
#define sloverProc1 1
#define sloverProc2 2
#define sloverProc3 3
#define sloverProc4 4
#define sloverProc5 5
#define sloverProc6 6
#define sloverProc7 7
#define sloverNum 7


#include <xdc/std.h>

/*  -----------------------------------XDC.RUNTIME module Headers    */
#include <xdc/runtime/System.h>

/*  ----------------------------------- IPC module Headers           */
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/Notify.h>
#include <ti/ipc/Ipc.h>
/*  ----------------------------------- BIOS6 module Headers         */
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>

/*  ----------------------------------- To get globals from .cfg Header */
#include <xdc/cfg/global.h>


#include <time.h>

//----------------------------------------------------------------------------0
#define TimeInSec(Cycle)  ((Cycle) / 1000.0 / 1000 / 1000)
//----------------------------------------------------------------------------0

/*
 *  ======== cbFxn ========
 *  This function was registered with Notify. It is called when any event is
 *  sent to this processor.
 */
Void cbFxn(UInt16 procId, UInt16 lineId,
           UInt32 eventId, UArg arg, UInt32 payload)
{
    /* The payload is a sequence number. */

    if(procId!=masterProc) // 主核注册函数
    {
        recvnumes++;  // 接收从核的数目
        if(recvnumes==sloverNum) // 当收到全部从核回复的信息
        {
            recvnumes=0;
           // Semaphore_post(semHandle);
            Semaphore_post(mysem2);
        }
    }
    else{
        times = payload; // 执行次数 mysem2
       // Semaphore_post(semHandle);
        Semaphore_post(mysem2);
    }
}


void mcip_core_task()
{
    while (times < 4)
    {
      Int status;
	  Int lin;
      //int core=MultiProc_self();
      //* wait forever on a semaphore, semaphore is posted in callback */
      //Semaphore_pend(semHandle, BIOS_WAIT_FOREVER); // 等待主核通知开始执行任务
      Semaphore_pend(mysem2, BIOS_WAIT_FOREVER); // 等待主核通知开始执行任务
      System_printf("SloverCore%d Received Event from MasterCore in %d\n", MultiProc_self(),times);

     /* 这里可以添加从核执行的任务*/

     memcpy(&index_firs,&index_firs_native,sizeof(index_firs));
     memcpy(&firs,&firs_native,sizeof(firs));
     
     lin=testData[coreNum][1];//hxl:提取当前的数据线的线序号
     zeroPhaseFilter_F((const short*) &testData[coreNum][0],firs[index_firs[lin]]);
     //for(k=0;k<4096;k++)//hxl:采用2倍下采样,2018-01-08
     //{
     //   outData[coreNum][k]=Hilbert_y[coreNum][k];
     //}



     /* Send an event to the next processor */
     status = Notify_sendEvent(masterProc, INTERRUPT_LINE, EVENTID, times,TRUE);
     if (status < 0) 
	  {
        System_abort("sendEvent to MasterCore failed\n");
      }

     System_printf("SloverCore%d sent Event from MasterCore in %d\n", MultiProc_self(),times);

    }
}
void master_core_task()
{
    int i=0;
	int j;
    int k;
    while (i < 1)
	{
      //---------20171019----zhou: 添加为了计算时间-----------------------------------1
      unsigned long long StartTime, EndTime, OverHead;
      float time = 0.0;
      TSCH = 0;
      TSCL = 0;
      StartTime = _itoll(TSCH, TSCL);
      EndTime = _itoll(TSCH, TSCL);
      OverHead = EndTime - StartTime;
      //---------------------------zhou: 添加为了计算时间-----------------------------------1
      Int status;
      /* 这里可以添加主核需要执行的任务代码*/
      //-------------------------------test-------------------------------------------------------------
      gen_twiddle_fft_sp_hxl (w_sp, TOTALPOINTS);
      gen_twiddle_fft_sp_ifft (w_spi, TOTALPOINTS);

      memcpy(&index_firs,&index_firs_native,sizeof(index_firs));
      memcpy(&firs,&firs_native,sizeof(firs));
                           
      for(k=0; k<Point_image; k++)
        {
           for(j=0; j<Num_line; j++)
             {
               tx_rx_distance[j][k] = 1;
             }
        }

      for(k=0; k<4096; k++)
        {
            for(j=0; j<8; j++)
                {
                    testData[j][k] = j*200+100;
                }
        }
      for(k = 0; k < 8; k++)
        {
            printf("  testData[%d] = ", k);
            for (j = 0; j < 10; j++)
                {
                    printf("%2d ", testData[k][j]);
                }
            printf("\n");
        }
       /*
       int lin;
       lin=testData[0][1];//hxl:提取当前的数据线的线序号
       zeroPhaseFilter_F((const short*) &testData[0][0],firs[index_firs[lin]]);
       for(k=0;k<4096;k++)//hxl:采用2倍下采样,2018-01-08
        {
            outData[0][k]=Hilbert_y[0][k];
        }
      */

      //-----------------------------------------------------------------------------------------
      /* Send an event to the next processor */
      status = Notify_sendEvent(sloverProc1, INTERRUPT_LINE, EVENTID, i, TRUE);
      status = Notify_sendEvent(sloverProc2, INTERRUPT_LINE, EVENTID, i, TRUE);
      status = Notify_sendEvent(sloverProc3, INTERRUPT_LINE, EVENTID, i, TRUE);
      status = Notify_sendEvent(sloverProc4, INTERRUPT_LINE, EVENTID, i, TRUE);
      status = Notify_sendEvent(sloverProc5, INTERRUPT_LINE, EVENTID, i, TRUE);
      status = Notify_sendEvent(sloverProc6, INTERRUPT_LINE, EVENTID, i, TRUE);
      status = Notify_sendEvent(sloverProc7, INTERRUPT_LINE, EVENTID, i, TRUE);

      /* Continue until remote side is up */
      if (status < 0)
        {
          continue;
        }

        platform_write("MasterCore Sent Event to SloverCores in %d\n", i);
        int lin;
        lin=testData[0][1];//hxl:提取当前的数据线的线序号
        zeroPhaseFilter_F((const short*) &testData[0][0],firs[index_firs[lin]]);
     /*
      for(k=0;k<4096;k++)//hxl:采用2倍下采样,2018-01-08
        {
          outData[0][k]=Hilbert_y[0][k];
        }
     
      for(k=0; k<4096; k++)
       {
         outData[0][k] = testData[0][k]+1;
         tmp = testData[0][k]+2;
         testData[0][k]  = tmp;
       }
     */
     
	  /* Wait to be released by the cbFxn posting the semaphore */
      // Semaphore_pend(semHandle, BIOS_WAIT_FOREVER); // 主核等待所有从核完成其工作返回
      Semaphore_pend(mysem2, BIOS_WAIT_FOREVER);
      platform_write("MasterCore Received Event from All SloverCores in %d\n",i);
      i++;
     //----------------------------------------------------------------------------------------------------

      for (j = 0; j < Num_line*Point_image; j++)
	    {
           ready_go_x[j] = ready_x[0][j]+ready_x[0][j]+ready_x[1][j]+ready_x[2][j]+ready_x[3][j]+ready_x[4][j]+ready_x[5][j]+ready_x[6][j]+ready_x[7][j];
           ready_go_y[j] = ready_y[0][j]+ready_y[0][j]+ready_y[1][j]+ready_y[2][j]+ready_y[3][j]+ready_y[4][j]+ready_y[5][j]+ready_y[6][j]+ready_y[7][j];
        }

       status = Notify_sendEvent(sloverProc1, INTERRUPT_LINE, EVENTID, i, TRUE);
       status = Notify_sendEvent(sloverProc2, INTERRUPT_LINE, EVENTID, i, TRUE);
       status = Notify_sendEvent(sloverProc3, INTERRUPT_LINE, EVENTID, i, TRUE);
       status = Notify_sendEvent(sloverProc4, INTERRUPT_LINE, EVENTID, i, TRUE);
       status = Notify_sendEvent(sloverProc5, INTERRUPT_LINE, EVENTID, i, TRUE);
       status = Notify_sendEvent(sloverProc6, INTERRUPT_LINE, EVENTID, i, TRUE);
       status = Notify_sendEvent(sloverProc7, INTERRUPT_LINE, EVENTID, i, TRUE);
       /* Continue until remote side is up */
       if (status < 0)
            {
              continue;
            }

        platform_write("MasterCore Sent Event to SloverCores in %d\n", i);



        lin=testData[0][1];//hxl:提取当前的数据线的线序号
        zeroPhaseFilter_F((const short*) &testData[0][0],firs[index_firs[lin]]);
        Semaphore_pend(mysem2, BIOS_WAIT_FOREVER);
        platform_write("MasterCore Received Event from All SloverCores in %d\n",i);
        i++;
       //----------------------------------------------------------------------------------------------------
        for (j = 0; j < Num_line*Point_image; j++)
            {
              ready_go_x[j] = ready_x[0][j]+ready_x[0][j]+ready_x[1][j]+ready_x[2][j]+ready_x[3][j]+ready_x[4][j]+ready_x[5][j]+ready_x[6][j]+ready_x[7][j];
              ready_go_y[j] = ready_y[0][j]+ready_y[0][j]+ready_y[1][j]+ready_y[2][j]+ready_y[3][j]+ready_y[4][j]+ready_y[5][j]+ready_y[6][j]+ready_y[7][j];
            }


        for(k = 0; k < 8; k++)
            {
                printf("  ready_x[%d] = ", k);
                for (j = 0; j < 10; j++)
                    {
                        printf("%2d ", ready_x[k][j]);
                    }
               printf("\n");
            }

        for(k = 0; k < 8; k++)
            {
                printf("  ready_y[%d] = ", k);
                for (j = 0; j < 10; j++)
                    {
                        printf("%2d ", ready_y[k][j]);
                    }
                     printf("\n");
            }

        printf("\n");
        printf("  ready_go_x = " );
        for (j = 0; j < 10; j++)
            {
                printf("%2d ", ready_go_x[j]);
            }
        printf("\n");
        printf("  ready_go_y = " );
        for (j = 0; j < 10; j++)
            {
                printf("%2d ", ready_go_y[j]);
            }
        printf("\n");

         //-----------------------------------------------------------------------------------------------------------
    }
}


//---------------------------------------------------------------------
// Main Entry Point
//---------------------------------------------------------------------
Void main(Void)
{
    Int status;
    Semaphore_Params  Params_sem;//2016-6-29,hxl:added from  main() of client.c
    Semaphore_Params_init(&Params_sem);
    Params_sem.mode=Semaphore_Mode_COUNTING ;//创建2进制信号

    mysem1 = Semaphore_create(0, &Params_sem, NULL);//信号count值初始化为1；
    mysem2 = Semaphore_create(0, &Params_sem, NULL);//信号count值初始化为1；

    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);//hxl:读取coreNUM,确定运行的core

    status = Ipc_start();
    if (status < 0)
	    {
            platform_write("Ipc_start failed\n");
        }
    if(MultiProc_self()==masterProc)
        {
            while(Ipc_attach(sloverProc1))
			    {
                    Task_sleep(1);
                }// 完成从核1的连接
            while(Ipc_attach(sloverProc2))
			    {
                    Task_sleep(1);
                }// 完成从核2的连接

            while(Ipc_attach(sloverProc3))
			    {
                    Task_sleep(1);
                }// 完成从核3的连接

            while(Ipc_attach(sloverProc4))
			    {
                    Task_sleep(1);
                }// 完成从核4的连接

            while(Ipc_attach(sloverProc5))
			    {
                    Task_sleep(1);
                }// 完成从核5的连接

            while(Ipc_attach(sloverProc6))
			    {
                   Task_sleep(1);
                }// 完成从核6的连接
            while(Ipc_attach(sloverProc7))
			    {
                    Task_sleep(1);
                }// 完成从核7的连接

            status = Notify_registerEvent(sloverProc1, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
            status = Notify_registerEvent(sloverProc2, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
            status = Notify_registerEvent(sloverProc3, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
            status = Notify_registerEvent(sloverProc4, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
            status = Notify_registerEvent(sloverProc5, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
            status = Notify_registerEvent(sloverProc6, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
            status = Notify_registerEvent(sloverProc7, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
        }
    else
	   {
           while(Ipc_attach(masterProc))
		        {
                    Task_sleep(1);
                }// 完成主核0的连接
            status = Notify_registerEvent(masterProc, INTERRUPT_LINE, EVENTID,(Notify_FnNotifyCbck)cbFxn, NULL);
           // if (status < 0)
		   //    {
           //      platform_write("Notify_registerEvent for masterCore0 failed\n");
           //   }// 完成主核0的事件注册
	    }



    if(MultiProc_self()==masterProc)
        {
	        QMSS_CFG_T      qmss_cfg;//2016-6-30 hxl:added this from stackTest()
            CPPI_CFG_T      cppi_cfg;//2016-6-30 hxl:added this from stackTest()
	        //Semaphore_Params  Params_sem;//2016-6-29,hxl:added from  main() of client.c
	        memset((void*)fpgaData0,0xFF,sizeof(fpgaData0));//hxl added it for test to FPGA
	        memset((void*)fpgaData1,0xFF,sizeof(fpgaData1));//hxl added it for test to FPGA
	        memset((void*)beforeData,0xFF,sizeof(beforeData));//hxl added it for test to FPGA
	        //Get information about the platform so we can use it in various places //
	        memset( (void *) &gPlatformInfo, 0, sizeof(platform_info));//2016-6-30 hxl:added this from stackTest()
	        (void) platform_get_info(&gPlatformInfo);//2016-6-30 hxl:added this from stackTest()
	        (void) platform_uart_init();//2016-6-30 hxl:added this from stackTest()
	        (void) platform_uart_set_baudrate(115200);//2016-6-30 hxl:added this from stackTest()
	        (void) platform_write_configure(PLATFORM_WRITE_ALL);//2016-6-30 hxl:added this from stackTest()

	        //MSM_init();//hxl added this line.与MSM有关的初始化。
	        /*---2016-6-29 hxl added the following codes from main() of client.c----------------*/
	        /* Get the core number. */
	        //coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);//hxl:读取coreNUM,确定运行的core

            #ifdef SIMULATOR_SUPPORT
              System_printf ("SRIO DIO LSU ISR  and Ethernet application is not supported on SIMULATOR. Exiting!\n");//hxl:modified the information
              return;
              #else
              //System_printf ("Executing the SRIO and Ethernet application on the DEVICE\n");//hxl:modified the information
              platform_write ("Executing the SRIO and Ethernet application on the DEVICE\n");//hxl:modified the information
           #endif 

    /* Initialize the system only if the core was configured to do so. */
    if (coreNum == CORE_SYS_INIT)
    {
		System_printf ("Debug(Core %d): System Initialization for PKTDMA(in PA) & QMSS\n", coreNum);//hxl:modified the print information
		/*---------------2016-6-30  hxl added the following part from stackTest() func--------------------------------------------------------- */
		// Initialize the components required to run the network application:
		//  (1) QMSS
		//  (2) CPPI (in PA)  //hxl:packet DMA,PA中的PKTDMA测试
		//  (3) Packet Accelerator  //hxl:属于NETCP
		//
		// Initialize QMSS //
		if (platform_get_coreid() == 0) //2016-6-30 hxl:deleted this will cause Qmss_insertMemoryRegion(&memCfg) return with error;
		{
			qmss_cfg.master_core        = 1;
		}
		else
		{
		   qmss_cfg.master_core        = 0;
		}
		//qmss_cfg.master_core        = coreNum;//2016-6-30 hxl:added this line
		qmss_cfg.max_num_desc       = MAX_NUM_DESC;//hxl:需要修改，加上SRIO需要的描述符数量   当前：128
		qmss_cfg.desc_size          = MAX_DESC_SIZE;//hxl:描述符大小，128=16*8
		qmss_cfg.mem_region         = Qmss_MemRegion_MEMORY_REGION0;//hxl:网络使用memory region0；还需要设置SRIO的memory region
		if (res_mgr_init_qmss (&qmss_cfg) != 0)
		{
		   platform_write ("Failed to initialize the QMSS subsystem \n");
		   goto main_exit;
		}
		else
		{
		   platform_write ("QMSS successfully initialized \n");
		}
		//------hxl added the following lines for SRIO-----------------------------------//
		if(SRIOMemoryRegion_init()<0)//2016-6-30 hxl added memory region1 for SRIO descriptor
		goto main_exit;
		//---end---hxl added the following lines for SRIO-----------------------------------//
		// Initialize CPPI //
		if (platform_get_coreid() == 0) //2016-6-30 hxl:deleted this will cause error in res_mgr_init_cppi()
		{
		   cppi_cfg.master_core        = 1;
		}
		else
		{
		  cppi_cfg.master_core        = 0;
		}
		//cppi_cfg.master_core        = 0;//2016-6-30 hxl:addded this
		cppi_cfg.dma_num            = Cppi_CpDma_PASS_CPDMA;//hxl:配置PA中的PKTDMA
		cppi_cfg.num_tx_queues      = NUM_PA_TX_QUEUES;//hxl:NETCP的PacketDMA通道映射
		cppi_cfg.num_rx_channels    = NUM_PA_RX_CHANNELS;
		if (res_mgr_init_cppi (&cppi_cfg) != 0)
		{
			platform_write ("Failed to initialize CPPI subsystem \n");
			goto main_exit;
		}
		else
		{
			//platform_write ("CPPI successfully initialized \n");
			platform_write ("PKTDMA(in PA) successfully initialized \n");//hxl:add this

		}
		if (res_mgr_init_pass()!= 0)
		{
		   platform_write ("Failed to initialize the Packet Accelerator \n");
		   goto main_exit;
		}
		else
		{
		  platform_write ("PASS successfully initialized \n");//hxl:change PA to PASS
		}
		/*----------end-----2016-6-30  hxl added the following part from stackTest()--------------------------------------------------------- */
		/* Power on SRIO peripheral before using it */
		if (enable_srio () < 0)//hxl:使能电源和时钟
		{
			System_printf ("Error: SRIO PSC Initialization Failed\n");
			return;
		}

		/* Device Specific SRIO Initializations: This should always be called before
		 * initializing the SRIO Driver. */
		if (SrioDevice_init() < 0)//hxl:SRIO设备初始化序列
			return;

		/* Initialize the SRIO Driver */ //hxl:SRIO Driver为 Application code 提供良好的API layer,由TI PDK包提供
		if (Srio_init () < 0)//hxl:该函数仅能调用一次
		{
			System_printf ("Error: SRIO Driver Initialization Failed\n");
			return;
		}

		/* SRIO Driver is operational at this time. */
		//System_printf ("Debug(Core %d): SRIO Driver has been initialized\n", coreNum);
		platform_write ("Debug(Core %d): SRIO Driver has been initialized\n", coreNum);
		/* Write to the SHARED memory location at this point in time. The other cores cannot execute
		 * till the SRIO Driver is up and running. */
		isSRIOInitialized[0] = 1;//hxl:位于".srioSharedMem"的全局变量
		/* The SRIO IP block has been initialized. We need to writeback the cache here because it will
		 * ensure that the rest of the cores which are waiting for SRIO to be initialized would now be
		 * woken up. */
		CACHE_wbL1d ((void *) &isSRIOInitialized[0], 128, CACHE_WAIT);//向L1dcatch写128字节数据 isSRIOInitialized
    }
    else
    {
        /* All other cores need to wait for the SRIO to be initialized before they proceed. */ 
        System_printf ("Debug(Core %d): Waiting for SRIO to be initialized.\n", coreNum);

        /* All other cores loop around forever till the SRIO is up and running. 
         * We need to invalidate the cache so that we always read this from the memory. */
        while (isSRIOInitialized[0] == 0)
            CACHE_invL1d ((void *) &isSRIOInitialized[0], 128, CACHE_WAIT);//hxl:使L1d的128字节无效

        //System_printf ("Debug(Core %d): SRIO can now be used.\n", coreNum);
        platform_write("Debug(Core %d): SRIO can now be used.\n", coreNum);//hxl:added this for print  information to uart
    }
}
else
{
    memset( (void *) &gPlatformInfo, 0, sizeof(platform_info));//2016-6-30 hxl:added this from stackTest()
        (void) platform_get_info(&gPlatformInfo);//2016-6-30 hxl:added this from stackTest()
        (void) platform_uart_init();//2016-6-30 hxl:added this from stackTest()
        (void) platform_uart_set_baudrate(115200);//2016-6-30 hxl:added this from stackTest()
        (void) platform_write_configure(PLATFORM_WRITE_ALL);//2016-6-30 hxl:added this from stackTest()
}
    /*----end------The following part is for SRIO ---------------------------------------------------*/


    /*----hxl-将fpgaData0和fpgaData1 设置为noncachable-------------*/
    //CACHE_init();
    /*---2016-6-29 hxl added the following codes from main() of client.c----------------*/
//    Semaphore_Params_init(&Params_sem);
//    Params_sem.mode=Semaphore_Mode_COUNTING ;//创建2进制信号
    /* Create a Semaphore object to be use as a resource lock */
//    mysem1 = Semaphore_create(0, &Params_sem, NULL);//信号count值初始化为1；
//    mysem2 = Semaphore_create(0, &Params_sem, NULL);//信号count值初始化为1；
   // memset((void*)index_firs_native,0x00,sizeof(index_firs_native));//hxl:默认采用5.5M的宽带带通滤波,用于通用应用
    /* Start the BIOS */
    platform_write("starting BIOS!\n", coreNum);//hxl:added this for print  information to uart
    BIOS_start();
    main_exit:return;
}
