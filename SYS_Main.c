/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* COPYRIGHT                                                                */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
	(c) 2018 Crane Payment Innovations, Inc. All rights reserved.
	Decompilation prohibited except as permitted by law. No using, disclosing,
	reproducing, accessing or modifying without prior written consent.
	
	CPI
	3222 Phoenixville Pike, Suite 200. Malvern, PA 19355 USA
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FILE DESCRIPTION                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
	Project:			10012
	Filename:			SYS_Main.c
	File Description:	main_application() for the application.
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* SOURCE CONTROL                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* INCLUDES                                                                 */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#include "SYS_StandardHeader.h"
#include "usb_lib.h"
#include "HAL_CLOCK_Driver.h"
#include "HAL_ISR_Vectors.h"
#include "HAL_MEMORY_Driver.h"
#include "HAL_MMI_Driver.h"
#include "HAL_OS_OperatingSystem.h"
#include "HAL_FPGA_Driver.h"
#include "HAL_USB_Driver.h"
#include "EVT_EventManager.h"
#include "MEI_Identification.h"
#include "DRV_IFC_Common.h"
#include "DRV_USB_Common.h"
#include "DRV_EEPROM_Driver.h"
#include "DRV_DBM_Startup.h"
#include "DRV_BLINKY_Driver.h"
#include "DRV_MMI_Driver.h"
#include "DRV_MTR_Driver.h"
#include "HST_HostTask.h"
#include "HST_SVC_Interface.h"
#include "LIB_AccessMap.h"
#include "HST_SVC_Messages.h"
#include "SYS_ComponentsVerification.h"
#include "HAL_IFLASH_Driver.h"

/*
	'#include' the API header of each task that will be installed here.
	Modify 'tasTaskList[] below as required.
*/
#include "ACQ_AcquisitionTaskApi.h"
#include "TRT_TransportTaskApi.h"
#include "HST_HostTaskApi.h"
#include "AUD_AuditTaskApi.h"
#include "REC_RecognitionTaskApi.h"
#include "SYS_Main.h"

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* DEFINES                                                                  */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

typedef tVOID(*vfpSTARTUP_FTN)(tVOID) ;
typedef tVOID(*vfpINSTALL_FTN)(tUINT8 u_Priority, tUINT16 uw_StackSize) ;

//J10012-20976 - This bootstrap prevents IFLASH read protection
#define BAD_BOOTSTRAP "286099501"

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* ENUMS and STRUCTURES                                                     */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

typedef struct _TASK_LIST				//Structure for starting and installing a task.
{
	vfpSTARTUP_FTN vfpTaskStartup ;		//The task's startup function.
	vfpINSTALL_FTN vfpTaskInstall ;		//The task's installation function.
	tUINT8 uPriority ;					//The task's priority.
	tUINT16 uwStackSize ;				//The size of the task's stack (in bytes).
} tTASK_LIST ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GLOBAL VARIABLES                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* LOCAL VARIABLES                                                          */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*
	The variable 'tasTaskList[]' is used to install a task.
	
	At least one task must be installed. Oherwise there is no task to run
	that can install other tasks. In a typical system, all tasks are
	installed prior to starting the OS.

	Each task must be assigned a unique priority from highest (0) to
	lowest (MAX_NUM_TASKS - 1).		
	
	For convenience, list tasks by priority.
	
	There must be MAX_NUM_TASKS number of entries. If there are more entries
	than tasks, then set 'ftnpTask' to NULL. If 'ftnpTask' is NULL then
	'uPriority' and 'uwStackSize' are 'don't cares'.
*/

static tTASK_LIST tasTaskList[MAX_NUM_TASKS] =
{
	{ (vfpSTARTUP_FTN) ACQ_AcquisitionTaskStartup,	(vfpINSTALL_FTN) ACQ_AcquisitionTask,	0,	MAX_STACK_SIZE	},
	{ (vfpSTARTUP_FTN) TRT_TransportTaskStartup,	(vfpINSTALL_FTN) TRT_TransportTask,		1,	MAX_STACK_SIZE	},
	{ (vfpSTARTUP_FTN) HST_HostTaskStartup,			(vfpINSTALL_FTN) HST_HostTask,			2,	MAX_STACK_SIZE	},
	{ (vfpSTARTUP_FTN) AUD_AuditTaskStartup,		(vfpINSTALL_FTN) AUD_AuditTask,			3,	MAX_STACK_SIZE 	},
	{ (vfpSTARTUP_FTN) REC_RecognitionTaskStartup,	(vfpINSTALL_FTN) REC_RecognitionTask,	4,	MAX_STACK_SIZE	},
} ;

static tTIME tsUsbDetectTime ;
static tBOOL bsDebounceUsbConnection ;
static tBOOL bsPrevUsbDetectState ;
static tBOOL bsCurrUsbDetectState ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FUNCTION PROTOTYPES                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

static tVOID sHandleUsbDetect(tVOID) ;
static tVOID sVerifyPowerFailStatus(tVOID) ;
static tVOID sDisableUnusedPins(tVOID) ;
static tVOID sDetectScnAntiSkewHardware(tVOID) ; //J10012-19053
#ifndef NDEBUG
static tVOID sCopyVariantOnDebugBuild(tVOID) ;	//J10012-15626
#else
static tBOOL sCanEnableReadProtection(tVOID) ; //J10012-20976
#endif
tINT32 main_application(tVOID) ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* CODE                                                                     */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/****************************************************************************/
/*
	Function:		main_application()
	Description:	Entry point for the application.
	Parameters:		None.
	Return Value:	None.
	Notes:			Assumes all tasks will successfully install.
*/
/****************************************************************************/
tINT32 main_application(tVOID)
{
	tTASK_LIST *tp_Task ;
	tUINT8 i ;
	
	LIB_PetWatchdog() ;
	HAL_OS_SaveAndDisableInterrupts() ;
	HAL_CLOCK_InitializeSystemClock() ;
	HAL_CLOCK_ConfigureIWDG() ;
	
#ifdef NDEBUG	
		//J10012-20976 - Enable IFLASH read protection (only for certain bootstraps)
	if (sCanEnableReadProtection())
	{
		HAL_IFLASH_EnableReadProtection() ;
	}
#endif	
	
	HAL_ISR_InitializeIsrVectorTable() ;
	HAL_MMI_Initialize() ;
	mHAL_MMI_TurnOnGreenYellowRedLed() ;
	HAL_DEBUG_Install() ;										//Debug features (e.g. scope trigger) are not available prior to this.
	sDisableUnusedPins() ;
		//Required to install clock before using timer. This enables global interrupts...
	HAL_CLOCK_InstallSystemTick(TRUE, TICKS_PER_SECOND, LIB_ISR_SystemTickHandler) ;
	LIB_PetWatchdog() ;
		//Place new functionality after this point and before the HAL_CLOCK_InstallSystemTick() below.
		//Determine the status of power fail...
	sVerifyPowerFailStatus() ;
	LIB_PetWatchdog() ;
	ACQ_GetBoardRevisions() ;
		/*
			J10012-19053: Merge SCR's antiskew algorithm to create a common production software (SCR/SCN)
			Determine if the anti-skew hardware is installed.
		*/
	sDetectScnAntiSkewHardware() ;

		//Disable unused pins...

		/*
			DRV_BLINKY_Configure() must be after ACQ_GetBoardRevisions() because
			the BLINKY pin is used for both. As an analog input when getting the
			control board's revision and as an output when used as BLINKY.
		*/
	DRV_BLINKY_Configure() ;
	LIB_PetWatchdog() ;
	HAL_MEMORY_Configure() ;
    HAL_MTR_DisableMotors() ;       //Make sure motors are off...
	DRV_EEPROM_Initialize() ;	
//TODO:CP:Retrieve power fail data.
//TODO:CP:Get system configuration data.
//TODO:CP:Handle PROM requirements.
	SYS_ComponentVerification() ;

#ifndef NDEBUG	
	/*
		J10012-15626
	
		The variant and application must both reside on the same page of XFLASH.
		If they don't, the background task takes care of copying the variant to
		the correct page and erasing the incorrect page.
	
		On Debug builds this causes an issue due to the fact that Audit is placed
		in XFLASH.  It is not available to handle events that are raised by the
		other tasks while the XFLASH copy is in progress.

		To resolve that we do the variant copy here, in the foreground, for Debug builds.		
	*/
	sCopyVariantOnDebugBuild() ;
#endif	

        /*
            Initialize common USB functionality.  From here, we can either go to preOS USB SVC functionality
            or regular USB functionality.  preOS configuration continues in SYS_OutOfOrder.  Regular
            USB configuration continues in sConfigureComms in the startup state.
        */
    HAL_USB_SystemInitialization() ;
	HAL_USB_InstallUsbEventsHandler() ;
    DRV_USB_InitStage1() ;
	DRV_USB_InitStage2() ;

		//Run each tasks startup script.
	for (i = 0; i < MAX_NUM_TASKS; ++i)
	{
		tp_Task = &tasTaskList[i] ;
		if (tp_Task->vfpTaskStartup != (vfpSTARTUP_FTN) NULL)
		{
			(*tp_Task->vfpTaskStartup)() ;
		}	
	}
	
//TODO:CP:Check for BNF.
		//Must disable before starting the Event Manager. This disables global interrupts.
	HAL_CLOCK_InstallSystemTick(FALSE, 0L, (vfpISR_FUNCTION) NULL) ;
	EVT_InitializeEventManager() ;
	LIB_PetWatchdog() ;

		//Install all tasks after event manager has been initialized...
	for (i = 0; i < MAX_NUM_TASKS; ++i)
	{
		tp_Task = &tasTaskList[i] ;
		if (tp_Task->vfpTaskInstall != (vfpINSTALL_FTN) NULL)
		{
			(*tp_Task->vfpTaskInstall)(tp_Task->uPriority, tp_Task->uwStackSize) ;
		}	
	}

		//Should never return from EVT_Run().
	EVT_Run() ;
	return (0) ;
}

/****************************************************************************/
/*
	Function:		SYS_OutOfOrder()
	Description:	Pre-OS Out of Order state.
	Parameters:		e_FaultReason = reason for going out of order.
	Return Value:	None
	Notes:			Can only be used prior to entering the OS or if the OS
					is stopped. Supports a limited number of service tool
					commands.
					Make sure this is not called before XFlash is configured since SVC lives out there!
*/
/****************************************************************************/
tVOID SYS_OutOfOrder(eFAULT_REASON e_FaultReason)
{
	tTIME t_MmiTime ;

		//J10012-12952: Set the system state to Out Of Order so it's reported to the service tool...
	tHstSystemData.eSystemState = SYSTEM_STATE_OUT_OF_ORDER ;
		//J10012-12952: Initialize memory so that we can get valid ranges for SVC commands...
	LIB_GetProcessorMemorySize() ;

	HAL_CLOCK_InstallSystemTick(TRUE, TICKS_PER_SECOND, LIB_ISR_SystemTickHandler) ;

	DRV_IFC_CommonInit(NULL, hst_svc_ProcessMessage, NULL, NULL, NULL) ;
	HAL_ISR_InstallInterruptHandler(ISR_VECTOR_ID_EXTI15_10, sHandleUsbDetect, ISR_PRIORITY_LOW) ;
	(tSvcUsbConfigPreOs.bfpInstallFtn)(uaCommsBufferUsbRx, uaCommsBufferUsbTx) ;
	DRV_USB_InitStage3() ;

		//Just leave the switch in the front...
	GPIO_SetBits(GPIOC, PIN_USB_SWITCH) ;
	mHAL_USB_Init() ;	//J10012-12952: Initialize USB.
		//Start debouncing...
	tsUsbDetectTime = LIB_GetCurrentTime() ;
	bsCurrUsbDetectState = HAL_USB_IsrIsFrontConnectorActive() ;
	bsDebounceUsbConnection = TRUE ;
	
		//Set the MMI pattern...
	t_MmiTime = LIB_GetCurrentTime() ;
	HAL_MMI_Initialize() ;
	DRV_MMI_Initialize() ;
	if (e_FaultReason == FAULT_DBM)
	{
		DRV_MMI_SetBlinkParameters(MMI_COMMUNICATIONS_HARDWARE_FAULT) ;
	}
	else
	{
		DRV_MMI_SetBlinkParameters(MMI_ACCEPTOR_HARDWARE_FAULT) ;
	}
	
		//Set the blinky pattern...
	DRV_BLINKY_Configure() ;
	DRV_BLINKY_SetPattern((eBLINKY_CODE) e_FaultReason) ;
	
	do
	{
		if (bsDebounceUsbConnection)
		{
			if (bsCurrUsbDetectState != HAL_USB_IsrIsFrontConnectorActive())
			{
				bsCurrUsbDetectState = HAL_USB_IsrIsFrontConnectorActive() ;
				tsUsbDetectTime = LIB_GetCurrentTime() ;
			}
			
			if (LIB_TestElapsedTime(tsUsbDetectTime, FIVE_MILLISECONDS))
			{
				if (bsPrevUsbDetectState != bsCurrUsbDetectState)
				{
					bsPrevUsbDetectState = bsCurrUsbDetectState ;
					bsDebounceUsbConnection = FALSE ;
				}
				else
				{
					/*
						Spurious USB Detect.  Nothing to do.  We also hit this case on
						startup if no USB cable is plugged in because the prev state
						defaults to 0 and the new state is also.
					*/
				}
			}			
		}
		
		if (LIB_TestElapsedTime(t_MmiTime, TWENTY_FIVE_MILLISECONDS))
		{
				//No need to pet the dog every time through the dowhile loop.  In here is plenty sufficient.
			LIB_PetWatchdog() ;		
			DRV_BLINKY_Update() ;
			DRV_MMI_Refresh() ;
			t_MmiTime = LIB_GetCurrentTime() ;
		}
	} while (1) ;
}

/****************************************************************************/
/*
	Function:		sHandleUsbDetect()
	Description:	Handle USB Detect interrupts before the OS starts.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sHandleUsbDetect(tVOID)
{
	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		bsCurrUsbDetectState = HAL_USB_IsrIsFrontConnectorActive() ;
		EXTI_ClearITPendingBit(EXTI_Line14);
		bsDebounceUsbConnection = TRUE ;
		tsUsbDetectTime = LIB_GetCurrentTime() ;
	}
}

/****************************************************************************/
/*
	Function:		sVerifyPowerFailStatus()
	Description:	Checks the power fail status register flag. If the flag is set
					then this function stays in an infinite loop until the watchdog reset
					or actual power fail. The system will stay in this function during
					brownout.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sVerifyPowerFailStatus(tVOID)
{
	GPIO_InitTypeDef t_GpioInitTypeDef ;
	tUINT32 ul_FilterCount ;
	tBOOL b_MaybeOutOfBrownOut ;
	tBOOL b_Done ;
	
	ul_FilterCount = 0 ;
	b_Done = FALSE ;
	b_MaybeOutOfBrownOut = FALSE ;
  		//Configure	PC15, PIN_POWER_FAIL, as floating input...
	GPIO_StructInit(&t_GpioInitTypeDef) ;
	t_GpioInitTypeDef.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
	t_GpioInitTypeDef.GPIO_Pin = PIN_POWER_FAIL ;
	GPIO_Init(GPIOC, &t_GpioInitTypeDef) ;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE) ;	
		//POWER_FAIL is active low...	
	while (!b_Done)
	{
	  		//Check to see if we are out of brownout...
		if ((GPIOC->IDR & PIN_POWER_FAIL) == PIN_POWER_FAIL)
		{
		  	if (!b_MaybeOutOfBrownOut)
			{
			  		//Start the filter counter...
				ul_FilterCount = LIB_GetCurrentTime() ;
			}
				//Possibly out of brownout...
			b_MaybeOutOfBrownOut = TRUE ;				
		}
		else
		{
			b_MaybeOutOfBrownOut = FALSE ;
		}
			//We are out of brownout if the power fail signal stays high for at least 1ms...
	    if (b_MaybeOutOfBrownOut && LIB_TestElapsedTime(ul_FilterCount, TEN_MILLISECONDS))
		{
			b_Done = TRUE ;
		}
	}	
}

/****************************************************************************/
/*
	Function:		sDisableUnusedPins()
	Description:	Sets unused pins to floating input.
	Parameters:		None.
	Return Value:	None.
	Notes:			Currently the only unused pins are PIN_PB_CLK and PIN_PB_DATA.
					Once the PB (Peripheral Bus) feature is implemented, this
					function can be removed.
*/
/****************************************************************************/
static tVOID sDisableUnusedPins(tVOID)
{
	GPIO_InitTypeDef GPIO_InitStructure ;						//Small enough to declare local.

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE) ;		//Enable GPIOE peripheral.
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz ;
	GPIO_InitStructure.GPIO_Pin = PIN_DBM_RESET_SEL ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOE, &GPIO_InitStructure) ;
	GPIOE->BSRR = PIN_DBM_RESET_SEL ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE) ;		//Enable GPIOD peripheral.
	GPIO_InitStructure.GPIO_Pin = PIN_PB_CLK ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;
	GPIO_Init(GPIOD, &GPIO_InitStructure) ;
	GPIOD->BSRR = PIN_PB_CLK ;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE) ;		//Enable GPIOC peripheral.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
	GPIO_InitStructure.GPIO_Pin = PIN_PB_DATA ;
	GPIO_Init(GPIOC, &GPIO_InitStructure) ;
}

/****************************************************************************/
/*
	Function:		sDetectScnAntiSkewHardware()
	Description:	Detects if the SCN contains the anti-skew hardware.
	Parameters:		None.
	Return Value:	None.
	Notes:			J10012-19053: Merge SCR's antiskew algorithm to create a common production software (SCR/SCN)
*/
/****************************************************************************/
static tVOID sDetectScnAntiSkewHardware(tVOID)
{
		//Safeguard against SCN application et al having antiskew set at all.
	tHstSystemData.bAntiSkewScr = FALSE ;

	if (COMPONENT_TYPE_PRODUCTION == tComponentHeader.eComponentType)
	{
		if (BOARD_REVISION_MW_LED_NO_BARCODE_WITH_ANTISKEW == uAcqRevisionLedBoard)
		{
			tHstSystemData.bAntiSkewScr = TRUE ;
		}
	}
}

/****************************************************************************/
/*
	Function:		sCanEnableReadProtection()
	Description:	Detects if the bootstrap allows IFLASH read protection.
	Parameters:		None.
	Return Value:	TRUE = read protection allowed
					FALSE = bootstrap prevents read protection
	Notes:			Bootstrap 286099501 will disable read protection if
					it detects that it is enabled.  When this happens,
					all IFLASH gets erased.
					Added for J10012-20976
*/
/****************************************************************************/
#ifdef NDEBUG
static tBOOL sCanEnableReadProtection(tVOID)
{
	return (memcmp(((tCOMPONENT_HEADER *) BOOTSTRAP_HEADER_LOCATION)->caComponentPartNumber, BAD_BOOTSTRAP, LENGTH_PART_NUMBER)) ;
}
#endif

/****************************************************************************/
/*
	Function:		sCopyVariantOnDebugBuild()
	Description:	Copies the variant to the correct XFLASH page on debug build
	Parameters:		None.
	Return Value:	None.
	Notes:			Added for J10012-15626
*/
/****************************************************************************/
#ifndef NDEBUG
static tVOID sCopyVariantOnDebugBuild(tVOID)
{	
		//May need to copy variant
	if (mSYS_XflashNeedsUpdate())
	{
			//This kicks off XFlash erasure
		SYS_XflashUpdateCheck() ;
			//Perform next step of copy
		while (SYS_XflashUpdateExec())
		{
				//Handle copy error
			if (mSYS_XflashUpdateIsError())
			{
				SYS_OutOfOrder(FAULT_XFLASH) ;	
			}
			LIB_PetWatchdog() ;
		}
			//Reset so component verification flags get set properly
		LIB_Reset() ;
	}	
}
#endif	
/****************************************************************************/
/*
	Function:		
	Description:	
	Parameters:		
	Return Value:	
	Notes:			
*/
/****************************************************************************/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
