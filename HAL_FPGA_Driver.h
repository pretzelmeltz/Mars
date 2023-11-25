/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* COPYRIGHT                                                                */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
	(c) 2008 MEI.  All rights reserved.
	Contains proprietary information, copyright and database rights MEI.
	Decompilation prohibited save as permitted by law. No using, disclosing,
	reproducing, accessing or modifying without MEI's prior written consent.

	MEI
	1301 Wilson Drive, West Chester, PA 19380 USA
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FILE DESCRIPTION                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
	Project:			10012
	Filename:			HAL_FPGA_Driver.h
	File Description:	FPGA definitions.
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* SOURCE CONTROL                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
#ifndef _HAL_FPGA_DRIVER_H_
#define _HAL_FPGA_DRIVER_H_
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* INCLUDES                                                                 */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* DEFINES                                                                  */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#define	XFPGA_ADDRESS					((tUINT32) 0x6C000000L)				//Base address.

#define	mup_FPGA_REG_BC_DAC_0			((tUINT8 *) (XFPGA_ADDRESS + 0))
#define	mup_FPGA_REG_BC_DAC_1			((tUINT8 *) (XFPGA_ADDRESS + 1))
#define	mup_FPGA_REG_BC_DAC_2			((tUINT8 *) (XFPGA_ADDRESS + 2))
#define	mup_FPGA_REG_BC_DAC_3			((tUINT8 *) (XFPGA_ADDRESS + 3))
#define	mup_FPGA_REG_BC_CTL				((tUINT8 *) (XFPGA_ADDRESS + 4))
#define	mup_FPGA_REG_CC_DAC				((tUINT8 *) (XFPGA_ADDRESS + 5))
#define	mup_FPGA_REG_MTR_CTL			((tUINT8 *) (XFPGA_ADDRESS + 6))
#define	mup_FPGA_REG_GPIO				((tUINT8 *) (XFPGA_ADDRESS + 7))

//The following bits are defined for mup_FPGA_REG_BC_CTL...
#define	BIT_BC_ENABLE					((tUINT8) BIT_0)
#define BIT_BC_DAC_0_EN					((tUINT8) BIT_1)
#define BIT_BC_DAC_1_EN                 ((tUINT8) BIT_2)
#define BIT_BC_DAC_2_EN                 ((tUINT8) BIT_3)
#define BIT_BC_DAC_3_EN                 ((tUINT8) BIT_4)
#define BIT_BC_TWO_SENSOR               ((tUINT8) BIT_5)

//The following bits are defined for mup_FPGA_REG_MTR_CTL...
#define	BIT_MTR_TACH_EN					((tUINT8) BIT_0)    //1 = Enabled.
#define	BIT_MTR_TRANSPORT_ENABLE		((tUINT8) BIT_1)	//1 = Motor enabled.
#define	BIT_MTR_STACKER_ENABLE			((tUINT8) BIT_2)	//1 = Motor enabled.
#define BIT_MTR_CTL_MASK                ((tUINT8)(BIT_MTR_TACH_EN|BIT_MTR_TRANSPORT_ENABLE|BIT_MTR_STACKER_ENABLE))

//The following bits are defined for mup_FPGA_REG_GPIO...
#define	BIT_GPIO_START_LED				((tUINT8) BIT_0)	//1 = START LED enable.
#define	BIT_GPIO_HOME_LED				((tUINT8) BIT_1)	//1 = HOME LED enable.

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* ENUMS and STRUCTURES                                                     */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* VARIABLES                                                                */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FUNCTION PROTOTYPES                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
#endif	//This file.
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

