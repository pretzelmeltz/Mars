/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* COPYRIGHT                                                                */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*
	(c) 2015 Crane Payment Innovations, Inc. All rights reserved.
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
	Filename:			BC_Barcode.c
	File Description:	Barcode Algorithms.
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
#include "BC_I2of5.h"
#include "BC_Barcode.h"
#include "ACQ_AcquisitionDataApi.h"		//J10012-15627.

/*
	BC_Barcode.c and BC_I2of5.c were ported over from SC. Changes were
	basically limited to those needed to meet NPD coding standards. In some
	cases, but not all, variables of type tUINT16 or tINT16 were changed to
	tUINT8 or tINT8 since the ARM process is a byte based machine while the
	SC is native word based.
*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* DEFINES                                                                  */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#define BAR_CODE_CHARACTER_COUNT_MIN    		((tUINT16) 16)
#define BAR_CODE_CHARACTER_COUNT_MAX    		((tUINT16) 24)

#define DELTA_BUFFER_SIZE               		((tUINT16) 170)

//#define SCAN_SIZE                  				((tINT16) 8192)		//Samples per document.

#define AGC_ELEMENT_COUNT             			((tUINT16) 6)		//Left or right of center.

#define QUIET_ZONE_FACTOR_2           			((tUINT16) 9)		//X Narrows, for 2 known deltas.
#define QUIET_ZONE_FACTOR_3           			((tUINT16) 6)		//X Narrows, for 3 known deltas.
#define QUIET_ZONE_WIDTH_MAX        			((tUINT16) 167)		//Samples.

#define TREND_PEAK_COUNT_MAX          			((tUINT16) 4)

#define LOW_PASS_ORDER               			((tINT16) 26)

//J10012-19928.Changed from 75 to 120 to fit broader units with different barcode noise level.  
#define Y_PRIME_THRESHOLD_MIN					((tINT16) 120)		//Counts (set above noise floor).
#define Y_PRIME_THRESHOLD_PERCENT     			((tINT16) 8)		//Percent of scale.
#define Y_PRIME_THRESHOLD_FACTOR  				(Y_PRIME_THRESHOLD_PERCENT * 0x10000 / 100)
#define BUFFER_A_SIZE               			((tINT16) 300)
#define BUFFER_B_SIZE               			((tINT16) 600)
#define Y_PRIME_HYSTERESIS           			((tINT16) 10)		//Counts.
#define Y_PRIME_PRIME_HYSTERESIS      			((tINT16) 2) 		//Counts.
#define FRACTION_SIZE                			((tINT16) 16)		//Bits.


#define msAbsoluteValue(x)						(x >= 0 ? x : -x)	//Absolute value (inline).

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* ENUMS and STRUCTURES                                                     */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

typedef enum _BC_STATE
{
    BC_STATE_NEED_Y_PRIME,
	BC_STATE_NEED_Y_PRIME_PRIME_DEFERRED_Y_PRIME,
	BC_STATE_NEED_Y_PRIME_PRIME
} eBC_STATE ;

typedef struct _BARCODE_DSP
{
	tUINT16 *uwpDelta ;					//Place to store deltas.
	tUINT16 uwDeltaCount ;				//Number of deltas found.
	tUINT16 uwDeltaCountMax ;			//Maximum number of deltas permitted.
} tBARCODE_DSP ;

typedef struct _TREND                   	//History queue for sliding window of demodulator.
{
    tUINT16 uwDeltaMinus1 ;
    tUINT16 uwDelta0 ;
    tUINT16 uwDelta1 ;
    tUINT16 uwIndexMinus2 ;
    tUINT16 uwIndexMinus1 ;
    tUINT16 uwIndex0 ;
    tUINT16 uwIndex1 ;
    tINT16  wYPrimeMinus1 ;
    tINT16  wYPrime0 ;
    tINT16  wYPrime1 ;
    tUINT16 uwPeakCount ;
} tTREND ;

typedef struct _AGC_TREND                   //History queue for sliding window of AGC.
{
    tUINT16 uwLeftDeltaMinus2 ;
    tUINT16 uwLeftDeltaMinus1 ;
    tUINT16 uwLeftDelta0 ;
    tUINT16 uwRightDelta0 ;
    tUINT16 uwRightDelta1 ;
    tUINT16 uwRightDelta2 ;
    tUINT16 uwLeftIndexMinus1 ;
    tUINT16 uwRightIndexMinus1 ;
} tAGC_TREND ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GLOBAL VARIABLES                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

tBARCODE_DECODE tBarCodeDecode ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* LOCAL VARIABLES                                                          */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

static tBARCODE_DECODE tBarCodeDecodePromo ;

static eBC_STATE esBcState ;

static tUINT16 uwaDelta[DELTA_BUFFER_SIZE] ;

static tUINT16 uwsSampleIndex ;

static tBARCODE_DSP *tpsDsp ;

static tTREND tsTrend ;
static tAGC_TREND tsAgcTrend ;

static tINT16 wsYPrimeMin ;
static tINT16 wsYPrimeMax ;
static tINT16 wsYPrimeThreshold ;

static tUINT16 uwsBufferACount ;
static tUINT16 uwsBufferBCount ;

static tINT16 wsYPrimeHysteresis ;
static tINT16 wsYPrimePrimeHysteresis ;

static tINT16 wsY1 ;
static tINT16 wsY2 ;
static tINT16 wsYPrime0 ;
static tINT16 wsYPrime1 ;
static tINT16 wsYPrimePrime0 ;
static tINT16 wsYPrimePrime1 ;
static tUINT16 uwYPrimeDeferredIndex ;

static const tINT16 wasLowPassFilterB[LOW_PASS_ORDER + 1] =
{
    37,    107,    215,    367,    564,    802,   1073,   1363,   1656,
  1932,   2173,   2360,   2478,   2518,   2478,   2360,   2173,   1932,
  1656,   1363,   1073,    802,    564,    367,    215,    107,     37
} ;
static tINT16 wasLowPassBuffer[LOW_PASS_ORDER + 1] ;

static tUINT16 uwasBufferA[BUFFER_A_SIZE] ;
static tUINT16 uwasBufferB[BUFFER_B_SIZE] ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FUNCTION PROTOTYPES                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

static tVOID sProcessSignal(tBARCODE_DSP *tp_BarCodeDsp) ;
static tVOID sFindYpPeaks(tVOID) ;
static tVOID sDemodulate(tVOID) ;

static tVOID sAgc(tUINT16 uw_CenterIndexIndex) ;
static tBOOL sAgcFindPeakLeft(tUINT16 *uwp_StartIndexIndex, tBOOL *bp_Restart) ;
static tBOOL sAgcFindPeakRight(tUINT16 *uwp_StartIndexIndex, tBOOL *bp_Restart) ;
static tBOOL sAgcPeakExceedsMinMax(tINT16 w_YPrimePeak) ;
static tBOOL sAgcLeftQuietZone(tUINT16 uw_PeakIndex) ;
static tBOOL sAgcRightQuietZone(tUINT16 uw_PeakIndex) ;
static tVOID sAgcComputeNewThreshold(tVOID) ;
static tVOID sProcessYPrimePeak(tUINT16 uw_IndexIndex) ;
static tBOOL sQuietOnRight(tVOID) ;
static tBOOL sQuietInMiddle(tVOID) ;
static tVOID sPeakAccept(tVOID) ;
static tVOID sPeakAcceptKillLast(tVOID) ;
static tVOID sFlushTrend(tVOID) ;
static tVOID sPutDelta(tUINT16 uw_Delta) ;
static tINT16 sFixedPointToInteger(tINT32 ul_Number) ;

static tVOID sComputeHysteresis(tVOID) ;
static tVOID sPutBufferA(tUINT16 uIndex) ;
static tVOID sPutBufferB(tUINT16 uIndex, tINT16 nYp) ;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* CODE                                                                     */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/****************************************************************************/
/*
	Function:		BC_BarcodeStart()
	Description:	Initializes the Barcode system for data gathering.
					Called when sampling begins.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
tVOID BC_BarcodeStart(tVOID)
{
    uwsSampleIndex = 0 ;
    uwsBufferACount = uwsBufferBCount = 0 ;
    wsYPrimeHysteresis = wsYPrimePrimeHysteresis = 0 ;
    esBcState = BC_STATE_NEED_Y_PRIME ;
	memset(&wasLowPassBuffer[0], 0, sizeof(wasLowPassBuffer)) ;
}

/****************************************************************************/
/*
	Function:		BC_BarcodeSample()
	Description:	Processes each barcode sample.
					Due to the large quantities of samples and the limited
					amount of post-processing time, the samples are reduced and
					stored in two static buffers.  The first buffer (A)
					contains the indices of y' zero crossings while the second
					(B) contains the indices of the y" zero crossings along
					with their corresponding y' values.
					
					A low pass filter precedes the derivative calculations so
					that electrical noise is effectively eliminated.
					
					Hysteresis is used to avoid unecessary generation of
					edges which otherwise appear when a signal dithers around
					zero.
	Parameters:		w_Sample = sample to process.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
tVOID BC_BarcodeSample(tINT16 w_Sample)
{
    wsY1 = wsY2 ;                         //Keep recent data so that derivatives can be calculated.
    wsYPrime0 = wsYPrime1 ;
    wsYPrimePrime0 = wsYPrimePrime1 ;
		//Filter low frequencies...
	wsY2 = BC_FirFilter(w_Sample) ;

		// J10012-15627. Moved adjusting reading and inverting here after filtering.
    wsY2 = ((NUM_BARCODE_SAMPLES - 1) - (wsY2 << 3)) ;

    wsYPrime1 = wsY2 - wsY1 ;             //Compute y'.
    if ((wsYPrime0 ^ wsYPrime1) < 0)      //Store index when y' crosses zero.
    {
        if ((wsYPrime1 ^ wsYPrimeHysteresis) < 0)
        {
            if (esBcState == BC_STATE_NEED_Y_PRIME)
            {
                sPutBufferA(uwsSampleIndex) ;
                esBcState = BC_STATE_NEED_Y_PRIME_PRIME ;
            }
            else
            {
                uwYPrimeDeferredIndex = uwsSampleIndex ;
                esBcState = BC_STATE_NEED_Y_PRIME_PRIME_DEFERRED_Y_PRIME ;
            }
        }
    }
		//Compute y"...
    wsYPrimePrime1 = wsYPrime1 - wsYPrime0 ;
		//Store index and y' when y" crosses zero and y' bows outward...
    if ((wsYPrimePrime0 ^ wsYPrimePrime1) < 0)
    {
        if ((wsYPrimePrime1 ^ wsYPrimePrimeHysteresis) < 0)
        {
            if ((wsYPrimePrime0 ^ wsYPrime1) >= 0)
            {
                if (msAbsoluteValue(wsYPrime1) >= Y_PRIME_THRESHOLD_MIN)
                {
                    if (esBcState == BC_STATE_NEED_Y_PRIME_PRIME_DEFERRED_Y_PRIME)
                    {
                        sPutBufferA(uwYPrimeDeferredIndex) ;
                    }
                    sPutBufferB(uwsSampleIndex, wsYPrime1) ;
                    esBcState = BC_STATE_NEED_Y_PRIME ;
                }
            }
        }
    }
    sComputeHysteresis() ;
    uwsSampleIndex++ ;
}

/****************************************************************************/
/*
	Function:		BC_BarcodeDecode()
	Description:	Decodes the data read by the bar code sensor.
	Parameters:		cp_Destination = place to store the decoded barcode characters.
					up_Length = place to store the number of characters found.
					cp_PromoDestination = place to store the promotional barcode characters (if any).
					up_PromoLength = place to store the number of characters found.
	Return Value:	
	Notes:			
*/
/****************************************************************************/
tBOOL BC_BarcodeDecode(tCHAR *cp_Destination, tUINT8 *up_Length, tCHAR *cp_PromoDestination, tUINT8 *up_PromoLength)
{
    tBARCODE_DSP t_BarCodeDSP ;

    tBarCodeDecode.bValid = FALSE ;					//Assume invalid barcode voucher.
    *up_Length = 0 ;                     			//Assume 0 valid barcode characters.
    tBarCodeDecode.uwCharacterCountMin = BAR_CODE_CHARACTER_COUNT_MIN ;
    tBarCodeDecode.uwCharacterCountMax = BAR_CODE_CHARACTER_COUNT_MAX ;
    tBarCodeDecode.cpCharacter = cp_Destination ;
		//Process signal into deltas...
    t_BarCodeDSP.uwpDelta = &uwaDelta[0] ;
    t_BarCodeDSP.uwDeltaCountMax = DELTA_BUFFER_SIZE ;
    sProcessSignal(&t_BarCodeDSP) ;
		//Decode deltas into characters....
    tBarCodeDecode.uwpDelta = &uwaDelta[0] ;
    tBarCodeDecode.uwDeltaCount = t_BarCodeDSP.uwDeltaCount ;
	tBarCodeDecode.cpPromoCharacter = cp_PromoDestination ;	
	tBarCodeDecode.uwPromoCharacterCountExpected = BAR_CODE_PROMO_CHARACTER_COUNT_EXPECTED ;

	tBarCodeDecodePromo.bValid = tBarCodeDecode.bValid ;
	tBarCodeDecodePromo.uwCharacterCountMin = tBarCodeDecode.uwCharacterCountMin ;
	tBarCodeDecodePromo.uwCharacterCountMax = tBarCodeDecode.uwCharacterCountMax ;
		//Use the Promo Destination buffer, since this struct is use to detect the Promo barcode...
	tBarCodeDecodePromo.cpCharacter = cp_PromoDestination ;
	tBarCodeDecodePromo.uwpDelta = tBarCodeDecode.uwpDelta ;
	tBarCodeDecodePromo.uwDeltaCount = tBarCodeDecode.uwDeltaCount ;

	tBarCodeDecodePromo.bPromoValid = FALSE;
	tBarCodeDecodePromo.uwPromoCharacterCountExpected = BAR_CODE_PROMO_CHARACTER_COUNT_EXPECTED ;
	tBarCodeDecodePromo.cpPromoCharacter = cp_PromoDestination ;	
	
	BC_I2of5Decode(&tBarCodeDecode, &tBarCodeDecodePromo) ;
    *up_Length = (tUINT8) tBarCodeDecode.uwCharacterCount ;
	*up_PromoLength = (tUINT8) tBarCodeDecode.uwPromoCharacterCount;
    return (tBarCodeDecode.bValid | tBarCodeDecode.bPromoValid) ;
}

/****************************************************************************/
/*
	Function:		sProcessSignal()
	Description:    Reduces the data gathered by the interrupt to a series of
					deltas representing the widths of the bars and spaces.
	Parameters:		tp_BarCodeDsp.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sProcessSignal(tBARCODE_DSP *tp_BarCodeDsp)
{
    tpsDsp = tp_BarCodeDsp ;
    tpsDsp->uwDeltaCount = 0 ;

    sFindYpPeaks() ;
    sDemodulate() ;
}

/****************************************************************************/
/*
	Function:		sFindYpPeaks()
	Description:    Reduces the data collected by the interrupt into a series
					of y' peaks which can be fed into a demodulator.

					Interrupt data is provided by buffers A and B.  Buffer A
					holds y' zero crossing indices.  Buffer B holds y" zero
					crossing indices interlaced with y' values at those points.
					
					The output is placed back in buffers A and B.  Buffer A
					holds the y' peak indices while buffer B holds the y'
					peaks.  The original y' zero crossings information is
					discarded since it is no longer needed.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sFindYpPeaks(tVOID)
{
    tUINT16 *uwp_YPrimeZeroCrossIndex ;			//Inputs to algorithm.
    tUINT16 *uwp_YPrimePrimeX ;
    tUINT16 *uwp_YPrimeIndex ;					//Outputs from algorithm.
    tUINT16 *uwp_YPrime ;
    tUINT16 uw_IndexLeft ;
    tUINT16 uw_IndexRight ;
    tUINT16 uw_IndexCenter ;
    tUINT16 uw_Index ;
    tINT16 w_Delta ;
    tUINT16 uw_DeltaMin ;
    tUINT16 i, j, k ;

	uwp_YPrimeZeroCrossIndex = &uwasBufferA[0] ;
	uwp_YPrimePrimeX = &uwasBufferB[0] ;
	uwp_YPrimeIndex = &uwasBufferA[0] ;
	uwp_YPrime = &uwasBufferB[0] ;

    uw_IndexRight = *uwp_YPrimeZeroCrossIndex++ ;
    for (i = 1, j = k = 0; i < uwsBufferACount; i++)
    {
			//Establish y' zero crossing pair and center...
        uw_IndexLeft = uw_IndexRight ;
        uw_IndexRight = *uwp_YPrimeZeroCrossIndex++ ;
        uw_IndexCenter = (uw_IndexLeft + uw_IndexRight) >> 1 ;

        uw_DeltaMin = UINT16_MAX ;        			//Find closest y" zero crossing.
        for ( ; j < uwsBufferBCount; j += 2, uwp_YPrimePrimeX += 2)
        {
            if ((uw_Index = *uwp_YPrimePrimeX) > uw_IndexRight)
            {
                break ;
            }
            w_Delta = (tINT16) (uw_Index - uw_IndexCenter) ;
            w_Delta = msAbsoluteValue(w_Delta) ;
            if (w_Delta > uw_DeltaMin)
            {
                break ;
            }
            uw_DeltaMin = w_Delta ;
        }
        if (uw_DeltaMin < UINT16_MAX)
        {
            uwp_YPrimePrimeX -= 2 ;       			//Store y' peak data.
            *uwp_YPrimeIndex++ = *uwp_YPrimePrimeX++ ;
            *uwp_YPrime++ = *uwp_YPrimePrimeX++ ;
            k++ ;
        }
    }
    uwsBufferACount = uwsBufferBCount = k ;
}

/****************************************************************************/
/*
	Function:		sDemodulate()
	Description:    Converts the series of y' peaks into a series of deltas.

					Not all y' peaks translate into deltas.  Some are ignored
					because of amplitude and polarity.  AGC is used at this
					level to weed out those of unfavorable amplitude.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sDemodulate(tVOID)
{
    tINT16 *uwp_YPrime ;
    tINT16 w_YPrime ;
    tUINT16 i ;

	uwp_YPrime = (tINT16 *) &uwasBufferB[0] ;
    memset(&tsTrend, 0, sizeof(tTREND)) ;  		//Start new trend queue.
    for (i = 0; i < uwsBufferACount; i++)
    {
        sAgc(i) ;                       		//Adjust gain for this location.
        w_YPrime = *uwp_YPrime++ ;         		//Ignore small amplitudes.
        if (msAbsoluteValue(w_YPrime) >= wsYPrimeThreshold)
        {
            sProcessYPrimePeak(i) ;     		//Continue demodulation.
        }
    }
		//Put extra peak at end of scan to account for end of document...
    tsTrend.uwIndex1 = SCAN_SIZE - 1 ;
    tsTrend.uwDelta1 = - tsTrend.uwIndex0 + tsTrend.uwIndex1 ;
    if (tsTrend.uwDelta1 > 0)
    {
        tsTrend.wYPrime1 = Y_PRIME_THRESHOLD_MIN ;
        tsTrend.uwPeakCount++ ;
        sPeakAccept() ;
    }
    sFlushTrend() ;		                     	//Flush out trapped data from trend queue.
}

/****************************************************************************/
/*
	Function:		sAgc()
	Description:    Calculates a y' threshold from the y' peaks
					detected immediately around a given point in a series of
					p's.  The result is a value which is proportional to the
					difference between the largest positive and negative peaks
					within a fixed window.  To prevent corruption from
					neighboring signals, the window is prevented from crossing
					quiet zones.
	Parameters:     uw_CenterIndexIndex : Zero based index of starting point in y' buffers (A & B).
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sAgc(tUINT16 uw_CenterIndexIndex)
{
    tUINT16 i ;
    tUINT16 j ;
    tUINT16 uw_CenterIndex ;
    tBOOL b_EnableLeft ;
    tBOOL b_EnableRight ;
    tUINT16 uw_PeakCountLeft ;
    tUINT16 uw_PeakCountRight ;
    tBOOL b_Restart ;

	i = j = uw_CenterIndexIndex ;
    uw_CenterIndex = uwasBufferA[uw_CenterIndexIndex] ;
    b_EnableLeft = TRUE ;
    b_EnableRight = TRUE ;
    uw_PeakCountLeft = 0 ;
    uw_PeakCountRight = 0 ;
		//Start Agc trend queue...
    memset(&tsAgcTrend, 0, sizeof(tAGC_TREND)) ;
    sAgcLeftQuietZone(uw_CenterIndex) ;
    sAgcRightQuietZone(uw_CenterIndex) ;
		//Start near noise level and go up...
    wsYPrimeThreshold = Y_PRIME_THRESHOLD_MIN ;
		/*
			The starting Y'Max is "that threshold which the noise threshold
			is 8% of" (or whatever % is Y_PRIME_THRESHOLD_PERCENT) i.e. this
			is the "100%" level. The div 2 is because the "100% level" is
			split here between plus and minus, whereas the noise threshold
			Y_PRIME_THRESHOLD_MIN was defined one-sided (i.e. the noise level
			is plus or minus that amount).
		*/
    wsYPrimeMax = +Y_PRIME_THRESHOLD_MIN * 100 / 2 / Y_PRIME_THRESHOLD_PERCENT ;
    wsYPrimeMin = -Y_PRIME_THRESHOLD_MIN * 100 / 2 / Y_PRIME_THRESHOLD_PERCENT ;
    while (b_EnableLeft || b_EnableRight)
    {
        if (b_EnableLeft)                	//Find peak on left.
        {
            if (sAgcFindPeakLeft(&i, &b_Restart))
            {
                if (++uw_PeakCountLeft >= AGC_ELEMENT_COUNT)
                {
                    b_EnableLeft = FALSE ;
                }
            }
            else
            {
                b_EnableLeft = FALSE ;
            }
        }
        if (b_EnableRight && !b_Restart)	//Find peak on right.
        {
            if (sAgcFindPeakRight(&j, &b_Restart))
            {
                if (++uw_PeakCountRight >= AGC_ELEMENT_COUNT)
                {
					b_EnableRight = FALSE ;
                }
            }
            else
            {
				b_EnableRight = FALSE ;
            }
        }
        if (b_Restart)                   	//Group has bigger peak: restart.
        {
            i = j = uw_CenterIndexIndex ;
            memset(&tsAgcTrend, 0, sizeof(tAGC_TREND)) ;
            sAgcLeftQuietZone(uw_CenterIndex) ;
            sAgcRightQuietZone(uw_CenterIndex) ;
            uw_PeakCountLeft = uw_PeakCountRight = 0 ;
            b_EnableLeft = b_EnableRight = TRUE ;
        }
    }
}

/****************************************************************************/
/*
	Function:		sAgcFindPeakLeft()
	Description:    Continues the Agc algorithm by finding the
					next peak to the left of a given starting point, running
					the peak through a peak (min/max) detector, and, based upon
					the size of the peak, making a recommendation to the caller
					as to whether or not to restart the Agc process.
					
					The list is searched to the left until the search limit
					is reached OR a Y' is found > Y'Threshold. If no such Y'
					is found, then "found" and "restart" are both rtnd FALSE.
					
					If a Y' is found greater than Y'Threshold, then the
					"restart" and "found" outputs are decided according
					to this truth table.
					
					Rel. of Y' to current   Are we in "left"   Restart  Found
					   MinMax window ?       quiet-zone?
					--------------------   ----------------   -------  -----
						 Outside             Not Q-zone        TRUE    FALSE
						 Inside               "     "          FALSE   TRUE
							X                Is in Q-zone      FALSE   FALSE
	Parameters:     uwp_StartIndexIndex - Zero based index of starting point
                    bp_Restart         - Recommendation to restart algorithm
	Return Value:	TRUE if found.
	Notes:			
*/
/****************************************************************************/
static tBOOL sAgcFindPeakLeft(tUINT16 *uwp_StartIndexIndex, tBOOL *bp_Restart)
{
    tUINT16 *uwp_YPrimeIndex ;
    tUINT16 *uwp_YPrime ;
    tINT16 w_YPrime ;
    tUINT16 uw_Index ;
    tUINT16 i ;
    tBOOL b_Found ;

    uwp_YPrimeIndex = &uwasBufferA[*uwp_StartIndexIndex - 1] ;
    uwp_YPrime = &uwasBufferB[*uwp_StartIndexIndex - 1] ;
    i = *uwp_StartIndexIndex ;
    b_Found = FALSE ;
	*bp_Restart = FALSE ;
    if (i-- > 0)
    {
        for (; i < UINT16_MAX; i--)     //Consider every y' peak of minimum magnitude.
        {
            uw_Index = *uwp_YPrimeIndex-- ;
            w_YPrime = *uwp_YPrime-- ;
            if (msAbsoluteValue(w_YPrime) >= wsYPrimeThreshold)	//Stop on quiet zone.
            {
                if (!sAgcLeftQuietZone(uw_Index))
                {
						//Test peak against min/max...
                    if (sAgcPeakExceedsMinMax(w_YPrime))
                    {
                        *bp_Restart = TRUE ;
                    }
                    else
                    {
                        b_Found = TRUE ;
                    }
                }
                break ;
            }
        }
        *uwp_StartIndexIndex = i ;		//Mark current position for next iteration.
    }
    return (b_Found) ;
}

/****************************************************************************/
/*
	Function:		sAgcFindPeakRight()
	Description:    Continues the Agc algorithm by finding the
					next peak to the right of a given starting point, running
					the peak through a peak (min/max) detector, and, based upon
					the size of the peak, making a recommendation to the caller
					as to whether or not to restart the Agc process.
					
					See process description for sAgcFindPeakLeft.
	Parameters:     uwp_StartIndexIndex - Zero based index of starting point
                    bp_Restart         - Recommendation to restart algorithm
	Return Value:	TRUE if found.
	Notes:			
*/
/****************************************************************************/
static tBOOL sAgcFindPeakRight(tUINT16 *uwp_StartIndexIndex, tBOOL *bp_Restart)
{
    tUINT16 *uwp_YPrimeIndex ;
    tUINT16 *uwp_YPrime ;
    tINT16 w_YPrime ;
    tUINT16 uw_Index ;
    tUINT16 i ;
    tBOOL b_Found ;

    uwp_YPrimeIndex = &uwasBufferA[*uwp_StartIndexIndex + 1] ;
    uwp_YPrime = &uwasBufferB[*uwp_StartIndexIndex + 1] ;
    i = *uwp_StartIndexIndex ;
    b_Found = FALSE ;
    *bp_Restart = FALSE ;
    if (++i < uwsBufferACount)
    {
        for (; i < uwsBufferACount; i++)  //Consider every y' peak of  minimum magnitude.
        {
            uw_Index = *uwp_YPrimeIndex++ ;
            w_YPrime = *uwp_YPrime++ ;
            if (msAbsoluteValue(w_YPrime) >= wsYPrimeThreshold)	//Stop on quiet zone.
            {
                if (!sAgcRightQuietZone(uw_Index))
                {
						//Test peak against min/max...
                    if (sAgcPeakExceedsMinMax(w_YPrime))
                    {
                        *bp_Restart = TRUE ;
                    }
                    else
                    {
                        b_Found = TRUE ;
                    }
                }
                break ;
            }
        }
        *uwp_StartIndexIndex = i ;        //Mark current position for next iteration.
    }
    return (b_Found) ;
}

/****************************************************************************/
/*
	Function:		sAgcPeakExceedsMinMax()
	Description:    Determines if a given peak exceeds the
					limits of a peak (min/max) detector.  The min/max values
					and the threshold are adjusted in the process.
	Parameters:     w_YPrimePeak   - Peak to be considered
	Return Value:   TRUE if peak exceeds min/max.
	Notes:			
*/
/****************************************************************************/
static tBOOL sAgcPeakExceedsMinMax(tINT16 w_YPrimePeak)
{
    tBOOL b_Exceed ;

	b_Exceed = FALSE ;
    if (w_YPrimePeak > wsYPrimeMax)       //Test against maximum.
    {
        wsYPrimeMax = w_YPrimePeak ;
        b_Exceed = TRUE ;
    }
    else if (w_YPrimePeak < wsYPrimeMin)  //Test against minimum.
    {
        wsYPrimeMin = w_YPrimePeak ;
        b_Exceed = TRUE ;
    }
    if (b_Exceed)                         //Compute new threshold.
    {
        sAgcComputeNewThreshold() ;
    }
    return (b_Exceed) ;
}

/****************************************************************************/
/*
	Function:		sAgcLeftQuiteZone()
	Description:    Determines if the most recent peak (on the
					left) makes a quiet zone with respect to it neighboring
					peaks.  The algorithm uses as much trend information as
					possible to make the determination.
	Parameters:     uw_PeakIndex = Zero based index of new peak.
	Return Value:   TRUE if peak makes quiet zone.
	Notes:			
*/
/****************************************************************************/
static tBOOL sAgcLeftQuietZone(tUINT16 uw_PeakIndex)
{
	tBOOL b_Quiet ;

	b_Quiet = FALSE ;
		//Compute delta and store in trend buffer...
    tsAgcTrend.uwLeftDelta0 = tsAgcTrend.uwLeftDeltaMinus1 ;
    tsAgcTrend.uwLeftDeltaMinus1 = tsAgcTrend.uwLeftDeltaMinus2 ;
    if (tsAgcTrend.uwLeftIndexMinus1 > 0)
    {
        tsAgcTrend.uwLeftDeltaMinus2 = tsAgcTrend.uwLeftIndexMinus1 - uw_PeakIndex ;
    }
    tsAgcTrend.uwLeftIndexMinus1 = uw_PeakIndex ;
		//Test for quiet zone, use single big delta...
    if (tsAgcTrend.uwLeftDeltaMinus2 >= QUIET_ZONE_WIDTH_MAX)
    {
        b_Quiet = TRUE ;
    }
    else if (tsAgcTrend.uwLeftDelta0 > 0)  //Test for quiet zone, use 3 deltas.
    {
        if (tsAgcTrend.uwLeftDeltaMinus2 >= (tsAgcTrend.uwLeftDeltaMinus1 + tsAgcTrend.uwLeftDelta0) * QUIET_ZONE_FACTOR_3)
        {
            b_Quiet = TRUE ;
        }
    }
		//Test for quiet zone, use 2 deltas...
    else if (tsAgcTrend.uwLeftDeltaMinus1 > 0)
    {
        if (tsAgcTrend.uwLeftDeltaMinus2 >= (tsAgcTrend.uwLeftDeltaMinus1 * QUIET_ZONE_FACTOR_2))
        {
            b_Quiet = TRUE ;
        }
    }
    return (b_Quiet) ;
}

/****************************************************************************/
/*
	Function:		sAgcRightQuiteZone()
	Description:    Determines if the most recent peak (on the
					right) makes a quiet zone with respect to it neighboring
					peaks.  The algorithm uses as much trend information as
					possible to make the determination.
	Parameters:     uw_PeakIndex = Zero based index of new peak.
	Return Value:   TRUE if peak makes quiet zone.
	Notes:			
*/
/****************************************************************************/
static tBOOL sAgcRightQuietZone(tUINT16 uw_PeakIndex)
{
    tBOOL b_Quiet ;

	b_Quiet = FALSE ;
		//Compute delta and store in trend buffer...
    tsAgcTrend.uwRightDelta0 = tsAgcTrend.uwRightDelta1 ;
    tsAgcTrend.uwRightDelta1 = tsAgcTrend.uwRightDelta2 ;
    if (tsAgcTrend.uwRightIndexMinus1 > 0)
    {
        tsAgcTrend.uwRightDelta2 = uw_PeakIndex - tsAgcTrend.uwRightIndexMinus1 ;
    }
    tsAgcTrend.uwRightIndexMinus1 = uw_PeakIndex ;
		//Test for quiet zone, use single big delta...
    if (tsAgcTrend.uwRightDelta2 >= QUIET_ZONE_WIDTH_MAX)
    {
        b_Quiet = TRUE ;
    }
    else if (tsAgcTrend.uwRightDelta0 > 0) //Test for quiet zone, use 3 deltas.
    {
        if (tsAgcTrend.uwRightDelta2 >= (tsAgcTrend.uwRightDelta1 + tsAgcTrend.uwRightDelta0) * QUIET_ZONE_FACTOR_3)
        {
			b_Quiet = TRUE ;
        }
    }
    else if (tsAgcTrend.uwRightDelta1 > 0) //Test for quiet zone, use 2 deltas.
    {
        if (tsAgcTrend.uwRightDelta2 >= (tsAgcTrend.uwRightDelta1 * QUIET_ZONE_FACTOR_2))
        {
			b_Quiet = TRUE ;
        }
    }
    return (b_Quiet) ;
}

/****************************************************************************/
/*
	Function:		sAgcComputeNewThreshold()
	Description:    Computes a new first derivative threshold
					based upon a previously obtained min/max combination.  The
					new threshold is set proportionally to the absolution
					difference in min/max.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sAgcComputeNewThreshold(tVOID)
{
    tINT32 l_ypT ;

    l_ypT = (tINT32) (wsYPrimeMax - wsYPrimeMin) * Y_PRIME_THRESHOLD_FACTOR ;
    wsYPrimeThreshold = sFixedPointToInteger(l_ypT) ;
}

/****************************************************************************/
/*
	Function:		sProcessYPrimePeak()
	Description:    Continues the demodulation process on a candidate y' peak.

					Here, the y' peaks are tested for polarity.  Peaks which
					don't fit an acceptable pattern are ignored.
					
					Normally, a good peak is indentified when its polarity is
					determined to be the opposite polarity as that of its
					previous neighbor.  When a quiet zone is encountered, the
					pattern is broken and a new one is started -- positive
					peak first.
	Parameters:     uw_Index = Zero based index of candidate peak.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sProcessYPrimePeak(tUINT16 uw_IndexIndex)
{
		//Load peak into trend queue (a position of test).
    tsTrend.uwIndex1 = uwasBufferA[uw_IndexIndex] ;
    tsTrend.uwDelta1 = - tsTrend.uwIndex0 + tsTrend.uwIndex1 ;
    tsTrend.wYPrime1 = (tINT16) uwasBufferB[uw_IndexIndex] ;
    tsTrend.uwPeakCount++ ;
    if (tsTrend.uwPeakCount <= 2)          	//Accept first and second peaks unconditionally.
    {
        sPeakAccept() ;
    }
    else if (sQuietOnRight())           	//Accept peak which makes quiet zone.
    {
        sPeakAccept() ;
    }
    else if (sQuietInMiddle())          	//Accept peak and make sure previous peak following quite zone is postivie.
    {
        if (tsTrend.wYPrime0 >= 0)
        {
            sPeakAccept() ;
        }
        else
        {
            sPeakAcceptKillLast() ;
        }
    }
    else                                	//Accept peak which continues to alternate polarity.
    {
        if ((tsTrend.wYPrime0 ^ tsTrend.wYPrime1) < 0)
        {
            sPeakAccept() ;
        }
        else                            	//Accept peak of same polarity only if bigger, then kill previous peak.
        {
            if (msAbsoluteValue(tsTrend.wYPrime0) < msAbsoluteValue(tsTrend.wYPrime1))
            {
                sPeakAcceptKillLast() ;
            }
        }
    }
}

/****************************************************************************/
/*
	Function:		sQuiteOnRight()
	Description:    Determines if the newest delta (on the right)
					in the trend queue is a quiet zone.  The algorithm uses
					as much trend information as possible to make the
					determination.
	Parameters:		None.
	Return Value:	TRUE if quiet zone.
	Notes:			
*/
/****************************************************************************/
static tBOOL sQuietOnRight(tVOID)
{
    tBOOL b_Quiet ;

	b_Quiet = FALSE ;
		//Use single big delta...
    if (tsTrend.uwDelta1 >= QUIET_ZONE_WIDTH_MAX)
    {
        b_Quiet = TRUE ;
    }
    else if (tsTrend.uwPeakCount > 3)      //Use 3 deltas.
    {
        if (((tsTrend.uwDeltaMinus1 + tsTrend.uwDelta0) * QUIET_ZONE_FACTOR_3) <= tsTrend.uwDelta1)
        {
            b_Quiet = TRUE ;
        }
    }
    else if (tsTrend.uwPeakCount > 2)      //Use 2 deltas.
    {
        if ((tsTrend.uwDelta0 * QUIET_ZONE_FACTOR_2) <= tsTrend.uwDelta1)
        {
            b_Quiet = TRUE ;
        }
    }
    return (b_Quiet) ;
}

/****************************************************************************/
/*
	Function:		sQuiteInMiddle()
	Description:    Cetermines if the second to newest delta (in
					the middle) in the trend queue is a quiet zone.  The
					algorithm uses as much trend information as possible to
					make the determination.
	Parameters:		None.
	Return Value:	TRUE if quiet zone.
	Notes:			
*/
/****************************************************************************/
static tBOOL sQuietInMiddle(tVOID)
{
    tBOOL b_Quiet ;

	b_Quiet = FALSE ;
		//Use single big delta...
    if (tsTrend.uwDelta0 >= QUIET_ZONE_WIDTH_MAX)
    {
        b_Quiet = TRUE ;
    }
    else if (tsTrend.uwPeakCount > 3)      //Use 3 deltas.
    {
        if (tsTrend.uwDelta0 >= ((tsTrend.uwDeltaMinus1 + tsTrend.uwDelta1) * QUIET_ZONE_FACTOR_3))
        {
            b_Quiet = TRUE ;
        }
    }
    else if (tsTrend.uwPeakCount > 2)      //Use 2 deltas.
    {
        if (tsTrend.uwDelta0 >= (tsTrend.uwDelta1 * QUIET_ZONE_FACTOR_2))
        {
            b_Quiet = TRUE ;
        }
    }
    return (b_Quiet) ;
}

/****************************************************************************/
/*
	Function:		sPeakAccept()
	Description:    Accepts the candidate peak at the end of the
					trend queue and outputs the old peak at the opposite end.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sPeakAccept(tVOID)
{
        //Output oldest delta...
    if (tsTrend.uwPeakCount >= TREND_PEAK_COUNT_MAX)
    {
        sPutDelta(tsTrend.uwDeltaMinus1) ;
        tsTrend.uwPeakCount-- ;
    }
		//Accept new peak...
    tsTrend.wYPrimeMinus1 = tsTrend.wYPrime0 ;
    tsTrend.wYPrime0 = tsTrend.wYPrime1 ;

    tsTrend.uwIndexMinus2 = tsTrend.uwIndexMinus1 ;
    tsTrend.uwIndexMinus1 = tsTrend.uwIndex0 ;
    tsTrend.uwIndex0 = tsTrend.uwIndex1 ;

    tsTrend.uwDeltaMinus1 = tsTrend.uwDelta0 ;
    tsTrend.uwDelta0 = tsTrend.uwDelta1 ;
}

/****************************************************************************/
/*
	Function:		sPeakAcceptKillLast()
	Description:    Accepts the candidate peak at the end of the
					trend queue while replacing the last peak.  The other end
					of the queue holds still.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sPeakAcceptKillLast(tVOID)
{
    tsTrend.wYPrime0 = tsTrend.wYPrime1 ;
	tsTrend.uwIndex0 = tsTrend.uwIndex1 ;
	tsTrend.uwDelta0 += tsTrend.uwDelta1 ;
	tsTrend.uwPeakCount-- ;
}

/****************************************************************************/
/*
	Function:		sFlushTrend()
	Description:    Outputs the peaks trapped in the trend queue
					at the time of call.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sFlushTrend(tVOID)
{
    if (tsTrend.uwPeakCount >= 3)          //Output oldest delta.
    {
        sPutDelta(tsTrend.uwDeltaMinus1) ;
        tsTrend.uwPeakCount-- ;
    }
    if (tsTrend.uwPeakCount >= 2)          //Output oldest delta.
    {
        sPutDelta(tsTrend.uwDelta0) ;
        tsTrend.uwPeakCount-- ;
    }
}

/****************************************************************************/
/*
	Function:		sPutDelta()
	Description:    Stores a given delta in the location
					specified by the module caller.  The deltas are stored in
					an array in the sequence that they occur.
	Parameters:		uw_Delta = Delta to be stored.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sPutDelta(tUINT16 uw_Delta)
{
    if (tpsDsp->uwDeltaCount < tpsDsp->uwDeltaCountMax)
    {
        *tpsDsp->uwpDelta++ = uw_Delta ;
        tpsDsp->uwDeltaCount++ ;
    }
}

/****************************************************************************/
/*
	Function:		sFixedPointToInteger()
	Description:    Converts a fixed point number to an integer.
					The format is:
						<32-FRACTION_SIZE> bits of mantissa,
						(implied binary point),
						<FRACTION_SIZE> bits of fraction < 1
	Parameters:		l_Number = fixed point number.
	Return Value:	Integer.
	Notes:			
*/
/****************************************************************************/
static tINT16 sFixedPointToInteger(tINT32 l_Number)
{
    tINT32 l_RoundOff ;
    tINT16 w_Result ;

    l_RoundOff = 1L;                  //Calculate round-off.
    l_RoundOff <<= FRACTION_SIZE - 1 ;
		//Convert number and add round-off...
    l_Number += l_RoundOff ;
	l_Number >>= FRACTION_SIZE ;
    w_Result = (tINT16) l_Number ;
    return (w_Result) ;
}

/****************************************************************************/
/*
	Function:		sComputeHysteresis()
	Description:    Computes  hysteresis references for y' and
					y".  The values are used to determine when the signals
					cross zero after being away from zero at some previous
					point.
					
					The hysteresis reference can be thought of as a point
					centered in a vertical window.  The height of the window
					is determined by a constant.  Each sample is compared to
					to the upper and lower window edges.  If the point falls
					outside of the window, the window is then pushed or
					pulled higher or lower by the appropriate edge.
	Parameters:		None.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sComputeHysteresis(tVOID)
{
		//Compute y' hysteresis...
    if (wsYPrime1 > (wsYPrimeHysteresis + Y_PRIME_HYSTERESIS))
    {
        wsYPrimeHysteresis = wsYPrime1 - Y_PRIME_HYSTERESIS ;
    }
    else if (wsYPrime1 < (wsYPrimeHysteresis - Y_PRIME_HYSTERESIS))
    {
        wsYPrimeHysteresis = wsYPrime1 + Y_PRIME_HYSTERESIS ;
    }
		//Compute y" hysteresis...
    if (wsYPrimePrime1 > (wsYPrimePrimeHysteresis + Y_PRIME_PRIME_HYSTERESIS))
    {
        wsYPrimePrimeHysteresis = wsYPrimePrime1 - Y_PRIME_PRIME_HYSTERESIS ;
    }
    else if (wsYPrimePrime1 < (wsYPrimePrimeHysteresis - Y_PRIME_PRIME_HYSTERESIS))
    {
        wsYPrimePrimeHysteresis = wsYPrimePrime1 + Y_PRIME_PRIME_HYSTERESIS ;
    }
}

/****************************************************************************/
/*
	Function:		sPutBufferA()
	Description:	Stores the index of a y' zero crossing in buffer A.
	Parameters:		uw_Index = index to store.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sPutBufferA(tUINT16 uw_Index)
{
    if (uwsBufferACount < BUFFER_A_SIZE)
    {
        uwasBufferA[uwsBufferACount++] = uw_Index ;
    }
}

/****************************************************************************/
/*
	Function:		sPutBufferB()
	Description:	Stores the index of a y" zero crossing and the corresponding
					y' value in buffer B. the values are packed head-to-toe.
	Parameters:		uw_Index = index to store.
					w_Yp = y' to store.
	Return Value:	None.
	Notes:			
*/
/****************************************************************************/
static tVOID sPutBufferB(tUINT16 uw_Index, tINT16 w_Yp)
{
    if (uwsBufferBCount < BUFFER_B_SIZE)
    {
        uwasBufferB[uwsBufferBCount++] = uw_Index ;
        uwasBufferB[uwsBufferBCount++] = (tUINT16) w_Yp ;
    }
}

/****************************************************************************/
/*
	Function:		BC_FirFilter()
	Description:	FIR low pass filter.
	Parameters:		w_Sample = sample to filter.
	Return Value:	Filtered sample.
	Notes: 			J10012-13631. Change to global for other REC_ module 		
*/
/****************************************************************************/
tINT16 BC_FirFilter(tINT16 w_Sample)
{
	tINT32 l_Accumulator ;
	tINT16 i ;
	static tINT16 w_Oldest = 0 ;

	wasLowPassBuffer[w_Oldest] = w_Sample ;
	l_Accumulator = 0L ;
	for (i = 0; i < (LOW_PASS_ORDER + 1); ++i)
	{
		l_Accumulator += wasLowPassFilterB[i] * wasLowPassBuffer[(w_Oldest + i) % (LOW_PASS_ORDER + 1)] ;
	}
	--w_Oldest ;
	if (w_Oldest < 0)
	{
		w_Oldest = LOW_PASS_ORDER ;
	}
	return ((tINT16) (l_Accumulator >> 15)) ;
}

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

