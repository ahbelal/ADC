/* 
 * File:   temperature.h
 * Author: a.belal
 *
 * Created on August 16, 2021, 9:46 AM
 */

#ifndef TEMPERATURE_H
#define	TEMPERATURE_H

#ifdef	__cplusplus
extern "C" {
#endif

    /* 
     * Libraries 
     */
//#include "global.h"

    // PIC32
#include <xc.h>
#include <plib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "limits.h"    
#include "STIRA_Lib.h"
#include "task_info.h"
    
    /* Functions */
    /**
     * @brief
     * @param resultOfOperation
     * @return operation_status_t
     */
    BaseType_t GetAdcMessageTail( void ) ;

    /**
     * @brief
     * @param resultOfOperation
     * @return operation_status_t
     */
    operation_status_t AppendAdcMessage( BaseType_t resultOfOperation );

    /**
     * @brief
     * @param resultOfOperation
     * @return operation_status_t
     */
    operation_status_t ConsumeAdcMessage( BaseType_t resultOfOperation );

    /**
     * @brief
     * @param resultOfOperation
     * @return operation_status_t
     */
    operation_status_t AppendAdcInterrupt( BaseType_t resultOfOperation );

    /**
     * @brief
     * @param resultOfOperation
     * @return operation_status_t
     */
    operation_status_t ConsumeAdcInterrupt( BaseType_t resultOfOperation );

    /**
     * @brief
     * @param resultOfOperation
     * @return 
     */
    operation_status_t AppendAdcContInterrupt ( BaseType_t resultOfOperation );

    /**
     * @brief
     * @param resultOfOperation
     * @return 
     */
    operation_status_t ConsumeAdcContInterrupt ( BaseType_t resultOfOperation );

    /* 
     *  Tasks 
     */
    // Prototypes
    portTASK_FUNCTION(AdcTask, pParams);
    // Handles
    TaskHandle_t ghAdcTask; /**< IrLens task handle */
    
#ifdef	__cplusplus
}
#endif

#endif	/* TEMPERATURE_H */

