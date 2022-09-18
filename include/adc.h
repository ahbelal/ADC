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
#include "global.h"
#include "task_info.h"

    /* 
     * Definitions 
     */
        // SPI ( POT )
#define SPI_x_CHNL                  SPI_CHANNEL1        /**< POT SPI channel used. */
#define _SPI_x_VECTOR               _SPI_1_VECTOR       /**< POT SPI channel interrupt vector number. */
#define SPI_x_VECTOR                INT_SPI_1_VECTOR    /**< POT SPI channel interrupt vector number. */
#define SPI_x_TX_SOURCE             INT_SPI1TX          /**< POT SPI transmit interrupt source. */
#define SPI_x_RX_SOURCE             INT_SPI1RX          /**< POT SPI receive interrupt source. */
#define SPI_x_ERR_SOURCE            INT_SPI1E           /**< POT SPI error interrupt source. */

#define IPLxAUTO                    IPL2AUTO                    /**< POT Interrupt Priority Level number. */
#define SPI_x_PRIORITY              INT_PRIORITY_LEVEL_2        /**< POT Interrupt priority level number. */
#define SPI_x_SUB_PRIORITY          INT_SUB_PRIORITY_LEVEL_3    /**< POT Interrupt sub priority level number. */

#define SPI_x_INT_USE               INT_DISABLED
#define SPI_x_RX_INT_USE            INT_DISABLED
#define SPI_x_TX_INT_USE            INT_DISABLED
#define SPI_x_ERR_INT_USE           INT_DISABLED
    
    /* SPI FLAGS */
    // CONFIG
    /* POT */
#define SPI_x_OPEN_FLAGS        ( SPI_OPEN_ON \
                                | SPI_OPEN_MSTEN \
                                | SPI_OPEN_MODE16 \
                                | SPI_OPEN_TBE_SR_EMPTY \
                                | SPI_OPEN_RBF_NOT_EMPTY \
                                | SPI_OPEN_CKE_REV \
                                /*| SPI_OPEN_SMP_END \
                                | SPI_OPEN_MSSEN \
                                | SPI_OPEN_CKP_HIGH*/ )     /**< ADC SPI open flags. */

    // ENABLE
    // POT
#define SPI_x_CLK_DIV       ( 40 )   /**< POT SPI desired frequency. */
   
    /* PINS */
    /* POT */
    // CS Temperature
#define SPI_CS_ADC_PORT            IOPORT_D    /**< SPI Chip select elevation port. */
#define SPI_CS_ADC_PIN             BIT_9       /**< SPI Chip select elevation pin number. */
#define SPI_CS_ADC_SHIFT           9           /**< SPI Chip select elevation shift amount. */
    
    // ADC    
#define ADC_CENTER_VALUE            0x80        /**< ADC center value. */
#define ADC_VOLTAGE_AT_ZERO         0.400f      /**< ADC Temp volt at zero. */
#define ADC_TEMPERATURE_COEF        0.0195f     /**< ADC Temp coefficient. */
    /* Output Pins */
    /* ADC Limits */
    // Low Limit
#define ADC_LOW_END                 0.0f    /**< ADC low end limit value. */
    // High Limit
#define ADC_HIGH_END                2.5f    /**< ADC high end limit value. */
    // Low Limit
#define ADC_HIGH_LIMIT_END          2.4f    /**< ADC low high limit value. */
    // High Limit
#define ADC_LOW_LIMIT_END           0.0f    /**< ADC high low limit value. */
    // Low Temp
#define ADC_LOW_TEMP_END            ADC_LOW_END    /**< ADC low end limit value. */
    // High Temp
#define ADC_HIGH_TEMP_END           ADC_HIGH_END    /**< ADC high end limit value. */
#define ADC_MAX_VALUE               ( 2 << 15 )
    // Critical level
#define CRITICAL_LIMIT_HIGH_VALUE   2.25f
#define CRITICAL_LIMIT_LOW_VALUE    1.19f
    // Safety limit
#define SAFE_LIMIT_HIGH_VALUE       2.1f
#define SAFE_LIMIT_LOW_VALUE        1.2f
    // Safety Margins
#define CRITICAL_LIMIT_PERCENT      0.05f
#define ERROR_LIMIT_PERCENT         0.01f

    /* 
     * Enum 
     */

    /**
     * @brief ADC channel combinations.
     * @details The different ADC channel combinations.
     */
    typedef enum 
    {
        // CHx - CHx
        ADC_CH0_P_CH1_N = 0b0000010, /**< Channel 0 Positive, Channel 1 Negative. */
        ADC_CH2_P_CH3_N = 0b0001010, /**< Channel 2 Positive, Channel 3 Negative. */
        ADC_CH4_P_CH5_N = 0b0010010, /**< Channel 4 Positive, Channel 5 Negative. */
        ADC_CH6_P_CH7_N = 0b0011010, /**< Channel 6 Positive, Channel 7 Negative. */
        // Reverse
        ADC_CH1_P_CH0_N = 0b0100010, /**< Channel 1 Positive, Channel 0 Negative. */
        ADC_CH3_P_CH2_N = 0b0101010, /**< Channel 3 Positive, Channel 2 Negative. */
        ADC_CH5_P_CH4_N = 0b0110010, /**< Channel 5 Positive, Channel 4 Negative. */
        ADC_CH7_P_CH6_N = 0b0111010, /**< Channel 7 Positive, Channel 6 Negative. */

        // CHx - GND
        ADC_CH0_P_GND_N = 0b1000010, /**< Channel 0 Positive, GND Negative. */
        ADC_CH2_P_GND_N = 0b1001010, /**< Channel 2 Positive, GND Negative. */
        ADC_CH4_P_GND_N = 0b1010010, /**< Channel 4 Positive, GND Negative. */
        ADC_CH6_P_GND_N = 0b1011010, /**< Channel 6 Positive, GND Negative. */
        // Reverse
        ADC_CH1_P_GND_N = 0b1100010, /**< Channel 1 Positive, GND Negative. */
        ADC_CH3_P_GND_N = 0b1101010, /**< Channel 3 Positive, GND Negative. */
        ADC_CH5_P_GND_N = 0b1110010, /**< Channel 5 Positive, GND Negative. */
        ADC_CH7_P_GND_N = 0b1111010, /**< Channel 7 Positive, GND Negative. */

        // CHx - CH7/COM
        ADC_CH0_P_CH7_COM_N = 0b1000110, /**< Channel 0 Positive, Channel 7 (COM) Negative. */
        ADC_CH2_P_CH7_COM_N = 0b1001110, /**< Channel 2 Positive, Channel 7 (COM) Negative. */
        ADC_CH4_P_CH7_COM_N = 0b1010110, /**< Channel 4 Positive, Channel 7 (COM) Negative. */
        ADC_CH6_P_CH7_COM_N = 0b1011110, /**< Channel 6 Positive, Channel 7 (COM) Negative. */
        // Reverse
        ADC_CH1_P_CH7_COM_N = 0b1100110, /**< Channel 1 Positive, Channel 7 (COM) Negative. */
        ADC_CH3_P_CH7_COM_N = 0b1101110, /**< Channel 3 Positive, Channel 7 (COM) Negative. */
        ADC_CH5_P_CH7_COM_N = 0b1110110, /**< Channel 5 Positive, Channel 7 (COM) Negative. */

    } adc_channel_id_t;

    /* ADC ID */

    /**
     * @brief ADC Pot channel movements.
     * @details The different movement possibilities with regards to the potentiometer.
     */
    typedef enum
    {
        ADC_ELEVATION = 0x00, /**< Elevation movment. */
        ADC_AZIMUTH = 0x01, /**< Azimuth movment. */
    } adc_id_t;

    /*
     *  Struct
     */

    /**
     * @brief ADC channel type.
     * @details Holds both the channel combination to read as well as the data read.
     */
    typedef struct _adc_channel_t 
    {
        uint16_t Channel; /**< Channel combination to read from. */
        float Data; /**< Data read. */
    } adc_channel_t;

    /**
     * @brief Dual channel ADC data type.
     * @details Includes both A, B, AB, and BA data combinations. 
     */
    typedef struct __attribute__((__packed__)) _adc_position_data_t 
    {
        struct
        {
            float A; /**< Data from channel A */
            float B; /**< Data from channel B */
            float AB; /**< Data from channel AB */
            float BA; /**< Data from channel BA */            
        } floatData;
        struct 
        {
            uint16_t A; /**< Channel A */
            uint16_t B; /**< Channel B */
            uint16_t AB; /**< Channel AB */
            uint16_t BA; /**< Channel BA */            
        } intData;
    } adc_position_data_t;

    /**
     * @brief ADC device type.
     * @details Holds the information regarding the dual channel POT inputs. 
     */
    typedef struct __attribute__((__packed__)) _adc_t 
    {
        // Device ID
        adc_id_t adcId; /**< ADC device movement ID. */
        // Channels
        adc_channel_t Temperature; /**< ADC temperature channel. */
//        adc_channel_t Voltage; /**< ADC voltage monitor channel (not used). */
//        adc_channel_t Elevation; /**< ADC elevation potentiomenter channel. */
    } adc_t;

    BaseType_t adcInterruptNotifyTail;
    BaseType_t adcInterruptNotifyHead;
    BaseType_t adcContInterruptNotifyTail;
    BaseType_t adcContInterruptNotifyHead;
    BaseType_t adcMsgNotifyTail;
    BaseType_t adcMsgNotifyHead;
    
    
    
    /* Functions */
    operation_status_t GetAdc ( message_t * pMessage );

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

