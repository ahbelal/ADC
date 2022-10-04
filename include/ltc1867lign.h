/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _LTC1867LIGN_H    /* Guard against multiple inclusion */
#define _LTC1867LIGN_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
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


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */   
    typedef enum
    {
        TEMPERATURE = 0x0C,
    } adc_function_t;

    /**
     * @brief ADC Pot channel movements.
     * @details The different movement possibilities with regards to the potentiometer.
     */
    typedef enum
    {
        ADC_ELEVATION = 0x00, /**< Elevation movment. */
        ADC_AZIMUTH = 0x01, /**< Azimuth movment. */
    } adc_id_t;
    
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
     * @brief ADC channel type.
     * @details Holds both the channel combination to read as well as the data read.
     */
    typedef struct _adc_channel_t 
    {
        uint16_t Channel; /**< Channel combination to read from. */
        float Data; /**< Data read. */
    } adc_channel_t;
    
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
    
    
    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */

    

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _LTC1867LIGN_H */

/* *****************************************************************************
 End of File
 */
