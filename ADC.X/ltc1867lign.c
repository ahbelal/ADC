
/* Libraries */
#include "adc.h"
#include "ltc1867lign.h"
#include "HelperFunctions.h"
//#include "subsystem.h"

/* Definitions */

/* Variables */
static BaseType_t adcInterruptNotifyTail;
static BaseType_t adcInterruptNotifyHead;
static BaseType_t adcContInterruptNotifyTail;
static BaseType_t adcContInterruptNotifyHead;
static BaseType_t adcMsgNotifyTail;
static BaseType_t adcMsgNotifyHead;
    
static adc_t adc ;
SpiComm_t spiCommunication;

/**
 * @brief ADC device init.
 * @details Function to initialize the ADC device object.
 * 
 * @param padc Pointer to ADC device object.
 * @param pstADCAzimuthPot Pointer to ADC azimuth POT channel.
 */
static void InitAdc ( void );
static void InitAdcChannel ( void );
static void InitAdcCommunication ( void );
static operation_status_t GetAdc ( message_t * pMessage );
    
/**
 * @brief ADC read.
 * @details Function to read from a single ADC channel.
 * 
 * @param writeData Channel combination to read from.
 * @param twosComplementFlag Return twos complement or not (1 if twos complement required).
 * @param spiCommunication SPI channel.
 * @param valueMap The range of values to map the read value to.
 * @return Data read.
 */
static float ReadAdcChannel ( uint16_t writeData, uint8_t twosComplementFlag, float valueMap  );

static operation_status_t GetAdcTemperature ( message_t * pMessage );

/* Interrupts */

/* Tasks */

/*
 * Task:            ADCTask
 *
 * Overview:        Task to handle ADC 
 */
portTASK_FUNCTION ( AdcTask, pParams )
{
    /* Variables */
    operation_status_t  resultOfOperation;
    TickType_t          ticksBlockTime;
    message_t *         pMessage;
    uint32_t            recvItem;
    task_info_t *       pTaskInfo;
    
    /* Variables Init */
    ticksBlockTime                          = portMAX_DELAY ;
    pMessage                                = NULL ;
    resultOfOperation                       = 0;
    recvItem                                = 0;
    adcMsgNotifyTail                        = 0;
    adcMsgNotifyHead                        = 0;
    adcInterruptNotifyTail                  = configTASK_NOTIFICATION_ARRAY_ENTRIES / 2;
    adcInterruptNotifyHead                  = configTASK_NOTIFICATION_ARRAY_ENTRIES / 2;
    adcContInterruptNotifyTail              = 0;
    adcContInterruptNotifyHead              = 0;
    
    /* Init functions */
    InitAdc ();
    
    /* Task Core */
    for ( ; ; )
    {
        /* 
         * Block indefinitely (without a timeout, so no need to check the functionâ€™s
         * return value) to wait for a notification.
         * Bits in this RTOS taskâ€™s notification value are set by the notifying
         * tasks and interrupts to indicate which events have occurred.         
         */
        resultOfOperation = xTaskNotifyWaitIndexed ( adcMsgNotifyHead,  // Wait for 0th notification.
                                          0x00,                             // Don't clear any notification bits on entry.
                                          ULONG_MAX,                        // Reset the notification value to 0 on exit.
                                          &recvItem,                        // Notified value pass out in ulNotifiedValue.
                                          ticksBlockTime ) ;                // Block indefinitely. 
        resultOfOperation = ConsumeAdcMessage ( resultOfOperation ) ;
        if( SUCCESSFUL_OPERATION == resultOfOperation )
        {            
            pMessage = ( message_t * ) recvItem ;
            pTaskInfo = ( task_info_t * ) pMessage->info;
            resultOfOperation = NO_OPERATION_STATUS ;
        
            switch ( pMessage->body.command )
            {
                case GET_COMMAND:
                    resultOfOperation = GetAdc ( pMessage );
                    break ;
                    
                case SET_COMMAND:
                    break ;

                case RESET_COMMAND:
                    break ;
                    
                case TEST_COMMAND:
                    break ;
                    
                case BRIDGE_COMMAND:
                    break ;
                    
                default:
                    resultOfOperation = INVALID_OPRATION_ERROR ;
                    break ;
            }

            // Handle feedback
            if ( NULL != pTaskInfo )
            {
                if ( NULL != pTaskInfo->handle )
                {
                    taskENTER_CRITICAL();
                    resultOfOperation = xTaskNotifyIndexed (    pTaskInfo->handle, *pTaskInfo->pIndex, 
                                                                ( resultOfOperation << SHIFT_3_BYTE ) | 
                                                                ADC_TASK_BITS, eSetBits );
                    resultOfOperation = IsNotifyValid ( resultOfOperation, pTaskInfo->pIndex );
                    // Free ( pTaskInfo );
                    taskEXIT_CRITICAL();               
                }
                else
                {

                }            
            }
            else
            {
                //TODO: Handle error
            }
        }
        else
        {
            taskYIELD();
        }        
    }
}

static operation_status_t GetAdc ( message_t * pMessage )
{
    /* Variables */
    operation_status_t  result ;
    
    /* Variables Init */
    result = NO_OPERATION_STATUS ;

    /* Function Core */
    switch ( pMessage->body.peripheralFunction )
    {
        case TEMPERATURE:
            result = GetAdcTemperature (pMessage);
            break;
        default:
            result = INVALID_MESSAGE_ERROR;
            break;
    }
        
    /* Function Exit */
    return result ;    
}

static operation_status_t GetAdcTemperature ( message_t * pMessage )
{
    operation_status_t result;
    float_t currentTemperature;
        
    /* TEMP */
    result = AllocateMessageData ( pMessage, sizeof( currentTemperature ) );
    if ( SUCCESSFUL_OPERATION == result )
    { // Data buffer allocated successfully
        adc.Temperature.Data = ReadAdcChannel ( adc.Temperature.Channel, 0, ADC_HIGH_END ) ;
        currentTemperature.DataFloat =
                ( adc.Temperature.Data - ADC_VOLTAGE_AT_ZERO ) / 
                ( ADC_TEMPERATURE_COEF ) ;

        // Update message data length
        pMessage->body.dataLength = sizeof( currentTemperature ) ;

        // Update message data
        memcpy ( pMessage->data, currentTemperature.Data, pMessage->body.dataLength );
    }
    else
    {
    }
    
    return result;
}

/* Functions */

/*
 * Function:        vADCInit
 *
 * Overview:        ADC device init.
 */
static void InitAdc ( void )
{
    // Initialize ADC channel
    InitAdcChannel ();

    // Initialize ADC communication
    InitAdcCommunication ();
}

static void InitAdcChannel ( void )
{
    // Device ID
    adc.adcId = 0 ;
    // Temperature
    adc.Temperature.Channel = ADC_CH4_P_GND_N ;
    adc.Temperature.Data = 0.0 ;
//    // Elevation limit
//    adc.Elevation.Channel = ADC_CH1_P_GND_N ;
//    adc.Elevation.Data = 0.0 ;
//    adc.ElevationLimit = NOT_REACHED ;
//    // System Voltage
//    adc.Voltage.Channel = ADC_CH5_P_GND_N ;
//    adc.Voltage.Data = 0.0 ;
}

static void InitAdcCommunication ( void )
{
    IntFlags_t stInterruptFlags_SPI ;
    
    // Interrupt Object Flags ( SPI )
    stInterruptFlags_SPI.Vector         = _SPI_x_VECTOR ;
    stInterruptFlags_SPI.Priority       = SPI_x_PRIORITY ;
    stInterruptFlags_SPI.SubPriority    = SPI_x_SUB_PRIORITY ;

    stInterruptFlags_SPI.Source [ 0 ]   = SPI_x_RX_SOURCE ;
    stInterruptFlags_SPI.Source [ 1 ]   = SPI_x_TX_SOURCE ;
    stInterruptFlags_SPI.Source [ 2 ]   = SPI_x_ERR_SOURCE ;

    stInterruptFlags_SPI.Enable [ 0 ]   = SPI_x_RX_INT_USE ;
    stInterruptFlags_SPI.Enable [ 1 ]   = SPI_x_TX_INT_USE ;
    stInterruptFlags_SPI.Enable [ 2 ]   = SPI_x_ERR_INT_USE ;

    stInterruptFlags_SPI.Length         = 3 ;
    stInterruptFlags_SPI.UseInterrupt   = SPI_x_INT_USE ;

    spiCommunication.Chnl = SPI_x_CHNL;
    spiCommunication.Mode = CS_MANUAL;
    spiCommunication.Cs.Port = SPI_CS_ADC_PORT ;
    spiCommunication.Cs.Pin = SPI_CS_ADC_PIN ;
    spiCommunication.Cs.Shift = SPI_CS_ADC_SHIFT ;
    
    PORTSetPinsDigitalOut (SPI_CS_ADC_PORT, SPI_CS_ADC_PIN);
    PORTSetBits (SPI_CS_ADC_PORT, SPI_CS_ADC_PIN);
    
    // SPI
    eSPIPeripheralConfig ( SPI_x_CHNL,
                           SPI_x_OPEN_FLAGS,
                           &stInterruptFlags_SPI,
                           SPI_x_CLK_DIV ) ;
}

/*
 * Function:        ReadAdcChannel
 *
 * Overview:        ADC read.      
 */
float ReadAdcChannel ( uint16_t writeData, uint8_t twosComplementFlag, float valueMap )
{
    /* Variables */
    float dDataVoltage = 0.0 ;
    uint16_t usReadData ;
    writeData = writeData << 9 ;

    /* Function Core */
    usReadData = ucSPIPeripheralWriteRead ( spiCommunication, writeData ) ;
    vTaskDelay ( pdMS_TO_TICKS ( 1 ) ) ;
    usReadData = ucSPIPeripheralWriteRead ( spiCommunication, writeData ) ;

    if ( twosComplementFlag )
    { // Return Twos complement value
        usReadData = ( ~usReadData ) + 1 ;
        dDataVoltage = usReadData * ( valueMap / ADC_MAX_VALUE ) ;
    }
    else
    { // Return Normal reading
        dDataVoltage = usReadData * ( valueMap / ADC_MAX_VALUE ) ;
    }

    /* Function Exit */
    return dDataVoltage ;
}

BaseType_t GetAdcMessageTail( void ) 
{
    return adcMsgNotifyTail ;
}

operation_status_t AppendAdcMessage( BaseType_t resultOfOperation )
{    
    return IsNotifyValid ( resultOfOperation, &adcMsgNotifyTail );    
}

operation_status_t ConsumeAdcMessage( BaseType_t resultOfOperation )
{
    return IsNotifyValid ( resultOfOperation, &adcMsgNotifyHead );    
}

operation_status_t AppendAdcInterrupt( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &adcInterruptNotifyTail );
}

operation_status_t ConsumeAdcInterrupt( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &adcInterruptNotifyHead );    
}

operation_status_t AppendAdcContInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &adcContInterruptNotifyTail );
}

operation_status_t ConsumeAdcContInterrupt ( BaseType_t resultOfOperation )
{
    return IsNotifyValidFromISR ( resultOfOperation, &adcContInterruptNotifyHead );    
}