#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/pxa95x-regs.h>
#include <mach/hardware.h>

#define SET     1
#define CLEAR   0
#define TRUE true
#define FALSE false
typedef bool BOOL;
typedef unsigned char		UINT8;
typedef unsigned short		UINT16;
typedef unsigned int		UINT32;

typedef enum
{
    I2C_RC_OK,
    I2C_RC_NOT_OK,
    I2C_RC_INVALID_DATA_SIZE,
    I2C_RC_INVALID_DATA_PTR,
    I2C_RC_TOO_MANY_REGISTERS,
    I2C_RC_TIMEOUT_ERROR,                               // 5
    I2C_RC_CHIP_BUSY,                                   // 6
    I2C_RC_INVALID_GENERAL_CALL_SLAVE_ADDRESS,          // 7
	I2C_RC_UNREGISTER_ERR,                              // 8
	I2C_RC_MESSAGE_QUEUE_IS_FULL,                       // 9
    I2C_ISR_UNEXPECTED_INTERRUPT,                       // 0xA
	I2C_ISR_BUS_ERROR,                                  // 0xB
	I2C_ISR_BUS_BUSY,                                   // 0xC
	I2C_ISR_EARLY_BUS_BUSY,                             // 0xD
	I2C_ISR_CALL_BACK_FUNCTION_ERR,                     // 0xE
	I2C_ISR_ARBITRATION_LOSS,                           // 0xF
	I2C_RC_ILLEGAL_USE_OF_API
}I2C_ReturnCode;

#if 1
#if 0
#define I2C_REG_WRITE(reg,wval) \
        ( (* ( (volatile UINT32*)( (reg)) ) ) = wval);

#define I2C_REG_READ(reg,rval) \
        rval = (* ( (volatile UINT32*)( (reg)) ) );
#else
#define I2C_REG_WRITE(reg,wval) \
        (reg = wval);

#define I2C_REG_READ(reg,rval) \
        (rval = reg);
#endif
#else
#define I2C_REG_WRITE(reg,wval) \
        ( __raw_writel(reg,wval));

#define I2C_REG_READ(reg,rval) \
        rval = (__raw_readl(reg));
#endif
#define I2C_REG_BIT_WRITE(reg,btNm,sc) \
            {   uint32_t regVl; \
                I2C_REG_READ(reg,regVl); \
                sc ? ( regVl = (1 << btNm) | regVl ) : (regVl = ((regVl) & (~(1 << btNm)))); \
                I2C_REG_WRITE(reg,regVl); \
            }


#define I2C_STATUS_REG_CLEAR_BIT(bitNum)   			\
            {   uint32_t regVal; 						\
                regVal = (1 << bitNum); 			\
                I2C_REG_WRITE(PISR_REG,regVal); 	\
			}
			
#define PIBMR_REG       __REG(0x40F500C0) /* PI2C Bus Monitor register */
#define PIDBR_REG       __REG(0x40F500C4) /* PI2C Data Buffer Register */
#define PICR_REG        __REG(0x40F500C8) /* PI2C Control Register*/
#define PISR_REG        __REG(0x40F500CC) /* PI2C Status Register */
#define PISAR_REG       __REG(0x40F500D0) /* PI2C Slave Address Register */

#define PCFR_REG       __REG(0x40F5000C)

#define PMCR_REG       __REG(0x40F50000)

#define I2C_REG_CLEAR_VALUE         		0x0000
#define I2C_ICR_CLEAR_ALL_CONTROL_BITS      0xfff0				// Clear the TB, STOP, START, ACK bits in the ICR register

/** bit map of the ICR register **/
#define I2C_ICR_DMA_ENABLE_BIT 						27
#define I2C_ICR_RECEIVE_FIFO_OVERRUN_INT_ENABLE_BIT 26
#define I2C_ICR_RECEIVE_FIFO_FULL_INT_ENABLE_BIT    25
#define I2C_ICR_TRANSMIT_FIFO_EMPTY_INT_ENABLE_BIT  24
#define I2C_ICR_RECEIVE_FIFO_HALF_INT_ENABLE_BIT    23
#define I2C_ICR_TRANSACTION_DONE_INT_ENABLE_BIT     22
#define I2C_ICR_                					21
#define I2C_ICR_FIFO_MODE_ENABLE_BIT                20
#define I2C_ICR_GPIO_MODE_ENABLE_BIT                19
#define I2C_ICR_MASTER_STOP_DETECT_ENABLE_BIT       18			// Detect by the status register
#define I2C_ICR_MASTER_STOP_DETECT_INT_ENABLE_BIT   17
#define I2C_ICR_HS_MODE_BIT                         16
#define I2C_ICR_FAST_MODE_BIT                       15
#define I2C_ICR_UNIT_RESET_BIT                      14
#define I2C_ICR_IDBR_RECEIVE_FULL_INT_ENABLE_BIT    9
#define I2C_ICR_UNIT_ENABLE_BIT                     6
#define I2C_ICR_SCLEA_ENABLE_BIT                    5
#define I2C_ICR_MASTER_ABORT_BIT                    4
#define I2C_ICR_TRANSFER_BYTE_BIT                   3
#define I2C_ICR_ACK_NACK_CONTROL_BIT                2
#define I2C_ICR_STOP_BIT                            1
#define I2C_ICR_START_BIT                           0


#define I2C_ICR_UNIT_RESET                            (1 << I2C_ICR_UNIT_RESET_BIT)
#define I2C_ICR_IDBR_RECEIVE_FULL_INT_ENABLE          (1 << I2C_ICR_IDBR_RECEIVE_FULL_INT_ENABLE_BIT)
#define I2C_ICR_UNIT_ENABLE                           (1 << I2C_ICR_UNIT_ENABLE_BIT)
#define I2C_ICR_SCLEA_ENABLE                          (1 << I2C_ICR_SCLEA_ENABLE_BIT)
#define I2C_ICR_MASTER_ABORT                          (1 << I2C_ICR_MASTER_ABORT_BIT)
#define I2C_ICR_TRANSFER_BYTE                         (1 << I2C_ICR_TRANSFER_BYTE_BIT)
#define I2C_ICR_ACK_NACK_CONTROL                      (1 << I2C_ICR_ACK_NACK_CONTROL_BIT)
#define I2C_ICR_STOP                                  (1 << I2C_ICR_STOP_BIT)
#define I2C_ICR_START                                 (1 << I2C_ICR_START_BIT)





/** bit map of the ISR register **/
#define I2C_ISR_UNEXPECTED_INTERRUPTS_REPORT			0xff
#define I2C_ISR_UNEXPECTED_INTERRUPTS					0x00000310
#define I2C_ISR_CLEAR_ALL_INTERRUPTS					0x000017f2

#define I2C_ICR_RECEIVE_FIFO_OVERRUN_BIT                17
#define I2C_ISR_RECEIVE_FIFO_FULL_BIT                   16
#define I2C_ISR_TRANSMIT_FIFO_EMPTY_BIT                 15
#define I2C_ISR_RECEIVE_FIFO_HALF_BIT                   14
#define I2C_ISR_TRANSACTION_DONE_BIT		     		13
#define I2C_ISR_MASTER_STOP_DETECTED_BIT				12
#define I2C_ISR_EARLY_IC_BUS_BUSY_BIT					11
#define I2C_ISR_BUS_ERROR_DETECTED_BIT                  10
#define I2C_ISR_SLAVE_ADDRESS_DETECTED_BIT              9
#define I2C_ISR_GENERAL_CALL_ADDRESS_DETECTED_BIT       8
#define I2C_ISR_IDBR_RECEIVE_FULL_BIT                   7
#define I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT                 6
#define I2C_ISR_ARBITRATION_LOSS_DETECTED_BIT           5
#define I2C_ISR_SLAVE_STOP_DETECTED_BIT                 4
#define I2C_ISR_I2C_BUS_BUSY_BIT                        3
#define I2C_ISR_UNIT_BUSY_BIT                           2
#define I2C_ISR_ACK_NACK_STATUS_BIT                     1
#define I2C_ISR_READ_WRITE_MODE_BIT                     0


#define I2C_IBMR_SDA_BIT                     		0
#define I2C_IBMR_SCL_BIT                     		1
#define I2C_IBMR_SCL_AND_SDA_LINES_HIGH				0x3

#define I2C_OWN_HS_MASTER_CODE     (0x08 | 0x04 | 0x02)

#define I2C_REG_BIT_READ(regVl,btNm)  ( (regVl >> btNm) & 0x00000001 )

#define I2C_REG_BIT(btNm) (((uint32_t)1)<<(btNm))

#define I2C_SLAVE_WRITE(slv)        ( (slv) | 0x00000000 )			/* Master is writing to the slave */
#define I2C_SLAVE_READ(slv)         ( (slv) | 0x00000001 )      	/* Master is reading from the slave */

#define LOOP_COUNTER_LIMIT  4000L

static volatile bool        _error_was_detected = false , _arbitration_loss_was_detected = false;
static uint32_t i2c_hs_own_master_code = I2C_OWN_HS_MASTER_CODE;

static I2C_ReturnCode  I2C_busIsIdle(void) 
{
  #define NON_GPB_LOOP_THRESHOLD  500
  volatile uint32_t    delayLoop;

    uint32_t  rv = 0;
    uint32_t  retry = 3;
    I2C_ReturnCode rc = I2C_RC_OK;

    //NOTE: Minimum one I2C_DELAY has already been done in the Clock-ON while checking IBMR 
    while(retry--)
    {
        I2C_REG_READ(PISR_REG,rv);
        
        for(delayLoop=0; delayLoop<NON_GPB_LOOP_THRESHOLD; delayLoop++); //I2C_DELAY

        if(rv & ((1<<I2C_ISR_EARLY_IC_BUS_BUSY_BIT) | (1<<I2C_ISR_UNIT_BUSY_BIT) | (1<<I2C_ISR_I2C_BUS_BUSY_BIT)) )
            continue;

        I2C_REG_READ(PISR_REG,rv);
        if(rv & ((1<<I2C_ISR_EARLY_IC_BUS_BUSY_BIT) | (1<<I2C_ISR_UNIT_BUSY_BIT) | (1<<I2C_ISR_I2C_BUS_BUSY_BIT)) )
        {
            for(delayLoop=0; delayLoop<NON_GPB_LOOP_THRESHOLD; delayLoop++); //I2C_DELAY
            continue;
        }
        break;
    }

    if(rv & (1<<I2C_ISR_EARLY_IC_BUS_BUSY_BIT))
        rc = I2C_ISR_EARLY_BUS_BUSY;
    else
    if(rv & ((1<<I2C_ISR_UNIT_BUSY_BIT) | (1<<I2C_ISR_I2C_BUS_BUSY_BIT)) )
        rc = I2C_ISR_BUS_BUSY;
    else
    {
        I2C_REG_READ(PIBMR_REG,rv);
        if ( rv != I2C_IBMR_SCL_AND_SDA_LINES_HIGH )
            rc = I2C_ISR_BUS_BUSY;
    }
    return (rc);
}   

static I2C_ReturnCode i2cWaitStatus ( uint32_t    bitsSet , uint32_t    bitsCleared )
{

	uint32_t 		statusRegValue , i2c_flags_receive;
    uint32_t 		countLimit = LOOP_COUNTER_LIMIT;
	uint32_t 		mask = bitsSet|bitsCleared;    // all bits we care of
	uint32_t 		value = bitsSet;

	do
    {
        I2C_REG_READ(PISR_REG,statusRegValue);
        countLimit--;
    }
    while ((( statusRegValue & mask ) != value ) && ( _error_was_detected == false ) && (countLimit > 0) && (_arbitration_loss_was_detected == false) );

    if ( _arbitration_loss_was_detected == true )
	{
		  /* In case we got an arbitration loss error, the SISR send a FLAG to the task, we have */
		  /* to clear the sent flag by doing this waitng for the next receive data stage */
          //OSAFlagWait ( I2CFlagRef,
          //              I2C_ERROR,
          //              OSA_FLAG_OR_CLEAR,
          //              &i2c_flags_receive,
          //              OSA_NO_SUSPEND );

		  return I2C_ISR_ARBITRATION_LOSS;
	}

    // check for timeout, reset the I2C device for next operations and return error if needed
    if ( countLimit == 0 )
    {
		uint32_t  ICRRegValue = 0;

		I2C_REG_READ(PICR_REG,ICRRegValue);
		ICRRegValue |= I2C_ICR_MASTER_ABORT;
		ICRRegValue &= ~I2C_ICR_TRANSFER_BYTE;
    	I2C_REG_WRITE(PICR_REG, ICRRegValue );
    	return I2C_RC_TIMEOUT_ERROR;
	}
    else
	   if ( _error_was_detected == true )
	        return I2C_ISR_BUS_ERROR;

#ifdef I2C_ERROR_INFO
    byteCount++;
#endif
	return I2C_RC_OK;
}
static I2C_ReturnCode masterSend(uint8_t *data, uint16_t dataSize, uint8_t slaveAddress, bool repeatedStart , bool   MasterReceiveCalled )
{
    uint32_t  		statusRegValue , ICRRegValue = 0;
    int 			i;
	bool			SlaveAddressWasSent = false , MasterCodeWasSent = false;
	I2C_ReturnCode 	I2CReturnCode = I2C_RC_OK;

	/* Check the bus for free - If there is an arbitration loss, stay until it become free */
    do
	{
        // check if 'Arbitration Loss' was detected
       	if ( I2CReturnCode == I2C_ISR_ARBITRATION_LOSS )
		{
			if(MasterCodeWasSent && SlaveAddressWasSent)
			{ //Arbitration could occures only on master stage in HS mode
				//We should never be here!
				//DIAG_FILTER(SW_PLAT,I2C,ARBITRATION_LOSS,DIAG_INFORMATION)
				//diagPrintf ("I2c HS mode, but ARBITRATION_LOSS obtained on SLAVE-ADDR stage");
				//WARNING(0);
			}
	         _arbitration_loss_was_detected = false;
			 MasterCodeWasSent = false;

			 // Read the control register
			 I2C_REG_READ(PICR_REG,ICRRegValue);

		  	 ICRRegValue &= ~I2C_ICR_START;               // Clear the start bit
		  	 ICRRegValue &= ~I2C_ICR_TRANSFER_BYTE;       // Clear the transfer_byte bit

    		 // Write to the ICR register
    		 I2C_REG_WRITE(PICR_REG, ICRRegValue);

			 msleep(1); //OSATaskSleep( 1 );           	/* A little delay to enable others master to finish without interruption */
											/* the bus arbiter, and reduce the arbitration sequences */
#ifdef I2C_ERROR_INFO
  			 byteCount = 0;
#endif
		}

        I2C_REG_READ(PICR_REG,ICRRegValue);

		if ( (ICRRegValue & (1<<I2C_ICR_HS_MODE_BIT)) && ( MasterCodeWasSent == false ) )
		{
			MasterCodeWasSent = true;
			// write IDBR register the Master code.
    		I2C_REG_WRITE(PIDBR_REG, i2c_hs_own_master_code );
		}
		else
		{
			SlaveAddressWasSent = true;
			// write IDBR register: target slave address and R/W# bit=0 for write transaction.
    		I2C_REG_WRITE(PIDBR_REG, I2C_SLAVE_WRITE(slaveAddress) );
		}

    	// write ICR register: set START bit, clear STOP bit, set Transfer Byte bit to initiate the access
		ICRRegValue &= ~I2C_ICR_STOP;
    //GPIO_TB_SET();
		ICRRegValue |= (I2C_ICR_START | I2C_ICR_TRANSFER_BYTE);
    	I2C_REG_WRITE(PICR_REG, ICRRegValue );
    //GPIO_TB_CLR();

	    I2CReturnCode = i2cWaitStatus ( (I2C_REG_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT) | I2C_REG_BIT(I2C_ISR_UNIT_BUSY_BIT)),
			    	                     I2C_REG_BIT(I2C_ISR_READ_WRITE_MODE_BIT)  );

    	// clear 'IDBR Transmit Empty' bit (write '1')
    	I2C_STATUS_REG_CLEAR_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT);

		if ( ( I2CReturnCode != I2C_ISR_ARBITRATION_LOSS ) && ( I2CReturnCode != I2C_RC_OK ))
		{
        	I2C_REG_READ(PICR_REG,ICRRegValue);
			ICRRegValue &= 0xfff0;
   			I2C_REG_WRITE(PICR_REG, ICRRegValue );

			return I2CReturnCode;
		}
    }
    // check if 'Arbitration Loss' was detected
	while ( ( I2CReturnCode == I2C_ISR_ARBITRATION_LOSS ) || ( SlaveAddressWasSent == false ) );

    // write ICR register: clear START bit, clear STOP bit
    I2C_REG_READ(PICR_REG,ICRRegValue);
  	ICRRegValue &= ~I2C_ICR_STOP;
  	ICRRegValue &= ~I2C_ICR_START;
   	I2C_REG_WRITE(PICR_REG, ICRRegValue );

    /*** Send data bytes - all, except the last byte  ***/
    for ( i = 0; i < ( dataSize - 1 ); i++ )
    {
        // write data byte to the IDBR register
        I2C_REG_WRITE(PIDBR_REG, *data);
        data++;

        // Set 'Tranfer Byte' bit to intiate the access
   // GPIO_TB_SET();
        I2C_REG_BIT_WRITE(PICR_REG, I2C_ICR_TRANSFER_BYTE_BIT, SET);
   // GPIO_TB_CLR();

	    if((I2CReturnCode = i2cWaitStatus(I2C_REG_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT)|I2C_REG_BIT(I2C_ISR_UNIT_BUSY_BIT),
				                        I2C_REG_BIT(I2C_ISR_READ_WRITE_MODE_BIT)))!=I2C_RC_OK)
						return I2CReturnCode;

        // clear 'IDBR Transmit Empty' bit (write '1')
        I2C_STATUS_REG_CLEAR_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT);
    }


    /*** Send the last byte with STOP bit ***/

    // write the last data byte to the IDBR register
    I2C_REG_WRITE(PIDBR_REG, *data);

    // write ICR: clear START bit
    I2C_REG_BIT_WRITE(PICR_REG, I2C_ICR_START_BIT, CLEAR);

    if (( repeatedStart == false ) || ( MasterReceiveCalled == false ))
        I2C_REG_BIT_WRITE(PICR_REG, I2C_ICR_STOP_BIT, SET);

   // GPIO_TB_SET();
    I2C_REG_BIT_WRITE(PICR_REG, I2C_ICR_TRANSFER_BYTE_BIT, SET);
   // GPIO_TB_CLR();

	if((I2CReturnCode = i2cWaitStatus ( I2C_REG_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT),
	                                    I2C_REG_BIT(I2C_ISR_READ_WRITE_MODE_BIT))) != I2C_RC_OK)
				return I2CReturnCode;

	// clear 'IDBR Transmit Empty' bit (write '1')
    I2C_STATUS_REG_CLEAR_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT);
    I2C_REG_BIT_WRITE(PICR_REG, I2C_ICR_STOP_BIT, CLEAR);

#ifdef _I2C_CLK_WORKAROUND_
    if (( ICRRegValue & FAST_MODE_ENABLE ) == 0 )       // Check if we are in slow mode
    {
       i=0;
       while( i++ < 13 )    // Problem in slow mode, make a little delay before clock off
       {
           I2C_REG_READ(PISR_REG,statusRegValue);
           if(statusRegValue == 0xFFFFFFFF) break; //just for compiler warning
       }
    }
#endif
    return I2C_RC_OK;
}

I2C_ReturnCode I2CMasterReceiveData ( UINT8 							*cmd,
									  UINT16 	 						cmdLength,
									  UINT8 							writeSlaveAddress,
									  BOOL 		 						protected,
                                      UINT8 							*designatedRxBufferPtr,
									  UINT16 	 						dataSize,
									  UINT8 							readSlaveAddress,
									  UINT16      						userId)
{
    UINT32 			cpsr = 0;
    I2C_ReturnCode 	I2CReturnCode = I2C_RC_OK;
    UINT32 			ICRRegValue = 0;

    // store the request parameters (to be used from the ISR and by the I2C task when error)
    //_receiveReqParams.activeSlaveAddress = readSlaveAddress;
    //_receiveReqParams.ID = userId;
    //_receiveReqParams.RxBufferPtr = designatedRxBufferPtr;
    //_receiveReqParams.dataSize = dataSize;
    //_receiveReqParams.callBack = ReadCallBack;


   // if ( dataSize == 0 )
   //     return I2C_RC_INVALID_DATA_SIZE;

   // I2CReturnCode = I2C_clock( PMU_ON );
	//if ( I2CReturnCode != I2C_RC_OK )
   //     return I2C_ISR_BUS_BUSY;

    //if ( CHECK_IF_GENERAL_CALL(writeSlaveAddress) )
	//	ASSERT(FALSE);

    //if ( CHECK_IF_GENERAL_CALL(readSlaveAddress) )
	//	ASSERT(FALSE);

    I2CReturnCode = I2C_busIsIdle();
    if(I2CReturnCode != I2C_RC_OK)
        return I2CReturnCode;

    if ( (cmdLength != 0) && (cmd != NULL) )  /* if command is associated with the received request */
    {
     //   if (protected)
     //       cpsr = disableInterrupts();

 // GPIO_START_ON();
        I2CReturnCode = masterSend(cmd, cmdLength, writeSlaveAddress, FALSE, TRUE);

	//	if (protected)
     //       restoreInterrupts(cpsr);

        if(I2C_RC_OK != I2CReturnCode)
            return I2CReturnCode;
    }

	/* this is work around for Harbell - there is a problem with the I2C signal in Tavor - System issue */
	//msleep(1);
	//I2C_DELAY(80);
 // GPIO_ALL_OFF();

    /*** send Read Request ***/
    // write IDBR: target slave address and R/W# bit (1 for read)
    I2C_REG_WRITE(PIDBR_REG, I2C_SLAVE_READ(readSlaveAddress) );

    // write ICR: set START bit, clear STOP bit, set Transfer Byte bit to initiate the access
    I2C_REG_READ(PICR_REG,ICRRegValue);
	ICRRegValue &= ~I2C_ICR_STOP;
    //GPIO_TB_SET();
	ICRRegValue |= (I2C_ICR_START | I2C_ICR_TRANSFER_BYTE);
    I2C_REG_WRITE(PICR_REG, ICRRegValue );
    //GPIO_TB_CLR();

	I2CReturnCode = i2cWaitStatus(I2C_REG_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT)|I2C_REG_BIT(I2C_ISR_UNIT_BUSY_BIT)|I2C_REG_BIT(I2C_ISR_READ_WRITE_MODE_BIT), 0 );

    // clear 'IDBR Transmit Empty' bit (write '1')
    I2C_STATUS_REG_CLEAR_BIT(I2C_ISR_IDBR_TRANSMIT_EMPTY_BIT);

	if ( I2CReturnCode != I2C_RC_OK )
		return I2CReturnCode;

    // enable 'IDBR Buffer Full' interrupt
    I2C_REG_BIT_WRITE(PICR_REG, I2C_ICR_IDBR_RECEIVE_FULL_INT_ENABLE_BIT, SET);

    // Initiate the read process:

    if ( dataSize == 1 )   // only one byte to read
    {
        // write ICR: set STOP bit, set ACK/NACK bit (1 for NACK)
        I2C_REG_READ(PICR_REG,ICRRegValue);
  		ICRRegValue &= ~I2C_ICR_START;
  	    ICRRegValue |= (I2C_ICR_STOP | I2C_ICR_ACK_NACK_CONTROL);
   	    I2C_REG_WRITE(PICR_REG, ICRRegValue );
    }
    else   // more than one byte to read
    {
        // write ICR: clear STOP bit, clear ACK/NACK bit (0 for ACK)
    	I2C_REG_READ(PICR_REG,ICRRegValue);
		ICRRegValue &= ~I2C_ICR_STOP;
		ICRRegValue &= ~I2C_ICR_ACK_NACK_CONTROL;
    	I2C_REG_WRITE(PICR_REG, ICRRegValue );
    }

    I2C_REG_READ(PICR_REG,ICRRegValue);
  	ICRRegValue &= ~I2C_ICR_START;
   // GPIO_TB_SET();
  	ICRRegValue |= I2C_ICR_TRANSFER_BYTE;
   	I2C_REG_WRITE(PICR_REG, ICRRegValue );
   // GPIO_TB_CLR();

    return I2C_RC_OK;
}
static I2C_ReturnCode I2CMasterSendData( uint8_t *data , uint16_t dataSize , uint8_t slaveAddress , bool protected , uint16_t userId )
{
    uint32_t 			cpsr;
    I2C_ReturnCode 	I2CReturnCode;

	//_receiveReqParams.activeSlaveAddress = slaveAddress;
	//_receiveReqParams.ID = userId;
    //_receiveReqParams.RxBufferPtr = NULL;

    //I2CReturnCode = I2C_clock( PMU_ON);
	//if ( I2CReturnCode != I2C_RC_OK )
    //    return I2C_ISR_BUS_BUSY;

    //if ( CHECK_IF_GENERAL_CALL(slaveAddress) )
	//	ASSERT(FALSE);

    I2CReturnCode = I2C_busIsIdle();
    if(I2CReturnCode != I2C_RC_OK)
        return I2CReturnCode;

   // if (protected)
	//{
   //     cpsr = disableInterrupts();
    //    I2CReturnCode = masterSend(data, dataSize, slaveAddress, _repeat_start , FALSE);
   //     restoreInterrupts(cpsr);
//	}
//	else
        I2CReturnCode = masterSend(data, dataSize, slaveAddress, false , false);

	//if ( I2CReturnCode == I2C_RC_OK )
   //      I2C_clock(PMU_OFF);

    return I2CReturnCode;
}

void d1980_pi2c_reset(void)
{
    I2C_REG_WRITE(PICR_REG,  I2C_ICR_UNIT_RESET );
    I2C_REG_WRITE(PISR_REG,  I2C_ISR_CLEAR_ALL_INTERRUPTS );
    I2C_REG_WRITE(PICR_REG,  0 );


    I2C_REG_WRITE(PICR_REG, I2C_ICR_UNIT_ENABLE | I2C_ICR_SCLEA_ENABLE );
}
static I2C_ReturnCode  I2C_clock ( uint8_t      onOff )
{

    UINT32 ICRRegValue, IbmrRegValue;
    UINT32 retry=7;

	//if (currentPmuState == onOff)
	//	return I2C_RC_OK;
	// currentPmuState = onOff;

    if ( onOff == 1 )
    {
        //I2C_CLK_ON;
        I2C_REG_READ(PISR_REG,IbmrRegValue);

        printk(KERN_ERR "d1980_pi2c_write2bulk1 PISR_REG = 0x%0x \n",IbmrRegValue);
        
        I2C_REG_READ(PIBMR_REG,IbmrRegValue);

        printk(KERN_ERR "d1980_pi2c_write2bulk1 PIBMR_REG = 0x%0x \n",IbmrRegValue);


        //I2C_REG_READ(PICR_REG,IbmrRegValue);

      //  I2C_REG_WRITE(PCFR_REG,0x880000FC);
       // printk(KERN_ERR "d1980_pi2c_write2bulk1 PCFR_REG = 0x%0x \n",IbmrRegValue);


      //  I2C_REG_READ(PMCR_REG,IbmrRegValue);
      //  printk(KERN_ERR "d1980_pi2c_write2bulk1 PMCR_REG = 0x%0x \n",IbmrRegValue);
        
        
     //     I2C_REG_WRITE(PISAR_REG, _I2CRegistersBackup.ISARBackup);
       // I2C_REG_WRITE(PICR_REG,  I2C_ICR_UNIT_RESET );
       // I2C_REG_WRITE(PISR_REG,  I2C_ISR_CLEAR_ALL_INTERRUPTS );
       // I2C_REG_WRITE(PICR_REG,  0 );

//#if defined (CLOCK_ADJUST_ILCR)
//		I2C_REG_WRITE(I2C_ILCR_REG, _I2CRegistersBackup.ILCRBackup);
//#endif
//#if defined (CLOCK_ADJUST_IWCR)
//		I2C_REG_WRITE(I2C_IWCR_REG, _I2CRegistersBackup.IWCRBackup);
//#endif
       // I2C_REG_WRITE(PICR_REG, I2C_ICR_UNIT_ENABLE | I2C_ICR_SCLEA_ENABLE );
       // GPIO_UE_ON();
        
        //Wait for Unit Enable with retry
       // while(retry--)
      //  {
      ///      msleep(1);
       //     I2C_REG_READ(PIBMR_REG,IbmrRegValue);
      //      if ( IbmrRegValue == I2C_IBMR_SCL_AND_SDA_LINES_HIGH )
      //          break;
      //  }
       // GPIO_ALL_OFF();
    }
    else
    {   //The HW transaction may be still in progres. Waite a bit.
        //If still busy, apply MasterAbort which adds the missing stop signal
        //and close clock anyway
        while(retry--)
        {
            I2C_REG_READ(PIBMR_REG,IbmrRegValue);
            if ( IbmrRegValue == I2C_IBMR_SCL_AND_SDA_LINES_HIGH )
                break;
            msleep(1);
        }
        if(retry<=0)
        {
            I2C_REG_READ(PICR_REG,ICRRegValue);
            ICRRegValue |= I2C_ICR_MASTER_ABORT;
            I2C_REG_WRITE(PICR_REG, ICRRegValue );
            msleep(1);
        }
        I2C_REG_WRITE(PICR_REG, I2C_ICR_UNIT_RESET );
        I2C_REG_WRITE(PICR_REG, 0);
        msleep(1); //OSATaskSleep(2); //Delay before I2C_CLK_OFF
        //I2C_CLK_OFF;
	  }
	return I2C_RC_OK;
}

extern unsigned char g_d1981_power_state;
void d1980_pi2c_write2bulk1(int value)
{
  I2C_ReturnCode status;
  uint8_t txdata[8],rxdata[8];
  uint16_t dataSize;

//  printk(KERN_ERR "d1980_pi2c_write2bulk1 volt = %d \n",value);
///Bulk1 voltage set
  txdata[0] = 0x2E;	
	
#if 0  
  switch(value)
  {
     case 1375:
        txdata[1] = 0x63;
        break;
     case 1275:
        txdata[1] = 0x5f;
        break;
     case 1200:
        txdata[1] = 0x5c;
        break;
     case 1150:
        txdata[1] = 0x5a;
        break;
     default:
        printk(KERN_ERR "d1980_pi2c_write2bulk1 error value = %d \n",value);
        return;

  }
#else
    txdata[1] = value/25+0x2c;
#endif
  status = I2CMasterSendData( txdata,
                            2,
                            0x92,
                            FALSE,
                            0 );

  if(status != I2C_RC_OK)
  {
    printk(KERN_ERR "d1980_pi2c_write2bulk1 Error1 = %d \n",status);
    return;
  }

    #if 0
  txdata[0] = 0x46;

  status = I2CMasterReceiveData(txdata,           	/* Command to the slave */
								   1,
							   0x92,
							   FALSE,
            	       		   rxdata,           	/* Pointer for the receive data */
								   1,
							   0x93,
							   0);
                          
///Bulk1 Go register   
  if(status != I2C_RC_OK)
  {
     printk(KERN_ERR "d1980_pi2c_write2bulk1 Error2 = %d \n",status);
     return;
  }
  #endif
  txdata[0] = 0x46;	
  //txdata[1] = rxdata[0] | 0x1;
  if(g_d1981_power_state==1)
    txdata[1] =0x70 | 0x1;
  else
    txdata[1] =0x30 | 0x1;
  status = I2CMasterSendData( txdata,
                            2,
                            0x92,
                            FALSE,
                            0 );   
  if(status != I2C_RC_OK)
  {
     printk(KERN_ERR "d1980_pi2c_write2bulk1 Error3 = %d \n",status);
    return;
  }                            

}

