// /home/dchen/lynx_utils/dsp_releases/070311_trunk/scripts/MB_API_CMDS

#define MAILBOX_I2C_PAGE_SIZE	30 //60

/* Return values */
typedef enum {
	FRAMEWORK_STATUS_OK                                = 0x000000,  // Status of framework OK
	CMD_INTERP_STATUS_OK                               = 0x000001,  //MB is ready to accept commands
	CMD_RECEIVING		                               = 0x000002,  //MB is currently receive a command list from the host
	CMD_EXECUTING		                               = 0x000003,  //MB is currently executing a command list from the host
	CMD_PROCESSING_FAIL                               = 0x000004,  //DSP has found an error and has abandoned executing and receiving commands
	FRAMEWORK_INIT_OK                              		= 0x000005,  //DSP has successfully initialised and is waiting for the host to reset the FW
	FRAMEWORK_FATAL_ERR                               = 0x000006,  //DSP has encountered a fatal error and is waiting for the host to completely re-initialise it

	FRAMEWORK_STATUS_UNKNOW_ERROR_CODE                 = 0xEEEE00,  // Unknown error code in error handler

	CMD_INTERP_ERR_INVALID_COMMAND                     = 0xF00100,  // Invalid command received in command interpreter
	CMD_INTERP_ERR_INVALID_COUNTER                     = 0xF00200,  // Invalid counter value in command interpreter state machine
	CMD_INTERP_ERR_BAD_STATE                           = 0xF00300,  // Bad state in command interpreter state machine

	CMD_INTERP_ERR_INVALID_N_FN_INST                   = 0xF00400,  // Too many function instances created when trying to create another function instance 
	CMD_INTERP_ERR_INVALID_FN_IDENTIFIER               = 0xF00500,  // Invalid function instance identifier when creating a function instance
	CMD_INTERP_ERR_FN_INST_OVERFLOW                    = 0xF00600,  // Overflow of function instance parameter structure when storing a function instance parameter
	CMD_INTERP_ERR_FN_INST_HEAP_OVERFLOW               = 0xF00700,  // Overflow of function instance heap when allocating memory for a new function instance
	CMD_INTERP_ERR_INVALID_BUFFER_IDENTIFIER           = 0xF00800,  // Invalid buffer identifier when interpreting identifier

	CMD_INTERP_ERR_INVALID_FN_INDEX                    = 0xF00900,  // Invalid function index when changing a parameter, scheduling a function instance or reading a function instance parameter
	CMD_INTERP_ERR_INVALID_N_FN_INST_SCHEDULE          = 0xF00A00,  // Too many function instances being scheduled
	CMD_INTERP_ERR_INVALID_FN_SCHEDULE_START_INDEX     = 0xF00B00,  // Start index requested is greater than number of function instances in a processing schedule

	CMD_INTERP_ERR_DYN_BUFF_HEAP_OVERFLOW              = 0xF00C00,  // Allocated dynamic buffer will overflow dynamic buffer heap
	CMD_INTERP_ERR_DYN_BUFFER_NOT_ALLOC                = 0xF00D00,  // Dynamic buffer identifier received is that of a dynamic buffer which has not been created
	CMD_INTERP_ERR_NO_MORE_DYN_BUFF                    = 0xF00E00,  // No more dynamic buffers to allocate (all identifiers are already allocated)

	CMD_INTERP_ERR_INVALID_N_BG_FN_INST                = 0xF00F00,  // Too many background function instances created when trying to create another background  function instance 
	CMD_INTERP_ERR_INVALID_BG_FN_IDENTIFIER            = 0xF01000,  // Invalid background function instance identifier when creating a background function instance
	CMD_INTERP_ERR_BG_FN_INST_HEAP_OVERFLOW            = 0xF01100,  // Overflow of background function instance parameter structure when storing a background function instance parameter

	CMD_INTERP_ERR_INVALID_BG_FN_INDEX                 = 0xF01200,  // Invalid background function index when changing a parameter, scheduling a background function instance or reading a background function instance parameter
	CMD_INTERP_ERR_INVALID_N_BG_FN_INST_SCHEDULE       = 0xF01300,  // Too many background function instances being scheduled
	CMD_INTERP_ERR_INVALID_BG_FN_SCHEDULE_START_INDEX  = 0xF01400,  // Start index requested is greater than number of background function instances in a background processing schedule

	CMD_INTERP_ERR_INVALID_BLOCK_SOURCE                = 0xF01500,  // Invalid block source identifier when setting a block size or scheduling a block pointer update
	CMD_INTERP_ERR_INVALID_BLOCK_SIZE                  = 0xF01600,  // Invalid value for block size for an IO buffer (block size must be a factor of synchronous buffer length)
	CMD_INTERP_ERR_INVALID_N_BP_SCHED                  = 0xF01700,  // Too many block pointer updates in schedule

	//CMD_INTERP_ERR_FILT_WEIGHT_SET_OVERFLOW            = 0xF01800,  // Received weight which overflows allocated filter weight set memory allocation
	CMD_INTERP_ERR_INVALID_N_FILT_WEIGHT_SET           = 0xF01900,  // All identifiers for filter weight sets are being used
	CMD_INTERP_ERR_FILT_WEIGHT_HEAP_OVERFLOW           = 0xF01A00,  // Allocating a new filter weight will overflow the filter weight heap

	CMD_INTERP_ERR_INVALID_PP_PTR_INDEX                = 0xF01B00,  // Invalid ping pong pointer index when scheduling a ping pong pointer update or changing a ping pong pointer
	CMD_INTERP_ERR_INVALID_N_PP_PTR                    = 0xF01C00,  // Too many ping pong pointers created
	CMD_INTERP_ERR_INVALID_N_PP_PTR_SCHEDULE           = 0xF01D00,  // Too many ping pong pointers in ping pong update schedule

	CMD_INTERP_ERR_INVALID_N_ADC                       = 0xF01E00,  // Invalid value for NADC when configuring framework
	CMD_INTERP_ERR_INVALID_N_DAC                       = 0xF01F00,  // Invalid value for NDAC when configuring framework
	CMD_INTERP_ERR_INVALID_MASTER_CLOCK                = 0xF02000,  // Invalid value for master clock when configuring framework
	CMD_INTERP_ERR_INVALID_I2S_MASK                    = 0xF02100,  // Invalid I2S mask when configuring framework

	CMD_INTERP_ERR_TOO_MANY_PARM_CHANGES               = 0xF02200,  // Too many parameter changes scheduled
	CMD_INTERP_ERR_TOO_MANY_BG_PARM_CHANGES            = 0xF02300,  // Too many background parameter changes scheduled
	CMD_INTERP_ERR_TOO_MANY_PPPTR_CHANGES              = 0xF02400,  // Too many ping pong pointer changes scheduled
	CMD_INTERP_ERR_INVALID_PARAM_OFFSET                = 0xF02500,  // Invalid parameter offset within a function parameter structure when changing or reading a parameter
	CMD_INTERP_ERR_INVALID_BG_PARAMETER_OFFSET         = 0xF02600,  // Invalid background parameter offset within a background function parameter structure when changing or reading a background parameter
	MAILBOX_CMD_ERR_IN_EXEC_STATE                      = 0xF02700 
} dspErrorCode;

const u8 DSP_ABORT[] = {
0x75, 0x02,
0x76, 0xba,
0x77, 0xba,
0x78, 0xba,
};
const u16 DSP_ABORT_SIZE = 4;


const u8 DSP_ABORT_RESET_FW[] = {
0x75, 0x02,
0x76, 0xba,
0x77, 0xba,
0x78, 0xba,
0x76, 0x02,
0x77, 0x00,
0x78, 0x05,
0x76, 0xaa,
0x77, 0xaa,
0x78, 0xaa,
};
const u16 DSP_ABORT_RESET_FW_SIZE = 10;


const u8 DSP_MB_CMD_COUNT_REQ[] = {
0x75, 0x04,
0x76, 0xac,
0x77, 0xac,
0x78, 0xac,
};
const u16 DSP_MB_CMD_COUNT_REQ_SIZE = 4;


const u8 DSP_MB_STATUS_REQ[] = {
0x75, 0x01,
0x76, 0xab,
0x77, 0xab,
0x78, 0xab,
};
const u16 DSP_MB_STATUS_REQ_SIZE = 4;


const u8 DSP_RESET_FW[] = {
0x76, 0x02,
0x77, 0x00,
0x78, 0x05,
0x76, 0xaa,
0x77, 0xaa,
0x78, 0xaa,
};
const u16 DSP_RESET_FW_SIZE = 6;
