/*****************************************************************************
						Copyright(c) 2009 YokogawaDigitalComputer Corporation
機能：システムマクロトレースカーネルランドAPIライブラリ

注意：

変更履歴
 +-------------- 履歴番号 (000 〜 999)
 |	  +--------- 修正しているシステムバージョン
 |	  | 	+--- 新規、変更、追加、削除の分類
 v	  v 	v
 No  Ver  分類 年月日	  名前			説明
---+-----+----+----------+-------------+--------------------------------------
000 00.00 新規 2010/04/19 S.Tonoshita	新規開発
*****************************************************************************/

/* インクルード指定 *********************************************************/
#include <linux/module.h>
#include <smt/SMTAPI.h>

/* 定数宣言 ******************************************************************/
#define SET_FUNCPOINTER(api)		_SMT_##api = (func ? func : _SMT_##api##_nop)

/* 関数プロトタイプ宣言 *****************************************************/
void smt_set_PortOut( int (*func)(unsigned long addr,unsigned long data,_SMT_PSZ size,_SMT_PRW rw ) );
void smt_set_Printf( int (*func)(int level,const char *format , ...) );
void smt_set_Puts( int (*func)(int level,const char *s) );
void smt_set_UsrMsgTag0( int (*func)(int level,unsigned long TagNum ) );
void smt_set_UsrMsgTag1( int (*func)(int level,unsigned long TagNum,unsigned long arg1 ) );
void smt_set_UsrMsgTag2( int (*func)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2 ) );
void smt_set_UsrMsgTag3( int (*func)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3 ) );
void smt_set_UsrMsgTag4( int (*func)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4 ) );

void smt_set_OsSwitch_Process( int (*func)(unsigned long processid) );
void smt_set_OsSwitch_ThreadProcess( int (*func)(unsigned long threadid,unsigned long processid) );
void smt_set_OsSwitch_Process_Name( int (*func)(unsigned long processid,const char *str) );
void smt_set_OsSwitch_ThreadProcess_Name( int (*func)(unsigned long threadid,unsigned long processid,const char *tname,const char *pname) );
void smt_set_OsSwitch_Irq_in( int (*func)(unsigned long irqid) );
void smt_set_OsSwitch_Irq_out( int (*func)(unsigned long irqid) );
void smt_set_OsSwitch_Idle( int (*func)( void ) );
void smt_set_OsCall0( int (*func)(int osc,_SMT_OS_CALL attr) );
void smt_set_OsCall1( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1) );
void smt_set_OsCall2( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2) );
void smt_set_OsCall3( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3) );
void smt_set_OsCall4( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4) );
void smt_set_OsCall5( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5) );
void smt_set_OsCall6( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6) );
void smt_set_OsCall7( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7) );
void smt_set_OsCall8( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8) );
void smt_set_OsCall9( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9) );
void smt_set_OsCall10( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10) );
void smt_set_OsCall11( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11) );
void smt_set_OsCall12( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12) );
void smt_set_OsCall13( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13) );
void smt_set_OsCall14( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14) );
void smt_set_OsCall15( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15) );
void smt_set_OsCall16( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16) );

void smt_set_Hook0( void (*func)(unsigned long TagNum) );
void smt_set_Hook1( void (*func)(unsigned long TagNum,unsigned long arg1) );
void smt_set_Hook2( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2) );
void smt_set_Hook3( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3) );
void smt_set_Hook4( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4) );
void smt_set_Hook5( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5) );
void smt_set_Hook6( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6) );
void smt_set_Hook7( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7) );
void smt_set_Hook8( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8) );
void smt_set_Hook9( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9) );
void smt_set_Hook10( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10) );
void smt_set_Hook11( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11) );
void smt_set_Hook12( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12) );
void smt_set_Hook13( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13) );
void smt_set_Hook14( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14) );
void smt_set_Hook15( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15) );
void smt_set_Hook16( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16) );

/* 内部関数プロトタイプ宣言 *************************************************/
static int _SMT_PortOut_nop(unsigned long addr,unsigned long data,_SMT_PSZ size,_SMT_PRW rw);
static int _SMT_Printf_nop(int level,const char *format , ...);
static int _SMT_Puts_nop(int level,const char *s);
static int _SMT_UsrMsgTag0_nop(int level,unsigned long TagNum);
static int _SMT_UsrMsgTag1_nop(int level,unsigned long TagNum,unsigned long arg1);
static int _SMT_UsrMsgTag2_nop(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2);
static int _SMT_UsrMsgTag3_nop(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3);
static int _SMT_UsrMsgTag4_nop(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4);
static int _SMT_OsSwitch_Process_nop(unsigned long processid);
static int _SMT_OsSwitch_ThreadProcess_nop(unsigned long threadid,unsigned long processid);
static int _SMT_OsSwitch_Process_Name_nop(unsigned long processid,const char *str);
static int _SMT_OsSwitch_ThreadProcess_Name_nop(unsigned long threadid,unsigned long processid,const char *tname,const char *pname);
static int _SMT_OsSwitch_Irq_in_nop(unsigned long irqid);
static int _SMT_OsSwitch_Irq_out_nop(unsigned long irqid);
static int _SMT_OsSwitch_Idle_nop( void );

static int _SMT_OsCall0_nop(int osc,_SMT_OS_CALL attr);
static int _SMT_OsCall1_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1);
static int _SMT_OsCall2_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2);
static int _SMT_OsCall3_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3);
static int _SMT_OsCall4_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4);
static int _SMT_OsCall5_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5);
static int _SMT_OsCall6_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6);
static int _SMT_OsCall7_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7);
static int _SMT_OsCall8_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8);
static int _SMT_OsCall9_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9);
static int _SMT_OsCall10_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10);
static int _SMT_OsCall11_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11);
static int _SMT_OsCall12_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12);
static int _SMT_OsCall13_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13);
static int _SMT_OsCall14_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14);
static int _SMT_OsCall15_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15);
static int _SMT_OsCall16_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16);

static void _TRQ_Hook0_nop(unsigned long TagNum);
static void _TRQ_Hook1_nop(unsigned long TagNum,unsigned long arg1);
static void _TRQ_Hook2_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2);
static void _TRQ_Hook3_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3);
static void _TRQ_Hook4_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4);
static void _TRQ_Hook5_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5);
static void _TRQ_Hook6_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6);
static void _TRQ_Hook7_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7);
static void _TRQ_Hook8_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8);
static void _TRQ_Hook9_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9);
static void _TRQ_Hook10_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10);
static void _TRQ_Hook11_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11);
static void _TRQ_Hook12_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12);
static void _TRQ_Hook13_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13);
static void _TRQ_Hook14_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14);
static void _TRQ_Hook15_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15);
static void _TRQ_Hook16_nop(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16);

/* 内部変数宣言 *************************************************************/
static int SMTDebugLevel 		  = SMT_LV_MSK;

int (*_SMT_PortOut)(unsigned long addr ,unsigned long data,_SMT_PSZ size,_SMT_PRW rw)												= _SMT_PortOut_nop;
int (*_SMT_Printf)(int level,const char *format , ...)																				= _SMT_Printf_nop;
int (*_SMT_Puts)(int level,const char *s)																							= _SMT_Puts_nop;
int (*_SMT_UsrMsgTag0)(int level,unsigned long TagNum)																				= _SMT_UsrMsgTag0_nop;
int (*_SMT_UsrMsgTag1)(int level,unsigned long TagNum,unsigned long arg1)															= _SMT_UsrMsgTag1_nop;
int (*_SMT_UsrMsgTag2)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2)										= _SMT_UsrMsgTag2_nop;
int (*_SMT_UsrMsgTag3)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3)						= _SMT_UsrMsgTag3_nop;
int (*_SMT_UsrMsgTag4)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4)	= _SMT_UsrMsgTag4_nop;
int (*_SMT_OsSwitch_Process)(unsigned long processid) 																				= _SMT_OsSwitch_Process_nop;
int (*_SMT_OsSwitch_ThreadProcess)(unsigned long threadid,unsigned long processid) 													= _SMT_OsSwitch_ThreadProcess_nop;
int (*_SMT_OsSwitch_Process_Name)(unsigned long processid,const char *str) 															= _SMT_OsSwitch_Process_Name_nop;
int (*_SMT_OsSwitch_ThreadProcess_Name)(unsigned long threadid,unsigned long processid,const char *tname,const char *pname) 		= _SMT_OsSwitch_ThreadProcess_Name_nop;
int (*_SMT_OsSwitch_Irq_in)(unsigned long irqid) 																					= _SMT_OsSwitch_Irq_in_nop;
int (*_SMT_OsSwitch_Irq_out)(unsigned long irqid) 																					= _SMT_OsSwitch_Irq_out_nop;
int (*_SMT_OsSwitch_Idle)( void ) 																									= _SMT_OsSwitch_Idle_nop;

int (*_SMT_OsCall0)(int osc,_SMT_OS_CALL attr) 																						= _SMT_OsCall0_nop;
int (*_SMT_OsCall1)(int osc,_SMT_OS_CALL attr,unsigned long arg1) 																	= _SMT_OsCall1_nop;
int (*_SMT_OsCall2)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2) 												= _SMT_OsCall2_nop;
int (*_SMT_OsCall3)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3) 							= _SMT_OsCall3_nop;
int (*_SMT_OsCall4)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4) 			= _SMT_OsCall4_nop;
int (*_SMT_OsCall5)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5) 																																																							= _SMT_OsCall5_nop;
int (*_SMT_OsCall6)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6) 																																																		= _SMT_OsCall6_nop;
int (*_SMT_OsCall7)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7) 																																													= _SMT_OsCall7_nop;
int (*_SMT_OsCall8)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8) 																																								= _SMT_OsCall8_nop;
int (*_SMT_OsCall9)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9)																																				= _SMT_OsCall9_nop;
int (*_SMT_OsCall10)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10) 																														= _SMT_OsCall10_nop;
int (*_SMT_OsCall11)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11) 																									= _SMT_OsCall11_nop;
int (*_SMT_OsCall12)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12) 																				= _SMT_OsCall12_nop;
int (*_SMT_OsCall13)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13) 															= _SMT_OsCall13_nop;
int (*_SMT_OsCall14)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14) 										= _SMT_OsCall14_nop;
int (*_SMT_OsCall15)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15) 					= _SMT_OsCall15_nop;
int (*_SMT_OsCall16)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16) = _SMT_OsCall16_nop;

void (*_TRQ_Hook0)(unsigned long TagNum)																							= _TRQ_Hook0_nop;
void (*_TRQ_Hook1)(unsigned long TagNum,unsigned long arg1)																			= _TRQ_Hook1_nop;
void (*_TRQ_Hook2)(unsigned long TagNum,unsigned long arg1,unsigned long arg2)														= _TRQ_Hook2_nop;
void (*_TRQ_Hook3)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3)									= _TRQ_Hook3_nop;
void (*_TRQ_Hook4)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4)				= _TRQ_Hook4_nop;
void (*_TRQ_Hook5)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5)																																																								= _TRQ_Hook5_nop;
void (*_TRQ_Hook6)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6) 																																																			= _TRQ_Hook6_nop;
void (*_TRQ_Hook7)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7)																																														= _TRQ_Hook7_nop;
void (*_TRQ_Hook8)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8) 																																									= _TRQ_Hook8_nop;
void (*_TRQ_Hook9)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9)																																					= _TRQ_Hook9_nop;
void (*_TRQ_Hook10)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10)																															= _TRQ_Hook10_nop;
void (*_TRQ_Hook11)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11)																										= _TRQ_Hook11_nop;
void (*_TRQ_Hook12)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12)																					= _TRQ_Hook12_nop;
void (*_TRQ_Hook13)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13)																= _TRQ_Hook13_nop;
void (*_TRQ_Hook14)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14)											= _TRQ_Hook14_nop;
void (*_TRQ_Hook15)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15)						= _TRQ_Hook15_nop;
void (*_TRQ_Hook16)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16)	= _TRQ_Hook16_nop;

EXPORT_SYMBOL(_SMT_PortOut);
EXPORT_SYMBOL(_SMT_Printf);
EXPORT_SYMBOL(_SMT_Puts);
EXPORT_SYMBOL(_SMT_UsrMsgTag0);
EXPORT_SYMBOL(_SMT_UsrMsgTag1);
EXPORT_SYMBOL(_SMT_UsrMsgTag2);
EXPORT_SYMBOL(_SMT_UsrMsgTag3);
EXPORT_SYMBOL(_SMT_UsrMsgTag4);
EXPORT_SYMBOL(_SMT_OsSwitch_Process);
EXPORT_SYMBOL(_SMT_OsSwitch_ThreadProcess);
EXPORT_SYMBOL(_SMT_OsSwitch_Process_Name);
EXPORT_SYMBOL(_SMT_OsSwitch_ThreadProcess_Name);
EXPORT_SYMBOL(_SMT_OsSwitch_Irq_in);
EXPORT_SYMBOL(_SMT_OsSwitch_Irq_out);
EXPORT_SYMBOL(_SMT_OsSwitch_Idle);
EXPORT_SYMBOL(_SMT_OsCall0);
EXPORT_SYMBOL(_SMT_OsCall1);
EXPORT_SYMBOL(_SMT_OsCall2);
EXPORT_SYMBOL(_SMT_OsCall3);
EXPORT_SYMBOL(_SMT_OsCall4);
EXPORT_SYMBOL(_SMT_OsCall5);
EXPORT_SYMBOL(_SMT_OsCall6);
EXPORT_SYMBOL(_SMT_OsCall7);
EXPORT_SYMBOL(_SMT_OsCall8);
EXPORT_SYMBOL(_SMT_OsCall9);
EXPORT_SYMBOL(_SMT_OsCall10);
EXPORT_SYMBOL(_SMT_OsCall11);
EXPORT_SYMBOL(_SMT_OsCall12);
EXPORT_SYMBOL(_SMT_OsCall13);
EXPORT_SYMBOL(_SMT_OsCall14);
EXPORT_SYMBOL(_SMT_OsCall15);
EXPORT_SYMBOL(_SMT_OsCall16);
EXPORT_SYMBOL(_TRQ_Hook0);
EXPORT_SYMBOL(_TRQ_Hook1);
EXPORT_SYMBOL(_TRQ_Hook2);
EXPORT_SYMBOL(_TRQ_Hook3);
EXPORT_SYMBOL(_TRQ_Hook4);
EXPORT_SYMBOL(_TRQ_Hook5);
EXPORT_SYMBOL(_TRQ_Hook6);
EXPORT_SYMBOL(_TRQ_Hook7);
EXPORT_SYMBOL(_TRQ_Hook8);
EXPORT_SYMBOL(_TRQ_Hook9);
EXPORT_SYMBOL(_TRQ_Hook10);
EXPORT_SYMBOL(_TRQ_Hook11);
EXPORT_SYMBOL(_TRQ_Hook12);
EXPORT_SYMBOL(_TRQ_Hook13);
EXPORT_SYMBOL(_TRQ_Hook14);
EXPORT_SYMBOL(_TRQ_Hook15);
EXPORT_SYMBOL(_TRQ_Hook16);

/*****************************************************************************
1.機能：デバックプリント文デバックレベルを参照する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------

4.戻り値：
*****************************************************************************/
int	_SMT_GetDebugLevel( void )
{
	return SMTDebugLevel;
}
EXPORT_SYMBOL(_SMT_GetDebugLevel);

/*****************************************************************************
1.機能：デバックプリント文デバックレベルを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------

4.戻り値：
*****************************************************************************/
void _SMT_SetDebugLevel( int level )
{
	SMTDebugLevel = level;
}
EXPORT_SYMBOL(_SMT_SetDebugLevel);

/*****************************************************************************
1.機能：_SMT_PortOut()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_PortOut( int (*func)(unsigned long addr ,unsigned long data ,_SMT_PSZ size ,_SMT_PRW rw ) )
{
	if( func ){
		_SMT_PortOut = func;
	}
	else{
		_SMT_PortOut = _SMT_PortOut_nop;
	}
}
EXPORT_SYMBOL(smt_set_PortOut);

/*****************************************************************************
1.機能：_SMT_Printf()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Printf( int (*func)(int level,const char *format , ...) )
{
	if( func ){
		_SMT_Printf = func;
	}
	else{
		_SMT_Printf = _SMT_Printf_nop;
	}
}
EXPORT_SYMBOL(smt_set_Printf);

/*****************************************************************************
1.機能：_SMT_Puts()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Puts( int (*func)(int level,const char *s) )
{
	if( func ){
		_SMT_Puts = func;
	}
	else{
		_SMT_Puts = _SMT_Puts_nop;
	}
}
EXPORT_SYMBOL(smt_set_Puts);

/*****************************************************************************
1.機能：_SMT_UsrMsgTag0()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_UsrMsgTag0( int(*func)(int level,unsigned long TagNum ) )
{
	if( func ){
		_SMT_UsrMsgTag0 = func;
	}
	else{
		_SMT_UsrMsgTag0 = _SMT_UsrMsgTag0_nop;
	}
}
EXPORT_SYMBOL(smt_set_UsrMsgTag0);

/*****************************************************************************
1.機能：_SMT_UsrMsgTag1()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_UsrMsgTag1( int (*func)(int level,unsigned long TagNum,unsigned long arg1 ) )
{
	if( func ){
		_SMT_UsrMsgTag1 = func;
	}
	else{
		_SMT_UsrMsgTag1 = _SMT_UsrMsgTag1_nop;
	}
}
EXPORT_SYMBOL(smt_set_UsrMsgTag1);

/*****************************************************************************
1.機能：_SMT_UsrMsgTag2()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_UsrMsgTag2( int (*func)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2 ) )
{
	if( func ){
		_SMT_UsrMsgTag2 = func;
	}
	else{
		_SMT_UsrMsgTag2 = _SMT_UsrMsgTag2_nop;
	}
}
EXPORT_SYMBOL(smt_set_UsrMsgTag2);

/*****************************************************************************
1.機能：_SMT_UsrMsgTag3()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_UsrMsgTag3( int (*func)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3 ) )
{
	if( func ){
		_SMT_UsrMsgTag3 = func;
	}
	else{
		_SMT_UsrMsgTag3 = _SMT_UsrMsgTag3_nop;
	}
}
EXPORT_SYMBOL(smt_set_UsrMsgTag3);

/*****************************************************************************
1.機能：_SMT_UsrMsgTag4()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_UsrMsgTag4( int (*func)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4 ) )
{
	if( func ){
		_SMT_UsrMsgTag4 = func;
	}
	else{
		_SMT_UsrMsgTag4 = _SMT_UsrMsgTag4_nop;
	}
}
EXPORT_SYMBOL(smt_set_UsrMsgTag4);

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_PortOut()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	addr						アドレス値
I/	data						データ値
I/	size						ポートサイズ 1:8bit 2:16bit 3:32bit
I/	rw							リードライト種別 0:ライト 1:リード

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_PortOut_nop(unsigned long addr ,unsigned long data ,_SMT_PSZ size ,_SMT_PRW rw )
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMTPrintf()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	*format						書式
I/	agg1						引数....

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_Printf_nop(int level,const char *format , ...)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMTPuts()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	*format						書式
I/	agg1						引数....

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_Puts_nop(int level,const char *s)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_UsrMsgTag0()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	TagNum						メッセージタグ番号

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_UsrMsgTag0_nop(int level,unsigned long TagNum)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_UsrMsgTag1()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	TagNum						メッセージタグ番号
I/	arg1						引数1

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_UsrMsgTag1_nop(int level,unsigned long TagNum,unsigned long arg1)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_UsrMsgTag2()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	TagNum						メッセージタグ番号
I/	arg1						引数1
I/	arg2						引数2

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_UsrMsgTag2_nop(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_UsrMsgTag4()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	TagNum						メッセージタグ番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_UsrMsgTag3_nop(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_UsrMsgTag4()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	level						出力レベル
I/	TagNum						メッセージタグ番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_UsrMsgTag4_nop(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：_SMT_OsSwitch_Process()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_Process( int (*func)(unsigned long processid) )
{
	SET_FUNCPOINTER(OsSwitch_Process);
}
EXPORT_SYMBOL(smt_set_OsSwitch_Process);

/*****************************************************************************
1.機能：_SMT_OsSwitch_ThreadProcess()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_ThreadProcess( int (*func)(unsigned long threadid,unsigned long processid) )
{
	SET_FUNCPOINTER(OsSwitch_ThreadProcess);
}
EXPORT_SYMBOL(smt_set_OsSwitch_ThreadProcess);

/*****************************************************************************
1.機能：_SMT_OsSwitch_Process_Name()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_Process_Name( int (*func)(unsigned long processid,const char *str) )
{
	SET_FUNCPOINTER(OsSwitch_Process_Name);
}
EXPORT_SYMBOL(smt_set_OsSwitch_Process_Name);

/*****************************************************************************
1.機能：_SMT_OsSwitch_ThreadProcess_Name()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_ThreadProcess_Name( int (*func)(unsigned long threadid,unsigned long processid,const char *tname,const char *pname) )
{
	SET_FUNCPOINTER(OsSwitch_ThreadProcess_Name);
}
EXPORT_SYMBOL(smt_set_OsSwitch_ThreadProcess_Name);

/*****************************************************************************
1.機能：_SMT_OsSwitch_Irq_in()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_Irq_in( int (*func)(unsigned long irqid) )
{
	SET_FUNCPOINTER(OsSwitch_Irq_in);
}
EXPORT_SYMBOL(smt_set_OsSwitch_Irq_in);

/*****************************************************************************
1.機能：_SMT_OsSwitch_Irq_out()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_Irq_out( int (*func)(unsigned long irqid) )
{
	SET_FUNCPOINTER(OsSwitch_Irq_out);
}
EXPORT_SYMBOL(smt_set_OsSwitch_Irq_out);

/*****************************************************************************
1.機能：_SMT_OsSwitch_Idle()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsSwitch_Idle( int (*func)( void ) )
{
	SET_FUNCPOINTER(OsSwitch_Idle);
}
EXPORT_SYMBOL(smt_set_OsSwitch_Idle);

/*****************************************************************************
1.機能：_SMT_OsCall1()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall0( int (*func)(int osc,_SMT_OS_CALL attr) )
{
	SET_FUNCPOINTER(OsCall0);
}
EXPORT_SYMBOL(smt_set_OsCall0);

/*****************************************************************************
1.機能：_SMT_OsCall1()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall1( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1) )
{
	SET_FUNCPOINTER(OsCall1);
}
EXPORT_SYMBOL(smt_set_OsCall1);

/*****************************************************************************
1.機能：_SMT_OsCall2()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall2( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2) )
{
	SET_FUNCPOINTER(OsCall2);
}
EXPORT_SYMBOL(smt_set_OsCall2);

/*****************************************************************************
1.機能：_SMT_OsCall3()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall3( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3) )
{
	SET_FUNCPOINTER(OsCall3);
}
EXPORT_SYMBOL(smt_set_OsCall3);

/*****************************************************************************
1.機能：_SMT_OsCall4()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall4( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4) )
{
	SET_FUNCPOINTER(OsCall4);
}
EXPORT_SYMBOL(smt_set_OsCall4);

/*****************************************************************************
1.機能：_SMT_OsCall5()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall5( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5) )
{
	SET_FUNCPOINTER(OsCall5);
}
EXPORT_SYMBOL(smt_set_OsCall5);

/*****************************************************************************
1.機能：_SMT_OsCall6()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall6( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6) )
{
	SET_FUNCPOINTER(OsCall6);
}
EXPORT_SYMBOL(smt_set_OsCall6);

/*****************************************************************************
1.機能：_SMT_OsCall7()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall7( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7) )
{
	SET_FUNCPOINTER(OsCall7);
}
EXPORT_SYMBOL(smt_set_OsCall7);

/*****************************************************************************
1.機能：_SMT_OsCall8()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall8( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8) )
{
	SET_FUNCPOINTER(OsCall8);
}
EXPORT_SYMBOL(smt_set_OsCall8);

/*****************************************************************************
1.機能：_SMT_OsCall9()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall9( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9) )
{
	SET_FUNCPOINTER(OsCall9);
}
EXPORT_SYMBOL(smt_set_OsCall9);

/*****************************************************************************
1.機能：_SMT_OsCall10()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall10( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10) )
{
	SET_FUNCPOINTER(OsCall10);
}
EXPORT_SYMBOL(smt_set_OsCall10);

/*****************************************************************************
1.機能：_SMT_OsCall11()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall11( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11) )
{
	SET_FUNCPOINTER(OsCall11);
}
EXPORT_SYMBOL(smt_set_OsCall11);

/*****************************************************************************
1.機能：_SMT_OsCall12()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall12( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12) )
{
	SET_FUNCPOINTER(OsCall12);
}
EXPORT_SYMBOL(smt_set_OsCall12);

/*****************************************************************************
1.機能：_SMT_OsCall13()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall13( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13) )
{
	SET_FUNCPOINTER(OsCall13);
}
EXPORT_SYMBOL(smt_set_OsCall13);

/*****************************************************************************
1.機能：_SMT_OsCall14()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall14( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14) )
{
	SET_FUNCPOINTER(OsCall14);
}
EXPORT_SYMBOL(smt_set_OsCall14);

/*****************************************************************************
1.機能：_SMT_OsCall15()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall15( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15) )
{
	SET_FUNCPOINTER(OsCall15);
}
EXPORT_SYMBOL(smt_set_OsCall15);

/*****************************************************************************
1.機能：_SMT_OsCall16()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_OsCall16( int (*func)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16) )
{
	SET_FUNCPOINTER(OsCall16);
}
EXPORT_SYMBOL(smt_set_OsCall16);


/*****************************************************************************
1.機能：_TRQ_Hook0()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook0( void (*func)(unsigned long TagNum) )
{
	if( func ){
		_TRQ_Hook0 = func;
	}
	else{
		_TRQ_Hook0 = _TRQ_Hook0_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook0);

/*****************************************************************************
1.機能：_TRQ_Hook1()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook1( void (*func)(unsigned long TagNum,unsigned long arg1) )
{
	if( func ){
		_TRQ_Hook1 = func;
	}
	else{
		_TRQ_Hook1 = _TRQ_Hook1_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook1);

/*****************************************************************************
1.機能：_TRQ_Hook2()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook2( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2) )
{
	if( func ){
		_TRQ_Hook2 = func;
	}
	else{
		_TRQ_Hook2 = _TRQ_Hook2_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook2);

/*****************************************************************************
1.機能：_TRQ_Hook3()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook3( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3) )
{
	if( func ){
		_TRQ_Hook3 = func;
	}
	else{
		_TRQ_Hook3 = _TRQ_Hook3_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook3);

/*****************************************************************************
1.機能：_TRQ_Hook4()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook4( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4) )
{
	if( func ){
		_TRQ_Hook4 = func;
	}
	else{
		_TRQ_Hook4 = _TRQ_Hook4_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook4);

/*****************************************************************************
1.機能：_TRQ_Hook5()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook5( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5) )
{
	if( func ){
		_TRQ_Hook5 = func;
	}
	else{
		_TRQ_Hook5 = _TRQ_Hook5_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook5);

/*****************************************************************************
1.機能：_TRQ_Hook6()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook6( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6) )
{
	if( func ){
		_TRQ_Hook6 = func;
	}
	else{
		_TRQ_Hook6 = _TRQ_Hook6_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook6);

/*****************************************************************************
1.機能：_TRQ_Hook7()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook7( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7) )
{
	if( func ){
		_TRQ_Hook7 = func;
	}
	else{
		_TRQ_Hook7 = _TRQ_Hook7_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook7);

/*****************************************************************************
1.機能：_TRQ_Hook8()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook8( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8) )
{
	if( func ){
		_TRQ_Hook8 = func;
	}
	else{
		_TRQ_Hook8 = _TRQ_Hook8_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook8);

/*****************************************************************************
1.機能：_TRQ_Hook9()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook9( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9) )
{
	if( func ){
		_TRQ_Hook9 = func;
	}
	else{
		_TRQ_Hook9 = _TRQ_Hook9_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook9);

/*****************************************************************************
1.機能：_TRQ_Hook10()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook10( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10) )
{
	if( func ){
		_TRQ_Hook10 = func;
	}
	else{
		_TRQ_Hook10 = _TRQ_Hook10_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook10);

/*****************************************************************************
1.機能：_TRQ_Hook11()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook11( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11) )
{
	if( func ){
		_TRQ_Hook11 = func;
	}
	else{
		_TRQ_Hook11 = _TRQ_Hook11_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook11);

/*****************************************************************************
1.機能：_TRQ_Hook12()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook12( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12) )
{
	if( func ){
		_TRQ_Hook12 = func;
	}
	else{
		_TRQ_Hook12 = _TRQ_Hook12_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook12);

/*****************************************************************************
1.機能：_TRQ_Hook13()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook13( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13) )
{
	if( func ){
		_TRQ_Hook13 = func;
	}
	else{
		_TRQ_Hook13 = _TRQ_Hook13_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook13);

/*****************************************************************************
1.機能：_TRQ_Hook14()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook14( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14) )
{
	if( func ){
		_TRQ_Hook14 = func;
	}
	else{
		_TRQ_Hook14 = _TRQ_Hook14_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook14);

/*****************************************************************************
1.機能：_TRQ_Hook15()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook15( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15) )
{
	if( func ){
		_TRQ_Hook15 = func;
	}
	else{
		_TRQ_Hook15 = _TRQ_Hook15_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook15);

/*****************************************************************************
1.機能：_TRQ_Hook16()関数のアドレスを設定する

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	func						関数ポインタ

4.戻り値：
*****************************************************************************/
void smt_set_Hook16( void (*func)(unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16) )
{
	if( func ){
		_TRQ_Hook16 = func;
	}
	else{
		_TRQ_Hook16 = _TRQ_Hook16_nop;
	}
}
EXPORT_SYMBOL(smt_set_Hook16);


/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_Process()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	processid					プロセスID

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_Process_nop(unsigned long processid)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_ThreadProcess()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	threadid					スレッドID
I/	processid					プロセスID

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_ThreadProcess_nop(unsigned long threadid,unsigned long processid)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_Process_Name()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	processid					プロセスID
I/	str							プロセス名称(最大32文字）

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_Process_Name_nop(unsigned long processid,const char *str)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_ThreadProcess_Name()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	threadid					スレッドID
I/	processid					プロセスID
I/	tname						スレッド名称(最大32文字）
I/	pname						プロセス名称(最大32文字）

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_ThreadProcess_Name_nop(unsigned long threadid,unsigned long processid,const char *tname,const char *pname)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_Irq_in()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	irqid						割込ID

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_Irq_in_nop(unsigned long irqid)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_Irq_out()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	irqid						割込ID

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_Irq_out_nop(unsigned long irqid)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsSwitch_Idle()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsSwitch_Idle_nop( void )
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall0()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall0_nop(int osc,_SMT_OS_CALL attr)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall1()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	arg1						引数1の値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall1_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall2()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜2)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall2_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall3()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜3)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall3_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall4()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜4)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall4_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall5()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜5)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall5_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall6()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜6)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall6_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall7()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜7)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall7_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall8()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜8)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall8_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall9()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜9)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall9_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall10()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜10)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall10_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall11()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜11)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall11_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall12()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜12)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall12_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall13()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜13)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall13_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall14()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜14)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall14_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall15()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜15)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall15_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：ライブラリ未初期化時用_SMT_OsCall16()関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	osc							OSコール種別
I/	attr						発行種別(_SMT_OS_ATTR_CALL/_SMT_OS_ATTR_RET)
I/	argN(N=1〜16)				引数Nの値

4.戻り値：						_SMT_OK:正常終了
								_SMT_NG:異常終了
*****************************************************************************/
static int _SMT_OsCall16_nop(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16)
{
	return( _SMT_OK );
}

/*****************************************************************************
1.機能：関数入口HOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook0_nop(unsigned long TagNum)
{
}

/*****************************************************************************
1.機能：関数入口[引数個数=1]/出口[return値あり]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数/return値

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook1_nop(unsigned long TagNum,unsigned long arg1 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=2]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook2_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=3]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook3_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=4]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook4_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=5]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook5_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=6]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook6_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=7]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook7_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=8]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook8_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=9]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook9_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=10]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook10_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=11]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10
I/	arg11						引数11

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook11_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 ,unsigned long arg11 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=12]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10
I/	arg11						引数11
I/	arg12						引数12

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook12_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 ,unsigned long arg11 ,unsigned long arg12 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=13]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10
I/	arg11						引数11
I/	arg12						引数12
I/	arg13						引数13

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook13_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 ,unsigned long arg11 ,unsigned long arg12 ,unsigned long arg13 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=14]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10
I/	arg11						引数11
I/	arg12						引数12
I/	arg13						引数13
I/	arg14						引数14

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook14_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 ,unsigned long arg11 ,unsigned long arg12 ,unsigned long arg13 ,unsigned long arg14 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=15]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10
I/	arg11						引数11
I/	arg12						引数12
I/	arg13						引数13
I/	arg14						引数14
I/	arg15						引数15

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook15_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 ,unsigned long arg11 ,unsigned long arg12 ,unsigned long arg13 ,unsigned long arg14 ,unsigned long arg15 )
{
}

/*****************************************************************************
1.機能：
	関数入口[引数個数=16]の場合のHOOK関数

2.制限、注意：

3.入出力引数：
I/O|変数名					   |説明
---+---------------------------+----------------------------------------------
I/	TagNum						関数チェックポイント番号
I/	arg1						引数1
I/	arg2						引数2
I/	arg3						引数3
I/	arg4						引数4
I/	arg5						引数5
I/	arg6						引数6
I/	arg7						引数7
I/	arg8						引数8
I/	arg9						引数9
I/	arg10						引数10
I/	arg11						引数11
I/	arg12						引数12
I/	arg13						引数13
I/	arg14						引数14
I/	arg15						引数15
I/	arg16						引数16

4.戻り値：
*****************************************************************************/
static void _TRQ_Hook16_nop(unsigned long TagNum ,unsigned long arg1 ,unsigned long arg2 ,unsigned long arg3 ,unsigned long arg4 ,unsigned long arg5 ,unsigned long arg6 ,unsigned long arg7 ,unsigned long arg8 ,unsigned long arg9 ,unsigned long arg10 ,unsigned long arg11 ,unsigned long arg12 ,unsigned long arg13 ,unsigned long arg14 ,unsigned long arg15 ,unsigned long arg16 )
{
}
