/*****************************************************************************
						Copyright(c) 2010 YokogawaDigitalComputer Corporation
機能：システムマクロトレースカーネルランドライブラリ

注意：

変更履歴
 +-------------- 履歴番号 (000 〜 999)
 |	  +--------- 修正しているシステムバージョン
 |	  | 	+--- 新規、変更、追加、削除の分類
 v	  v 	v
 No  Ver  分類 年月日	  名前			説明
---+-----+----+----------+-------------+--------------------------------------
000 00.00 新規 2010/06/15 S.Tonoshita	新規開発
*****************************************************************************/
#ifndef __SMTAPI_H__
#define __SMTAPI_H__

//
// 関数リターン値定義
//
#define	_SMT_OK			0			// 正常終了
#define	_SMT_NG			-1			// 異常終了

/******* デバッグプリント出力レベル設定 初期値 ******/
	/* SMT_LV_MSK

	bit
	 1 1 1 1 1 1
	 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|L|L|L|L|L|L|L|L|L|L|L|L|L|L|L|L|
	|V|V|V|V|V|V|V|V|V|V|V|V|V|V|V|V|
	|1|1|1|1|1|1| | | | | | | | | | |
	|5|4|3|2|1|0|9|8|7|6|5|4|3|2|1|0|
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	LV0-15= レベル
	
	NOTE: 各bitがSMTデバッグ文のレベルに対応し、対応するレベルをマスクする
	 1でマスク、0で出力

	例1.全てのデバッグ文レベルを出力する
	#define	SMT_LV_MSK	0x0000

	例2.レベル4以上のみ出力する
	#define	SMT_LV_MSK	0x000F

	例3.レベル2とレベル8のみ出力する
	#define	SMT_LV_MSK	0xFEFB

 */
#define	SMT_LV_MSK	0x0000

/****** デバッグ関数無効化設定 ******/
/* 以下の定義を行うとデバッグ関数が無効となる
	・全デバッグ出力を無効化
	#define	_SMT_ALL_DIS
	・ポート出力関数を無効化
	#define	_SMT_PORT_DIS
	・プリント系関数を無効化
	#define	_SMT_PRINT_DIS		*/

#if !defined(_SMT_PORT_DIS) && !defined(_SMT_ALL_DIS)
	#define	_SMT_PORTOUT		_SMT_PortOut
#else	// _SMT_PORT_DIS
	#define	_SMT_PORTOUT		1 ? (void)0 : _SMT_PortOut
#endif	// _SMT_PORT_DIS

#if !defined(_SMT_PRINT_DIS) && !defined(_SMT_ALL_DIS)
	#define	_SMT_PRINTF		_SMT_printf
	#define	_SMT_PUTS		_SMT_puts
	#define	_SMT_USRMSGTAG0		_SMT_UsrMsgTag0
	#define	_SMT_USRMSGTAG1		_SMT_UsrMsgTag1
	#define	_SMT_USRMSGTAG2		_SMT_UsrMsgTag2
	#define	_SMT_USRMSGTAG3		_SMT_UsrMsgTag3
	#define	_SMT_USRMSGTAG4		_SMT_UsrMsgTag4
#else	// _SMT_PRINT_DIS
	#define	_SMT_PRINTF		1 ? (void)0 : _SMT_printf
	#define	_SMT_PUTS		1 ? (void)0 : _SMT_puts
	#define	_SMT_USRMSGTAG0		1 ? (void)0 : _SMT_UsrMsgTag0
	#define	_SMT_USRMSGTAG1		1 ? (void)0 : _SMT_UsrMsgTag1
	#define	_SMT_USRMSGTAG2		1 ? (void)0 : _SMT_UsrMsgTag2
	#define	_SMT_USRMSGTAG3		1 ? (void)0 : _SMT_UsrMsgTag3
	#define	_SMT_USRMSGTAG4		1 ? (void)0 : _SMT_UsrMsgTag4
#endif	// _SMT_PRINT_DIS

#if !defined(_SMT_OS_DIS) && !defined(_SMT_ALL_DIS)
	#define _SMT_OSSWITCH_PROCESS				_SMT_OsSwitch_Process
	#define _SMT_OSSWITCH_THREADPROCESS			_SMT_OsSwitch_ThreadProcess
	#define _SMT_OSSWITCH_PROCESS_NAME			_SMT_OsSwitch_Process_Name
	#define _SMT_OSSWITCH_THREADPROCESS_NAME	_SMT_OsSwitch_ThreadProcess_Name
	#define _SMT_OSSWITCH_IRQ_IN				_SMT_OsSwitch_Irq_in
	#define _SMT_OSSWITCH_IRQ_OUT				_SMT_OsSwitch_Irq_out
	#define _SMT_OSSWITCH_IDLE					_SMT_OsSwitch_Idle
	#define _SMT_OSCALL0						_SMT_OsCall0
	#define _SMT_OSCALL1						_SMT_OsCall1
	#define _SMT_OSCALL2						_SMT_OsCall2
	#define _SMT_OSCALL3						_SMT_OsCall3
	#define _SMT_OSCALL4						_SMT_OsCall4
	#define _SMT_OSCALL5						_SMT_OsCall5
	#define _SMT_OSCALL6						_SMT_OsCall6
	#define _SMT_OSCALL7						_SMT_OsCall7
	#define _SMT_OSCALL8						_SMT_OsCall8
	#define _SMT_OSCALL9						_SMT_OsCall9
	#define _SMT_OSCALL10						_SMT_OsCall10
	#define _SMT_OSCALL11						_SMT_OsCall11
	#define _SMT_OSCALL12						_SMT_OsCall12
	#define _SMT_OSCALL13						_SMT_OsCall13
	#define _SMT_OSCALL14						_SMT_OsCall14
	#define _SMT_OSCALL15						_SMT_OsCall15
	#define _SMT_OSCALL16						_SMT_OsCall16
#else
	#define _SMT_OSSWITCH_PROCESS				1 ? (void)0 : _SMT_OsSwitch_Process
	#define _SMT_OSSWITCH_THREADPROCESS			1 ? (void)0 : _SMT_OsSwitch_ThreadProcess
	#define _SMT_OSSWITCH_PROCESS_NAME			1 ? (void)0 : _SMT_OsSwitch_Process_Name
	#define _SMT_OSSWITCH_THREADPROCESS_NAME	1 ? (void)0 : _SMT_OsSwitch_ThreadProcess_Name
	#define _SMT_OSSWITCH_IRQ_IN				1 ? (void)0 : _SMT_OsSwitch_Irq_in
	#define _SMT_OSSWITCH_IRQ_OUT				1 ? (void)0 : _SMT_OsSwitch_Irq_out
	#define _SMT_OSSWITCH_IDLE					1 ? (void)0 : _SMT_OsSwitch_Idle
	#define _SMT_OSCALL0						1 ? (void)0 : _SMT_OsCall0
	#define _SMT_OSCALL1						1 ? (void)0 : _SMT_OsCall1
	#define _SMT_OSCALL2						1 ? (void)0 : _SMT_OsCall2
	#define _SMT_OSCALL3						1 ? (void)0 : _SMT_OsCall3
	#define _SMT_OSCALL4						1 ? (void)0 : _SMT_OsCall4
	#define _SMT_OSCALL5						1 ? (void)0 : _SMT_OsCall5
	#define _SMT_OSCALL6						1 ? (void)0 : _SMT_OsCall6
	#define _SMT_OSCALL7						1 ? (void)0 : _SMT_OsCall7
	#define _SMT_OSCALL8						1 ? (void)0 : _SMT_OsCall8
	#define _SMT_OSCALL9						1 ? (void)0 : _SMT_OsCall9
	#define _SMT_OSCALL10						1 ? (void)0 : _SMT_OsCall10
	#define _SMT_OSCALL11						1 ? (void)0 : _SMT_OsCall11
	#define _SMT_OSCALL12						1 ? (void)0 : _SMT_OsCall12
	#define _SMT_OSCALL13						1 ? (void)0 : _SMT_OsCall13
	#define _SMT_OSCALL14						1 ? (void)0 : _SMT_OsCall14
	#define _SMT_OSCALL15						1 ? (void)0 : _SMT_OsCall15
	#define _SMT_OSCALL16						1 ? (void)0 : _SMT_OsCall16
#endif


//
//	ポートサイズ設定値
//
typedef enum {
	_SMT_PSZ64 = 0,				// 64Bit幅（未使用）
	_SMT_PSZ8,					//  8Bit幅
	_SMT_PSZ16,					// 16Bit幅
	_SMT_PSZ32					// 32Bit幅
} _SMT_PSZ ;

//
//	アクセス設定値
//
typedef enum{
	_SMT_PW = 0,				// Writeアクセス
	_SMT_PR						// Readアクセス
} _SMT_PRW;


typedef enum {
	_SMT_OS_ATTR_CALL = 0,
	_SMT_OS_ATTR_RET
} _SMT_OS_CALL ;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
	int	 _SMT_GetDebugLevel( void );
	void _SMT_SetDebugLevel( int level );
	void _SMT_IntrruptInit(void);
	void _SMT_IntrruptEnable(void);
	void _SMT_IntrruptDisable(void);
#ifdef __cplusplus
}
#endif // __cplusplus

/* 関数プロトタイプ宣言 *****************************************************/
extern int (*_SMT_PortOut)(unsigned long addr,unsigned long data,_SMT_PSZ size,_SMT_PRW rw);
extern int (*_SMT_Printf)(int level,const char *format , ...);
extern int	(*_SMT_Puts)(int level,const char *s);
extern int	(*_SMT_UsrMsgTag0)(int level,unsigned long TagNum);
extern int	(*_SMT_UsrMsgTag1)(int level,unsigned long TagNum,unsigned long arg1);
extern int	(*_SMT_UsrMsgTag2)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2);
extern int	(*_SMT_UsrMsgTag3)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3);
extern int	(*_SMT_UsrMsgTag4)(int level,unsigned long TagNum,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4);
extern int (*_SMT_OsSwitch_Process)(unsigned long processid);
extern int (*_SMT_OsSwitch_ThreadProcess)(unsigned long threadid,unsigned long processid);
extern int (*_SMT_OsSwitch_Process_Name)(unsigned long processid,const char *str);
extern int (*_SMT_OsSwitch_ThreadProcess_Name)(unsigned long threadid,unsigned long processid,const char *tname,const char *pname);
extern int (*_SMT_OsSwitch_Irq_in)(unsigned long irqid);
extern int (*_SMT_OsSwitch_Irq_out)(unsigned long irqid);
extern int (*_SMT_OsSwitch_Idle)( void );
extern int (*_SMT_OsCall0)(int osc,_SMT_OS_CALL attr);
extern int (*_SMT_OsCall1)(int osc,_SMT_OS_CALL attr,unsigned long arg1);
extern int (*_SMT_OsCall2)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2);
extern int (*_SMT_OsCall3)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3);
extern int (*_SMT_OsCall4)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4);
extern int (*_SMT_OsCall5)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5);
extern int (*_SMT_OsCall6)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6);
extern int (*_SMT_OsCall7)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7);
extern int (*_SMT_OsCall8)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8);
extern int (*_SMT_OsCall9)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9);
extern int (*_SMT_OsCall10)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10);
extern int (*_SMT_OsCall11)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11);
extern int (*_SMT_OsCall12)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12);
extern int (*_SMT_OsCall13)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13);
extern int (*_SMT_OsCall14)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14);
extern int (*_SMT_OsCall15)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15);
extern int (*_SMT_OsCall16)(int osc,_SMT_OS_CALL attr,unsigned long arg1,unsigned long arg2,unsigned long arg3,unsigned long arg4,unsigned long arg5,unsigned long arg6,unsigned long arg7,unsigned long arg8,unsigned long arg9,unsigned long arg10,unsigned long arg11,unsigned long arg12,unsigned long arg13,unsigned long arg14,unsigned long arg15,unsigned long arg16);


#endif /* __SMTAPI_H__ */
