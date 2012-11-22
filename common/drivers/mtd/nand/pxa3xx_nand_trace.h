#if !defined(_PXA3XX_NAND_H) || defined(TRACE_HEADER_MULTI_READ)
#define _PXA3XX_NAND_H
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>
#undef TRACE_SYSTEM
#define TRACE_SYSTEM pxa3xx_nand
TRACE_EVENT(pxa3xx_nand_stat,

		TP_PROTO(int bar),

		TP_ARGS(bar),

		TP_STRUCT__entry(
			__field(int, bar)
			),

		TP_fast_assign(
			__entry->bar    = bar;
			),

		TP_printk("foo %x", __entry->bar)
	   );
#endif

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH ../../drivers/mtd/nand/
/*
 * TRACE_INCLUDE_FILE is not needed if the filename and TRACE_SYSTEM are equal
 */
#define TRACE_INCLUDE_FILE pxa3xx_nand_trace
#include <trace/define_trace.h>

