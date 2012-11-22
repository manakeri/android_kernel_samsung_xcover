/*
 * Arm specific backtracing code for MPDC
 *
 * Copyright 2007 Marvell Ltd.
 *
 * Arm oprofile backtrace code by Richard Purdie is referenced.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <asm/ptrace.h>
#include <asm/uaccess.h>

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30))
#include <asm/stacktrace.h>
#endif

#include "PXD_css.h"
#include "ring_buffer.h"
#include "css_dsa.h"

#define N 5

static pid_t g_current_pid;
static u32   g_current_pc;

const int WRITE_CALL_STACK_ERR = -1;

/* prologue instruction pattern */
#define PROLOG_INSTR1 0xE92DD800   /* stmdb sp!, {fp, ip, lr, pc} */
#define PROLOG_INSTR1_MASK 0xFFFFF800
#define PROLOG_INSTR2 0xE24CB004  /* sub fp, ip, #4*/

#define GET_USERREG() ((struct pt_regs *)(THREAD_START_SP + (unsigned long)current_thread_info()) - 1)

#define SWI_INSTR(inst)                       ((inst & 0x0f000000) == 0x0f000000)
#define BL_INSTR(inst)                        ((inst & 0x0f000000) == 0x0b000000)              // BL
#define BLX_INSTR(inst)                       ((inst & 0x0ffffff0) == 0x012fff30)              // BLX
#define BX_LR_INSTR(inst)                     ((inst & 0x0fffffff) == 0x012fff1e)              // BX(cond) LR 
#define ADD_SP_SP_immed_INSTR(inst)           ((inst & 0xfffff000) == 0xe28dd000)              // ADD SP, SP, #immed
#define SUB_SP_SP_immed_INSTR(inst)           ((inst & 0xfffff000) == 0xe24dd000)              // SUB SP, SP, #immed
#define ADD_SP_FP_immed_INSTR(inst)           ((inst & 0xfffff000) == 0xe28bd000)              // ADD SP, FP, #immed
#define SUB_SP_FP_immed_INSTR(inst)           ((inst & 0xfffff000) == 0xe24bd000)              // SUB SP, FP, #immed
#define LDMIA_SP_WITH_LR_INSTR(inst)          ((inst & 0xffff4000) == 0xe8bd4000)              // LDMIA sp!, {Rx, ... lr}
#define LDMIA_SP_WITH_PC_INSTR(inst)          ((inst & 0xffff8000) == 0xe89d8000)              // LDMIA sp!, {Rx, ... pc}
#define LDMIA_SP_WITHOUT_LR_PC_INSTR(inst)    ((inst & 0xffffc000) == 0xe8bd0000)              // LDMIA sp!, {Rx, ... Ry} (without pc and lr)
#define LDMIA_1_SPBASE_INSTR(inst)            ((inst & 0xffdf0000) == 0xe89d0000)              // LDMIA sp{!}, {Rx, ... Ry}
#define STMDB_1_SPBASE_INSTR(inst)            ((inst & 0xffff0000) == 0xe92d0000)              // STMDB sp!, {Rx, ... Ry}
#define LDR_LR_SPBASE_immed_INSTR(inst)       ((inst & 0xfffff000) == 0xe59de000)              // LDR lr, [sp, #immed]
#define LDR_FP_SPBASE_immed_INSTR(inst)       ((inst & 0xfffff000) == 0xe59db000)              // LDR fp, [sp, #immed]
#define LDR_LR_SPBASE_immed_pre_INSTR(inst)   ((inst & 0xfffff000) == 0xe5bde000)              // LDR lr, [sp, #immed]!
#define LDR_LR_SPBASE_immed_post_INSTR(inst)  ((inst & 0xfffff000) == 0xe49de000)              // LDR lr, [sp], #immed
#define LDR_FP_SPBASE_immed_pre_INSTR(inst)   ((inst & 0xfffff000) == 0xe5bdb000)              // LDR fp, [sp, #immed]!
#define LDR_FP_SPBASE_immed_post_INSTR(inst)  ((inst & 0xfffff000) == 0xe49db000)              // LDR fp, [sp], #immed
#define MOV_PC_LR(inst)                       (inst == 0xe1a0f00e)                             // MOV pc, lr

#define SWI_INSTR_THUMB(inst)                 ((inst & 0xff00) == 0xdf00)                      // THUMB: swi
#define BL_INSTR_THUMB(inst1, inst2)          (((inst1 & 0xf800) == 0xf000) && ((inst2 & 0xf800) == 0xf800))              // THUMB: BL <addr>
#define BLX_INSTR_THUMB(inst1, inst2)         (((inst1 & 0xf800) == 0xf000) && ((inst2 & 0xf800) == 0xe800))              // THUMB: BLX <addr>
#define BLX_Rm_INSTR_THUMB(inst)              ((inst & 0xff87) == 0x4780)                      // THUMB: BLX Rm

#define ADD_SP_immed_THUMB_INSTR(inst)        ((inst & 0xff80) == 0xb000)                      // THUMB: ADD SP, #immed
#define SUB_SP_immed_THUMB_INSTR(inst)        ((inst & 0xff80) == 0xb080)                      // THUMB: SUB SP, #immed
#define POP_THUMB_INSTR(inst)                 ((inst & 0xfe00) == 0xbc00)                      // THUMB: POP {Rx, ..., Ry}
#define PUSH_THUMB_INSTR(inst)                ((inst & 0xfe00) == 0xb400)                      // THUMB: PUSH {Rx, ..., Ry}
#define BX_LR_THUMB_INSTR(inst)               ((inst & 0xffff) == 0x4770)                      // THUMB: BX LR
#define BX_THUMB_INSTR(inst)                  ((inst & 0xffc7) == 0x4700)                      // THUMB: BX Rm
#define POP_WITH_PC_THUMB_INSTR(inst)         ((inst & 0xff00) == 0xbd00)                      // THUMB: POP {Rx, ..., pc} 
#define MOV_PC_Rm_THUMB_INSTR(inst)           ((inst & 0xff87) == 0x4687)                      // THUMB: MOV PC, Rm


bool ldm_stm_inst_update_base_reg(u32 inst)
{
	return (inst & 0x00200000);
}

extern struct RingBufferInfo g_sample_buffer;
extern wait_queue_head_t pxcss_kd_wait;
extern unsigned int timer_int_count;
extern unsigned int g_timer_count;
extern struct px_css_dsa *g_dsa;

struct unwind_regs
{
	unsigned int pc;
	unsigned int sp;
	unsigned int fp;
	unsigned int lr;
} __attribute__((packed));

/*
 * The registers we're interested in are at the end of the variable
 * length saved register structure. The fp points at the end of this
 * structure so the address of this struct is:
 * (struct frame_tail_1 *)(xxx->fp)-1
 */
struct frame_tail_1 {
	unsigned long fp;
	unsigned long sp;
	unsigned long lr;
} __attribute__((packed));

struct frame_tail_2 {
	unsigned long fp;
	unsigned long lr;
} __attribute__((packed));

/* metrics support */
struct scg_metrics_struct {
	unsigned int collect_user_samples;
	unsigned int discard_user_samples;
	unsigned int collect_kernel_samples;
	unsigned int discard_kernel_samples;
};

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30))
struct stackframe {
	unsigned long fp;
	unsigned long sp;
	unsigned long lr;
	unsigned long pc;
};
#endif

static unsigned int arm_inst_size(void)
{
	return sizeof(u32);
}

static unsigned int thumb_inst_size(void)
{
	return sizeof(u16);
}

static struct scg_metrics_struct scg_metrics;

void scg_init_metrics(void)
{
	memset(&scg_metrics, 0, sizeof(struct scg_metrics_struct));
}

void scg_print_metrics(void)
{
	printk("collected user samples:%d, discarded user samples:%d\n", 
			scg_metrics.collect_user_samples, scg_metrics.discard_user_samples);

	printk("collected kernel samples:%d, discarded kernel samples:%d\n", 
			scg_metrics.collect_kernel_samples, scg_metrics.discard_kernel_samples);
}

static inline int valid_kernel_stack(unsigned long fp, struct pt_regs *regs)
{
	unsigned long stack = (unsigned long)regs;
	unsigned long stack_base = (stack & ~(THREAD_SIZE - 1)) + THREAD_SIZE;

	return (fp > stack) && (fp < stack_base) && (fp > regs->ARM_sp);
}

static inline int valid_user_stack(struct frame_tail_1 *tail, struct pt_regs *regs)
{ 
	/* check accessibility of one struct frame_tail_1 beyond. 
	 *  Normally tail should be greater than sp. But for the first stack frame, 
	 *  there is possibility that they are equal.
	 */
	return tail 
		&& !((unsigned long) tail & 3) 
		&& ((unsigned long) tail >= regs->ARM_sp)
		&& access_ok(VERIFY_READ, tail, sizeof(struct frame_tail_1) * 2);
}


/* Check if the interruptted instruction is in the prologue of a function 
 * where call frame has not been pushed on to the stack. 
 */
static inline int in_prologue(u32 instrs[2])
{
	return (   ((instrs[0] & PROLOG_INSTR1_MASK) == PROLOG_INSTR1)
	        || ((instrs[1] & PROLOG_INSTR1_MASK) == PROLOG_INSTR1)
			|| (instrs[0] == PROLOG_INSTR2));
}


static bool need_flush;

static void roll_back_buffer_to_offset(unsigned int offset)
{
	g_sample_buffer.buffer.write_offset = offset;
}

/* add css data to buffer, return true if the data has been successfully added to the buffer */
static bool add_css_data(unsigned int pc, unsigned int pid)
{
	PXD32_CSS_Call cs;
	bool buffer_full;
//	bool need_flush;
	
	cs.address = pc;
	cs.pid     = pid;
	
	//return true;

	write_ring_buffer(&g_sample_buffer.buffer, &cs, sizeof(cs), &buffer_full, &need_flush);

	
	if (buffer_full)
	{
		/* buffer is full, roll back the current call stack data */
		printk("buffer is full 1\n");
		return false;
	}
	
	return true;
}

static bool write_css_data_head(unsigned int reg_id, pid_t pid, pid_t tid)
{
//	bool need_flush;
	bool buffer_full;
	
	PXD32_CSS_Call_Stack data;


	//return true;
	data.registerId = reg_id;
	data.pid        = pid;
	data.tid        = tid;
	data.depth      = 0;
	data.cs[0].address = 0;
	data.cs[0].pid     = 0;
	
	write_ring_buffer(&g_sample_buffer.buffer, &data, offsetof(PXD32_CSS_Call_Stack, cs), &buffer_full, &need_flush);

	if (buffer_full)
	{
		/* buffer is full, roll back the current call stack data */
		printk("buffer is full 2\n");
		
		scg_metrics.discard_kernel_samples++;
		return false;
	}

	return true;
}

static void update_css_data_depth(unsigned int depth_offset, unsigned int depth)
{
	update_ring_buffer(&g_sample_buffer.buffer, depth_offset, &depth, sizeof(depth));
}

static void check_buffer_flush(void)
{
	if (need_flush && !g_sample_buffer.is_full_event_set)
	{
		g_sample_buffer.is_full_event_set = true;

		if (waitqueue_active(&pxcss_kd_wait))
			wake_up_interruptible(&pxcss_kd_wait);
	}	
}

struct report_trace_data
{
	unsigned int depth;
	pid_t        pid;
	bool         write_success;
};

static int kernel_report_trace(struct stackframe *frame, void *d)
{
	struct report_trace_data *data = (struct report_trace_data *)d;

	if (data != NULL)
	{
		if (!add_css_data(frame->pc - arm_inst_size(), data->pid))
		{
			++scg_metrics.discard_kernel_samples;

			data->write_success = false;
			return -1;
		}
		else
		{
			data->write_success = true;
		}

		(data->depth)++;
	}
	
	return 0;
}

int unwind_frame(struct stackframe *frame)
{
	unsigned long high, low;
	unsigned long fp = frame->fp;

	/* only go to a higher address on the stack */
	low = frame->sp;
	high = ALIGN(low, THREAD_SIZE) + THREAD_SIZE;

	/* check current frame pointer is within bounds */
	if (fp < (low + 12) || fp + 4 >= high)
		return -EINVAL;

	/* restore the registers from the stack frame */
	frame->fp = *(unsigned long *)(fp - 12);
	frame->sp = *(unsigned long *)(fp - 8);
	frame->pc = *(unsigned long *)(fp - 4);

	return 0;
}

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30))
static void walk_stackframe(struct stackframe *frame, int (*fn)(struct stackframe *, void *), void *data)
{
	while (1)
	{
		int ret;

		if (fn(frame, data))
			break;

		ret = unwind_frame(frame);
		if (ret < 0)
			break;
	}
}
#endif

#if 0
int walk_stackframe(unsigned long fp, unsigned long low, unsigned long high,
		int (*fn)(struct stackframe *, void *), void *data);
#endif

#if 0
static struct frame_tail_1* user_backtrace(struct frame_tail_1 *tail)
{
	unsigned long t;
	struct frame_tail_1 buftail[2];

	/* Also check accessibility of one struct frame_tail_1 beyond */
	if (!access_ok(VERIFY_READ, tail, sizeof(buftail)))
	{
		return NULL;
	}

	if (__copy_from_user_inatomic(buftail, tail, sizeof(buftail)))
	{
		return NULL;
	}

	if (g_dsa->sample_count < N)
		printk("0x%x ", buftail[0].lr - 4);
	//add_css_data(buftail[0].lr - 4, current->pid);
	
	/* frame pointers should strictly progress back up the stack
	 * (towards higher addresses) */
	if (tail >= buftail[0].fp)
	{
		return NULL;
	}

	return buftail[0].fp-1;
}
#endif

unsigned int kernel_backtrace(struct pt_regs * const regs, pid_t pid, pid_t tid)
{
	unsigned int depth = 0;
#if 1

#if 1
	struct report_trace_data data;
	struct stackframe frame;
	
	data.depth = depth;
	data.pid   = pid;

	frame.fp = regs->ARM_fp;
	frame.sp = regs->ARM_sp;
	frame.lr = regs->ARM_lr;
	frame.pc = regs->ARM_pc;
	
	g_current_pc = regs->ARM_pc;

	if (in_prologue((u32 *)(regs->ARM_pc)))
	{
		frame.pc = (regs->ARM_lr) - 4;
	}

	walk_stackframe(&frame, kernel_report_trace, &data);

	if (!data.write_success)
	{
		return WRITE_CALL_STACK_ERR;
	}
	
	depth = data.depth;
#else
//	unsigned long base = ((unsigned long)regs) & ~(THREAD_SIZE - 1);
//	walk_stackframe(regs->ARM_fp, base, base + THREAD_SIZE, report_trace, &depth);
	unsigned long low = regs->ARM_sp;
	unsigned long high = ALIGN(low, THREAD_SIZE) + THREAD_SIZE;

	walk_stackframe(regs->ARM_fp, low, high, kernel_report_trace, &depth);

#endif
#else
#ifdef CONFIG_FRAME_POINTER
	u32 pc;
	u32 fp;
	u32 prev_fp;

	pc = regs->ARM_pc;
	fp = regs->ARM_fp;
	prev_fp = fp - 1;
	do
	{
		if (!add_css_data(pc, pid))
		{
			++scg_metrics.discard_kernel_samples;
			return depth;
		}
		++depth;

		/* check if in prologue */
		if (depth == 1 && in_prologue((u32 *)pc))
		{
			pc = regs->ARM_lr - 4;
			continue;
		}

		/* backtrace to upper frame 
		 * Note: frame pointers should strictly progress back up the stack
		 * (towards higher addresses) 
		 * */
		if (fp <= prev_fp || !valid_kernel_stack(fp, regs))
			break;

		tail = ((struct frame_tail_1 *) fp) - 1;
		pc = tail->lr - 4;
		if (pc == 0)
			break;
		prev_fp = fp;
		fp = tail->fp;
	} while (1);
#endif
#endif
	return depth;
}

static bool read_data_from_addr(u32 address, void * data, unsigned int size)
{
	if (!access_ok(VERIFY_READ, address, size))
	{
		return false;
	}

	if (__copy_from_user_inatomic(data, (void *)address, size))
	{
		return false;
	}

	return true;
}

static bool read_instruction(u32 address, void *insts, unsigned int inst_num, bool is_thumb)
{
	unsigned int size;

	if (!is_thumb)
	{
		size = inst_num * arm_inst_size();
	}
	else
	{
		size = inst_num * thumb_inst_size();
	}

	return read_data_from_addr(address, insts, size);
}

static unsigned int bit_count(unsigned int n)
{
	int i;
	unsigned sum = 0;

	for (i=0; i < sizeof(n) * 8; i++)
	{
		sum += (n >> i) & 0x1;
	}

	return sum;
}

static unsigned int rotr(unsigned int value, int shift, int value_bit_num)
{
	return ((value >> shift) | (value << (value_bit_num - shift)));
}

static unsigned int get_immed_operand_for_data_proc_instr(unsigned int instr)
{
	u8 immed_8, rotate_imm;
	
	immed_8 = instr & 0xff;
	rotate_imm = instr & 0xf00;
	
	return rotr(immed_8, 2 * rotate_imm, 8);
}

static bool is_swi_instruction_arm(u32 inst)
{
	if (SWI_INSTR(inst))
		return true;

	return false;
}

static bool is_swi_instruction_thumb(u32 inst)
{
	if (SWI_INSTR_THUMB(inst))
		return true;

	return false;
}

static bool is_branch_instruction_arm(u32 inst)
{
	if (BL_INSTR(inst) || BLX_INSTR(inst))
		return true;
	
	return false;
}

static bool is_branch_instruction_thumb(u16 inst1, u16 inst2)
{
	if (BL_INSTR_THUMB(inst1, inst2) || BLX_INSTR_THUMB(inst1, inst2))
		return true;
	
	if (BLX_Rm_INSTR_THUMB(inst1))
		return true;

	return false;
}

static bool is_branch_instruction(u32 pc)
{
	u32 inst;

	if (pc & 0x1)
	{
		u16 inst1, inst2;

		pc = pc & ~0x1;

		if (!read_instruction(pc, &inst, 2, true))
			return false;

		inst1 = inst & 0xffff;
		inst2 = (inst & 0xffff0000) >> 16;

		if (is_branch_instruction_thumb(inst1, inst2))
			return true;
		else
			return false;
	}
	else
	{
		if (!read_instruction(pc, &inst, 1, false))
			return false;

		if (is_branch_instruction_arm(inst))
			return true;
		else
			return false;
	}
	
}

static bool is_swi_instruction(u32 pc)
{
	u32 inst;

	if (pc & 0x1)
	{
		pc = pc & ~0x1;

		if (!read_instruction(pc, &inst, 1, true))
			return false;

		if (is_swi_instruction_thumb(inst))
			return true;
		else
			return false;
	}
	else
	{
		if (!read_instruction(pc, &inst, 1, false))
			return false;

		if (is_swi_instruction_arm(inst))
			return true;
		else
			return false;
	}
	
}

static bool thumb_unwind(struct unwind_regs *frame, bool *sp_modified, unsigned int depth)
{
	u32 regs[16];
	
	u32 pc, sp, fp, lr;
	u32 cur;
	u32 pc1, sp1;
	
	u16 inst1, inst2;
	int count;
	
	pc1 = frame->pc;
	sp1 = frame->sp;

	pc = frame->pc;
	sp = frame->sp;
	fp = frame->fp;
	lr = frame->lr;
	
	count = 0;

	*sp_modified = false;

	if (!read_instruction(pc, &inst1, 1, false))
	{
		return false;
	}
	
	if (!read_instruction(pc + thumb_inst_size(), &inst2, 1, false))
	{
		return false;
	}

	if (BL_INSTR_THUMB(inst1, inst2) || BLX_INSTR_THUMB(inst1, inst2))
	{
		cur = pc + arm_inst_size();	
	}
	else
	{
		cur = pc + thumb_inst_size();
	}

	cur = pc;

	while (true)
	{
		if (!read_instruction(cur, &inst1, 1, true))
		{
			return false;
		}

		if (count++ >= 0xffff)
		{
			return false;
		}

		if (ADD_SP_immed_THUMB_INSTR(inst1))                 // ADD SP, #immed
		{
			u8 immed;
			
			immed = inst1 & 0x7f;
			
			sp += (immed << 2);
			
			*sp_modified = true;
		}
		else if (SUB_SP_immed_THUMB_INSTR(inst1))            // SUB SP, #immed
		{
			u8 immed;
			
			immed = inst1 & 0x7f;
			
			sp -= (immed << 2);
			
			*sp_modified = true;
		}
		else if (POP_THUMB_INSTR(inst1))                     // POP {Rx, ..., Ry}
		{
			int i;
			u32 data[5];
			
			read_data_from_addr(sp - 8, data, 5 * sizeof(u32));

			for (i=0; i<8; i++)
			{
				if ((inst1 & 0xff) & (0x1 << i))
				{
					if (!read_data_from_addr(sp, &regs[i], sizeof(u32)))
						return false;

					sp += 4;
					
					*sp_modified = true;
				}
			}

			if (inst1 & 0x0100)
			{
				if (!read_data_from_addr(sp, &pc, sizeof(u32)))
				{
					return false;
				}
				
				sp += 4;
				
				*sp_modified = true;

				break;
			}
		}
		else if (PUSH_THUMB_INSTR(inst1))                     // PUSH {Rx, ..., Ry}
		{
			int i;
			int n = 0;
			
			for (i=0; i<=8; i++)
			{
				if ((inst1 & 0x1ff) & (0x1 << i))
				{
					sp -= 4;
					
					n++;
					
					*sp_modified = true;
				}
			}
		}
		else if (BX_LR_THUMB_INSTR(inst1))                    // BX LR
		{
			pc = lr;

			break;
		}
		else if (MOV_PC_Rm_THUMB_INSTR(inst1))                // mov pc, Rx
		{
			int reg_num;
			
			reg_num = (inst1 & 0x78) >> 3;

			pc = regs[reg_num];

			break;
		}
		else if (BX_THUMB_INSTR(inst1))                       // BX Rm
		{
			int reg_num;
			
			reg_num = (inst1 & 0x78) >> 3;
			
			pc = regs[reg_num];
			
			break;
		}

		cur += thumb_inst_size();
	}	

	frame->pc = pc;
	frame->sp = sp;
	frame->fp = fp;
	
	return true;
}

static bool arm_unwind_2(struct unwind_regs *frame, bool *sp_modified, unsigned int depth)
{
	u32 fp, sp, pc, lr;
	u32 cur;
	int count = 0;
	
	u32 pc1;
	
	pc = frame->pc;
	sp = frame->sp;
	fp = frame->fp;
	lr = frame->lr;
	
	pc1 = pc;

	//cur = pc + arm_inst_size();
	cur = pc;

	*sp_modified = false;

	while (true)
	{
		u32 instr;

		if (!read_instruction(cur, &instr, 1, false))
		{
			return false;
		}

		if (count++ >= 0xffff)
		{
			return false;
		}

		if (ADD_SP_SP_immed_INSTR(instr))               // ADD SP, SP #immed
		{
			u8 shifter_operand;
			
			shifter_operand = get_immed_operand_for_data_proc_instr(instr);
			
			sp += shifter_operand;
			
			*sp_modified = true;
		}
		else if (LDMIA_1_SPBASE_INSTR(instr))                 // LDMIA SP{!}, {Rx, ..., Ry}
		{
			int i;
			u32 regs[16];
			bool update_base_reg;
			u32 address;
			
			memset(regs, 0, sizeof(regs));

			update_base_reg = ldm_stm_inst_update_base_reg(instr);

			address = sp;
			
			for (i=0; i<=15; i++)
			{
				if (instr & (1 << i))
				{
					if (!read_data_from_addr(address, &regs[i], arm_inst_size()))
					{
						return false;
					}
					
					address += arm_inst_size();
				}
			}
			
			if (update_base_reg)                // SP is base register and will be updated
			{
				sp += bit_count(instr & 0xffff) * arm_inst_size();
				*sp_modified = true;
			}

			if (instr & (1 << 11))
			{
				fp = regs[11];
			}
			
			if (instr & (1 << 13))
			{
				sp = regs[13];
				*sp_modified = true;
			}
			
			if (instr & (1 << 14))
			{
				lr = regs[14];
			}

			if (instr & (1 << 15))
			{
				pc = regs[15];
				break;
			}
		}
		else if (SUB_SP_SP_immed_INSTR(instr))           // SUB   SP, SP, #immed
		{
			u8 shifter_operand;
			
			shifter_operand = get_immed_operand_for_data_proc_instr(instr);
			
			sp -= shifter_operand;
			
			*sp_modified = true;
			
			//pc = frame->lr;
		}
		else if (STMDB_1_SPBASE_INSTR(instr))                  // STMDB SP!, {Rx, ..., Ry}
		{
			sp -= bit_count(instr & 0xffff) * arm_inst_size();
			*sp_modified = true;
			//pc = frame->lr;
		} 
		else if (SUB_SP_FP_immed_INSTR(instr))           // SUB SP, FP, #immed
		{
			u8 shifter_operand;
			
			shifter_operand = get_immed_operand_for_data_proc_instr(instr);
			
			sp = fp - shifter_operand;
			*sp_modified = true;
		}
		else if (ADD_SP_FP_immed_INSTR(instr))           // ADD SP, FP, #immed
		{
			u8 shifter_operand;
			
			shifter_operand = get_immed_operand_for_data_proc_instr(instr);
			
			sp = fp + shifter_operand;
			*sp_modified = true;
		}
		else if (LDR_LR_SPBASE_immed_INSTR(instr))       // LDR lr, [sp, #immed]
		{
			u16 immed;
			
			immed = instr & 0xfff;
			
			if (!read_data_from_addr(sp + immed, &lr, arm_inst_size()))
			{
				return false;
			}
			
		}
		else if (LDR_FP_SPBASE_immed_INSTR(instr))       // LDR fp, [sp, #immed]
		{
			u16 immed;
			
			immed = instr & 0xfff;
			
			if (!read_data_from_addr(sp + immed, &fp, arm_inst_size()))
			{
				return false;
			}
		}
		else if (LDR_LR_SPBASE_immed_pre_INSTR(instr))  // LDR lr, [sp, #immed]!
		{
			u16 immed;
			
			immed = instr & 0xfff;
			
			sp += immed;
			*sp_modified = true;

			if (!read_data_from_addr(sp, &lr, arm_inst_size()))
			{
				return false;
			}
		}
		else if (LDR_FP_SPBASE_immed_pre_INSTR(instr))  // LDR fp, [sp, #immed]!
		{
			u16 immed;
			
			immed = instr & 0xfff;
			
			sp += immed;
			*sp_modified = true;

			if (!read_data_from_addr(sp, &fp, arm_inst_size()))
			{
				return false;
			}
		}
		else if (LDR_LR_SPBASE_immed_post_INSTR(instr))  // LDR lr, [sp], #immed
		{
			u16 immed;
			
			immed = instr & 0xfff;
			
			if (!read_data_from_addr(sp, &lr, arm_inst_size()))
			{
				return false;
			}
			
			sp += immed;
			*sp_modified = true;
		}
		else if (LDR_FP_SPBASE_immed_post_INSTR(instr))  // LDR fp, [sp], #immed
		{
			u16 immed;
			
			immed = instr & 0xfff;
			
			if (!read_data_from_addr(sp, &fp, arm_inst_size()))
			{
				return false;
			}
			
			sp += immed;
			*sp_modified = true;
		}
		else if (BX_LR_INSTR(instr))                     // BX lr 
		{
			pc = lr;
			
			break;
		}
		else if (MOV_PC_LR(instr))                       // MOV  PC, LR
		{
			pc = lr;
			break;
		}
		
		cur += arm_inst_size();
	}

	frame->pc = pc;
	frame->sp = sp;
	frame->fp = fp;
	
	return true;	
}
#if 0
static bool arm_unwind_1(struct unwind_regs *frame, bool *sp_modified, unsigned int depth)
{
	u32 pc;
	u32 fp;
	u32 sp;
	u32 cur;
	u32 lr;
	u32 pc1;

	int mode;

	pc = frame->pc;
	sp = frame->sp;
	fp = frame->fp;
	lr = frame->lr;
	
	pc1 = pc;
	
	cur = pc + arm_inst_size();

	while (true)
	{
		u32 instrs[4];

		if (!read_instruction(cur, instrs, 4, false))
		{
			return false;
		}
		
		if (   ADD_SP_SP_immed_INSTR(instrs[0])          // add     sp, sp, #xxx
		    && (instrs[1] == 0xe49de004)                 // ldr     lr, [sp], #4
		    && BX_LR_INSTR(instrs[2]))                   // bx      lr
		{
			u8 shifter_operand;
			
			mode = 1;
			
			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[0]);
			
			if (!read_data_from_addr(sp + shifter_operand, &pc, arm_inst_size()))
			{
				return false;
			}

			sp += shifter_operand;
			sp += 4;
			
			*sp_modified = true;

			break;
		}
		else if (   ADD_SP_SP_immed_INSTR(instrs[0])          // add     sp, sp, #xxx
		         && LDMIA_SP_WITH_PC_INSTR(instrs[1]))        // ldmia   sp!, {Rx, ..., pc}
		{
			u8 shifter_operand;
			
			mode = 2;
			
			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[0]);
			
			sp += shifter_operand;
			sp += bit_count(instrs[1] & 0xffff) * arm_inst_size();
			
			*sp_modified = true;

			if (!read_data_from_addr(sp - arm_inst_size(), &pc, arm_inst_size()))
			{
				return false;
			}

			if (instrs[1] & 0x0800)     // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xf800), &fp, sizeof(u32)))
				{
					return false;
				}
			}

			break;
		}
		else if (     LDMIA_SP_WITHOUT_LR_PC_INSTR(instrs[0])     // ldmia sp!, {Rx, ..., Ry}
			  &&  LDMIA_SP_WITH_LR_INSTR(instrs[1])           // ldmia sp!, {Rx, ..., lr}
			  &&  BX_LR_INSTR(instrs[2]))                     // bx    lr
		{
			mode = 3;
			sp += bit_count(instrs[0] & 0xffff) * arm_inst_size();
			
			*sp_modified = true;
			
			if (instrs[0] & 0x0800)     // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[0] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			sp += bit_count(instrs[1] & 0xffff) * arm_inst_size();

			if (instrs[1] & 0x0800)     // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[0] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			if (!read_data_from_addr(sp - arm_inst_size(), &pc, arm_inst_size()))
			{
				return false;
			}

			break;
		}
#if 1
		else if (LDMIA_SP_WITH_PC_INSTR(instrs[0]))      // ldmia sp!, {Rx, ..., pc}
		{
			mode = 4;
			sp += bit_count(instrs[0] & 0xffff) * arm_inst_size();

			*sp_modified = true;

			if (!read_data_from_addr(sp - arm_inst_size(), &pc, arm_inst_size()))
			{
				return false;
			}

			if (instrs[0] & 0x0800)       // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[0] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			break;
		}
		else if (   (instrs[0] == 0xe24bd00c)              // sub     sp, fp, #12
			 && (instrs[1] == 0xe89da800))             // ldmia   sp, {fp, sp, pc}
		{
			struct frame_tail_1 tail;

			mode = 5;
			sp = fp - 12;
			
			*sp_modified = true;

			if (!read_data_from_addr(sp, &tail, sizeof(struct frame_tail_1)))
			{
				return false;
			}
			//pc = tail.lr - arm_inst_size();
			pc = tail.lr;
			fp = tail.fp;
			sp = tail.sp;

			break;
		}
		else if (    SUB_SP_FP_immed_INSTR(instrs[0])           // sub     sp, fp, #immed
			  && ((instrs[1] & 0xffff0800) == 0xe8bd0800)   // ldmia   sp!, {Rx, ..., Ry, fp, ...}
			  && BX_LR_INSTR(instrs[2]))                    // bx      lr
		{
			u8 shifter_operand;
			
			mode = 6;
			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[0]);
			
			sp = fp - shifter_operand;
			
			sp += bit_count(instrs[1] & 0xffff) * arm_inst_size();

			if (instrs[1] & 0x0800)       // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			if (instrs[1] & 0x4000)       // if lr is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xc000), &lr, arm_inst_size()))
				{
					return false;
				}
			}

			if (instrs[1] & 0x2000)       // if sp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xe000), &sp, arm_inst_size()))
				{
					return false;
				}
			}

			*sp_modified = true;

			//pc = lr - arm_inst_size();
			pc = lr;

			break;
		}
		else if (    ADD_SP_FP_immed_INSTR(instrs[0])           // add     sp, fp, #immed
			  && ((instrs[1] & 0xffff0800) == 0xe8bd0800)   // ldmia   sp!, {Rx, ..., Ry, fp, ...}
			  && BX_LR_INSTR(instrs[2]))                    // bx      lr
		{
			u8 shifter_operand;
			
			mode = 7;
			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[0]);
			
			sp = fp + shifter_operand;
			
			sp += bit_count(instrs[1] & 0xffff) * arm_inst_size();

			if (instrs[1] & 0x0800)       // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			if (instrs[1] & 0x4000)       // if lr is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xc000), &lr, arm_inst_size()))
				{
					return false;
				}
			}

			if (instrs[1] & 0x2000)       // if sp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[1] & 0xe000), &sp, arm_inst_size()))
				{
					return false;
				}
			}

			*sp_modified = true;

			//pc = lr - arm_inst_size();
			pc = lr;

			break;
		}
/*			
			else if (    (instrs[0] == 0xe28bd000)          // sub     sp, fp, #0
				  && (instrs[1] == 0xe8bd0800)          // ldmia   sp!, {fp}
				  && BX_LR_INSTR(instrs[2]))            // bx      lr
			{
				mode = 8;
				sp = fp;
				
				*sp_modified = true;

				if (!read_data_from_addr(sp, &fp, arm_inst_size()))
				{
					return false;
				}
				
				sp += 4;

				//pc = lr - arm_inst_size();
				pc = lr;

				break;
			}
*/
		else if (    ADD_SP_SP_immed_INSTR(instrs[0])            // add     sp, sp, #xxx
		          && LDMIA_SP_WITH_LR_INSTR(instrs[1])           // ldmia   sp!, {Rx, ..., lr}
		          && BX_LR_INSTR(instrs[2]))                     // bx      lr
		{
			u8 shifter_operand;
			
			mode = 9;
			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[0]);
			
			sp += shifter_operand;

			sp += bit_count(instrs[1] & 0xffff) * arm_inst_size();
			
			*sp_modified = true;

			if (instrs[1] & 0x0800)        // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[0] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			if (!read_data_from_addr(sp - arm_inst_size(), &pc, arm_inst_size()))
			{
				return false;
			}

			break;
		}
		else if (LDMIA_SP_WITH_LR_INSTR(instrs[0]))          // ldmia sp!, {Rx, ..., lr}
		{
			mode = 10;
			sp += bit_count(instrs[0] & 0xffff) * arm_inst_size();
			
			*sp_modified = true;

			if (instrs[0] & 0x0800)     // if fp is in the list
			{
				if (!read_data_from_addr(sp - arm_inst_size() * bit_count(instrs[0] & 0xf800), &fp, arm_inst_size()))
				{
					return false;
				}
			}

			if (!read_data_from_addr(sp - arm_inst_size(), &pc, arm_inst_size()))
			{
				return false;
			}

			break; 
		}
		else if (   LDR_LR_SPBASE_immed_INSTR(instrs[0])         // LDR     lr, [sp, #immed]
		         && LDR_FP_SPBASE_immed_INSTR(instrs[1])         // LDR     fp, [sp, #immed]
		         && ADD_SP_SP_immed_INSTR(instrs[2])             // add     sp, sp, #xxx
		         && BX_LR_INSTR(instrs[3]))                      // bx      lr
		{
			u8 shifter_operand;
			u16 immed1, immed2;
			
			mode = 13;
			immed1 = instrs[0] & 0xfff;
			
			if (!read_data_from_addr(sp + immed1, &pc, arm_inst_size()))
			{
				return false;
			}
			
			immed2 = instrs[1] & 0xfff;
			if (!read_data_from_addr(sp + immed2, &fp, arm_inst_size()))
			{
				return false;
			}

			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[2]);
			
			sp += shifter_operand;
			*sp_modified = true;
			
			break;
		}
		else if (   ADD_SP_SP_immed_INSTR(instrs[0])          // add     sp, sp, #xxx
		         && BX_LR_INSTR(instrs[1]))                   // bx      lr
		{
			u8 shifter_operand;
			
			mode = 11;
			shifter_operand = get_immed_operand_for_data_proc_instr(instrs[0]);
			
			//pc = lr - arm_inst_size();
			pc = lr;

			sp += shifter_operand;
			
			*sp_modified = true;

			break;
		}
		else if (   ((instrs[0] & 0x0fffffff) == 0x012fff1e)            // BX(cond) lr
		         && (depth == 1))
		{
			mode = 12;

			//pc = lr - 4;
			pc = lr;

			*sp_modified = false;
			
			break;
		}
		else if (LDMIA_SP_WITHOUT_LR_PC_INSTR(instrs[0]))
		{
			sp += bit_count(instrs[0] & 0xffff) * arm_inst_size();
			
			*sp_modified = true;
		} 
		
#endif

		cur += arm_inst_size();
	}
	
	frame->pc = pc;
	frame->sp = sp;
	frame->fp = fp;
	
	return true;
}
#endif
static u32 get_branch_address_from_lr(u32 lr)
{
	u32 address;
	
	if (lr & 0x1)
	{
		u16 insts[2];
		u32 cur;

		//lr = lr & ~0x1;
		
		cur = (lr & ~0x1) - 2 * thumb_inst_size();
		
		if (!read_instruction(cur, &insts, 2, true))
			return 0;

		if (BL_INSTR_THUMB(insts[0], insts[1]) || BLX_INSTR_THUMB(insts[0], insts[1]))
			address = lr - 2 * thumb_inst_size();
		else
			address = lr - thumb_inst_size();
			
		return address;
	}
	else
	{
		return lr - arm_inst_size();
	}
}
unsigned int user_backtrace_2(struct pt_regs * const regs, pid_t pid, pid_t tid, bool kernel_sample)
{
	u32 pc, sp, lr, fp;
	u32 prev_sp = 0;
	u32 prev_fp = 0;
	u32 prev_pc = 0;
	
	u32 adjust_pc;
	u32 pc1;
	
	unsigned int depth = 0;
	struct unwind_regs frame;
	
	bool ret;
	
	bool sp_modified = false;

	//return 0;
	pc = regs->ARM_pc;
	sp = regs->ARM_sp;
	lr = regs->ARM_lr;
	fp = regs->ARM_fp;
	
	if (regs->ARM_cpsr & 0x20)            // It is an Thumb instruction when interrupt is triggered
	{
		// we need to set the lowest bit of the pc 
		// to make sure it will be handled as a thumb instrution
		pc |= 0x1;                    
	}

	pc1 = pc;

	g_current_pid = pid;

	//while (sp > prev_sp) 
	do
	{
		if (depth >= 1)
		{
			prev_sp = sp;
			prev_fp = fp;
		}

		prev_pc = pc;

		adjust_pc = pc & ~0x1;
		
		// check if the instruction is a BL or SWI instruction
		if (depth >= 1)
		{
			if (!is_branch_instruction(pc) && !is_swi_instruction(pc))
			{
				return depth;
			}
		}

		if (!add_css_data(adjust_pc, pid))
		{
			return WRITE_CALL_STACK_ERR;
		}

		depth++;

		if (depth >= 200)
		{
			break;
		}

		frame.pc = adjust_pc;
		frame.sp = sp;
		frame.fp = fp;
		frame.lr = lr;

		if (pc & 0x1)
			ret = thumb_unwind(&frame, &sp_modified, depth);
		else
			ret = arm_unwind_2(&frame, &sp_modified, depth);
		
		if (!ret)
		{
			return depth;
		}
		
		sp = frame.sp;
		pc = get_branch_address_from_lr(frame.pc);
		fp = frame.fp;
		
	} while ((!sp_modified || (sp > prev_sp)) && access_ok(VERIFY_READ, sp, sizeof(unsigned int)));

	return depth;
}

unsigned int user_backtrace(struct pt_regs * const regs, pid_t pid, pid_t tid, bool kernel_sample)
{
	unsigned int depth = 0;
#if 0
	tail = ((struct frame_tail_1 *) regs->ARM_fp) - 1; 
	//tail = ((struct frame_fail *) regs->ARM_fp);
        while (tail && !((unsigned long) tail & 3))
	{
		tail = user_backtrace(tail);

		if (tail == NULL)
			break;

		if ((unsigned long) tail & 3)
		{
			break;
		}
		depth++;

	}

	return depth;
#else
	struct frame_tail_1 *tail;
	struct frame_tail_1 buftail;
	u32 pc, fp, prev_fp;

	/* user space call stack back trace */
	pc = (regs->ARM_pc & ~0x1)- 4;
	fp = regs->ARM_fp;
	prev_fp = fp - 1;
	do
	{
		if (!add_css_data(pc, pid))
		{
			if (kernel_sample) 
				++scg_metrics.discard_kernel_samples;
			else
				++scg_metrics.discard_user_samples;
			return WRITE_CALL_STACK_ERR;
		}

		++depth;

		/* check if in prologue */
		if (depth == 1)
		{
			u32 instrs[2];
			if (!__copy_from_user_inatomic(instrs, (void *)pc, sizeof(instrs))
					&& in_prologue(instrs))
			{
			       	pc = regs->ARM_lr - arm_inst_size();
			       	continue;
			}
		}

		/* backtrace to upper frame 
		 * Note: frame pointers should strictly progress back up the stack
		 * (towards higher addresses) 
		 * */
		tail = ((struct frame_tail_1 *) fp) - 1;
		if (fp <= prev_fp || !valid_user_stack(tail, regs) 
				|| __copy_from_user_inatomic(&buftail, tail, sizeof(buftail)))
		       	break;
		prev_fp = fp;
		pc = buftail.lr - 4;
		if (pc == 0) 
			break;
		fp = buftail.fp;
	} while (true);

	return depth;
#endif
}


unsigned int get_depth_offset(unsigned int rec_begin_offset)
{
	unsigned int depth_offset;

	depth_offset = rec_begin_offset + offsetof(PXD32_CSS_Call_Stack, depth);

	if (depth_offset >= g_sample_buffer.buffer.size)
		depth_offset -= g_sample_buffer.buffer.size;
	
	return depth_offset;
}

/* 
 * return value:
 *      true:    buffer is full
 *      false:   buffer is not full
 */
bool backtrace(struct pt_regs * const orig_regs, pid_t pid, pid_t tid, unsigned int reg_id)
{
	u32 instr;
	unsigned int depth = 0;
	unsigned int n;
	struct pt_regs * regs = orig_regs;
	bool kernel_sample = false;

	/* the offset where the depth field in the current PXD32_CSS_Call_Stack records locates*/
	unsigned int depth_offset;
	unsigned int orig_write_offset;
	
//	if (in_sched_functions(regs->ARM_pc))
//		return false;

	orig_write_offset = g_sample_buffer.buffer.write_offset;
	
	if (!write_css_data_head(reg_id, pid, tid))
	{
		roll_back_buffer_to_offset(orig_write_offset);
		return true;
	}

	/* kernel space call stack back trace */	
	if (!user_mode(regs))
	{

		depth = kernel_backtrace(regs, pid, tid);

		if (depth == WRITE_CALL_STACK_ERR)
		{
			roll_back_buffer_to_offset(orig_write_offset);
			return true;
		}

		 /* if the last instruction in user mode is SWI then in system call.
		  * if in system call, trace back to user space.
		  */
		regs = GET_USERREG();

		if (__copy_from_user_inatomic(&instr, (void *)((regs->ARM_pc & ~0x1) - arm_inst_size()), 4) 
				|| !SWI_INSTR(instr))
		{
//#ifdef CONFIG_FRAME_POINTER
#if 1
			++scg_metrics.collect_kernel_samples;
			
			check_buffer_flush();

			depth_offset = get_depth_offset(orig_write_offset);
			update_css_data_depth(depth_offset, depth);

			g_dsa->sample_count++;

			return false;
#else
			++scg_metrics.discard_kernel_samples;
			
			check_buffer_flush();
			roll_back_buffer_to_offset(orig_write_offset);

			return false;
#endif
		}

		kernel_sample = true;
	}

	n = user_backtrace_2(regs, pid, tid, kernel_sample);

	if (n == WRITE_CALL_STACK_ERR)
	{
		roll_back_buffer_to_offset(orig_write_offset);
		return true;
	}

	depth += n;

	if (depth == 0)
	{
		roll_back_buffer_to_offset(orig_write_offset);
		return true;
	}
	else
	{
		depth_offset = get_depth_offset(orig_write_offset);
		update_css_data_depth(depth_offset, depth);
	}

	if (kernel_sample)
		++scg_metrics.collect_kernel_samples;
	else
		++scg_metrics.collect_user_samples;

	check_buffer_flush();
	
	g_dsa->sample_count++;
	
	return false;
}
