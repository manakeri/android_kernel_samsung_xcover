cmd_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.o := /home/teemu/sources/ANDROID/cyanogenmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-gcc -Wp,-MD,/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/.gc_hal_kernel_hardware.o.d  -nostdinc -isystem /home/teemu/sources/ANDROID/cyanogenmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include -I/home/teemu/sources/ANDROID/s5690m/common/arch/arm/include -Iinclude -I../modules/INC  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-pxa/include -Iarch/arm/plat-pxa/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=7 -march=armv7-a -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fomit-frame-pointer -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=0 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=1 -DgcdNO_POWER_MANAGEMENT=0 -DUSE_PLATFORM_DRIVER=1 -DVIVANTE_PROFILER=0 -DANDROID=1 -DENABLE_GPU_CLOCK_BY_DRIVER=1 -DUSE_NEW_LINUX_SIGNAL=0 -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0 -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/cmodel/inc -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/user  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(gc_hal_kernel_hardware)"  -D"KBUILD_MODNAME=KBUILD_STR(hal/driver/galcore)" -D"DEBUG_HASH=61" -D"DEBUG_HASH2=22" -c -o /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.o /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.c

deps_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.o := \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.c \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_types.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_options.h \
    $(wildcard include/config/cpu/pxa910.h) \
    $(wildcard include/config/pxa95x.h) \
    $(wildcard include/config/cpu/mmp2.h) \
    $(wildcard include/config/dvfm.h) \
    $(wildcard include/config/dvfm/pxa910.h) \
    $(wildcard include/config/dvfm/td.h) \
    $(wildcard include/config/dvfm/mg1.h) \
    $(wildcard include/config/dvfm/mmp2.h) \
    $(wildcard include/config/enable/dvfm.h) \
    $(wildcard include/config/axiclk/control.h) \
    $(wildcard include/config/has/earlysuspend.h) \
    $(wildcard include/config/earlysuspend.h) \
    $(wildcard include/config/enable/earlysuspend.h) \
    $(wildcard include/config/pxa910/dvfm/stats.h) \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_enum.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_base.h \
    $(wildcard include/config/.h) \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_dump.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_profiler.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_driver.h \
  include/linux/delay.h \
  include/linux/kernel.h \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/preempt/voluntary.h) \
    $(wildcard include/config/debug/spinlock/sleep.h) \
    $(wildcard include/config/prove/locking.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/dynamic/debug.h) \
    $(wildcard include/config/ring/buffer.h) \
    $(wildcard include/config/tracing.h) \
    $(wildcard include/config/numa.h) \
    $(wildcard include/config/ftrace/mcount/record.h) \
  /home/teemu/sources/ANDROID/cyanogenmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include/stdarg.h \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/linkage.h \
  include/linux/stddef.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/posix_types.h \
  include/linux/bitops.h \
    $(wildcard include/config/generic/find/first/bit.h) \
    $(wildcard include/config/generic/find/last/bit.h) \
    $(wildcard include/config/generic/find/next/bit.h) \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/bitops.h \
    $(wildcard include/config/smp.h) \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/arch/has/barriers.h) \
    $(wildcard include/config/arm/dma/mem/bufferable.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  include/linux/irqflags.h \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
  include/linux/typecheck.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/irqflags.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/hwcap.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/outercache.h \
    $(wildcard include/config/outer/cache/sync.h) \
    $(wildcard include/config/outer/cache.h) \
  include/asm-generic/cmpxchg-local.h \
  include/asm-generic/bitops/non-atomic.h \
  include/asm-generic/bitops/fls64.h \
  include/asm-generic/bitops/sched.h \
  include/asm-generic/bitops/hweight.h \
  include/asm-generic/bitops/arch_hweight.h \
  include/asm-generic/bitops/const_hweight.h \
  include/asm-generic/bitops/lock.h \
  include/linux/log2.h \
    $(wildcard include/config/arch/has/ilog2/u32.h) \
    $(wildcard include/config/arch/has/ilog2/u64.h) \
  include/linux/dynamic_debug.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/byteorder.h \
  include/linux/byteorder/little_endian.h \
  include/linux/swab.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/swab.h \
  include/linux/byteorder/generic.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/bug.h \
    $(wildcard include/config/bug.h) \
    $(wildcard include/config/debug/bugverbose.h) \
  include/asm-generic/bug.h \
    $(wildcard include/config/generic/bug.h) \
    $(wildcard include/config/generic/bug/relative/pointers.h) \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/delay.h \
  /home/teemu/sources/ANDROID/s5690m/common/arch/arm/include/asm/param.h \
    $(wildcard include/config/hz.h) \

/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.o: $(deps_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.o)

$(deps_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.o):
