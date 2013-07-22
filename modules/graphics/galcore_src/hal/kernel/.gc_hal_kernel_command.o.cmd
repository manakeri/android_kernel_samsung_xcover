cmd_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.o := /home/teemu/sources/ANDROID/cyanogenmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-gcc -Wp,-MD,/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/.gc_hal_kernel_command.o.d  -nostdinc -isystem /home/teemu/sources/ANDROID/cyanogenmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include -I/home/teemu/sources/ANDROID/s5690m/common/arch/arm/include -Iinclude -I../modules/INC  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-pxa/include -Iarch/arm/plat-pxa/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=7 -march=armv7-a -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fomit-frame-pointer -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=0 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=1 -DgcdNO_POWER_MANAGEMENT=0 -DUSE_PLATFORM_DRIVER=1 -DVIVANTE_PROFILER=0 -DANDROID=1 -DENABLE_GPU_CLOCK_BY_DRIVER=1 -DUSE_NEW_LINUX_SIGNAL=0 -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0 -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/cmodel/inc -I/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/user  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(gc_hal_kernel_command)"  -D"KBUILD_MODNAME=KBUILD_STR(hal/driver/galcore)" -D"DEBUG_HASH=61" -D"DEBUG_HASH2=16" -c -o /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.o /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.c

deps_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.o := \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.c \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_precomp.h \
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
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/inc/gc_hal_driver.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/arch/unified/hal/kernel/gc_hal_kernel_hardware.h \
  /home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/user/gc_hal_user_context.h \

/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.o: $(deps_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.o)

$(deps_/home/teemu/sources/ANDROID/s5690m/modules/graphics/galcore_src/hal/kernel/gc_hal_kernel_command.o):
