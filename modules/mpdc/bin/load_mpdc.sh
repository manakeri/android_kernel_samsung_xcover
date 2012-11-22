#! /system/bin/sh

# try to load driver 
echo "Loading Marvell Code Performance Analyzer Driver ..."

PWD=`pwd`

#cd /system/etc/mpdc/

PARAM_SYSTEM_CALL_TABLE_ADDR=0

PARAM_SYSTEM_CALL_TABLE_ADDR=`/system/bin/pxksymaddr sys_call_table`

#PARAM_SYSTEM_CALL_TABLE_ADDR=$?

#if [ -f /proc/kallsyms ] ; then
#	PARAM_SYSTEM_CALL_TABLE_ADDR=`./ksymaddr sys_call_table`
#fi

#if [ "$PARAM_SYSTEM_CALL_TABLE_ADDR" = "0" ] ; then
#	echo -n "Warning: can't find sys_call_table address in /proc/kallsyms"
#fi

cd ${PWD}

insmod /system/lib/modules/mpdc_cm.ko param_system_call_table_addr=${PARAM_SYSTEM_CALL_TABLE_ADDR}
insmod /system/lib/modules/mpdc_hs.ko param_system_call_table_addr=${PARAM_SYSTEM_CALL_TABLE_ADDR}
insmod /system/lib/modules/mpdc_css.ko param_system_call_table_addr=${PARAM_SYSTEM_CALL_TABLE_ADDR}


echo "Done"
