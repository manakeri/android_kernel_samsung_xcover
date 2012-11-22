================================================================================
		U S E R  M A N U A L  F O R  MBTCHAR

NAME
	mbtchar.ko - BT char driver which works with Marvell stack


FOR DRIVER BUILD
----------------
	For X86 platform:
	Modify ".config" to set "CONFIG_X86=y" or make X86
	For Moorestown platform:
	Modify ".config" to set "CONFIG_MOORESTOWN=y" or make MOORESTOWN
	For PXA9XX platfrom:
	Modify ".config" to set "CONFIG_PXA9XX=y" or make PXA9XX [2.6.25]
	For PXA955 platfrom:
	Modify ".config" to set "CONFIG_PXA955=y" or make PXA955
	For PXA950 platfrom:
	Modify ".config" to set "CONFIG_PXA950=y" or make PXA950
	For PXA920 platfrom:
	Modify ".config" to set "CONFIG_PXA920=y" or make PXA920 [2.6.32]

	make clean
	make build
	The binary can be found in ../bin_mbtchar directory.


FOR DRIVER INSTALL
------------------
	insmod mbtchar.ko
	The driver should be loaded before BT driver (w/ char support).