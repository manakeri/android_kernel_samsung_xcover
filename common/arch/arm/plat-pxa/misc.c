#include <linux/module.h>
#include <plat/misc.h>

static int board_version;

int get_board_version(void)
{
	return board_version;
}
EXPORT_SYMBOL(get_board_version);

void set_board_version(int version)
{
	board_version = version;
}
EXPORT_SYMBOL(set_board_version);
