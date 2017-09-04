/* watson: definitions */

#include <plat/bcm_cpufreq_drv.h>
#include <mach/bcm21553_cpufreq_gov.h>

#define BCM_CORE_CLK_SLEEPING_BEAUTY	BCM21553_CORECLK_KHZ_312
//#define BCM_CORE_CLK_LOWEST				BCM21553_CORECLK_KHZ_416
//#define BCM_CORE_CLK_LOW				BCM21553_CORECLK_KHZ_520
#define BCM_CORE_CLK_NORMAL				BCM21553_CORECLK_KHZ_624
#define BCM_CORE_CLK_MIDDLE				BCM21553_CORECLK_KHZ_754
#define BCM_CORE_CLK_TURBOLOW			BCM21553_CORECLK_KHZ_832
#define BCM_CORE_CLK_TURBOMID			BCM21553_CORECLK_KHZ_1040
//#define BCM_CORE_CLK_TURBOHIGH			BCM21553_CORECLK_KHZ_1248

u32 freq_tbl[5] = {
BCM_CORE_CLK_SLEEPING_BEAUTY,
BCM_CORE_CLK_NORMAL,
BCM_CORE_CLK_MIDDLE,
BCM_CORE_CLK_TURBOLOW,
BCM_CORE_CLK_TURBOMID,
}

struct bcm_freq_tbl update_volt_tbl(u32 volts[])
{
	struct bcm_freq_tbl new_tbl[];
	for (int i = 0; i < (sizeof(freq_tbl) / sizeof(u32)); i++;)
	{
		bcm215xx_cpu0_freq_tbl[i] = FTBL_INIT(freq_tbl[i] / 1000, volts[i]);
	}
	return new_tbl;
}
