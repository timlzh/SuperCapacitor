#include "lsqspi_msp.h"
#include "field_manipulate.h"
#include "compile_flag.h"
#include "reg_sysc_cpu.h"
#include "reg_sysc_awo.h"
#include "ARMCM3.h"
#include "platform.h"

XIP_BANNED void lsqspi_msp_init()
{
    SYSC_CPU->PD_CPU_CLKG = SYSC_CPU_CLKG_SET_LSQSPI0_MASK;
    MODIFY_REG(SYSC_AWO->PD_AWO_CLK_CTRL,SYSC_AWO_CLK_SEL_QSPI_MASK,4<<SYSC_AWO_CLK_SEL_QSPI_POS);
}

#if (defined(BOOT_ROM))
XIP_BANNED uint8_t lsqspi_rd_cap_dly_get()
{
    return 0;
}
#else
XIP_BANNED uint8_t lsqspi_rd_cap_dly_get()
{
    return 1;
}
#endif

XIP_BANNED uint8_t lsqspi_baudrate_get()
{
    return 0;
}

XIP_BANNED uint32_t lsqspi_dly_get()
{
    return FIELD_BUILD(LSQSPI_DLY_NSS,4) | FIELD_BUILD(LSQSPI_DLY_AFTER,2) | FIELD_BUILD(LSQSPI_DLY_INIT, 2);
}
