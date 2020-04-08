#include <stdint.h>
#include <nrf52840_bitfields.h>
#include "vdd_out.h"

#if defined (BOARD_PCA10056) || defined (BOARD_PCA10100)
#if defined ( __GNUC__ ) || defined ( __SES_ARM )
    volatile uint32_t UICR_REGOUT0 __attribute__ ((section(".uicr_regout0"))) = UICR_REGOUT0_VDD_OUT;
#endif
#endif
