#ifndef _LINUX_MXT1386E_P2_H
#define _LINUX_MXT1386E_P2_H

#if defined(CONFIG_ARCH_ACER_T30)
#include "../../../arch/arm/mach-tegra/board-acer-t30.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#endif
extern int acer_board_id;
extern int acer_board_type;

#define X_MIN                                 0x00
#define Y_MIN                                 0x00
#define X_MAX                                 1919
#define Y_MAX                                 1199
#define MXT_MAX_REPORTED_PRESSURE             255
#define MXT_MAX_TOUCH_SIZE                    255

#define Firmware_Number                       4099
#define ConfigChecksum                        1237707 /* 12E2CB */
#define Chip_Vendor                           "AT"
#define Reseved_Chip_Vendor                   "0"
#define ConfigVersion                         1118468 /* 111104 */
#define Reseved_Firmware_Info                 0
#define Reseved_ConfigVersion                 0
#define Reservedinfo                          0

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

enum {
	TOUCH_SENSITIVITY_SYMBOL_HIGH = 0,
	TOUCH_SENSITIVITY_SYMBOL_MEDIUM,
	TOUCH_SENSITIVITY_SYMBOL_LOW,
	TOUCH_SENSITIVITY_SYMBOL_COUNT,
};

#define TOUCH_SENSITIVITY_SYMBOL_DEFAULT TOUCH_SENSITIVITY_SYMBOL_MEDIUM

/* GEN_POWERCONFIG_T7 INSTANCE 0 */
u8 T07OBJ[3]  = {  50, 255,  10};

/* GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
u8 T08OBJ[10] = {  30,  10,  10,   0,   0,   0,   5,  10,  15,   0};

/* TOUCH_MULTITOUCHSCREEN_T9 INSTATNCE 0 */
u8 T09OBJ[35] = { 131,   0,   0,  30,  42,   0,  16,  60,   2,   5,
                   10,   5,   2,   0,  10,  20,  20,  10, 175,   4,
                  127,   7,   0,   0,   0,   0,   0,   0,  64,   0,
                   10,  15,   0,   0,   0};

/* SPT_COMMSCONFIG_T18 INSTANCE 0 */
u8 T18OBJ[2]  = {   0,   0};

/* PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTATNCE 0 */
u8 T24OBJ[19] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0};

/* SPT_SELFTEST_T25 INSTATNCE 0 */
u8 T25OBJ[6]  = {   3,   0,  60, 115, 156,  99};

/* PROCI_TWOTOUCHGESTUREPROCESSOR_T27 INSTATNCE 0 */
u8 T27OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_USERDATA_T38 INSTATNCE 0 */
u8 T38OBJ[64] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};

/* PROCI_GRIPSUPPRESSION_T40 INSTATNCE 0 */
u8 T40OBJ[5]  = {   0,   0,   0,   0,   0};

/* PROCI_TOUCHSUPPRESSION_T42 INSTATNCE 0 */
u8 T42OBJ[10] = {  35,  15,  20,  45,   0,   3,   0,   0,  11,   3};

/* SPT_DIGITIZER_T43 INSTANCE 0 */
u8 T43OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ[9]  = {  68,   0,   8,  16,   0,   0,   1,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ_0[9]= {   4,   0,   8,  16,   0,   0,   1,   0,   0};

/* PROCI_STYLUS_T47 INSTATNCE 0 */
u8 T47OBJ[10] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* PROCG_NOISESUPPRESSION_T48 INSTATNCE 0 */
u8 T48OBJ[54] = {   1, 192, 194,   0,   0,   0,   0,   0,   0,   0,
                   16,  36,   0,   6,   6,   0,   0,  48,   4,  64,
                   10,   0,  20,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};

/* PROCI_SHIELDLESS_T56 INSTATNCE 0  */
u8 T56OBJ[43] = {   1,   0,   1,  51,  24,  24,  24,  24,  24,  24,
                   24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
                   24,  24,  24,  24,  24,  24,  20,  20,  20,  20,
                   20,  20,  20,  24,   1,   1,   1,   0,   0,   1,
                    2,   6,   4};

struct sensitivity_mapping {
	int symbol;
	int value;
};

static struct sensitivity_mapping sensitivity_table[] = {
	{TOUCH_SENSITIVITY_SYMBOL_HIGH,           50},
	{TOUCH_SENSITIVITY_SYMBOL_MEDIUM,         60},
	{TOUCH_SENSITIVITY_SYMBOL_LOW,            70},
};

#endif /* _LINUX_MXT1386E_P2_H */
