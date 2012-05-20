#ifndef _LINUX_MXT768E_PE_H
#define _LINUX_MXT768E_PE_H

#define ATMEL768E_IOCTL_MAGIC 't'
#define ATMEL768E_FirmwareVersion                _IOR(ATMEL768E_IOCTL_MAGIC, 0x01, int)
#define ATMEL768E_CHGPinStatus                   _IOR(ATMEL768E_IOCTL_MAGIC, 0x02, int)
#define ATMEL768E_T6_REGVALUE                    _IOR(ATMEL768E_IOCTL_MAGIC, 0x03, int)

#define X_MIN                                 0x00
#define Y_MIN                                 0x00
#define X_MAX                                 0x4FF
#define Y_MAX                                 0x31F
#define MXT_MAX_REPORTED_PRESSURE             255
#define MXT_MAX_TOUCH_SIZE                    255

#define ATMEL_ReadResponseMsg_Noaddress       1
#define ATMEL_ReadResponseMsg                 2
#define ATMEL_ReadSendCMDgetCRCMsg            3
#define ATMEL_HandleTouchMsg                  4

/* Debug levels */
#define DEBUG_DETAIL                          2
#define DEBUG_BASIC                           1
#define DEBUG_ERROR                           0
/* The number of touches */
#define NUM_FINGERS_SUPPORTED                 10
/* Touch Firmware Version */
#define Firmware_Number                       8362
#define ConfigChecksum                        11426798
#define Chip_Vendor                           "AT"
#define Reseved_Chip_Vendor                   "0"
#define ConfigVersion                         1118233 /* 111019 */
#define Reseved_Firmware_Info                 0
#define Reseved_ConfigVersion                 0
#define Reservedinfo                          0
/* Interrupt Pin */
#define TEGRA_GPIO_PV6                        174

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

enum {
	TOUCH_SENSITIVITY_SYMBOL_HIGH = 0,
	TOUCH_SENSITIVITY_SYMBOL_MEDIUM,
	TOUCH_SENSITIVITY_SYMBOL_LOW,
	TOUCH_SENSITIVITY_SYMBOL_COUNT,
};

#define TOUCH_SENSITIVITY_SYMBOL_DEFAULT TOUCH_SENSITIVITY_SYMBOL_HIGH

struct point_data {
	short Status;
	short X;
	short Y;
};

struct sensitivity_mapping {
	int symbol;
	int value;
};

static int debug = DEBUG_ERROR;
static int LastUpdateID = 0;
static u8 checksum[3] = {0};
static int Firmware_Info = 0;
static u8 Firmware_Version = 0;
static u8 Firmware_Build = 0;
static bool ConfigChecksumError = 0;
static int i2cfail_esd = 0;
static int reenter_count = 0;
static int reenter_times = 6;
static u8 sens_buf[8] = {0};
static u8 sens_val[1] = {0};
static u16 sens_addr = 0;
static bool sencheck = 0;

static struct sensitivity_mapping sensitivity_table[] = {
	{TOUCH_SENSITIVITY_SYMBOL_HIGH,           50},
	{TOUCH_SENSITIVITY_SYMBOL_MEDIUM,         60},
	{TOUCH_SENSITIVITY_SYMBOL_LOW,            70},
};

u16 T05_OBJAddr; /* Message Processor Byte0 = ID, Byte1 = AddrL, Byte2 = AddrH */
u16 T06_OBJAddr; /* Command Processor */
u16 T07_OBJAddr; /* Power Configuration */
u16 T08_OBJAddr; /* Acquisition Configuration */
u16 T09_OBJAddr; /* Multiple Touch */
u16 T15_OBJAddr; /* Key Array */
u16 T18_OBJAddr;
u16 T19_OBJAddr;
u16 T22_OBJAddr;
u16 T24_OBJAddr;
u16 T27_OBJAddr;
u16 T25_OBJAddr;
u16 T28_OBJAddr;
u16 T37_OBJAddr;
u16 T38_OBJAddr;
u16 T40_OBJAddr;
u16 T41_OBJAddr;
u16 T42_OBJAddr;
u16 T43_OBJAddr;
u16 T44_OBJAddr;
u16 T46_OBJAddr;
u16 T47_OBJAddr;
u16 T48_OBJAddr;
u16 T52_OBJAddr;
u16 T55_OBJAddr;
u16 T56_OBJAddr;
u16 T57_OBJAddr;
u16 MaxTableSize;
u16 OBJTableSize;

#define mxt_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			printk(__VA_ARGS__); \
	} while (0)

enum {
	T01 = 1,
	T02,
	T03,
	T04,
	T05,
	T06,
	T07,
	T08,
	T09,
	T10,
	T11,
	T12,
	T13,
	T14,
	T15,
	T16,
	T17,
	T18,
	T19,
	T20,
	T21,
	T22,
	T23,
	T24,
	T25,
	T26,
	T27,
	T28,
	T29,
	T30,
	T31,
	T32,
	T33,
	T34,
	T35,
	T36,
	T37,
	T38,
	T39,
	T40,
	T41,
	T42,
	T43,
	T44,
	T45,
	T46,
	T47,
	T48,
	T49,
	T50,
	T51,
	T52,
	T53,
	T54,
	T55,
	T56,
	T57
};

u8 OBJTable[40][6];
u8 T05OBJInf[3]; /* Message Processor Byte0 = ID, Byte1 = AddrL, Byte2 = AddrH */
u8 T06OBJInf[3]; /* Command Processor */
u8 T07OBJInf[3]; /* Power Configuration */
u8 T08OBJInf[3]; /* Acquisition Configuration */
u8 T09OBJInf[3]; /* Multiple Touch */
u8 T15OBJInf[3]; /* Key Array */
u8 T18OBJInf[3];
u8 T19OBJInf[3];
u8 T22OBJInf[3];
u8 T24OBJInf[4];
u8 T25OBJInf[3];
u8 T27OBJInf[3];
u8 T28OBJInf[3];
u8 T37OBJInf[3];
u8 T38OBJInf[3];
u8 T40OBJInf[3];
u8 T41OBJInf[3];
u8 T42OBJInf[3];
u8 T43OBJInf[3];
u8 T44OBJInf[3];
u8 T46OBJInf[3];
u8 T47OBJInf[3];
u8 T48OBJInf[3];
u8 T52OBJInf[3];
u8 T55OBJInf[3];
u8 T56OBJInf[3];
u8 T57OBJInf[3];

/* GEN_POWERCONFIG_T7 INSTANCE 0 */
u8 T07OBJ[3]  = {  50, 255,  10};

/* GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
u8 T08OBJ[10] = {  56,   0,  20,  20,   0,   0,   5,  30,  10, -96};

/* TOUCH_MULTITOUCHSCREEN_T9 INSTATNCE 0 */
u8 T09OBJ[70] = { 143,   0,   0,  24,  32,   0, 128,  50,   2,   3,
                    0,   5,   2,   0,  10,  10,  10,  10,  31,   3,
                  255,   4,   0,   0,   0,   0,   0,   0,   0,   0,
                   10,  15,  57,  69,   0,   0,   0,   0,  24,  32,
                    0, 160,  50,  31,   1,   0,   0,   0,   0,  10,
                    0,   0,   0,   0,   0,   0,   0,  11,  11,  18,
                   18, 148,  34,   0,   0,  25,   0,   0,   0,   0};

/* TOUCH_KEYARRAY_T15 INSTATNCE 0 INSTATNCE 1 */
u8 T15OBJ[22] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0};

/* SPT_COMMSCONFIG_T18 INSTANCE 0 */
u8 T18OBJ[2]  = {   0,   0};

/* SPT_GPIOPWM_T19 INSTATNCE 0 */
u8 T19OBJ[16] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0};

/* SPT_SELFTEST_T25 INSTATNCE 0 */
u8 T25OBJ[18] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0};

/* SPT_USERDATA_T38 INSTATNCE 0 */
u8 T38OBJ[64] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};

/* PROCI_GRIPSUPPRESSION_T40 INSTATNCE 0 INSTATNCE 1 */
u8 T40OBJ[10] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* PROCI_TOUCHSUPPRESSION_T42 INSTATNCE 0 INSTATNCE 1 */
u8 T42OBJ[20] = {  35,  20,  40,  35,   0,   3,   0,   0,   2,   2,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* SPT_DIGITIZER_T43 INSTANCE 0 */
u8 T43OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ[10] = {   4,   0,   8,  10,   0,   0,   2,   0,   0,   0};

/* PROCI_STYLUS_T47 INSTATNCE 0 INSTATNCE 1 */
u8 T47OBJ[20] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* PROCG_NOISESUPPRESSION_T48 INSTATNCE 0 */
u8 T48OBJ[74] = {   3, 192, 194,   0,   0,   0,   0,   0,   0,   0,
                  128,  28,   0,   6,   6,   0,   0,  24,   4,  64,
                   10,   0,  20,   0,   0,  38,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};

/* TOUCH_PROXKEY_T52 INSTATNCE 0 INSTATNCE 1 */
u8 T52OBJ[30] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0};



/* PROCI_ADAPTIVETHRESHOLD_T55 INSTATNCE 0 INSTATNCE 1 */
u8 T55OBJ[12] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0};

/* PROCI_SHIELDLESS_T56 INSTATNCE 0  */
u8 T56OBJ[34] = {   3,   0,   1,  38,   9,   9,   9,   9,   9,   9,
                    9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
                    9,   9,   9,   9,   9,   9,   9,   9,   0,  64,
                    1,   2,  30,   4};

/* PROCI_EXTRATOUCHSCREENDATA_T57 INSTATNCE 0 INSTATNCE 1 */
u8 T57OBJ[6]  = {   0,   0,   0,   0,   0,   0};

struct mxt_data
{
	struct i2c_client    *client;
	struct input_dev     *input;
	struct semaphore     sema;
	struct delayed_work  dwork;
	struct delayed_work  init_dwork;
	struct delayed_work  touch_dwork;
	int init_irq;
	int touch_irq;
	short irq_type;
	struct point_data PointBuf[NUM_FINGERS_SUPPORTED];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct mxt_data *mxt;

static int ATMEL_Init_OBJ(struct mxt_data *mxt);
static int ATMEL_ConfigcheckCRC(struct mxt_data *mxt);
static int ATMEL_ConfigRecovery(struct mxt_data *mxt);
static int ATMEL_Touch_Init(struct mxt_data *mxt);
static int ATMEL_Backup(struct mxt_data *mxt);
static int ATMEL_Reset(struct mxt_data *mxt);
static int ATMEL_Deepsleep(struct mxt_data *mxt);
static int ATMEL_Issleep(struct mxt_data *mxt);
static int ATMEL_Resume(struct mxt_data *mxt);
static int ATMEL_IsResume(struct mxt_data *mxt);
static int ATMEL_Calibrate(struct mxt_data *mxt);
#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h);
void mxt_late_resume(struct early_suspend *h);
#endif

#endif /* _LINUX_MXT768E_PE_H */
