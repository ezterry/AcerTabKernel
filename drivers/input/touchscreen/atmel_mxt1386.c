#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

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
#define NUM_FINGERS                           10

/* The information of mXT1386 */
#define ConfigChecksum                        8159978 /* C830A */
#define Chip_Vendor                           "AT"
#define Reseved_Chip_Vendor                   "0"
#define ConfigVersion                         1114920 /* 110328 */
#define Reseved_Firmware_Info                 0
#define Reseved_ConfigVersion                 0
#define Reservedinfo                          0

/* The I2C write or read retry number of times */
#define I2C_RETRY_NUM                         10

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

struct sensitivity_mapping {
	int symbol;
	int value;
};

static struct sensitivity_mapping sensitivity_table[] = {
	{TOUCH_SENSITIVITY_SYMBOL_HIGH,           45},
	{TOUCH_SENSITIVITY_SYMBOL_MEDIUM,         55},
	{TOUCH_SENSITIVITY_SYMBOL_LOW,            65},
};

struct point_data {
	short Status;
	short X;
	short Y;
};

/* The information of firmware */
static int Firmware_Info = 0;
static u8 Firmware_Version = 0;
static u8 Firmware_Build = 0;

static int debug = DEBUG_ERROR;

/* The parameters about dealing interrupt */
static int ConfigError = 0;
static int LastUpdateID = 0;
static bool irq_coming = 0;
static bool ConfigChecksumError = 0;
static u8 checksum[3] = {0};
static u8 counts = 0;
static u8 i2cfail_esd = 0;
static u8 i2cfail_real = 0;

/* The address of the related OBJ function */
u16 T05_OBJAddr;  /* Message Processor Byte0 = ID, Byte1 = AddrL, Byte2 = AddrH */
u16 T06_OBJAddr;  /* Command Processor */
u16 T07_OBJAddr;  /* Power Configuration */
u16 T08_OBJAddr;  /* Acquisition Configuration */
u16 T09_OBJAddr;  /* Multiple Touch */
u16 T15_OBJAddr;  /* Key Array */
u16 T18_OBJAddr;  /* Comconfig */
u16 T22_OBJAddr;  /* Noise Suppression */
u16 T24_OBJAddr;  /* One-touch Gesture Processor */
u16 T25_OBJAddr;  /* Self Test */
u16 T27_OBJAddr;  /* Two-touch Gesture Processor */
u16 T28_OBJAddr;  /* CTE */
u16 T37_OBJAddr;  /* DIAGDEBUG */
u16 T38_OBJAddr;  /* Userdata */
u16 T40_OBJAddr;  /* Grip */
u16 T41_OBJAddr;  /* Palm */
u16 T43_OBJAddr;  /* Digitizer */
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
	T43
};

u8 OBJTable[20][6];
u8 T05OBJInf[3]; /* Message Processor Byte0 = ID, Byte1 = AddrL, Byte2 = AddrH */
u8 T06OBJInf[3]; /* Command Processor */
u8 T07OBJInf[3]; /* Power Configuration */
u8 T08OBJInf[3]; /* Acquisition Configuration */
u8 T09OBJInf[3]; /* Multiple Touch */
u8 T15OBJInf[3]; /* Key Array */
u8 T18OBJInf[3]; /* Comconfig */
u8 T22OBJInf[3]; /* Noise Suppression */
u8 T24OBJInf[4]; /* One-touch Gesture Processor */
u8 T25OBJInf[3]; /* Self Test */
u8 T27OBJInf[3]; /* Two-touch Gesture Processor */
u8 T28OBJInf[3]; /* CTE */
u8 T37OBJInf[3]; /* DIAGDEBUG */
u8 T38OBJInf[3]; /* Userdata */
u8 T40OBJInf[3]; /* Grip */
u8 T41OBJInf[3]; /* Palm */
u8 T43OBJInf[3]; /* Digitizer */

/* T10A_0328.cfg */
/* GEN_POWERCONFIG_T7 INSTANCE 0 */
u8 T07OBJ[3]  = {  50,  10,  25};

/* GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
u8 T08OBJ[10] = {  10,   0,  10,  10,   0,   0,   5,  10,  30,  25};

/* TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 0 */
u8 T09OBJ[34] = { 143,   0,   0,  28,  41,   0,  16,  55,   3,   1,
                    0,   5,   5,  32,  10,   5,  10,   5,  31,   3,
                  255,   4,   0,   0,   0,   0, 152,  34, 212,  22,
                   10,  10,   0,   0};

/* TOUCH_KEYARRAY_T15 INSTANCE 0 INSTANCE 1 */
u8 T15OBJ[22] = {   1,  24,  41,   4,   1,   0,   0, 255,   1,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0};

/* SPT_COMMSCONFIG_T18 INSTANCE 0 */
u8 T18OBJ[2]  = {   0,   0};

/* PROCG_NOISESUPPRESSION_T22 INSTANCE 0 */
u8 T22OBJ[17] = {   5,   0,   0,   0,   0,   0,   0,   0,  45,   0,
                    0,  11,  17,  22,  32,  36,   0};

/* PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTANCE 0 */
u8 T24OBJ[19] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0};

/* SPT_SELFTEST_T25 INSTANCE 0 */
u8 T25OBJ[14] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};
/* PROCI_TWOTOUCHGESTUREPROCESSOR_T27 INSTANCE 0 */
u8 T27OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_CTECONFIG_T28 INSTANCE 0 */
u8 T28OBJ[6]  = {   0,   0,   0,  16,  16,  60};

/* PROCI_GRIPSUPPRESSION_T40 INSTANCE 0 */
u8 T40OBJ[5]  = {   0,   0,   0,   0,   0};

/* PROCI_PALMSUPPRESSION_T41 INSTANCE 0 */
u8 T41OBJ[6]  = {   0,   0,   0,   0,   0,   0};

/* SPT_DIGITIZER_T43 INSTANCE 0 */
u8 T43OBJ[6]  = {   0,   0,   0,   0,   0,   0};

struct mxt_data
{
	struct i2c_client    *client;
	struct input_dev     *input;
	struct semaphore     sema;
	struct delayed_work  dwork;
	int irq;
	short irq_type;
	struct point_data PointBuf[NUM_FINGERS];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct mxt_data *mxt;
static int suspend_resume = 0;
static int ATMEL_Backup(struct mxt_data *mxt);
static int ATMEL_Reset(struct mxt_data *mxt);
static int ATMEL_Deepsleep(struct mxt_data *mxt);
static int ATMEL_Issleep(struct mxt_data *mxt);
static int ATMEL_Resume(struct mxt_data *mxt);
static int ATMEL_IsResume(struct mxt_data *mxt);
static int ATMEL_Calibrate(struct mxt_data *mxt);
static int ATMEL_SyncWithThreadToReadMsg(struct mxt_data *mxt, int boot);
#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h);
void mxt_late_resume(struct early_suspend *h);
#endif

/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	int i, j;
	int buf_size = 256;
	struct {
		__le16  le_addr;
		u8      data[buf_size];
	} i2c_block_transfer;
	struct mxt_data *mxt;

	if (length > buf_size)
		return -EINVAL;

	mxt = i2c_get_clientdata(client);
	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2)) {
		return length;
	} else {
		for(j=0; j<I2C_RETRY_NUM; j++) {
			mdelay(10);
			i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
			if (i == (length + 2)) {
				mxt_debug(DEBUG_ERROR, "mXT1386 i2c write %d time\n", j+2);
				return length;
			}
		}
		mxt_debug(DEBUG_ERROR, "mXT1386: i2c write failed\n");
		return -1;
	}
}

static int mxt_read_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr;
	struct mxt_data *mxt;
	int i;

	mxt = i2c_get_clientdata(client);
	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;
	if (i2c_transfer(adapter, msg, 2) == 2) {
		return length;
	} else {
		for(i=0; i<I2C_RETRY_NUM; i++) {
			mdelay(10);
			if (i2c_transfer(adapter, msg, 2) == 2) {
				mxt_debug(DEBUG_ERROR, "mXT1386: i2c read %d time\n", i+2);
				return length;
			}
		}
		mxt_debug(DEBUG_ERROR, "mXT1386: i2c read failed\n");
		return -1;
	}
}

static int mxt_read_block_onetime(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16  le_addr;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;
	if(i2c_transfer(adapter, msg, 2) == 2) {
		return length;
	} else {
		mdelay(10);
		mxt_debug(DEBUG_ERROR, "mXT1386: i2c read failed\n");
		return -1;
	}
}

static int mxt_read_block_wo_addr(struct i2c_client *client, u16 length, u8 *value)
{
	int i;

	if (i2c_master_recv(client, value, length) == length) {
		return length;
	} else {
		for(i=0; i<I2C_RETRY_NUM; i++){
			mdelay(10);
			if(i2c_master_recv(client, value, length) == length) {
				mxt_debug(DEBUG_ERROR, "mXT1386: i2c read %d time\n", i+2);
				return length;
			}
		}
		mxt_debug(DEBUG_ERROR, "mXT1386: i2c read failed\n");
		return -1;
	}
}

static void CalculateAddr16bits(u8 hbyte_input, u8 lbyte_input, u16 *output, u32 val)
{
	u16 temp;

	temp = hbyte_input;
	temp = (temp << 8) + lbyte_input;
	temp = temp + val;
	*output = temp;
}

static void mxt_worker(struct work_struct *work)
{
	struct mxt_data *mxt;
	u8 buffer[8] = {0};
	u16 Buf = 0;
	short ContactID = 0;
	bool first_touch = 0;
	int i;

	mxt = container_of(work, struct mxt_data, dwork.work);
	disable_irq(mxt->irq);
	mxt_debug(DEBUG_DETAIL, "mxt_worker, and irq_type is %d\n",mxt->irq_type);

	if (mxt->irq_type == ATMEL_ReadResponseMsg_Noaddress) {
		counts++;      /* identify touch source only for first message reading */
		if(counts == 1) {
			for(i=0;i<=1;i++) {    /* identify two touch sources two time */
				if(mxt_read_block_wo_addr(mxt->client, 8, buffer) < 0) {
					mxt_debug(DEBUG_ERROR, "mXT1386: identify CANDO source failed, start to identify Sintek source\n");
					mxt->client->addr = 0x4d;
					if (mxt_read_block_wo_addr(mxt->client, 8, buffer) < 0) {
						if (i == 1) {    /* i == 1 means this is second round */
							mxt_debug(DEBUG_ERROR, "mXT1386: identify two sources failed\n");
							up(&mxt->sema);
							goto fail;
						} else {
							mxt_debug(DEBUG_ERROR, "mXT1386: identify two sources failed, try again\n");
							mxt->client->addr = 0x4c;
							msleep(100);
						}
					} else {
						mxt_debug(DEBUG_ERROR, "mXT1386: It is Sintek source\n");
						break;
					}
				} else {
					mxt_debug(DEBUG_ERROR, "mXT1386: It is CANDO source\n");
					break;
				}
			}
		} else {
			if (mxt_read_block_wo_addr(mxt->client, 8, buffer) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block_wo_addr failed\n");
				i2cfail_real = 1;
				up(&mxt->sema);
				goto fail;
			}
		}

		i2cfail_real = 0;
		if (debug == DEBUG_DETAIL) {
			for(i=0;i<=7;i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]:0x%x\n", i, buffer[i]);
		}
		if (buffer[0] == 0x01) {
			if (buffer[1] != 0x00) {
				if ((buffer[1] & 0x08)==0x08) {
					ConfigError = 1;
					mxt_debug(DEBUG_DETAIL, "release sema\n");
					up(&mxt->sema);
					goto still_disable_irq;
				} else if((buffer[1] & 0x40)==0x40) {
					ConfigError = 0;
					mxt_debug(DEBUG_ERROR, "mXT1386: OFL overflow occurs\n");
					up(&mxt->sema);
					goto still_disable_irq;
				} else {
					ConfigError = 0;
				}
			} else {
				ConfigError = 0;
				mxt_debug(DEBUG_DETAIL, "release sema\n");
				up(&mxt->sema);
				goto still_disable_irq;
			}
		}
	} else if (mxt->irq_type == ATMEL_ReadResponseMsg) {
		if (mxt_read_block(mxt->client, T05_OBJAddr, 8, buffer) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
			i2cfail_real = 1;
			up(&mxt->sema);
			goto fail;
		}
		i2cfail_real = 0;
		if (debug == DEBUG_DETAIL) {
			for(i=0;i<=7;i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]:0x%x\n", i, buffer[i]);
		}
		if (buffer[0] == 0x01) {
			if (buffer[1] != 0x00) {
				if ((buffer[1] & 0x08)==0x08) {
					ConfigError = 1;
					mxt_debug(DEBUG_DETAIL, "release sema\n");
					up(&mxt->sema);
					goto still_disable_irq;
				 } else if((buffer[1] & 0x40)==0x40) {
					ConfigError = 0;
					mxt_debug(DEBUG_ERROR, "mXT1386: OFL overflow occurs\n");
					up(&mxt->sema);
					goto still_disable_irq;
				} else {
					ConfigError = 0;
				}
			} else {
				ConfigError = 0;
				mxt_debug(DEBUG_DETAIL, "release sema\n");
				up(&mxt->sema);
				goto still_disable_irq;
			}
		}
	} else if (mxt->irq_type == ATMEL_ReadSendCMDgetCRCMsg) {
		if (mxt_read_block(mxt->client, T05_OBJAddr, 8, buffer) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
			i2cfail_real = 1;
			up(&mxt->sema);
			goto fail;
		}
		i2cfail_real = 0;
		if (debug == DEBUG_DETAIL) {
			for(i=0;i<=7;i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]:0x%x\n", i, buffer[i]);
		}
		if (buffer[0] == 0x01) {
			checksum[0] = buffer[2];
			checksum[1] = buffer[3];
			checksum[2] = buffer[4];
		} else if (buffer[0] == 0x1b) {
			mxt_debug(DEBUG_DETAIL, "release sema\n");
			up(&mxt->sema);
			goto still_disable_irq;
		}
	} else if (mxt->irq_type == ATMEL_HandleTouchMsg) {
		if (mxt_read_block_onetime(mxt->client, T05_OBJAddr, 8, buffer) < 0) {
			i2cfail_esd = 1;
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed, try again\n");
			if (mxt_read_block(mxt->client, T05_OBJAddr, 8, buffer) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block fail\n");
				goto fail;
			}
		}
		if(i2cfail_esd == 1) {
			/* ESD recovery */
			for(i=0; i<NUM_FINGERS; i++) {
				mxt->PointBuf[i].X = 0;
				mxt->PointBuf[i].Y = 0;
				mxt->PointBuf[i].Status = -1;
			}
			i2cfail_esd = 0;
		}
		mxt_debug(DEBUG_DETAIL, "buffer[0] is 0x%x\n", buffer[0]);
		if (buffer[0] >= 2 && buffer[0] <= NUM_FINGERS+1) {
			mxt_debug(DEBUG_DETAIL, "report id: 0x%x\n",buffer[0]);
			ContactID = buffer[0] - 2 ; /* touch id from firmware begins from 2 */
			Buf = buffer[2];
			Buf = (Buf << 4) | (buffer[4] >> 4);
			mxt->PointBuf[ContactID].X = Buf;
			mxt_debug(DEBUG_DETAIL, "X hbyte is 0x%x xy lbyte is 0x%x\n",buffer[2],buffer[4]);
			mxt_debug(DEBUG_DETAIL, "X is 0x%x\n",Buf);
			Buf = buffer[3];
			Buf = (Buf << 4) | (buffer[4] & 0x0F);
			Buf = Buf >> 2;              /* y resolution is 800, use 10-bit format */
			mxt->PointBuf[ContactID].Y = Buf;
			mxt_debug(DEBUG_DETAIL, "Y hbyte is 0x%x xy lbyte is 0x%x\n",buffer[3],buffer[4]);
			mxt_debug(DEBUG_DETAIL, "Y is 0x%x\n",Buf);
			mxt_debug(DEBUG_DETAIL, "SIZE is 0x%x\n",buffer[5]);
			for(i=0;i<=7;i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]:0x%x\n", i, buffer[i]);

			if(mxt->PointBuf[ContactID].Status <= 0)
				first_touch = 1;
			else
				first_touch = 0;

			if ((buffer[1] & 0x20)==0x20) {
				mxt->PointBuf[ContactID].Status = 0;
				mxt_debug(DEBUG_DETAIL, "FingerRelease!!\n");
			} else if ((buffer[1] & 0x80)==0x80) {
				mxt->PointBuf[ContactID].Status = buffer[5];
				mxt_debug(DEBUG_DETAIL, "FingerTouch!!\n");
			} else if ((buffer[1] & 0x02)==0x02) {
				ContactID = 255;
				mxt_debug(DEBUG_DETAIL, "PalmSuppresion!!\n");
			} else if(buffer[1] == 0x00) {
				ContactID = 255;
				mxt_debug(DEBUG_DETAIL, "this point is not touch or release\n");
			}
		} else {
			mxt_debug(DEBUG_DETAIL, "not a member of touch point\n");
			if (debug == DEBUG_DETAIL) {
				for(i = 0; i <=7; i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]:0x%x\n", i, buffer[i]);
			}
			ContactID = 255 ;
		}
		if (ContactID == 255)
			goto next_irq;
		mxt_debug(DEBUG_BASIC, "Get Point[%d] Update: Status=%d X=%d Y=%d\n",
		ContactID, mxt->PointBuf[ContactID].Status, mxt->PointBuf[ContactID].X, mxt->PointBuf[ContactID].Y);
		/* Send point report to Android */
		if ((mxt->PointBuf[ContactID].Status == 0) || (ContactID <= LastUpdateID) || first_touch) {
			for(i=0; i<NUM_FINGERS; i++) {
				if (mxt->PointBuf[i].Status >= 0) {
					mxt_debug(DEBUG_BASIC,
						"Report Point[%d] Update: Status=%d X=%d Y=%d\n",
						i, mxt->PointBuf[i].Status, mxt->PointBuf[i].X, mxt->PointBuf[i].Y);
					if (mxt->PointBuf[i].Status > 0) {
						input_report_abs(mxt->input, ABS_MT_TRACKING_ID, i);
						input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, mxt->PointBuf[i].Status);
						input_report_abs(mxt->input, ABS_MT_POSITION_X, mxt->PointBuf[i].X);
						input_report_abs(mxt->input, ABS_MT_POSITION_Y, mxt->PointBuf[i].Y);
						input_report_abs(mxt->input, ABS_MT_PRESSURE, mxt->PointBuf[i].Status);
					}

					input_mt_sync(mxt->input);

					if (mxt->PointBuf[i].Status == 0)
						mxt->PointBuf[i].Status--;
				}
			}
			input_sync(mxt->input);
		}
		LastUpdateID = ContactID;
	}

next_irq:
fail:
	enable_irq(mxt->irq);
still_disable_irq:
	return;
}

static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;

	mxt_debug(DEBUG_DETAIL, "\n mxt_irq_handler\n");
	irq_coming = 1;

	schedule_delayed_work(&mxt->dwork, 0);

	return IRQ_HANDLED;
}

/* SYSFS_START */
char hbyte, lbyte, val;
int rlen;

int myatoi(const char *a)
{
	int s = 0;

	while(*a >= '0' && *a <= '9')
		s = (s << 3) + (s << 1) + *a++ - '0';
	return s;
}

static ssize_t hbyte_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;

	s += sprintf(s, "0x%x\n",hbyte);
	return (s - buf);
}

static ssize_t hbyte_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int ibuf;

	ibuf = myatoi(buf);
	hbyte = ibuf & 0x000000ff;
	mxt_debug(DEBUG_ERROR, "mXT1386: hbyte is 0x%x\n", hbyte);
	return n;
}

static ssize_t lbyte_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += sprintf(s, "0x%x\n",lbyte);
	return (s - buf);
}

static ssize_t lbyte_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int ibuf;

	ibuf = myatoi(buf);
	lbyte = ibuf & 0x000000ff;
	mxt_debug(DEBUG_ERROR, "mXT1386: lbyte is 0x%x\n", lbyte);
	return n;
}

static ssize_t rlen_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += sprintf(s, "0x%x\n",val);
	return (s - buf);
}

static ssize_t rlen_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	rlen = myatoi(buf);
	mxt_debug(DEBUG_ERROR, "mXT1386: rlen is %d\n", rlen);
	return n;
}

static ssize_t val_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	s += sprintf(s, "%c\n",val);
	return (s - buf);
}

static ssize_t val_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int ibuf,i;
	char wdatabuf[1] = {0};
	char *rdatabuf;
	u16 addr16;

	rdatabuf = kzalloc(sizeof(char)*rlen, GFP_KERNEL);
	ibuf = myatoi(buf);
	val = ibuf & 0x000000ff;
	wdatabuf[0] = val;

	mxt_debug(DEBUG_ERROR, "mXT1386: val is 0x%x\n", val);
	mxt_debug(DEBUG_ERROR, "mXT1386: hbyte is 0x%x, lbyte is 0x%x\n", hbyte, lbyte);

	CalculateAddr16bits(hbyte, lbyte, &addr16, 0);

	if (!strncmp(buf,"r", 1)) {
		mxt_read_block(mxt->client, addr16, rlen, rdatabuf);
		for(i=0;i<rlen;i++)
			mxt_debug(DEBUG_ERROR, "mXT1386: rdatabuf is 0x%x\n", rdatabuf[i]);
	} else if (!strncmp(buf,"b", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386: backup\n");
		ATMEL_Backup(mxt);
	} else if (!strncmp(buf,"t", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386: reset\n");
		ATMEL_Reset(mxt);
	} else if(!strncmp(buf,"s", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386: sleep\n");
		ATMEL_Deepsleep(mxt);
	} else if(!strncmp(buf,"m", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386: resume\n");
		ATMEL_Resume(mxt);
	} else {
		mxt_write_block(mxt->client, addr16, 1, wdatabuf);
	}

	kfree(rdatabuf);

	return n;
}

static ssize_t debugmsg_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += sprintf(s, "%d\n",debug);
	return (s - buf);
}

static ssize_t debugmsg_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	debug = myatoi(buf);
	printk("debug is %d\n", debug);
	return n;
}

static ssize_t sensitivity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int i = 0;
	int symbol = -1;
	unsigned int delta = ~0;
	unsigned int diff = 0;
	u8 sens_buffer[8] = {0};

	if (mxt_read_block(mxt->client, T09_OBJAddr, 8, sens_buffer) < 0)
		return sprintf(buf, "fail\n");
	for(i = 0; i < TOUCH_SENSITIVITY_SYMBOL_COUNT; i++) {
		if (sensitivity_table[i].value == sens_buffer[7]) {
			symbol = sensitivity_table[i].symbol;
			break;
		}
		diff = ABS((int)sensitivity_table[i].value - (int)sens_buffer[7]);
		if (diff < delta) {
			symbol = sensitivity_table[i].symbol;
			delta = diff;
		}
	}
	return sprintf(buf, "%d\n",symbol );
}

static ssize_t sensitivity_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int flags = myatoi(buf);
	int i = 0;
	int symbol = -1;
	u8 sen_value[1] = {0};
	u16 sens_addr16;

	for(i = 0; i < TOUCH_SENSITIVITY_SYMBOL_COUNT; i++) {
		if (sensitivity_table[i].symbol == flags) {
			symbol = sensitivity_table[i].symbol;
			break;
		}
	}
	if (symbol == -1) {
		mxt_debug(DEBUG_ERROR, "the flags of touch sensitivity is not invalid\n");
		symbol = TOUCH_SENSITIVITY_SYMBOL_DEFAULT;
	}
	sen_value[0] = sensitivity_table[symbol].value;

	CalculateAddr16bits(T09OBJInf[2], T09OBJInf[1], &sens_addr16, 7);
	if (mxt_write_block(mxt->client, sens_addr16, 1, sen_value) < 0)
		mxt_debug(DEBUG_ERROR, "sensitivity_store fail\n");
	if (ATMEL_Backup(mxt) < 0)
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Backup failed\n");
	return n;
}

static ssize_t filter_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	u8 filter[14] = {0};
	if (mxt_read_block(mxt->client, T09_OBJAddr, 14, filter) < 0)
		return sprintf(buf, "fail\n");
	return sprintf(buf, "filter1: 0x%x\nfilter2: 0x%x\nfilter3: 0x%x\n", filter[11], filter[12], filter[13]);
}

static ssize_t filter_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int XT9_DECISION = myatoi(buf);
	u8 filter_value[3] = {0};
	u16 addr;

	CalculateAddr16bits(T09OBJInf[2], T09OBJInf[1], &addr, 11);

	if(XT9_DECISION){
		/* The parameters for XT9 handwrite input */
		filter_value[0] = 0;
		filter_value[1] = 1;
		filter_value[2] = 78;
		if(mxt_write_block(mxt->client, addr, 3, filter_value) < 0)
			mxt_debug(DEBUG_ERROR, "filter_store fail\n");
	} else {
		/* The parameters for common status */
		filter_value[0] = 5;
		filter_value[1] = 5;
		filter_value[2] = 32;
		if(mxt_write_block(mxt->client, addr, 3, filter_value) < 0)
			mxt_debug(DEBUG_ERROR, "filter_store fail\n");
	}
	return n;
}

static ssize_t FirmwareVersion_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	int fw_version = 0;
	int cf_version = 0;

	fw_version = Firmware_Info;
	fw_version <<= 4;
	fw_version |= Reseved_Firmware_Info;
	cf_version = ConfigVersion;
	cf_version <<= 4;
	cf_version |= Reseved_ConfigVersion;
	cf_version <<= 4;
	cf_version |= Reservedinfo;
	s += sprintf(s, "%s%s-%X-%X\n", Chip_Vendor, Reseved_Chip_Vendor, fw_version, cf_version);
	return (s - buf);
}

static struct kobject *touchdebug_kobj;

#define debug_attr(_name) \
	static struct kobj_attribute _name##_attr = { \
	.attr = { \
	.name = __stringify(_name), \
	.mode = 0644, \
	}, \
	.show = _name##_show, \
	.store = _name##_store, \
	}

static struct kobj_attribute FirmwareVersion_attr = { \
	.attr = { \
	.name = __stringify(FirmwareVersion), \
	.mode = 0644, \
	}, \
	.show = FirmwareVersion_show, \
};

debug_attr(hbyte);
debug_attr(lbyte);
debug_attr(rlen);
debug_attr(val);
debug_attr(debugmsg);
debug_attr(sensitivity);
debug_attr(filter);

static struct attribute * g[] = {
	&hbyte_attr.attr,
	&lbyte_attr.attr,
	&rlen_attr.attr,
	&val_attr.attr,
	&debugmsg_attr.attr,
	&sensitivity_attr.attr,
	&filter_attr.attr,
	&FirmwareVersion_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};
/* SYSFS_END */

static int ATMEL_ReadIDInfo(struct mxt_data *mxt)
{
	u8 buffer[7] = {0};
	int i;

	if (mxt_read_block(mxt->client, 0, 7, buffer) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block fail\n");
		return -1;
	}
	if (debug == DEBUG_DETAIL) {
		for(i=0;i<=6;i++)
			mxt_debug(DEBUG_DETAIL, "ID info are 0x%x\n",buffer[i]);
	}
	Firmware_Version = buffer[2];
	Firmware_Build = buffer[3];
	Firmware_Info = Firmware_Version;
	Firmware_Info <<= 8;
	Firmware_Info |= Firmware_Build;
	MaxTableSize = (buffer[6] * 6) + 10;
	OBJTableSize = buffer[6];

	return 0;
}

static void ATMEL_CalOBJ_ID_Addr(u8 Type, u8 *Table)
{
	u8 i, ID, z;
	ID = 1;

	for(i = 0; i < MaxTableSize; i++) {
		if (OBJTable[i][0] == Type) {
			if (Type == T09) {
				Table[1] = OBJTable[i][1];
				Table[2] = OBJTable[i][2];
				for(z = 0; z < NUM_FINGERS; z++) {
					ID++;
				}
			} else if (Type == T24) {
				Table[2] = OBJTable[i][1];
				Table[3] = OBJTable[i][2];
				Table[0] = ID;
				ID++;
				Table[1] = ID;
			} else {
				if (OBJTable[i][5] != 0)
					Table[0] = ID;
				else
					Table[0] = 0x00;
					Table[1] = OBJTable[i][1];
					Table[2] = OBJTable[i][2];
			}
			return;
		} else {
			ID += (OBJTable[i][4]+1) * OBJTable[i][5];
		}
	}
}

static u32 ATMEL_CRCSoft24(u32 crc, u8 FirstByte, u8 SecondByte)
{
	u32 crcPoly;
	u32 Result;
	u16 WData;

	crcPoly = 0x80001b;

	WData = (u16) ((u16)(SecondByte << 8) | FirstByte);

	Result = ((crc << 1) ^ ((u32) WData));
	if (Result & 0x1000000) {
		Result ^= crcPoly;
	}

	return Result;
}

static int ATMEL_CheckOBJTableCRC(struct mxt_data *mxt)
{
  u8 *buffer;
	u8 T07_VAL[3] = {0}, T08_VAL[10] = {0}, T09_VAL[34] = {0}, T15_VAL[22] = {0};
	u8 T18_VAL[2] = {0}, T22_VAL[17] = {0}, T24_VAL[19] = {0}, T25_VAL[14] = {0};
	u8 T27_VAL[7] = {0}, T28_VAL[6]  = {0}, T40_VAL[5]  = {0}, T41_VAL[6]  = {0};
	u8 T43_VAL[6] = {0};
	u8 i, z, Value;
	u32 Cal_crc = 0, InternalCRC;
	buffer = kzalloc(sizeof(u8)*MaxTableSize + 1, GFP_KERNEL);

	/* read all table */
	if (mxt_read_block(mxt->client, 0, MaxTableSize, buffer) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		return -1;
	}
	i = 0;
	while(1) {
		if ((i + 2) > (MaxTableSize - 3)) {
			Cal_crc = ATMEL_CRCSoft24(Cal_crc, buffer[i], 0);
		} else {
			Cal_crc = ATMEL_CRCSoft24(Cal_crc, buffer[i], buffer[i + 1]);
		}
			i = i + 2;
		if ((i == (MaxTableSize - 3)) || (i > (MaxTableSize - 3)))
			break;
	}

	InternalCRC = buffer[MaxTableSize - 1];
	InternalCRC = (InternalCRC << 8) + buffer[MaxTableSize - 2];
	InternalCRC = (InternalCRC << 8) + buffer[MaxTableSize - 3];

	if ((Cal_crc & 0x00ffffff) == InternalCRC) {
		mxt_debug(DEBUG_ERROR, "mXT1386: Check OBJ Table CRC is ok....\n");
	} else {
		mxt_debug(DEBUG_ERROR, "mXT1386: Check OBJ Table CRC is fail....\n");
		return -1;
	}

	z = 0;
	Value = 7;
	mxt_debug(DEBUG_DETAIL, "OBJTable\n");

	while(1){
		for(i = 0; i < 6; i++) {
			OBJTable[z][i] = buffer[Value];
			mxt_debug(DEBUG_DETAIL, "0x%x ",buffer[Value]);
			Value++;
		}
		mxt_debug(DEBUG_DETAIL, "\n");
		z++;
		if (z == OBJTableSize) {
			break;
		}
	}

	ATMEL_CalOBJ_ID_Addr(T05, T05OBJInf);
	CalculateAddr16bits(T05OBJInf[2], T05OBJInf[1], &T05_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T06, T06OBJInf);
	CalculateAddr16bits(T06OBJInf[2], T06OBJInf[1], &T06_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T07, T07OBJInf);
	CalculateAddr16bits(T07OBJInf[2], T07OBJInf[1], &T07_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T08, T08OBJInf);
	CalculateAddr16bits(T08OBJInf[2], T08OBJInf[1], &T08_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T09, T09OBJInf);
	CalculateAddr16bits(T09OBJInf[2], T09OBJInf[1], &T09_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T15, T15OBJInf);
	CalculateAddr16bits(T15OBJInf[2], T15OBJInf[1], &T15_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T18, T18OBJInf);
	CalculateAddr16bits(T18OBJInf[2], T18OBJInf[1], &T18_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T22, T22OBJInf);
	CalculateAddr16bits(T22OBJInf[2], T22OBJInf[1], &T22_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T24, T24OBJInf);
	CalculateAddr16bits(T24OBJInf[3], T24OBJInf[2], &T24_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T25, T25OBJInf);
	CalculateAddr16bits(T25OBJInf[2], T25OBJInf[1], &T25_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T27, T27OBJInf);
	CalculateAddr16bits(T27OBJInf[2], T27OBJInf[1], &T27_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T28, T28OBJInf);
	CalculateAddr16bits(T28OBJInf[2], T28OBJInf[1], &T28_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T37, T37OBJInf);
	CalculateAddr16bits(T37OBJInf[2], T37OBJInf[1], &T37_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T38, T38OBJInf);
	CalculateAddr16bits(T38OBJInf[2], T38OBJInf[1], &T38_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T40, T40OBJInf);
	CalculateAddr16bits(T40OBJInf[2], T40OBJInf[1], &T40_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T41, T41OBJInf);
	CalculateAddr16bits(T41OBJInf[2], T41OBJInf[1], &T41_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T43, T43OBJInf);
	CalculateAddr16bits(T43OBJInf[2], T43OBJInf[1], &T43_OBJAddr, 0);


	if (debug == DEBUG_DETAIL) {
		mxt_debug(DEBUG_DETAIL, "Seperate Table\n");

		mxt_debug(DEBUG_DETAIL, "T05\n");
		mxt_debug(DEBUG_DETAIL, "T05[2]:0x%x, T05[1]:0x%x, T05[0]:0x%x T05:0x%x\n",
				T05OBJInf[2], T05OBJInf[1], T05OBJInf[0], T05_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T06\n");
		mxt_debug(DEBUG_DETAIL, "T06[2]:0x%x, T06[1]:0x%x, T06[0]:0x%x T06:0x%x\n",
				T06OBJInf[2], T06OBJInf[1], T06OBJInf[0], T06_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T07\n");
		mxt_debug(DEBUG_DETAIL, "T07[2]:0x%x, T07[1]:0x%x, T07[0]:0x%x T07:0x%x\n",
				T07OBJInf[2], T07OBJInf[1], T07OBJInf[0], T07_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T08\n");
		mxt_debug(DEBUG_DETAIL, "T08[2]:0x%x, T08[1]:0x%x, T08[0]:0x%x T08:0x%x\n",
				T08OBJInf[2], T08OBJInf[1], T08OBJInf[0], T08_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T09\n");
		mxt_debug(DEBUG_DETAIL, "T09[2]:0x%x, T09[1]:0x%x, T09[0]:0x%x T09:0x%x\n",
				T09OBJInf[2], T09OBJInf[1], T09OBJInf[0], T09_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T15\n");
		mxt_debug(DEBUG_DETAIL, "T15[2]:0x%x, T15[1]:0x%x, T15[0]:0x%x T15:0x%x\n",
				T15OBJInf[2], T15OBJInf[1], T15OBJInf[0], T15_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T18\n");
		mxt_debug(DEBUG_DETAIL, "T18[2]:0x%x, T18[1]:0x%x, T18[0]:0x%x T18:0x%x\n",
				T18OBJInf[2], T18OBJInf[1], T18OBJInf[0], T18_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T22\n");
		mxt_debug(DEBUG_DETAIL, "T22[2]:0x%x, T22[1]:0x%x, T22[0]:0x%x T22:0x%x\n",
				T22OBJInf[2], T22OBJInf[1], T22OBJInf[0], T22_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T24\n");
		mxt_debug(DEBUG_DETAIL, "T24[3]:0x%x, T24[2]:0x%x, T24[1]:0x%x, T24[0]:0x%x T24:0x%x\n",
				T24OBJInf[3], T24OBJInf[2], T24OBJInf[1], T24OBJInf[0], T24_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T25\n");
		mxt_debug(DEBUG_DETAIL, "T25[2]:0x%x, T25[1]:0x%x, T25[0]:0x%x T25:0x%x\n",
				T25OBJInf[2], T25OBJInf[1], T25OBJInf[0], T25_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T27\n");
		mxt_debug(DEBUG_DETAIL, "T27[2]:0x%x, T27[1]:0x%x, T27[0]:0x%x T27:0x%x\n",
				T27OBJInf[2], T27OBJInf[1], T27OBJInf[0], T27_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T28\n");
		mxt_debug(DEBUG_DETAIL, "T28[2]:0x%x, T28[1]:0x%x, T28[0]:0x%x T28:0x%x\n",
				T28OBJInf[2], T28OBJInf[1], T28OBJInf[0], T28_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T37\n");
		mxt_debug(DEBUG_DETAIL, "T37[2]:0x%x, T37[1]:0x%x, T37[0]:0x%x T37:0x%x\n",
				T37OBJInf[2], T37OBJInf[1], T37OBJInf[0], T37_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T38\n");
		mxt_debug(DEBUG_DETAIL, "T38[2]:0x%x, T38[1]:0x%x, T38[0]:0x%x T38:0x%x\n",
				T38OBJInf[2], T38OBJInf[1], T38OBJInf[0], T38_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T40\n");
		mxt_debug(DEBUG_DETAIL, "T40[2]:0x%x, T40[1]:0x%x, T40[0]:0x%x T40:0x%x\n",
				T40OBJInf[2], T40OBJInf[1], T40OBJInf[0], T40_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T41\n");
		mxt_debug(DEBUG_DETAIL, "T41[2]:0x%x, T41[1]:0x%x, T41[0]:0x%x T41:0x%x\n",
				T41OBJInf[2], T41OBJInf[1], T41OBJInf[0], T41_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T43\n");
		mxt_debug(DEBUG_DETAIL, "T43[2]:0x%x, T43[1]:0x%x, T43[0]:0x%x T43:0x%x\n",
				T43OBJInf[2], T43OBJInf[1], T43OBJInf[0], T43_OBJAddr);

		if (mxt_read_block(mxt->client, T07_OBJAddr, 3, T07_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<3;i++)
			mxt_debug(DEBUG_DETAIL, " T07[%d] 0x%x\n", i, T07_VAL[i]);

		if (mxt_read_block(mxt->client, T08_OBJAddr, 10, T08_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<10;i++)
			mxt_debug(DEBUG_DETAIL, " T08[%d] 0x%x\n", i, T08_VAL[i]);

		if (mxt_read_block(mxt->client, T09_OBJAddr, 34, T09_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<34;i++)
			mxt_debug(DEBUG_DETAIL, " T09[%d] 0x%x\n", i, T09_VAL[i]);

		if (mxt_read_block(mxt->client, T15_OBJAddr, 22, T15_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<22;i++)
			mxt_debug(DEBUG_DETAIL, " T15[%d] 0x%x\n", i, T15_VAL[i]);

		if (mxt_read_block(mxt->client, T18_OBJAddr, 2, T18_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<2;i++)
			mxt_debug(DEBUG_DETAIL, " T18[%d] 0x%x\n", i, T18_VAL[i]);

		if (mxt_read_block(mxt->client, T22_OBJAddr, 17, T22_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<17;i++)
			mxt_debug(DEBUG_DETAIL, " T22[%d] 0x%x\n", i, T22_VAL[i]);

		if (mxt_read_block(mxt->client, T24_OBJAddr, 19, T24_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<19;i++)
			mxt_debug(DEBUG_DETAIL, " T24[%d] 0x%x\n", i, T24_VAL[i]);

		if (mxt_read_block(mxt->client, T25_OBJAddr, 14, T25_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<14;i++)
			mxt_debug(DEBUG_DETAIL, " T25[%d] 0x%x\n", i, T25_VAL[i]);

		if (mxt_read_block(mxt->client, T27_OBJAddr, 7, T27_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<7;i++)
			mxt_debug(DEBUG_DETAIL, " T27[%d] 0x%x\n", i, T27_VAL[i]);

		if (mxt_read_block(mxt->client, T28_OBJAddr, 6, T28_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<6;i++)
			mxt_debug(DEBUG_DETAIL, " T28[%d] 0x%x\n", i, T28_VAL[i]);

		if (mxt_read_block(mxt->client, T40_OBJAddr, 5, T40_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<5;i++)
			mxt_debug(DEBUG_DETAIL, " T40[%d] 0x%x\n", i, T40_VAL[i]);

		if (mxt_read_block(mxt->client, T41_OBJAddr, 6, T41_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<6;i++)
			mxt_debug(DEBUG_DETAIL, " T41[%d] 0x%x\n", i, T41_VAL[i]);

		if (mxt_read_block(mxt->client, T43_OBJAddr,  6, T43_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386: mxt_read_block failed\n");
		for(i=0;i<6;i++)
			mxt_debug(DEBUG_DETAIL, " T43[%d] 0x%x\n", i, T43_VAL[i]);
	}
	kfree(buffer);

	return 0;
}

static int ATMEL_SetCTE(struct mxt_data *mxt)
{

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_SetCTE\n");

	if (mxt_write_block(mxt->client, T28_OBJAddr, 6, T28OBJ) < 0)
		return -1;

	return 0;
}

static int ATMEL_Backup(struct mxt_data *mxt)
{
	u8 val[1] = {85};
	u16 addr16;

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_Backup\n");

	CalculateAddr16bits(T06OBJInf[2],T06OBJInf[1], &addr16, 1);
	if(mxt_write_block(mxt->client, addr16, 1, val) < 0)
		return -1;

	return 0;
}

static int ATMEL_Reset(struct mxt_data *mxt)
{
	u8 val[1] = {1};

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_Reset\n");

	if(mxt_write_block(mxt->client, T06_OBJAddr, 1, val) < 0)
		return -1;

	return 0;
}

static int ATMEL_Calibrate(struct mxt_data *mxt)
{
	u8 val[1] = {1};
	u16 addr16;

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_Calibrate\n");

	CalculateAddr16bits(T06OBJInf[2],T06OBJInf[1], &addr16, 2);
	if(mxt_write_block(mxt->client, addr16, 1, val) < 0)
		return -1;

	return 0;
}

static int ATMEL_SendCMDgetCRC(struct mxt_data *mxt)
{
	u8 val[1] = {1};
	u16 addr16;
	u32 checksum32;

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_SendCMDgetCRC\n");

	CalculateAddr16bits(T06OBJInf[2],T06OBJInf[1], &addr16, 3);
	if(mxt_write_block(mxt->client, addr16, 1, val) < 0)
		return -1;

	mxt->irq_type = ATMEL_ReadSendCMDgetCRCMsg;
	if (ATMEL_SyncWithThreadToReadMsg(mxt,0) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_SyncWithThreadToReadMsg failed\n");
		return -1;
	}

	checksum32 = checksum[2];
	checksum32 = (checksum32 << 8) + checksum[1];
	checksum32 = (checksum32 << 8) + checksum[0];

	if (checksum32 != ConfigChecksum) {
		mxt_debug(DEBUG_ERROR,
				"mXT1386: ConfigChecksumError is set to 1, checksum32 now is %d -> should be %d\n",
				checksum32, ConfigChecksum);
		ConfigChecksumError = 1;
	} else {
		mxt_debug(DEBUG_ERROR,
				"mXT1386: There is no ConfigChecksumError, checksum32 now is %d -> should be %d\n",
				checksum32, ConfigChecksum);
		ConfigChecksumError = 0;
	}

	return 0;
}

static int ATMEL_WriteConfig(struct mxt_data *mxt)
{
	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_WriteConfig\n");

	if(mxt_write_block(mxt->client, T07_OBJAddr, 3, T07OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T08_OBJAddr, 10, T08OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T09_OBJAddr, 34, T09OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T15_OBJAddr, 22, T15OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T18_OBJAddr, 2, T18OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T22_OBJAddr, 17, T22OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T24_OBJAddr, 19, T24OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T25_OBJAddr, 14, T25OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T27_OBJAddr, 7, T27OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T28_OBJAddr, 6, T28OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T40_OBJAddr, 5, T40OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T41_OBJAddr, 6, T41OBJ) < 0)
		return -1;

	if(mxt_write_block(mxt->client, T43_OBJAddr, 6, T43OBJ) < 0)
		return -1;

	return 0;
}

/*  boot argument is for determine when to use request_irq or not*/
static int ATMEL_SyncWithThreadToReadMsg(struct mxt_data *mxt, int boot)
{
	int error, count = 0;

	mxt_debug(DEBUG_DETAIL, "ATMEL_SyncWithThreadToReadMsg, boot is %d\n", boot);

	if(boot) {
		mxt_debug(DEBUG_DETAIL, "mXT1386: boot request_irq\n");

		if(down_interruptible(&mxt->sema))
			return -1;
		error = request_irq(mxt->irq,
				mxt_irq_handler,
				IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				mxt->client->name,
				mxt);

		if (error < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: failed to allocate irq %d\n", mxt->irq);
			up(&mxt->sema);
			return -1;
		}
		/* no irq means that touch firmware may be crashed, return fail */
		while(!irq_coming) {
			mdelay(100);
			count++;
			if (count == 20) {
				mxt_debug(DEBUG_ERROR,
						"mXT1386: no interrupt, there may be no device or crashed\n");
				up(&mxt->sema);
				return -1;
			}
		}
	} else {
		mxt_debug(DEBUG_DETAIL, "mXT1386: enable_irq\n");
		enable_irq(mxt->irq);
	}

	mxt_debug(DEBUG_DETAIL, "PowerOnRead_noaddr before 2nd sema\n");
	if(down_interruptible(&mxt->sema))
		return -1;
	if(i2cfail_real == 1)
		return -1;
	return 0;
}

static int ATMEL_ConfigErrRecovery(struct mxt_data *mxt)
{
	if (ATMEL_SetCTE(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_SetCTE failed\n");
		return -1;
	}

	if (ATMEL_WriteConfig(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_WriteConfig failed\n");
		return -1;
	}

	if (ATMEL_Backup(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Backup failed\n");
		return -1;
	}

	mdelay(200);

	if (ATMEL_Reset(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Reset failed\n");
		return -1;
	}

	mdelay(200);

	mxt->irq_type = ATMEL_ReadResponseMsg;
	if(ATMEL_SyncWithThreadToReadMsg(mxt,0) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_SyncWithThreadToReadMsg failed\n");
		return -1;
	}

	return 0;
}

static int ATMEL_CheckConfig(struct mxt_data *mxt)
{
	if (ATMEL_SendCMDgetCRC(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_WriteConfig failed\n");
		return -1;
	}

	/* write config first time and check config error path */
	if (ConfigChecksumError) {
		do {
			if (ATMEL_WriteConfig(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_WriteConfig failed\n");
				return -1;
			}

			if (ATMEL_Backup(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Backup failed\n");
				return -1;
			}

			mxt->irq_type = ATMEL_ReadResponseMsg;
			if (ATMEL_SyncWithThreadToReadMsg(mxt,0) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_SyncWithThreadToReadMsg failed\n");
				return -1;
			}

			if(ConfigError) {
				mxt_debug(DEBUG_ERROR, "mXT1386: ConfigError after Backup\n");
				do {
					if (ATMEL_ConfigErrRecovery(mxt) < 0) {
						mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_ConfigErrRecovery failed\n");
						return -1;
					}

				} while(ConfigError);
			}

			if(ATMEL_SendCMDgetCRC(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_WriteConfig failed\n");
				return -1;
			}
		} while(ConfigChecksumError);
	}

	return 0;
}

static int ATMEL_Initial(struct mxt_data *mxt)
{
	u8 sens_buf[8] = {0};
	u8 sens_val[1];
	u16 sens_addr;

	mxt->irq_type = ATMEL_ReadResponseMsg_Noaddress;
	if (ATMEL_SyncWithThreadToReadMsg(mxt,1) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_SyncWithThreadToReadMsg failed\n");
		return -1;
	}

	if (ConfigError) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ConfigError\n");
		if (ATMEL_ReadIDInfo(mxt) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_ReadIDInfo failed\n");
			return -1;
		}

		if (ATMEL_CheckOBJTableCRC(mxt) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_CheckOBJTableCRC failed\n");
			return -1;
		}
		/* Read the sensitivity value */
		if (mxt_read_block(mxt->client, T09_OBJAddr, 8, sens_buf) < 0)
			mxt_debug(DEBUG_ERROR, "Read Multiple Touch fail\n");
		sens_val[0] = sens_buf[7];
		mxt_debug(DEBUG_DETAIL, "sensitivity value: 0x%x\n",sens_buf[7]);

		do {
			if (ATMEL_ConfigErrRecovery(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_ConfigErrRecovery failed\n");
				return -1;
			}
		}while(ConfigError);
	} else {
		if (ATMEL_ReadIDInfo(mxt) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_ReadIDInfo failed\n");
			return -1;
		}

		if (ATMEL_CheckOBJTableCRC(mxt) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_CheckOBJTableCRC failed\n");
			return -1;
		}
		/* Read the sensitivity value */
		if (mxt_read_block(mxt->client, T09_OBJAddr, 8, sens_buf) < 0)
			mxt_debug(DEBUG_ERROR, "Read Multiple Touch fail\n");
		sens_val[0] = sens_buf[7];
		mxt_debug(DEBUG_DETAIL, "sensitivity value: 0x%x\n",sens_buf[7]);
	}

	/* reserve this for write config to rom in the future */
	if (ATMEL_CheckConfig(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_CheckConfig failed\n");
		return -1;
	}

	/* Write the sensitivity value that the same as the AP */
	CalculateAddr16bits(T09OBJInf[2], T09OBJInf[1], &sens_addr, 7);
	if(mxt_write_block(mxt->client, sens_addr, 1, sens_val) < 0)
		mxt_debug(DEBUG_ERROR, "Write Multiple Touch fail\n");
	if (ATMEL_Backup(mxt) < 0)
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Backup failed\n");

	mxt->irq_type = ATMEL_HandleTouchMsg;

	enable_irq(mxt->irq);

	return 0;
}
static int ATMEL_Deepsleep(struct mxt_data *mxt)
{
	u8 val[2] = {0, 0};

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_Deepsleep\n");

	if (mxt_write_block(mxt->client, T07_OBJAddr, 2, val) < 0)
		return -1;

	return 0;

}

static int ATMEL_Issleep(struct mxt_data *mxt)
{
	u8 buffer[2] = {0};

	if (mxt_read_block(mxt->client, T07_OBJAddr, 2, buffer) < 0)
		return -1;

	if (buffer[0] != 0 || buffer[1] != 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: touch panel does not sleep\n");
		return 0;
	}

	return 1;

}

static int ATMEL_IsResume(struct mxt_data *mxt)
{
	u8 buffer[2] = {0};

	if (mxt_read_block(mxt->client, T07_OBJAddr, 2, buffer) < 0)
		return -1;

	if (buffer[0] == 0 || buffer[1] == 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: touch panel does not resume, still sleep\n");
		return 0;
	}

	return 1;

}

static int ATMEL_Resume(struct mxt_data *mxt)
{
	u8 val[2] = {0};

	val[0] = T07OBJ[0];
	val[1] = T07OBJ[1];

	mxt_debug(DEBUG_DETAIL, "mXT1386: ATMEL_Resume\n");

	if (mxt_write_block(mxt->client, T07_OBJAddr, 2, val) < 0)
		return -1;

	return 0;

}

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input;
	int error, i;

	mxt_debug(DEBUG_DETAIL, "mXT1386: mxt_probe\n");

	if (client == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386: client == NULL\n");
		return	-EINVAL;
	} else if (client->adapter == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386: client->adapter == NULL\n");
		return	-EINVAL;
	} else if (&client->dev == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386: client->dev == NULL\n");
		return	-EINVAL;
	} else if (&client->adapter->dev == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386: client->adapter->dev == NULL\n");
		return	-EINVAL;
	} else if (id == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386: id == NULL\n");
		return	-EINVAL;
	}

	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386: insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		mxt_debug(DEBUG_ERROR, "mXT1386: error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	input->name = "atmel-maxtouch";
	mxt->input  = input;
	mxt->client = client;
	mxt->irq = client->irq;

	INIT_DELAYED_WORK(&mxt->dwork, mxt_worker);

	set_bit(EV_ABS, input->evbit);

	set_bit(ABS_MT_TOUCH_MAJOR, input->keybit);
	set_bit(ABS_MT_POSITION_X, input->keybit);
	set_bit(ABS_MT_POSITION_Y, input->keybit);
	set_bit(ABS_X, input->keybit);
	set_bit(ABS_Y, input->keybit);

	/* single touch */
	input_set_abs_params(input, ABS_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_Y, Y_MIN, Y_MAX, 0, 0);

	/* multiple touch */
	input_set_abs_params(input, ABS_MT_POSITION_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, Y_MIN, Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, NUM_FINGERS, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE, 0, 0);

	i2c_set_clientdata(client, mxt);

	error = input_register_device(mxt->input);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: Failed to register input device\n");
		goto err_register_device;
	}

	sema_init(&mxt->sema, 1);

	for(i=0;i<NUM_FINGERS;i++) {
		mxt->PointBuf[i].X = 0;
		mxt->PointBuf[i].Y = 0;
		mxt->PointBuf[i].Status = -1;
	}

	error = ATMEL_Initial(mxt);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Initial failed\n");
		goto err_irq;
	}

	/* SYSFS_START */
	touchdebug_kobj = kobject_create_and_add("Touch", NULL);
	if (touchdebug_kobj == NULL)
		mxt_debug(DEBUG_ERROR, "%s: subsystem_register failed\n", __FUNCTION__);

	error = sysfs_create_group(touchdebug_kobj, &attr_group);
	if(error)
		mxt_debug(DEBUG_ERROR, "%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);

	/* SYSFS_END */

#ifdef CONFIG_HAS_EARLYSUSPEND
	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mxt->early_suspend.suspend = mxt_early_suspend;
	mxt->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&mxt->early_suspend);
#endif
	return 0;

err_irq:
	if (mxt->irq)
	   free_irq(mxt->irq, mxt);
err_register_device:
	input_free_device(input);
err_input_dev_alloc:
	kfree(mxt);
err_mxt_alloc:
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if (mxt->irq)
			free_irq(mxt->irq, mxt);

		cancel_delayed_work_sync(&mxt->dwork);
		input_unregister_device(mxt->input);
	}
	kfree(mxt);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
	int ret = 0;
	suspend_resume = 0;
	mxt_debug(DEBUG_ERROR, "mXT1386: mxt_early_suspend\n");

	disable_irq(data->irq);
	cancel_delayed_work_sync(&data->dwork);
	/*
	system will still go to suspend if i2c error,
	but it will be blocked if sleep configs are not written to touch successfully
	*/
	if (ATMEL_Deepsleep(data) == 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Deepsleep Done!\n");
	} else {
		ret = ATMEL_Issleep(data);
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL SUSPEND Fail: %d\n", ret);
	}
}

void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
	int i, ret;
	mxt_debug(DEBUG_ERROR, "mXT1386: mxt_late_resume\n");

	for(i=0;i<NUM_FINGERS;i++) {
		data->PointBuf[i].X = 0;
		data->PointBuf[i].Y = 0;
		data->PointBuf[i].Status = -1;
	}
	/*
	system will still resume back if i2c error,
	but it will be blocked if resume configs are not written to touch successfully
	*/
	if (ATMEL_Resume(data) == 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL_Resume Done!\n");
	} else {
		ret = ATMEL_IsResume(data);
		mxt_debug(DEBUG_ERROR, "mXT1386: ATMEL Resume Fail: %d\n", ret);
	}

	if(ATMEL_Calibrate(data) < 0)
		mxt_debug(DEBUG_ERROR, "mXT1386: calibration failed\n");

	enable_irq(data->irq);

}
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
static int mxt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	mxt_debug(DEBUG_ERROR, "mXT1386: mxt_suspend\n");
	return 0;
}

static int mxt_resume(struct i2c_client *client)
{
	mxt_debug(DEBUG_ERROR, "mXT1386: mxt_resume\n");
	return 0;
}
#endif
static const struct i2c_device_id mxt_id[] =
{
	{ "maXTouch", 0 },
	{},
};

static struct i2c_driver mxt_driver =
{
	.probe      = mxt_probe,
	.remove     = mxt_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = mxt_suspend,
	.resume     = mxt_resume,
#endif
	.id_table   = mxt_id,
	.driver     = {
		.name   = "maXTouch",
	},
};

static int __init mxt_init(void)
{
	int ret;
	ret = i2c_add_driver(&mxt_driver);

	return ret;
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

MODULE_DESCRIPTION("Driver for Atmel mxt1386 Touchscreen Controller");
MODULE_LICENSE("GPL");
