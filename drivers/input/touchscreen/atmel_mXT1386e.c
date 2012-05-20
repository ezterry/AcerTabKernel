#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/ioctl.h>
#include <linux/miscdevice.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#if defined(CONFIG_MACH_PICASSO_M)
#include <linux/input/mxt1386e_pm.h>
#elif defined(CONFIG_MACH_PICASSO2)
#include <linux/input/mxt1386e_p2.h>
#endif

#define ATMEL1386E_IOCTL_MAGIC 't'
#define ATMEL1386E_FirmwareVersion            _IOR(ATMEL1386E_IOCTL_MAGIC, 0x01, int)
#define ATMEL1386E_CHGPinStatus               _IOR(ATMEL1386E_IOCTL_MAGIC, 0x02, int)
#define ATMEL1386E_T6_REGVALUE                _IOR(ATMEL1386E_IOCTL_MAGIC, 0x03, int)

/* Debug levels */
#define DEBUG_DETAIL                          2
#define DEBUG_BASIC                           1
#define DEBUG_ERROR                           0
/* The number of touches */
#define NUM_FINGERS                           10

#define TP_LDO_EN                             TEGRA_GPIO_PB0
#define TP_RST                                TEGRA_GPIO_PI2
#define TP_INT                                TEGRA_GPIO_PJ0

struct point_data {
	short Status;
	short X;
	short Y;
};

static int debug = DEBUG_ERROR;

/* The information of firmware */
static int Firmware_Info = 0;
static u8 Firmware_Version = 0;
static u8 Firmware_Build = 0;
/* The caluculation of checksum */
static u8 checksum[3] = {0};
static bool ConfigChecksumError = 0;
static int reenter_count = 0;
static int reenter_times = 6;

/* The record of sensitivity */
static u8 sens_buf[8] = {0};
static u8 sens_val[1] = {0};
static u16 sens_addr = 0;
static bool sencheck = 0;
/* Other parameters for touch events */
static int i2cfail_esd = 0;
static int LastUpdateID = 0;

u16 T05_OBJAddr; /* Message Processor Byte0 = ID, Byte1 = AddrL, Byte2 = AddrH */
u16 T06_OBJAddr; /* Command Processor */
u16 T07_OBJAddr; /* Power Configuration */
u16 T08_OBJAddr; /* Acquisition Configuration */
u16 T09_OBJAddr; /* Multiple Touch */
u16 T15_OBJAddr;
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
u8 T15OBJInf[3];
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

struct mxt_data
{
	struct i2c_client    *client;
	struct input_dev     *input;
	struct work_struct   init_dwork;
	struct work_struct   touch_dwork;
	struct point_data    PointBuf[NUM_FINGERS];
	int init_irq;
	int touch_irq;
	short irq_type;
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

/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	int i, j;
	int buf_zise = 256;
	struct {
		__le16  le_addr;
		u8      data[buf_zise];
	} i2c_block_transfer;

	if (length > buf_zise)
		return -EINVAL;

	for(i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2)) {
		return length;
	} else {
		for(j=0; j<10; j++) {
			mdelay(10);
			i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
			if (i == (length + 2)) {
				mxt_debug(DEBUG_ERROR, "mXT1386E i2c write %d time\n", j+2);
				return length;
			}
		}
		/* HW reset for ESD test */
		mxt_debug(DEBUG_ERROR, "TOUCH HW RESET in i2c write\n");
		gpio_set_value(TP_LDO_EN, 0);
		gpio_set_value(TP_RST, 0);
		msleep(10);
		gpio_set_value(TP_LDO_EN, 1);
		msleep(10);
		gpio_set_value(TP_RST, 1);
		msleep(100);

		mxt_debug(DEBUG_ERROR, "mXT1386E: i2c write failed\n");
		return -1;
	}
}

static int mxt_read_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr;
	int i;

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
		for(i=0; i<10; i++) {
			mdelay(10);
			if (i2c_transfer(adapter, msg, 2) == 2) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: i2c read %d time\n", i+2);
				return length;
			}
		}
		/* HW reset for ESD test*/
		mxt_debug(DEBUG_ERROR, "TOUCH HW RESET in i2c read\n");
		gpio_set_value(TP_LDO_EN, 0);
		gpio_set_value(TP_RST, 0);
		msleep(10);
		gpio_set_value(TP_LDO_EN, 1);
		msleep(10);
		gpio_set_value(TP_RST, 1);
		msleep(100);

		mxt_debug(DEBUG_ERROR, "mXT1386E: i2c read failed\n");
		return -1;
	}
}

static int mxt_read_block_onetime(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16  le_addr;

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
		mdelay(10);
		mxt_debug(DEBUG_ERROR, "mXT1386E: i2c read failed\n");
		return -1;
	}
}

static int mxt_read_block_wo_addr(struct i2c_client *client, u16 length, u8 *value)
{
	int i;

	if (i2c_master_recv(client, value, length) == length) {
		return length;
	} else {
		for(i=0; i<10; i++) {
			mdelay(10);
			if (i2c_master_recv(client, value, length) == length) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: i2c read block wo addr %d time\n", i+2);
				return length;
			}
		}
		mxt_debug(DEBUG_ERROR, "mXT1386E: i2c read failed\n");
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

static void init_worker(struct work_struct *work)
{
	struct mxt_data *mxt;
	u8 buffer[7] = {0};
	int i;

	mxt_debug(DEBUG_ERROR, "mXT1386E: reenter_count: %d\n", reenter_count);
	mxt = container_of(work, struct mxt_data, init_dwork);
	disable_irq(mxt->init_irq);

	if (mxt_read_block_wo_addr(mxt->client, 7, buffer) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block_wo_addr failed\n");
		goto next_irq;
	}

	if (debug == DEBUG_DETAIL) {
		for(i=0;i<=6;i++)
			mxt_debug(DEBUG_DETAIL, "buffer[%d] is 0x%x\n", i, buffer[i]);
	}

	if ((buffer[0] == 0x01) && (reenter_count < reenter_times)) {
		/* the initial values of config checksum */
		checksum[0] = buffer[2];
		checksum[1] = buffer[3];
		checksum[2] = buffer[4];
		if (buffer[1] != 0x00) {
			mxt_debug(DEBUG_ERROR, "mXT1386E: Config has errors\n");
			if (buffer[1] & 0x08) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: Config ERROR occurs\n");
				goto next_irq;
			} else {
				if (buffer[1] & 0x40) {
					mxt_debug(DEBUG_ERROR, "mXT1386E: OFL overflow occurs\n");
				} else {
					mxt_debug(DEBUG_ERROR, "mXT1386E: The status is 0x%x\n", buffer[1]);
				}
				goto next_irq;
			}
		} else {
			mxt_debug(DEBUG_ERROR, "mXT1386E: Config status is ready\n");
			if (ATMEL_Init_OBJ(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Init_OBJ failed\n");
				goto next_irq;
			}
			if (ATMEL_ConfigcheckCRC(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_ConfigcheckCRC failed\n");
				goto next_irq;
			}
		}

		/* enable touch irq or rewrite config */
		if (ConfigChecksumError) {
			mxt_debug(DEBUG_ERROR, "mXT1386E: ConfigChecksum is error\n");
			if (ATMEL_ConfigRecovery(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_ConfigRecovery failed\n");
				goto next_irq;
			}
			goto next_irq;
		} else {
			mxt_debug(DEBUG_ERROR, "mXT1386E: ConfigChecksum is right\n");
			if (ATMEL_Touch_Init(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Touch_Init failed\n");
				goto next_irq;
			}
			goto still_disable_irq;
		}
	} else {
		if (reenter_count == reenter_times) {
			/* Directly enter touch irq */
			mxt_debug(DEBUG_DETAIL, "mXT1386E: Enable touch irq directly\n");
			if (ATMEL_Init_OBJ(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Init_OBJ failed\n");
				goto still_disable_irq;
			}
			if (ATMEL_ConfigRecovery(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_ConfigRecovery failed\n");
				goto still_disable_irq;
			}
			if (ATMEL_Touch_Init(mxt) < 0) {
				mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Touch_Init failed\n");
				goto still_disable_irq;
			}
			goto still_disable_irq;
		} else {
			goto next_irq;
		}
	}

next_irq:
	reenter_count++;
	enable_irq(mxt->init_irq);
still_disable_irq:
	return;
}

static void touch_worker(struct work_struct *work)
{
	struct mxt_data *mxt;
	u8 buffer[8] = {0};
	u16 Buf = 0;
	short ContactID = 0;
	int i;

	mxt = container_of(work, struct mxt_data, touch_dwork);
	disable_irq(mxt->touch_irq);

	if (mxt_read_block_onetime(mxt->client, T05_OBJAddr, 8, buffer) < 0) {
		i2cfail_esd = 1;
		mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block_onetime failed\n");
		if (mxt_read_block(mxt->client, T05_OBJAddr, 8, buffer) < 0) {
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block fail\n");
			goto next_irq;
		}
	}

	if (i2cfail_esd == 1) {
		/* ESD recovery */
		for(i=0; i<NUM_FINGERS; i++) {
			mxt->PointBuf[i].X = 0;
			mxt->PointBuf[i].Y = 0;
			mxt->PointBuf[i].Status = -1;
		}
		i2cfail_esd = 0;
	}

	/* touch id from firmware begins from 2 */
	if (buffer[0] >= 2 && buffer[0] <= NUM_FINGERS+1) {
		ContactID = buffer[0] - 2 ;
		Buf = buffer[2];
		Buf = (Buf << 4) | (buffer[4] >> 4);
		mxt->PointBuf[ContactID].X = Buf;
		Buf = buffer[3];
		Buf = (Buf << 4) | (buffer[4] & 0x0F);
#if defined(CONFIG_MACH_PICASSO_M)
		Buf = Buf >> 2;
#endif
		mxt->PointBuf[ContactID].Y = Buf;

		if (debug == DEBUG_DETAIL) {
			for(i=0;i<=7;i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]: 0x%x\n", i, buffer[i]);
		}

		if (buffer[1] & 0x20) {
			mxt->PointBuf[ContactID].Status = 0;
			mxt_debug(DEBUG_DETAIL, "Finger Release!!\n");
		} else if (buffer[1] & 0x80) {
			mxt->PointBuf[ContactID].Status = buffer[5];
			mxt_debug(DEBUG_DETAIL, "Finger Touch!!\n");
		} else if (buffer[1] & 0x02) {
			mxt->PointBuf[ContactID].Status = 0;
			mxt_debug(DEBUG_DETAIL, "Palm Suppresion!!\n");
		} else if (buffer[1] == 0x00) {
			ContactID = 255;
			mxt_debug(DEBUG_DETAIL, "this point is not touch or release\n");
		}
	} else {
		if (debug == DEBUG_DETAIL) {
			mxt_debug(DEBUG_DETAIL, "not a touch point\n");
			for(i = 0; i <=7; i++)
				mxt_debug(DEBUG_DETAIL, "buffer[%d]: 0x%x\n", i, buffer[i]);
		}
		ContactID = 255;
	}

	if (ContactID == 255)
		goto next_irq;

	mxt_debug(DEBUG_BASIC, "Point[%2d] status: %2d X: %4d Y: %4d\n", ContactID,
		mxt->PointBuf[ContactID].Status, mxt->PointBuf[ContactID].X, mxt->PointBuf[ContactID].Y);

	/* Send point report to Android */
	if ((ContactID <= LastUpdateID) || (mxt->PointBuf[i].Status >= 0)) {
		for(i=0; i<NUM_FINGERS; i++) {
			mxt_debug(DEBUG_BASIC, "Point[%2d] status: %2d X: %4d Y: %4d\n",
				i, mxt->PointBuf[i].Status, mxt->PointBuf[i].X, mxt->PointBuf[i].Y);
			/* Report Message*/
			input_report_abs(mxt->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, mxt->PointBuf[i].Status);
			input_report_abs(mxt->input, ABS_MT_POSITION_X, mxt->PointBuf[i].X);
			input_report_abs(mxt->input, ABS_MT_POSITION_Y, mxt->PointBuf[i].Y);
			input_report_abs(mxt->input, ABS_MT_PRESSURE, mxt->PointBuf[i].Status);

			input_mt_sync(mxt->input);
		}
		input_sync(mxt->input);
	}
	LastUpdateID = ContactID;

next_irq:
	enable_irq(mxt->touch_irq);
	return;
}


static irqreturn_t init_irq_handler(int irq, void *init_mxt)
{
	struct mxt_data *mxt = init_mxt;

	mxt_debug(DEBUG_DETAIL, "\ninit_irq_handler\n");

	schedule_work(&mxt->init_dwork);

	return IRQ_HANDLED;
}

static irqreturn_t touch_irq_handler(int irq, void *touch_mxt)
{
	struct mxt_data *mxt = touch_mxt;

	mxt_debug(DEBUG_DETAIL, "\ntouch_irq_handler\n");

	schedule_work(&mxt->touch_dwork);

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
	mxt_debug(DEBUG_ERROR, "mXT1386E: hbyte is 0x%x\n", hbyte);
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
	mxt_debug(DEBUG_ERROR, "mXT1386E: lbyte is 0x%x\n", lbyte);
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
	mxt_debug(DEBUG_ERROR, "mXT1386E: rlen is %d\n", rlen);
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

	mxt_debug(DEBUG_ERROR, "mXT1386E: val is 0x%x\n", val);
	mxt_debug(DEBUG_ERROR, "mXT1386E: hbyte is 0x%x, lbyte is 0x%x\n", hbyte, lbyte);

	CalculateAddr16bits(hbyte, lbyte, &addr16, 0);

	if (!strncmp(buf,"r", 1)) {
		mxt_read_block(mxt->client, addr16, rlen, rdatabuf);
		for(i=0;i<rlen;i++)
			mxt_debug(DEBUG_ERROR, "mXT1386E: rdatabuf is 0x%x\n", rdatabuf[i]);
	} else if (!strncmp(buf,"b", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: backup\n");
		ATMEL_Backup(mxt);
	} else if (!strncmp(buf,"t", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: reset\n");
		ATMEL_Reset(mxt);
	} else if (!strncmp(buf,"s", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: sleep\n");
		ATMEL_Deepsleep(mxt);
	} else if (!strncmp(buf,"m", 1)) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: resume\n");
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
	mxt_debug(DEBUG_ERROR, "TOUCH HW RESET in i2c read\n");
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
	return sprintf(buf, "%d\n", symbol);
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
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Backup failed\n");
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

	if (XT9_DECISION) {
		filter_value[0] = 0;
		filter_value[1] = 1;
		filter_value[2] = 78;
		if (mxt_write_block(mxt->client, addr, 3, filter_value) < 0)
			mxt_debug(DEBUG_ERROR, "filter_store fail\n");
	} else {
		filter_value[0] = 5;
		filter_value[1] = 5;
		filter_value[2] = 32;
		if (mxt_write_block(mxt->client, addr, 3, filter_value) < 0)
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
		mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block fail\n");
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
	u8 T07_VAL[3] = {0}, T08_VAL[10] = {0}, T09_VAL[35] = {0}, T18_VAL[2] = {0};
	u8 T24_VAL[19] = {0}, T25_VAL[6] = {0}, T27_VAL[7] = {0}, T38_VAL[64] = {0};
	u8 T40_VAL[5] = {0}, T42_VAL[10] = {0}, T43_VAL[7] = {0}, T46_VAL[9] = {0};
	u8 T47_VAL[10] = {0}, T48_VAL[54] = {0}, T56_VAL[43] = {0};
	u8 i, z, Value;
	u32 Cal_crc = 0, InternalCRC;
	buffer = kzalloc(sizeof(u8)*MaxTableSize + 1, GFP_KERNEL);

	/* read all table */
	if (mxt_read_block(mxt->client, 0, MaxTableSize, buffer) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
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
		mxt_debug(DEBUG_ERROR, "mXT1386E: Check OBJ Table CRC is ok....\n");
	} else {
		mxt_debug(DEBUG_ERROR, "mXT1386E: Check OBJ Table CRC is fail....\n");
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

	ATMEL_CalOBJ_ID_Addr(T18, T18OBJInf);
	CalculateAddr16bits(T18OBJInf[2], T18OBJInf[1], &T18_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T24, T25OBJInf);
	CalculateAddr16bits(T24OBJInf[2], T24OBJInf[1], &T24_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T25, T25OBJInf);
	CalculateAddr16bits(T25OBJInf[2], T25OBJInf[1], &T25_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T38, T38OBJInf);
	CalculateAddr16bits(T38OBJInf[2], T38OBJInf[1], &T38_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T40, T40OBJInf);
	CalculateAddr16bits(T40OBJInf[2], T40OBJInf[1], &T40_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T42, T42OBJInf);
	CalculateAddr16bits(T42OBJInf[2], T42OBJInf[1], &T42_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T43, T43OBJInf);
	CalculateAddr16bits(T43OBJInf[2], T43OBJInf[1], &T43_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T46, T46OBJInf);
	CalculateAddr16bits(T46OBJInf[2], T46OBJInf[1], &T46_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T47, T47OBJInf);
	CalculateAddr16bits(T47OBJInf[2], T47OBJInf[1], &T47_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T48, T48OBJInf);
	CalculateAddr16bits(T48OBJInf[2], T48OBJInf[1], &T48_OBJAddr, 0);

	ATMEL_CalOBJ_ID_Addr(T56, T56OBJInf);
	CalculateAddr16bits(T56OBJInf[2], T56OBJInf[1], &T56_OBJAddr, 0);

	if (debug == DEBUG_DETAIL) {
		mxt_debug(DEBUG_DETAIL, "Seperate Table\n");

		mxt_debug(DEBUG_DETAIL, "T07\n");
		mxt_debug(DEBUG_DETAIL, "T07[2]:0x%x, T07[1]:0x%x,  T07[0]:0x%x, T07 is 0x%x\n",
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

		mxt_debug(DEBUG_DETAIL, "T19\n");
		mxt_debug(DEBUG_DETAIL, "T19[2]:0x%x, T19[1]:0x%x, T19[0]:0x%x T19:0x%x\n",
				T19OBJInf[2], T19OBJInf[1], T19OBJInf[0], T19_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T24\n");
		mxt_debug(DEBUG_DETAIL, "T24[2]:0x%x, T24[1]:0x%x, T24[0]:0x%x T24:0x%x\n",
				T24OBJInf[2], T24OBJInf[1], T24OBJInf[0], T24_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T25\n");
		mxt_debug(DEBUG_DETAIL, "T25[2]:0x%x, T25[1]:0x%x, T25[0]:0x%x T25:0x%x\n",
				T25OBJInf[2], T25OBJInf[1], T25OBJInf[0], T25_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T38\n");
		mxt_debug(DEBUG_DETAIL, "T38[2]:0x%x, T38[1]:0x%x, T38[0]:0x%x T38:0x%x\n",
				T38OBJInf[2], T38OBJInf[1], T38OBJInf[0], T38_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T40\n");
		mxt_debug(DEBUG_DETAIL, "T40[2]:0x%x, T40[1]:0x%x, T40[0]:0x%x T40:0x%x\n",
				T40OBJInf[2], T40OBJInf[1], T40OBJInf[0], T40_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T42\n");
		mxt_debug(DEBUG_DETAIL, "T42[2]:0x%x, T42[1]:0x%x, T42[0]:0x%x T42:0x%x\n",
				T42OBJInf[2], T42OBJInf[1], T42OBJInf[0], T42_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T43\n");
		mxt_debug(DEBUG_DETAIL, "T43[2]:0x%x, T43[1]:0x%x, T43[0]:0x%x T43:0x%x\n",
				T43OBJInf[2], T43OBJInf[1], T43OBJInf[0], T43_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T46\n");
		mxt_debug(DEBUG_DETAIL, "T46[2]:0x%x, T46[1]:0x%x, T46[0]:0x%x T46:0x%x\n",
				T46OBJInf[2], T46OBJInf[1], T46OBJInf[0], T46_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T47\n");
		mxt_debug(DEBUG_DETAIL, "T47[2]:0x%x, T47[1]:0x%x, T47[0]:0x%x T47:0x%x\n",
				T47OBJInf[2], T47OBJInf[1], T47OBJInf[0], T47_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T48\n");
		mxt_debug(DEBUG_DETAIL, "T48[2]:0x%x, T48[1]:0x%x, T48[0]:0x%x T48:0x%x\n",
				T48OBJInf[2], T48OBJInf[1], T48OBJInf[0], T48_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T52\n");
		mxt_debug(DEBUG_DETAIL, "T52[2]:0x%x, T52[1]:0x%x, T52[0]:0x%x T52:0x%x\n",
				T52OBJInf[2], T52OBJInf[1], T52OBJInf[0], T52_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T55\n");
		mxt_debug(DEBUG_DETAIL, "T55[2]:0x%x, T55[1]:0x%x, T55[0]:0x%x T55:0x%x\n",
				T55OBJInf[2], T55OBJInf[1], T55OBJInf[0], T55_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T56\n");
		mxt_debug(DEBUG_DETAIL, "T56[2]:0x%x, T56[1]:0x%x, T56[0]:0x%x T56:0x%x\n",
				T56OBJInf[2], T56OBJInf[1], T56OBJInf[0], T56_OBJAddr);

		mxt_debug(DEBUG_DETAIL, "T57\n");
		mxt_debug(DEBUG_DETAIL, "T57[2]:0x%x, T57[1]:0x%x, T57[0]:0x%x T57:0x%x\n",
				T57OBJInf[2], T57OBJInf[1], T57OBJInf[0], T57_OBJAddr);


		if (mxt_read_block(mxt->client, T07_OBJAddr, 3, T07_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<3;i++)
			mxt_debug(DEBUG_DETAIL, " T07[%d] 0x%x\n", i, T07_VAL[i]);

		if (mxt_read_block(mxt->client, T08_OBJAddr, 10, T08_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<10;i++)
			mxt_debug(DEBUG_DETAIL, " T08[%d] 0x%x\n", i, T08_VAL[i]);

		if (mxt_read_block(mxt->client, T09_OBJAddr, 35, T09_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<35;i++)
			mxt_debug(DEBUG_DETAIL, " T09[%d] 0x%x\n", i, T09_VAL[i]);

		if (mxt_read_block(mxt->client, T18_OBJAddr, 2, T18_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<2;i++)
			mxt_debug(DEBUG_DETAIL, " T18[%d] 0x%x\n", i, T18_VAL[i]);

		if (mxt_read_block(mxt->client, T24_OBJAddr, 19, T24_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<19;i++)
			mxt_debug(DEBUG_DETAIL, " T24[%d] 0x%x\n", i, T24_VAL[i]);

		if (mxt_read_block(mxt->client, T25_OBJAddr, 6, T25_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<6;i++)
			mxt_debug(DEBUG_DETAIL, " T25[%d] 0x%x\n", i, T25_VAL[i]);

		if (mxt_read_block(mxt->client, T27_OBJAddr, 7, T27_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<7;i++)
			mxt_debug(DEBUG_DETAIL, " T27[%d] 0x%x\n", i, T27_VAL[i]);

		if (mxt_read_block(mxt->client, T38_OBJAddr, 64, T38_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<64;i++)
			mxt_debug(DEBUG_DETAIL, " T38[%d] 0x%x\n", i, T38_VAL[i]);

		if (mxt_read_block(mxt->client, T40_OBJAddr, 5, T40_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<5;i++)
			mxt_debug(DEBUG_DETAIL, " T40[%d] 0x%x\n", i, T40_VAL[i]);

		if (mxt_read_block(mxt->client, T42_OBJAddr, 10, T42_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<10;i++)
			mxt_debug(DEBUG_DETAIL, " T42[%d] 0x%x\n", i, T42_VAL[i]);

		if (mxt_read_block(mxt->client, T43_OBJAddr,  7, T43_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<7;i++)
			mxt_debug(DEBUG_DETAIL, " T43[%d] 0x%x\n", i, T43_VAL[i]);

		if (mxt_read_block(mxt->client, T46_OBJAddr, 9, T46_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<9;i++)
			mxt_debug(DEBUG_DETAIL, " T46[%d] 0x%x\n", i, T46_VAL[i]);

		if (mxt_read_block(mxt->client, T47_OBJAddr, 10, T47_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<10;i++)
			mxt_debug(DEBUG_DETAIL, " T47[%d] 0x%x\n", i, T47_VAL[i]);

		if (mxt_read_block(mxt->client, T48_OBJAddr, 54, T48_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<54;i++)
			mxt_debug(DEBUG_DETAIL, " T48[%d] 0x%x\n", i, T48_VAL[i]);

		if (mxt_read_block(mxt->client, T56_OBJAddr, 43, T56_VAL) < 0)
			mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_read_block failed\n");
		for(i=0;i<43;i++)
			mxt_debug(DEBUG_DETAIL, " T56[%d] 0x%x\n", i, T56_VAL[i]);

	}
	kfree(buffer);

	return 0;
}

static int ATMEL_SetCTE(struct mxt_data *mxt)
{

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_SetCTE\n");

	if (mxt_write_block(mxt->client, T46_OBJAddr, 9, T46OBJ) < 0)
		return -1;

	return 0;
}

static int ATMEL_Backup(struct mxt_data *mxt)
{
	u8 val[1] = {85};
	u16 addr16;

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Backup\n");

	CalculateAddr16bits(T06OBJInf[2],T06OBJInf[1], &addr16, 1);
	if (mxt_write_block(mxt->client, addr16, 1, val) < 0)
		return -1;

	return 0;
}

static int ATMEL_Reset(struct mxt_data *mxt)
{
	u8 val[1] = {1};

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Reset\n");

	if (mxt_write_block(mxt->client, T06_OBJAddr, 1, val) < 0)
		return -1;

	return 0;
}

static int ATMEL_Calibrate(struct mxt_data *mxt)
{
	u8 val[1] = {1};
	u16 addr16;

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Calibrate\n");

	CalculateAddr16bits(T06OBJInf[2],T06OBJInf[1], &addr16, 2);
	if (mxt_write_block(mxt->client, addr16, 1, val) < 0)
		return -1;

	return 0;
}

static int ATMEL_WriteConfig(struct mxt_data *mxt)
{
	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_WriteConfig\n");

	if (mxt_write_block(mxt->client, T07_OBJAddr,  3, T07OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T08_OBJAddr, 10, T08OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T09_OBJAddr, 35, T09OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T18_OBJAddr,  2, T18OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T24_OBJAddr, 19, T24OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T25_OBJAddr,  6, T25OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T27_OBJAddr,  7, T27OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T38_OBJAddr, 64, T38OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T40_OBJAddr,  5, T40OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T42_OBJAddr, 10, T42OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T43_OBJAddr,  7, T43OBJ) < 0)
		return -1;

	if (acer_board_type == BOARD_PICASSO_M) {
		if (mxt_write_block(mxt->client, T46_OBJAddr,  9, T46OBJ) < 0)
			return -1;
	} else if (acer_board_type == BOARD_PICASSO_2) {
		if (acer_board_id == BOARD_DVT1) {
			if (mxt_write_block(mxt->client, T46_OBJAddr,  9, T46OBJ_0) < 0)
				return -1;
		} else {
			if (mxt_write_block(mxt->client, T46_OBJAddr,  9, T46OBJ) < 0)
				return -1;
		}
	} else {
		if (mxt_write_block(mxt->client, T46_OBJAddr,  9, T46OBJ) < 0)
			return -1;
	}

	if (mxt_write_block(mxt->client, T47_OBJAddr, 10, T47OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T48_OBJAddr, 54, T48OBJ) < 0)
		return -1;

	if (mxt_write_block(mxt->client, T56_OBJAddr, 43, T56OBJ) < 0)
		return -1;

	return 0;
}

/* Config need to rewrite */
static int ATMEL_ConfigRecovery(struct mxt_data *mxt)
{
	if (ATMEL_SetCTE(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_SetCTE failed\n");
		return -1;
	}

	if (ATMEL_WriteConfig(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_WriteConfig failed\n");
		return -1;
	}

	if (ATMEL_Backup(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Backup failed\n");
		return -1;
	}

	mdelay(200);

	if (ATMEL_Reset(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Reset failed\n");
		return -1;
	}

	mdelay(200);

	return 0;
}

static int ATMEL_ConfigcheckCRC(struct mxt_data *mxt)
{
	u8 val[1] = {1};
	u16 addr16;
	u32 checksum32;

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_ConfigcheckCRC\n");

	CalculateAddr16bits(T06OBJInf[2],T06OBJInf[1], &addr16, 3);
	if (mxt_write_block(mxt->client, addr16, 1, val) < 0)
		return -1;

	checksum32 = checksum[2];
	checksum32 = (checksum32 << 8) + checksum[1];
	checksum32 = (checksum32 << 8) + checksum[0];

	mxt_debug(DEBUG_ERROR,"mXT1386E: checksum should be %d\n", ConfigChecksum);

	if (checksum32 != ConfigChecksum) {
		mxt_debug(DEBUG_ERROR,"mXT1386E: Now checksum is %d\n", checksum32);
		mxt_debug(DEBUG_ERROR,"mXT1386E: Now checksum is error\n");
		ConfigChecksumError = 1;
	} else {
		mxt_debug(DEBUG_ERROR,"mXT1386E: Now checksum is %d\n", checksum32);
		mxt_debug(DEBUG_ERROR,"mXT1386E: Now checksum is right\n");
		ConfigChecksumError = 0;
	}

	return 0;
}

static int ATMEL_Init(struct mxt_data *mxt){
	int error = 0;

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Init\n");

	error = request_irq(mxt->init_irq,
			init_irq_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_LOW,
			mxt->client->name,
			mxt);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: failed to allocate irq %d\n", mxt->init_irq);
		return -1;
	}

	return 0;
}

static int ATMEL_Touch_Init(struct mxt_data *mxt)
{
	int error = 0;

	mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Touch_Init\n");

	CalculateAddr16bits(T09OBJInf[2], T09OBJInf[1], &sens_addr, 7);
	if (mxt_write_block(mxt->client, sens_addr, 1, sens_val) < 0)
		mxt_debug(DEBUG_ERROR, "Write T09OBJInf fail\n");

	if (ATMEL_Backup(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Backup failed\n");
		return -1;
	}

	if (ATMEL_Reset(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Reset failed\n");
		return -1;
	}

	mdelay(200);
	free_irq(mxt->init_irq, mxt);

	error = request_irq(mxt->touch_irq,
			touch_irq_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_LOW,
			mxt->client->name,
			mxt);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: failed to allocate irq %d\n", mxt->touch_irq);
		return -1;
	}

	return 0;
}

/* count the address and CRC of OBJs */
static int ATMEL_Init_OBJ(struct mxt_data *mxt)
{
	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Init\n");
	if (ATMEL_ReadIDInfo(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_ReadIDInfo failed\n");
		return -1;
	}

	if (ATMEL_CheckOBJTableCRC(mxt) < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_CheckOBJTableCRC failed\n");
		return -1;
	}

	/* Read the sensitivity value */
	if (sencheck == 0) {
		if (mxt_read_block(mxt->client, T09_OBJAddr, 8, sens_buf) < 0)
			mxt_debug(DEBUG_ERROR, "Read T09_OBJAddr fail\n");
		sens_val[0] = sens_buf[7];
		sencheck = 1;
		mxt_debug(DEBUG_DETAIL, "sensitivity value: 0x%x\n",sens_buf[7]);
	}

	return 0;
}

static int ATMEL_Deepsleep(struct mxt_data *mxt)
{
	u8 val[2] = {0, 0};

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Deepsleep\n");

	if (mxt_write_block(mxt->client, T07_OBJAddr, 2, val) < 0)
		return -1;

	return 0;
}

/*
 *  This function is used for reply the current status of suspend.
 *  return -1: read block error, return 0: touch panel sleep fail
 *  return 1: touch panel sleep
 */
static int ATMEL_Issleep(struct mxt_data *mxt)
{
	u8 buffer[2] = {0};

	if (mxt_read_block(mxt->client, T07_OBJAddr, 2, buffer) < 0)
		return -1;

	if (buffer[0] != 0 || buffer[1] != 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: touch panel does not sleep\n");
		return 0;
	}

	return 1;
}

/*
 *  This function is used for reply the current status of resume.
 *  return -1: read block error, return 0: touch panel resume fail
 *  return 1: touch panel resume
 */
static int ATMEL_IsResume(struct mxt_data *mxt)
{
	u8 buffer[2] = {0};

	if (mxt_read_block(mxt->client, T07_OBJAddr, 2, buffer) < 0)
		return -1;

	if (buffer[0] == 0 || buffer[1] == 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: touch panel does not resume, still sleep\n");
		return 0;
	}

	return 1;
}

static int ATMEL_Resume(struct mxt_data *mxt)
{
	u8 val[2] = {0};

	val[0] = T07OBJ[0];
	val[1] = T07OBJ[1];

	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL_Resume\n");

	if (mxt_write_block(mxt->client, T07_OBJAddr, 2, val) < 0)
		return -1;

	return 0;
}

static int ATMEL1386E_open(struct inode *inode, struct file *file)
{
	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL1386E_open\n");

	return 0;
}

static int ATMEL1386E_release(struct inode *inode, struct file *file)
{
	mxt_debug(DEBUG_DETAIL, "mXT1386E: ATMEL1386E_release\n");

	return 0;
}

static long ATMEL1386E_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long rc = 0;
	int status = 0;

	switch (cmd) {

		case ATMEL1386E_FirmwareVersion:

			/* Touch event should not be dealt when firmware is updating */
			disable_irq(mxt->touch_irq);
			if (copy_to_user((void*)argp, &Firmware_Info, sizeof(Firmware_Info)))
				return -EFAULT;
			break;

		case ATMEL1386E_CHGPinStatus:

			status = gpio_get_value(TP_INT);
			if (copy_to_user((void*)argp, &status, sizeof(status)))
				return -EFAULT;
			break;

		case ATMEL1386E_T6_REGVALUE:

			if (copy_to_user((void*)argp, &T06_OBJAddr, sizeof(T06_OBJAddr)))
				return -EFAULT;
			break;

		default:

			mxt_debug(DEBUG_ERROR, "invalid command %d\n", _IOC_NR(cmd));
			rc = -EINVAL;
			break;
	}

	return rc;
}

static const struct file_operations ATMEL1386E_fops = {
	.owner				= THIS_MODULE,
	.open				= ATMEL1386E_open,
	.release			= ATMEL1386E_release,
	.unlocked_ioctl		= ATMEL1386E_ioctl,
};

static struct miscdevice ATMEL1386E_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "maXTouch",
	.fops = &ATMEL1386E_fops,
};

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input;
	int error, i;

	mxt_debug(DEBUG_DETAIL, "mXT1386E: mxt_probe\n");

	if (client == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: client == NULL\n");
		return	-EINVAL;
	} else if (client->adapter == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: client->adapter == NULL\n");
		return	-EINVAL;
	} else if (&client->dev == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: client->dev == NULL\n");
		return	-EINVAL;
	} else if (&client->adapter->dev == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: client->adapter->dev == NULL\n");
		return	-EINVAL;
	} else if (id == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: id == NULL\n");
		return	-EINVAL;
	}

	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	input->name = "acer-touch";
	mxt->input  = input;
	mxt->client = client;
	mxt->init_irq = client->irq;
	mxt->touch_irq = client->irq;

	INIT_WORK(&mxt->init_dwork, init_worker);
	INIT_WORK(&mxt->touch_dwork, touch_worker);

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
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, NUM_FINGERS,0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, MXT_MAX_TOUCH_SIZE, 0, 0);

	i2c_set_clientdata(client, mxt);

	error = misc_register(&ATMEL1386E_device);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR," ATMEL1386E_device register failed\n");
		goto err_register_misc;
	}

	error = input_register_device(mxt->input);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: Failed to register input device\n");
		goto err_register_device;
	}

	for(i=0;i<NUM_FINGERS;i++) {
		mxt->PointBuf[i].X = 0;
		mxt->PointBuf[i].Y = 0;
		mxt->PointBuf[i].Status = -1;
	}

	error = ATMEL_Init(mxt);
	if (error < 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Initial failed\n");
		goto err_irq;
	}

	/* SYSFS_START */
	touchdebug_kobj = kobject_create_and_add("Touch", NULL);
	if (touchdebug_kobj == NULL)
		mxt_debug(DEBUG_ERROR, "%s: subsystem_register failed\n", __FUNCTION__);

	error = sysfs_create_group(touchdebug_kobj, &attr_group);
	if (error)
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
	if (mxt->init_irq)
		free_irq(mxt->init_irq, mxt);
	input_unregister_device(mxt->input);
err_register_device:
	misc_deregister(&ATMEL1386E_device);
err_register_misc:
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
		if (mxt->touch_irq)
			free_irq(mxt->touch_irq, mxt);

		cancel_work_sync(&mxt->touch_dwork);
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
	mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_early_suspend\n");

	disable_irq(data->touch_irq);
	cancel_work_sync(&data->touch_dwork);
	/*
	system will still go to suspend if i2c error,
	but it will be blocked if sleep configs are not written to touch successfully
	*/
	if (ATMEL_Deepsleep(data) == 0) {
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Deepsleep OK!\n");
	} else {
		ret = ATMEL_Issleep(data);
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL SUSPEND Fail: %d\n", ret);
	}
}

void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
	int i, ret;
	mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_late_resume\n");

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
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL_Resume OK!\n");
	} else {
		ret = ATMEL_IsResume(data);
		mxt_debug(DEBUG_ERROR, "mXT1386E: ATMEL Resume Fail: %d\n", ret);
	}

	if (ATMEL_Calibrate(data) < 0)
		mxt_debug(DEBUG_ERROR, "mXT1386E: calibration failed\n");

	enable_irq(data->touch_irq);
}
#endif
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
static int mxt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_suspend\n");
	return 0;
}

static int mxt_resume(struct i2c_client *client)
{
	mxt_debug(DEBUG_ERROR, "mXT1386E: mxt_resume\n");
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
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
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

MODULE_DESCRIPTION("Driver for Atmel mxt1386E Touchscreen Controller");
MODULE_LICENSE("GPL");
