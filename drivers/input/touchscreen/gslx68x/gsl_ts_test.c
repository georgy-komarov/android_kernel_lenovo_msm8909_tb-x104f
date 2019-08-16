//#include "stdafx.h"//
//#include "Operation.h"//
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
//#include "gsl_ts_driver.h" 
#define	SILEAD_TEST_DAC			1
#define	SILEAD_TEST_BASE		2
#define	SILEAD_TEST_RELA		4
#define	CPU_TYPE_NONE			0
#define	CPU_TYPE_ALL			0xffff
#define	CPU_TYPE_1682			0x1682
#define	CPU_TYPE_1688			0x1688
#define	CPU_TYPE_1691			0x1691
#define	CPU_TYPE_168C			0x168c
#define	CPU_TYPE_2680			0x2680
#define	CPU_TYPE_3670			0x3670
#define	CPU_TYPE_9120			0x9120
#define	CPU_TYPE_9121			0x9121
#define	CPU_TYPE_9122			0x9122
#define	CPU_TYPE_9123			0x9123
#define	CPU_TYPE_968			0x968
#define	CPU_TYPE_3692			0x3692
#define	CPU_TYPE_5680			0x5680
#define CPU_TYPE_2133			0x2133
#define	CPU_TYPE_3628			0x3628
#define	CPU_TYPE_3680			0x3680
#define	CPU_TYPE_5642			0x5642


#define DATA_DRV_MAX			72
#define	DATA_SEN_MAX			40
#define	DAC_GROUP_MAX			4
#define ORDER_2133_MAX			30
#define KEY_2133_MAX			8
#define	CONF_PAGE_2680(n)		(0x04 + (n))
#define	UINT unsigned int
#define GATHER_DATA_BASE			1
#define GATHER_DATA_SUB				2
#define GATHER_DATA_REFE			3
#define GATHER_DATA_DAC				4
#define	GATHER_DATA_TEMP			5
#define	GATHER_DATA_OTHER			100

#define	LIST_CORE_SEN		0x00010000
#define LIST_CORE_DRV		0x00010001
#define LIST_CORE_BASE		0x00020000
#define LIST_CORE_BASE2		(LIST_CORE_BASE + 1)
#define LIST_CORE_REFE		(LIST_CORE_BASE + 2)
#define LIST_CORE_SUM		(LIST_CORE_BASE + 3)
#define LIST_CORE_SUB		(LIST_CORE_BASE + 4)
//#define LIST_CORE_STATIC	(LIST_CORE_BASE - 1)
#define LIST_CORE_REFE2		(LIST_CORE_BASE - 1)
#define LIST_CORE_SUM2		(LIST_CORE_BASE - 2)
#define	LIST_CORE_FUNADDR	0x00030000

#define	BASE_ADDR(memory,sen,drv)	((memory)-((drv)+2)*((sen)+2)*4 - (drv)*(sen)*2*4)
#ifndef UINT
#define	UINT unsigned int
#endif
/*
static union
{
    UINT sen_table_int[5];
    unsigned char sen_table[20];
}sen_data;
extern  u8 gsl_cfg_index ;
extern void gsl_I2C_ROnePage(unsigned int addr, char *buf); 
extern void gsl_I2C_RTotal_Address(unsigned int addr,unsigned int *data);
*/
extern UINT ReadReg(UINT addr);
extern UINT ReadMem(UINT addr);
extern void ReadPage(UINT addr, UINT *buf);
//extern UINT WriteMem(UINT addr,UINT data);
 //int gsl_ts_read(struct i2c_client *client, u8 reg, u8 *buf, u32 num);
 //int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num);
 //int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num);
extern   u8   gsl_cfg_index;
#define	CPU_TYPE	CPU_TYPE_5642
enum {
	FALSE = 0,
	TRUE,
};

typedef struct
{
	int kind;
	int sen_start;
	int sen_end;
	int drv_start;
	int drv_end;
	int data_min;
	int data_max;
}GSL_TEST_TYPE;
//struct gsl_ts_data *ts_data = NULL;
void GSL_DacInit(void);
void GSL_DacRead(int (*dac_data_in)[DATA_SEN_MAX]);
int  GSL_GatherInit(void);
//int GSL_GatherData(int read_type, int (*data_in)[DATA_SEN_MAX]);
int  GSL_Test(GSL_TEST_TYPE *test, int len);
static GSL_TEST_TYPE *test_config;
static int config_len;
static  int (*dac_data)[DATA_SEN_MAX];
static int (*data)[DATA_SEN_MAX];
static GSL_TEST_TYPE test_config_1[] =
{
		{ SILEAD_TEST_DAC , -1, -1,  0,  0,   5,   150  },
//sen 
		{ SILEAD_TEST_DAC , -1, -1,  1,  2,   5,   150 },
		//{ SILEAD_TEST_RELA, -1, -1,  0,  0,    0,   60 },
		//{ SILEAD_TEST_RELA, -1, -1,  1,  2,    0,  160 },
		{ SILEAD_TEST_BASE, -1, -1, -1, -1, 1000, 6500 },//3000~5000
	};
static GSL_TEST_TYPE test_config_2[] =
	{
		{ SILEAD_TEST_DAC , -1, -1,  0,  0,   15,   110 },
		{ SILEAD_TEST_DAC , -1, -1,  1,  2,   13,   90 },
		//{ SILEAD_TEST_RELA, -1, -1,  0,  0,    0,   60 },
		//{ SILEAD_TEST_RELA, -1, -1,  1,  2,    0,  160 },
		{ SILEAD_TEST_BASE, -1, -1, -1, -1, 2000, 6500 },//3000~5000
	};
static union
{
	UINT sen_table_int[(DATA_SEN_MAX + 3) / 4];
	unsigned char sen_table[DATA_SEN_MAX];
} table;
static UINT scan_sen[(DATA_SEN_MAX + 3) / 4 / 2];
static UINT scan_num;
static UINT dac_save[DAC_GROUP_MAX][DATA_SEN_MAX];
static int dac_sen_num;
static int dac_able;
static UINT sen[DATA_SEN_MAX];
static UINT sen_num;

static int drv_key;
static int sen_key, sen_scan_num;
static int sen_num_base, drv_num_base;
static int sen_order[DATA_SEN_MAX];
static UINT order_2133[2][ORDER_2133_MAX];
static UINT key_2133[KEY_2133_MAX];
static int dac_num;
static int drv_num_nokey;
static int sen_num_nokey;

//static int(*data)[DATA_SEN_MAX];
//static int(*dac_data)[DATA_SEN_MAX];

static void GetSenOrder5680(UINT sen[])
{
	UINT i, j, k;
	UINT scan[(DATA_SEN_MAX + 1) / 2];
	UINT base[DATA_SEN_MAX];
	for (i = 0; i<sizeof(scan) / sizeof(scan[0]); i++)
	{
		scan[i] = (scan_sen[sizeof(scan_sen) / sizeof(scan_sen[0]) - 1 - i / 4] >> (
3 - (i & 3)) * 8) & 0xff;
	}
	for (i = 0; i<scan_num&&i<DATA_SEN_MAX; i++)
	{
		if (i < scan_num / 2)
			base[i] = scan[i] * 2;
		else
			base[i] = scan[i - scan_num / 2] * 2 + 1;
	}
	for (i = 0; i<sen_num && i<DATA_SEN_MAX; i++)//get io
		sen[i] = base[table.sen_table[i ^ 3] ^ 1];
	for (; i<DATA_SEN_MAX; i++)
		sen[i] = 0;
	for (i = 0; i<DATA_SEN_MAX; i++)
	if (sen[i] >= DATA_SEN_MAX)
		sen[i] = 0;
	for (j = 1; j<DATA_SEN_MAX; j++)
	{
		for (i = 0; i<j; i++)
		{
			if (sen[j] == sen[i])
				break;
		}
		if (i >= j)
			continue;
		for (i = 0; i<DATA_SEN_MAX; i++)
		{
			for (k = 0; k<DATA_SEN_MAX; k++)
			{
				if (sen[k] == i)
					break;
			}
			if (k >= DATA_SEN_MAX)
				break;
		}
		if (i<DATA_SEN_MAX)
			sen[j] = i;
	}
}

static void GetSenOrder2680(UINT sen[])
{
	UINT i, j, k;
	UINT scan[12];
	UINT base[24];
	for (i = 0; i<sizeof(scan) / sizeof(scan[0]); i++)
	{
		if (i < 6)
			scan[i] = (scan_sen[0] >> (5 - i) * 4) & 0xf;
		else
			scan[i] = (scan_sen[1] >> (11 - i) * 4) & 0xf;
	}
	for (i = 0; i<scan_num&&i<24; i++)
	{
		if (i < scan_num / 2)
			base[i] = scan[i] * 2;
		else
			base[i] = scan[i - scan_num / 2] * 2 + 1;
	}
	for (i = 0; i<sen_num && i<24; i++)
		sen[i] = base[table.sen_table[i ^ 3] ^ 1];
	for (; i<24; i++)
		sen[i] = 0;
	for (i = 0; i<24; i++)
	if (sen[i] >= 24)
		sen[i] = 0;
	for (j = 1; j<24; j++)
	{
		for (i = 0; i<j; i++)
		{
			if (sen[j] == sen[i])
				break;
		}
		if (i >= j)
			continue;
		for (i = 0; i<24; i++)
		{
			for (k = 0; k<24; k++)
			{
				if (sen[k] == i)
					break;
			}
			if (k >= 24)
				break;
		}
		if (i<24)
			sen[j] = i;
	}
}

static void GetSenOrder2133(UINT sen[])
{
	int i;
	for (i = 0; i<6; i++)
	{
		sen[i] = i * 2;
		sen[i + 6] = i * 2 + 1;
	}
}

static void DataInit(void)
{
	int i,j;
	for (i = 0; i<DATA_SEN_MAX; i++)
		sen[i] = i;
	dac_sen_num = 10;
	dac_able = 1;
	for(j=0;j<DAC_GROUP_MAX;j++)
	{
	  for (i = 0; i<DATA_SEN_MAX; i++)		 
		dac_save[j][i] = 0;
	}
}

static int  InitSenData(void)
{
	UINT i;
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		sen_num = 12;
		scan_num = 12;
		scan_sen[0] = 0x12345;
		scan_sen[1] = 0x12345;
		for (i = 0; i<6; i++)
			table.sen_table_int[i] = 0;
	}
	else if (CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
	{
		for (i = 0; i<5; i++)
			scan_sen[i] = ReadMem(0xff080008 + i * 4);
		scan_num = ReadMem(CONF_PAGE_2680(4) * 0x80 + 0x7c);
		sen_num = ReadMem(CONF_PAGE_2680(1) * 0x80 + 0x7c);
		for (i = 0; i<10; i++)
			table.sen_table_int[i] = ReadMem(0x1d*0x80 + i * 4);
		return TRUE;
	}
	else
	{
		if ((ReadReg(0xfc) & 0xf0ff0000) == 0xb0820000)
		{
			scan_sen[0] = ReadMem(0xff080008);
			scan_sen[1] = ReadMem(0xff080008);
		}
		else
		{
			scan_sen[0] = ReadMem(0x7*0x80+ 0x78);//ReadMem(0xff08000c);
			scan_sen[1] = ReadMem(0x7*0x80+ 0x7c);//ReadMem(0xff080008);
		}
		scan_num = ReadMem(CONF_PAGE_2680(4)*0x80+ 0x7c);
		sen_num = ReadMem(CONF_PAGE_2680(1)*0x80+ 0x7c);
		for (i = 0; i<6; i++)
			table.sen_table_int[i] = ReadMem(CONF_PAGE_2680(3) * 0x80 + i * 4);
		return TRUE;
	}
	return TRUE;
}

void GSL_DacInit(void)
{
	DataInit();
	InitSenData();
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		dac_able = ReadMem(CONF_PAGE_2680(3)*0x80+ 0x54);
		GetSenOrder2133(sen);
	}
	else if (CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
	{
		printk("GSL_DacInit  5680\n");
		dac_able = 3;
		GetSenOrder5680(sen);
	}
	else
	{
		dac_able = ReadMem(CONF_PAGE_2680(3)*0x80+ 0x54);
		GetSenOrder2680(sen);
	}
	dac_sen_num = ReadMem(CONF_PAGE_2680(1)*0x80+ 0x7c);
	if (dac_able < 1 || dac_able > 4)
		dac_able = 1;
	if (dac_sen_num < 1 || dac_sen_num > DATA_SEN_MAX)
		dac_sen_num = DATA_SEN_MAX;
}

void GSL_DacRead(int (*dac_data_in)[DATA_SEN_MAX])
{
	int i, j, i2;
union
{
	UINT data_int[32];
	unsigned char data_char[128];
}data;
	dac_data= dac_data_in;
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		ReadPage(0xb,data.data_int);
		for (i = 0; i<0x80 - 0x30; i++)
			dac_save[i / 24][i % 24] = (data.data_char[0x30 + i] >> 2) + ((data.data_char[0x30 + i] & 0x3) << 6);//data_char[0x30+i]
		ReadPage(0xc,data.data_int);
		for (i=0; i<6 * 4 * 4; i++)
			dac_save[i / 24][i % 24] = (data.data_char[0x30 + i - 0x80] >> 2) + ((data.data_char[0x30 + i - 0x80] & 0x3) << 6);
	}
	else if (CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
	{
		ReadPage(0xff080100 / 0x80,data.data_int);
		for (i = 0; i<DATA_DRV_MAX; i++)
			dac_save[1+i/DATA_SEN_MAX][i%DATA_SEN_MAX] = data.data_char[i];
		for (i = 0; i<DATA_SEN_MAX; i++)
			dac_save[0][i] = data.data_char[(i + DATA_DRV_MAX)];
	}
	else
	{
		ReadPage(0xb,data.data_int);
		for (i = 0; i<5 * 4 * 4; i++)
			dac_save[i / 20][i % 20] = data.data_char[0x30 + i];
	}
	if (CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
	{
		for (j = 0; j<1; j++)
		{
			for (i = 0; i<DATA_SEN_MAX; i++)
			{
				i2 = sen[i];
				dac_data[j][i] = dac_save[j][i2];
				if (!(j<dac_able && i<dac_sen_num))
					dac_data[j][i] = 0;
			}
		}
		for (j = 1; j<3; j++)
		{
			for (i = 0; i<DATA_SEN_MAX; i++)
			{
				dac_data[j][i] = dac_save[j][i];
				if (j >= dac_able)
					dac_data[j][i] = 0;
			}
		}
	}
	else
	{
		for (j = 0; j<4; j++)
		{
			for (i = 0; i<DATA_SEN_MAX; i++)
			{
				i2 = sen[i];
				dac_data[j][i] = dac_save[j][i2];
				if (!(j<dac_able && i<dac_sen_num))
					dac_data[j][i] = 0;
			}
		}
	}
}

int GSL_GatherInit(void)
{
	UINT ret = TRUE;
	UINT i;
	union
	{
		UINT data_int[(DATA_SEN_MAX + 3) / 4];
		unsigned char data_char[DATA_SEN_MAX];
	} data;

	for (i = 0; i<ORDER_2133_MAX; i++)
	{
		order_2133[0][i] = 0;
		order_2133[1][i] = 0;
	}
	for (i = 0; i<KEY_2133_MAX; i++)
		key_2133[i] = 0;

	dac_num = ReadMem(CONF_PAGE_2680(3) * 0x80 + 0x54);
	drv_num_nokey = ReadMem(CONF_PAGE_2680(2) * 0x80 + 0x00);
	sen_num_nokey = ReadMem(CONF_PAGE_2680(2) * 0x80 + 0x08);
	drv_key = ReadMem(CONF_PAGE_2680(2) * 0x80 + 0x04);
	sen_key = ReadMem(CONF_PAGE_2680(2) * 0x80 + 0x7c);
	sen_scan_num = ReadMem(CONF_PAGE_2680(4) * 0x80 + 0x7c);
	sen_num_base = ReadMem(CONF_PAGE_2680(1) * 0x80 + 0x7c);
	drv_num_base = ReadMem(CONF_PAGE_2680(1) * 0x80 + 0x78);
	if (CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
	{
		for (i = 0; i<10; i++)
			data.data_int[i] = ReadMem(0x1d * 0x80 + i * 4);
		for (i = 0; i<40; i++)
			sen_order[i] = data.data_char[i ^ 3] ^ 1;
	}
	else
	{
		for (i = 0; i<6; i++)
			data.data_int[i] = ReadMem(CONF_PAGE_2680(3) * 0x80 + i * 4);
		for (i = 0; i<24; i++)
			sen_order[i] = data.data_char[i ^ 3] ^ 1;
	}
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		UINT t;
		for (i = 0; i<(UINT)(drv_num_nokey + 3) / 4 && i<ORDER_2133_MAX / 4; i++)
		{
			t = ReadMem(0x400 + i * 4);
			order_2133[0][i * 4 + 0] = ((char)((t & 0xff000000) >> 24)) ^ 0x1;
			order_2133[0][i * 4 + 1] = ((char)((t & 0x00ff0000) >> 16)) ^ 0x1;
			order_2133[0][i * 4 + 2] = ((char)((t & 0x0000ff00) >> 8)) ^ 0x1;
			order_2133[0][i * 4 + 3] = ((char)((t & 0x000000ff) >> 0)) ^ 0x1;
		}
		for (i = 0; i<(UINT)(drv_num_nokey + 3) / 4 && i<ORDER_2133_MAX / 4; i++)
		{
			t = ReadMem(0x400 + i * 4 + 0x20);
			order_2133[1][i * 4 + 0] = ((char)((t & 0xff000000) >> 24)) ^ 0x1;
			order_2133[1][i * 4 + 1] = ((char)((t & 0x00ff0000) >> 16)) ^ 0x1;
			order_2133[1][i * 4 + 2] = ((char)((t & 0x0000ff00) >> 8)) ^ 0x1;
			order_2133[1][i * 4 + 3] = ((char)((t & 0x000000ff) >> 0)) ^ 0x1;
		}
		for (i = 0; i<(UINT)(drv_key + 3) / 4 && i<KEY_2133_MAX / 4; i++)
		{
			t = ReadMem(0x7 * 0x80 + i * 4);
			key_2133[i * 4 + 0] = ((char)((t & 0xff000000) >> 24)) ^ 0x1;
			key_2133[i * 4 + 1] = ((char)((t & 0x00ff0000) >> 16)) ^ 0x1;
			key_2133[i * 4 + 2] = ((char)((t & 0x0000ff00) >> 8)) ^ 0x1;
			key_2133[i * 4 + 3] = ((char)((t & 0x000000ff) >> 0)) ^ 0x1;
		}
	}
	if (sen_key < 0 || sen_key > DATA_SEN_MAX)
		sen_key = 0;
	if (drv_key < 0 || drv_key > DATA_DRV_MAX)
		drv_key = 0;
	for (i = 0; i<DATA_SEN_MAX; i++)
	if (sen_order[i] < 0 || sen_order[i] > DATA_SEN_MAX)
		sen_order[i] = 0;
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		for (i = 0; i<(UINT)drv_num_nokey && i<ORDER_2133_MAX; i++)
		{
			if (order_2133[0][i] > 12 * 4)
				ret = FALSE;
			if (order_2133[1][i] > 12 * 4)
				ret = FALSE;
		}
		for (i = 0; i<(UINT)drv_key && i<KEY_2133_MAX; i++)
		if (key_2133[i] > 12 * 4)
			ret = FALSE;
	}
	if (ret != TRUE
		|| drv_num_nokey < 0 || (drv_num_nokey + drv_key > DATA_DRV_MAX && CPU_TYPE 
!= 0x2133)
		|| sen_num_nokey < 0 || (sen_num_nokey + sen_key > DATA_SEN_MAX && CPU_TYPE 
!= 0x2133)
		|| sen_scan_num <= 0 || sen_scan_num > DATA_SEN_MAX
		|| dac_num < 0 || dac_num > 4)
	{
		drv_num_nokey = DATA_DRV_MAX;
		sen_num_nokey = DATA_SEN_MAX;
		drv_key = 0;
		sen_key = 0;
		sen_scan_num = DATA_SEN_MAX;
		for (i = 0; i<DATA_SEN_MAX; i++)
			sen_order[i] = i;
		sen_num_base = sen_num_nokey + sen_key;
		drv_num_base = drv_num_nokey + drv_key;
		for (i = 0; i<ORDER_2133_MAX; i++)
		{
			order_2133[0][i] = 0;
			order_2133[1][i] = 0;
		}
		for (i = 0; i<KEY_2133_MAX; i++)
			key_2133[i] = 0;
	}
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		if (sen_num_nokey > 24 || sen_key != 0 || drv_num_nokey > 21 || drv_key > 42)
		{
			sen_num_nokey = 2;
			drv_num_nokey = 12;
			drv_key = 0;
			dac_num = 3;
		}
		drv_num_base = dac_num;
		sen_key = 0;
		sen_scan_num = 12;
		for (i = 0; i<DATA_SEN_MAX; i++)
			sen_order[i] = i;
		sen_num_base = 12;

	}
	if (ret == TRUE)
		return TRUE;
	else
		return FALSE;
}

static int GetSenAddr(int n)
{
	int i, t;
	for (i = 0; i<DATA_SEN_MAX; i++)
	{
		t = sen_order[i];
		if (t == n)
			break;
	}
	return i;
}

static void ReadDataSet(int start_address, int n_sen, int n_drv)
{
	int i, t, s, read_page;
	union
	{
		UINT read_data[32 * 2];
		UINT read_int[32];
	}read;
	read_page = 0;
	ReadPage(start_address / 0x80 + read_page, read.read_int);
	while (1)
	{
		for (i = 0; i<32 * 2; i++)
		{
			t = ((read_page + start_address / 0x80) * 0x80 - start_address + i * 2);
			if (t < 0)
				continue;
			if (t >= n_drv * n_sen * 2)
				break;
			t /= 2;
			s = t%n_sen;
			data[t / n_sen][s] = read.read_data[i];
		}
		if (i < 32 * 2)
			break;
		else
			read_page++;
		ReadPage(start_address / 0x80 + read_page, read.read_int);
	}

}

static void ReadDataShort(int start_address)
{
	int i, t, s, read_page;
	union
	{
		UINT read_data[32 * 2];
		UINT read_int[32];
	}read;
	for (t = 0; t<DATA_DRV_MAX; t++)
		for (i = 0; i<DATA_SEN_MAX; i++)
			data[t][i] = 0;//0x1234;
	read_page = 0;
	printk("ReadDataShort enter\n");
	ReadPage(start_address / 0x80 + read_page, read.read_int);
	printk("ReadDataShort enter111\n");
	while (1)
	{
		for (i = 0; i<32 * 2; i++)
		{
			t = ((read_page + start_address / 0x80) * 0x80 - start_address + i * 2);
			if (t < 0)
				continue;
			if (t >= drv_num_base * sen_scan_num * 2)
				break;
			t /= 2;
			s = GetSenAddr(t%sen_scan_num);
			//		s = t%sen_scan_num;
			if (s >= sen_num_base)
				continue;
			data[t / sen_scan_num][s] = read.read_data[i];
		}
		if (i < 32 * 2)
			break;
		else
			read_page++;
		ReadPage(start_address / 0x80 + read_page, read.read_int);
	}
	printk("ReadDataShort quit\n");

}

static UINT List(int core, int type)
{
#define	CORE_TYPE_MIN	0X100
	const struct
	{
		int core;
		int sen;
		int drv;
		int base;
		int fun_addr;
	}core_data[] =
	{
		{ CPU_TYPE_1682, 12, 23, 0x59d0, 0x1e, },
		{ CPU_TYPE_1688, 10, 16, 0x5f80, 0x1e, },
		{ CPU_TYPE_1691, 12, 18, BASE_ADDR(26 * 1024, 12, 18), 0x1e, },
		{ CPU_TYPE_168C, 12, 23, BASE_ADDR(30 * 1024, 12, 23), 0x1a, },
		{ CPU_TYPE_2680, 20, 31, 0x6100, 0x1e, },
		{ CPU_TYPE_3670, 14, 26, 0x5580, 0x1e, },
		{ CPU_TYPE_9120, 12, 20, 0x5B80, 0x1e, },
		{ CPU_TYPE_9121, 12, 21, 0x5B00, 0x1e, },
		{ CPU_TYPE_9122, 12, 22, 0x5A80, 0x1e, },
		{ CPU_TYPE_9123, 12, 23, BASE_ADDR(26 * 1024, 12, 23), 0x1e, },
		{ CPU_TYPE_968, 10, 18, 0x5ea0, 0x1e, },
		{ CPU_TYPE_3692, 24, 32, 0x5a00, 0x1e, },
		{ CPU_TYPE_3628, 18, 28, 0x66e0, 0x1e, },
		{ CPU_TYPE_3680, 20, 31, BASE_ADDR(32 * 1024, 20, 31), 0x1e, },
		{ CPU_TYPE_5680, 40, 72, 0x7570, 0x1e, },
		{ CPU_TYPE_5642, 30, 42, 0xC2A0, 0x1e, },
		{ CPU_TYPE_2133, 12, 23, 0x59d0, 0x1e, },
	};
	if (core >= sizeof(core_data) / sizeof(core_data[0]))
	{
		int i;
		for (i = 0; i<sizeof(core_data) / sizeof(core_data[0]); i++)
		{
			if (core == core_data[i].core)
			{
				core = i;
				break;
			}
		}
		if (core >= sizeof(core_data) / sizeof(core_data[0]))
			return 0;
	}
	if (core<0 || core >= sizeof(core_data) / sizeof(core_data[0]))
		return 0;
	if (type<0)
		return core_data[core].core;
	else if (type == LIST_CORE_SEN)
	{
		return core_data[core].sen;
	}
	else if (type == LIST_CORE_DRV)
	{
		return core_data[core].drv;
	}
	else if (type >= LIST_CORE_BASE - 0x10 && type <= LIST_CORE_BASE + 0x10)
	{
		return core_data[core].base - (LIST_CORE_BASE - type)*core_data[core].sen*core_data[core].drv * 2;
	}
	else if (type == LIST_CORE_FUNADDR)
	{
		return core_data[core].fun_addr;
	}
	return 0;
}

void  GSL_GatherData(int read_type, int (*data_in)[DATA_SEN_MAX])
{
	int addr;
	data = data_in;
	if (CPU_TYPE == CPU_TYPE_2133)
	{
		printk("1111 \n");
		if (read_type == GATHER_DATA_BASE)
			ReadDataSet(0x64E8, 12, dac_num);
		else if (read_type == GATHER_DATA_REFE)
			ReadDataSet(0x6668, 12, dac_num);
		else if (read_type == GATHER_DATA_SUB)
			;//ReadDataSelf(0x6728, drv_num_nokey, 0x67c0, drv_key);
		else if (read_type == GATHER_DATA_OTHER)
		{
			int i, j;
			int dt[4][12];
			ReadDataSet(0x6668, 12, dac_num);//0x66c8
			for (j = 0; j < 4; j++)
			for (i = 0; i < 12; i++)
				dt[j][i] = data[j][i];
			ReadDataSet(0x64E8, 12, 4 * 4);// + dac_num*12*2
			for (j = 0; j < 4 * 4; j++)
			{
				if ((j & 3) < dac_num)
				{
					for (i = 0; i < 12; i++)
						data[j][i] = dt[j & 3][i] - data[j][i];

				}
				else
				{
					for (i = 0; i < 12; i++)
						data[j][i] = 0;
				}
			}
		}
	}
	else
	{
		addr = List(CPU_TYPE, LIST_CORE_BASE);
		printk("gsl_dac_print %d\n",addr);
		if (read_type == GATHER_DATA_BASE){
			printk("2222 \n");
			ReadDataShort(addr);
		}
		else if (read_type == GATHER_DATA_REFE){
			printk("3333 \n");
			ReadDataShort(addr + List(CPU_TYPE, LIST_CORE_SEN)*List(CPU_TYPE, 
		
LIST_CORE_DRV) * 2 * 2);
			}
		//else if (read_type == GATHER_DATA_SUB)
			// ReadDataInt(addr + List(CPU_TYPE, LIST_CORE_SEN)*List(CPU_TYPE, 
//LIST_CORE_DRV) * 2 * 4, List(CPU_TYPE, LIST_CORE_DRV) + 2);
	}
	//return 0;
}

int  GSL_Test(GSL_TEST_TYPE *test, int len)
{
	int i, j;
	int k;
	int ret = 0;
	int test_max,test_min;
	for (k = 0; k < len; k++)
	{
		if (test[k].kind == SILEAD_TEST_DAC || test[k].kind == SILEAD_TEST_RELA)
		{
			if (CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
			{
				if (test[k].drv_start == 0 && test[k].drv_end == 0 && (test[k].sen_start 
< 0 || test[k].sen_end < 0))
				{
					test[k].sen_start = 0;
					test[k].sen_end = sen_num_base - 1;
				}
				if (test[k].drv_start == 1 && test[k].drv_end == 2 && (test[k].sen_start 
< 0 || test[k].sen_end < 0))
				{
					test[k].sen_start = 0;
					test[k].sen_end = DATA_SEN_MAX - 1;
				}
			}
			if (test[k].sen_start < 0 || test[k].sen_end < 0)
			{
				test[k].sen_start = 0;
				test[k].sen_end = sen_num_base - 1;
			}
			if (test[k].drv_start < 0 || test[k].drv_end < 0)
			{
				test[k].drv_start = 0;
				test[k].drv_end = dac_able-1;
			}
		}
		if (test[k].kind == SILEAD_TEST_BASE)
		{
			if (test[k].sen_start < 0 || test[k].sen_end < 0)
			{
				test[k].sen_start = 0;
				test[k].sen_end = sen_num_base - 1;
			}
			if (test[k].drv_start < 0 || test[k].drv_end < 0)
			{
				test[k].drv_start = 0;
				test[k].drv_end = drv_num_base-1;
			}
		}

		if (test[k].kind == SILEAD_TEST_DAC)
		{
			for (j = test[k].drv_start; j <= test[k].drv_end; j++)
			{
				for (i = test[k].sen_start; i <= test[k].sen_end; i++)
				{
					if ((CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642) && (j - 1)*DATA_SEN_MAX + i >= drv_num_base)
						continue;
					if (dac_data[j][i] > test[k].data_max || dac_data[j][i] < test[k].data_min)
					{
						printk("%d %d %d",j,i,dac_data[j][i]);
						ret |= SILEAD_TEST_DAC;
					}
				}
			}
		}
		if (test[k].kind == SILEAD_TEST_BASE)
		{
			for (j = test[k].drv_start; j <= test[k].drv_end; j++)
			{
				for (i = test[k].sen_start; i <= test[k].sen_end; i++)
				{
					if (data[j][i] > test[k].data_max || data[j][i] < test[k].data_min)
					{
						printk("%d %d %d",j,i,data[j][i]);
						ret |= SILEAD_TEST_BASE;
					}
				}
			}
		}
		if (test[k].kind == SILEAD_TEST_RELA)
		{
			test_max = 0;
			test_min = 0x7fffffff;
			for (j = test[k].drv_start; j <= test[k].drv_end; j++)
			{
				for (i = test[k].sen_start; i <= test[k].sen_end; i++)
				{
					if ((CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642) && (j - 1)*DATA_SEN_MAX + i >= drv_num_base)
						continue;
					if (dac_data[j][i] > test_max)
						test_max = dac_data[j][i];
					if (dac_data[j][i] < test_min)
						test_min = dac_data[j][i];
				}
			}
			if (test_max * 100 > test_min*(100 + test[k].data_max))
				ret |= SILEAD_TEST_RELA;
		}
	}
	return ret;
}
void  gsl_dac_print(void)
{
	int i, j;
	printk("gsl_dac_print\n");
	if(CPU_TYPE == CPU_TYPE_5680 || CPU_TYPE == CPU_TYPE_5642)
	{
		for (i = 0; i < sen_num_base; i++)
			printk("%5d",dac_data[0][i]);
		printk("\n");
		for (i = 0; i < drv_num_base; i++)
		{
			printk("%5d",dac_data[1+i/DATA_SEN_MAX][i%DATA_SEN_MAX]);
			if(i == sen_num_base-1)
				printk("\n");
		}
		printk("\n");
	}
	else
	{
		for (j = 0; j < dac_able; j++)
		{
			for (i = 0; i < sen_num_base; i++)
			{
				printk("%5d",dac_data[j][i]);
			}
			printk("\n");
		}
	}
}

void  gsl_original_print(void)
{
	int i, j;
	printk("gsl_original_print\n");
	for (j = 0; j < drv_num_base; j++)
	{
		for (i = 0; i < sen_num_base; i++)
		{
			printk("%5d",data[j][i]);
		}
		printk("\n");
	}
}
static   void RuleInit(void)
{
	if(gsl_cfg_index == 0)
	{
		test_config = test_config_1;
		config_len = sizeof(test_config_1) / sizeof(test_config_1[0]);
	}
	else
	{
		test_config = test_config_2;
		config_len = sizeof(test_config_2) / sizeof(test_config_2[0]);
	}
	
}
int gsl_tp_module_test(void)
{
	
	int ret = 0;
	int (*dac_data)[DATA_SEN_MAX];
	int (*data)[DATA_SEN_MAX];
	dac_data = (int(*)[DATA_SEN_MAX])kzalloc(DAC_GROUP_MAX*DATA_SEN_MAX*4,GFP_KERNEL);
	data = (int(*)[DATA_SEN_MAX])kzalloc(DATA_SEN_MAX*DATA_DRV_MAX*4,GFP_KERNEL);
	//WriteMem(0x508,0x5a);//TP_CPU
	//msleep(50);
	RuleInit();
	printk("GSL_DacInit\n");
	GSL_DacInit();
	printk("GSL_DacRead\n");
	GSL_DacRead(dac_data);
	printk("GSL_GatherInit\n");
	GSL_GatherInit();
	printk("GSL_GatherData\n");
	GSL_GatherData(GATHER_DATA_BASE, data);
	printk("GSL_Test\n");
	ret = GSL_Test(test_config, config_len);
	//WriteMem(0x508,0x00);
	gsl_dac_print();
	gsl_original_print();
	kfree(dac_data);
	kfree(data);

	return ret;
}
