/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support multitouch	Wenfs 2010-10-01
 */



#include <linux/i2c.h>
#include <linux/input.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/ft5x06_ts.h>
//#include <asm/jzsoc.h>

static struct i2c_client *this_client;
static int ct_irq_num;

#define CONFIG_FT5X0X_MULTITOUCH (1)
#define MAX_SUPPORT_POINTS (5)
#define EDT_STYLE (0)

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct ft5x0x_ts_data {
	struct i2c_client	client;
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif  //CONFIG_HAS_EARLYSUSPEND
	u16 num_x;
	u16 num_y;

};

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:	

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}


/***********************************************************************************************
Name	:	ft5x0x_read_reg 

Input	:	addr
                     pdata

Output	:	

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	struct i2c_msg msgs[2];
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;

	msgs[0].addr	= this_client->addr;
	msgs[0].flags	= 0;
	msgs[0].len	= 1;
	msgs[0].buf	= buf;
	
	msgs[1].addr	= this_client->addr;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].len	= 1;
	msgs[1].buf	= buf;

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0]; 

	//Modification to add delay between two operations for clock stretching issue. --Shriram
	/*ret = i2c_transfer(this_client->adapter, &msgs[0], 1);
	if (ret < 0)
		pr_err("msg %s i2c read error - write addr error: %d\n", __func__, ret);
	ret = i2c_transfer(this_client->adapter, &msgs[1], 1);
	if (ret < 0)
		pr_err("msg %s i2c read error - write suc, read reg error: %d\n", __func__, ret);
	*pdata = buf[0];*/

	return ret;
  
}


/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void
                     

Output	:	 firmware version 	

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}


#define CONFIG_SUPPORT_FTS_CTP_UPG


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(0xfc,0xaa);
    delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[TSP] Step 1: Reset CTPM test\n");
   
    delay_qt_ms(30);   


    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);
   
    delay_qt_ms(1500);
    printk("[TSP] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
   }

   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#endif


/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(this_client);
#ifdef CONFIG_FT5X0X_MULTITOUCH
	input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(tsdata->input_dev);
#else
	input_report_abs(tsdata->input_dev, ABS_PRESSURE, 0);
#endif
	input_report_key(tsdata->input_dev, BTN_TOUCH, 0);
	input_sync(tsdata->input_dev);
}

//static int ft5x0x_read_data(void *dev_id)
static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(this_client);
	struct ts_event *event = &tsdata->event;
	u8 buf[32] = {0};
#if EDT_STYLE
	int i, type, x, y, id;
#endif
	int ret = -1;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(buf, 31);
#else
	ret = ft5x0x_i2c_rxdata(buf, 7);
#endif
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

#if EDT_STYLE
	memset(buf, 0, sizeof(buf));
	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
                u8 *rdbuf = &buf[i * 4 + 5];
                bool down;

                type = rdbuf[0] >> 6;
                /* ignore Reserved events */
                if (type == TOUCH_EVENT_RESERVED)
                        continue;

                x = ((rdbuf[0] << 8) | rdbuf[1]) & 0x0fff;
                y = ((rdbuf[2] << 8) | rdbuf[3]) & 0x0fff;
                id = (rdbuf[2] >> 4) & 0x0f;
                down = (type != TOUCH_EVENT_UP);

                input_mt_slot(tsdata->input_dev, id);
                input_mt_report_slot_state(tsdata->input_dev, MT_TOOL_FINGER, down);

                if (!down)
                        continue;

                input_report_abs(tsdata->input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(tsdata->input_dev, ABS_MT_POSITION_Y, y);
        }

        input_mt_report_pointer_emulation(tsdata->input_dev, true);
        input_sync(tsdata->input_dev);
#else
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x03;// 0000 0011
	event->touch_point = buf[2] & 0x07;// 000 0111

	if (event->touch_point == 0) {
		ft5x0x_ts_release();
		return 1; 
	}

#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			break;
		default:
			return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
    }
#endif
	event->pressure = 200;
#endif
//	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
//			event->x1, event->y1, event->x2, event->y2);
	//printk("%d (%d, %d), (%d, %d)\n", event->touch_point, event->x1, event->y1, event->x2, event->y2);

	return 0;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(this_client);
	struct ts_event *event = &tsdata->event;
	u8 uVersion;

		//printk("==ft5x0x_report_value =\n");
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch(event->touch_point) {
		case 5:
			input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_X, event->x5);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(tsdata->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(tsdata->input_dev);
			//printk("===x5 = %d,y5 = %d ====\n",event->x2,event->y2);
		case 4:
			input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_X, event->x4);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(tsdata->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(tsdata->input_dev);
			//printk("===x4 = %d,y4 = %d ====\n",event->x2,event->y2);
		case 3:
			input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_X, event->x3);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(tsdata->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(tsdata->input_dev);
			//printk("===x3 = %d,y3 = %d ====\n",event->x2,event->y2);
		case 2:
			input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(tsdata->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(tsdata->input_dev);
			//printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 1:
			input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(tsdata->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(tsdata->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(tsdata->input_dev);
			//printk("===x1 = %d,y1 = %d ====\n",event->x1,event->y1);

		default:
			//printk("==touch_point default =\n");
			break;
	}
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(tsdata->input_dev, ABS_X, event->x1);
		input_report_abs(tsdata->input_dev, ABS_Y, event->y1);
		input_report_abs(tsdata->input_dev, ABS_PRESSURE, event->pressure);
	}
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	input_report_key(tsdata->input_dev, BTN_TOUCH, 1);
	input_sync(tsdata->input_dev);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
}	/*end ft5x0x_report_value*/

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	//printk("==work 1=\n");
	ret = ft5x0x_read_data();	
	if (ret == 0) {	
		ft5x0x_report_value();
	}
	//else printk("data package read error\n");
	//printk("==work 2=\n");
    	msleep(1);
//    enable_irq(this_client->irq);
//	enable_irq(IRQ_EINT(6));
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	int ret;
#if EDT_STYLE
	ret = ft5x0x_read_data();
	if (ret < 0) {
		dev_err(&this_client->dev, "Unable to fetch data\n");
	}

	msleep(1);
#else
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
//    	disable_irq(this_client->irq);		
//	disable_irq(IRQ_EINT(6));
	//printk("==int=");
	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}
#endif
	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
//	struct ft5x0x_ts_data *ts;
//	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk("==ft5x0x_ts_suspend=\n");
//	disable_irq(this_client->irq);
//	disable_irq(IRQ_EINT(6));
//	cancel_work_sync(&ts->pen_event_work);
//	flush_workqueue(ts->ts_workqueue);
	// ==set mode ==, 
//    	ft5x0x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	printk("==ft5x0x_ts_resume=\n");
	// wake the mode
//	__gpio_as_output(GPIO_FT5X0X_WAKE);		
//	__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
//	 msleep(100);
//	__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
//	msleep(100);
//	enable_irq(this_client->irq);
//	enable_irq(IRQ_EINT(6));
}
#endif  //CONFIG_HAS_EARLYSUSPEND

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __devinit ft5x0x_ts_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	const struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input;
	int err = 0;
	unsigned char ver = 0;
	unsigned char uc_reg_value;
	ct_irq_num = 0;
	this_client = client;


	if (!pdata) {
                dev_err(&client->dev, "missing platform data\n");
		goto out;
	}
	
	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	input = input_allocate_device();
	if (!ft5x0x_ts || !input)	{
		err = -ENOMEM;
		goto out;
	}

//#if 0	
	i2c_set_clientdata(client, ft5x0x_ts);

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));//dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
//#endif
	printk(KERN_INFO "ft5x06 waking up\n");
	/* wake pin */
	if ((gpio_request(pdata->wake_pin, "ft5x06_gpio_wake") == 0) &&
		(gpio_direction_output(pdata->wake_pin, 1) == 0)) {
		gpio_export(pdata->wake_pin, 0);
		gpio_set_value(pdata->wake_pin, 1);
	} else
		dev_err(&client->dev,
			"could not obtain gpio for ft5x06_ts wake");

	printk(KERN_INFO "ft5x06 setting IRQ\n");
	/* irq pin */
	if  ((gpio_request(pdata->irq_pin, FT5X0X_NAME) == 0) &&
		(gpio_direction_input(pdata->irq_pin) == 0)) {
		gpio_export(pdata->irq_pin, 0);
	} else
		dev_err(&client->dev,
			"could not obtain ft5x06_ts irq");
//#if 0
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
					IRQF_TRIGGER_FALLING, FT5X0X_NAME,
					ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);
//#endif
	ft5x0x_ts->input_dev = input;

	input->name = FT5X0X_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
#if EDT_STYLE
	//	input_set_drvdata(input, ft5x0x_ts);
	//	i2c_set_clientdata(client, ft5x0x_ts);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, ft5x0x_ts->num_x * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, ft5x0x_ts->num_y * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,
				0, ft5x0x_ts->num_x * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
				0, ft5x0x_ts->num_y * 64 - 1, 0, 0);
	err = input_mt_init_slots(input, MAX_SUPPORT_POINTS);
	if (err) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		goto exit_irq_request_failed;
	}
#else
#if 0
	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					FT5X0X_NAME, ft5x0x_ts);

	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);
#endif
	printk(KERN_INFO "EDT_STYLE is not set\n");

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	set_bit(ABS_MT_POSITION_X, input->absbit);
	set_bit(ABS_MT_POSITION_Y, input->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input->absbit);

	input_set_abs_params(input,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input->absbit);
	set_bit(ABS_Y, input->absbit);
	set_bit(ABS_PRESSURE, input->absbit);
	set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	//Define ABS_X&Y for tslib as it expects that
	set_bit(ABS_X, input->absbit);
	set_bit(ABS_Y, input->absbit);
	set_bit(BTN_TOUCH, input->keybit);

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
#if 0
	err = input_mt_init_slots(input, MAX_SUPPORT_POINTS);
	if (err) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		goto exit_irq_request_failed;
	}
#endif
#endif

	err = input_register_device(input);
	if (err)
		goto exit_input_register_device_failed;
#if 0
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));//dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = request_irq(ct_irq_num, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, FT5X0X_NAME, ft5x0x_ts);//IRQ_EINT(6)
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	//Define ABS_X&Y for tslib as it expects that
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
//	 err = input_mt_init_slots(input_dev, 4);//4 is max support points
//	 if (err) {
//			 dev_err(&client->dev, "Unable to init MT slots.\n");
//			 printk("Unable to init MT Slots.\n");
//			 //goto err_free_mem;
//	 }

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

    msleep(50);
    //get some register information
    uc_reg_value = ft5x0x_read_fw_ver();
    printk("[FST] FT55x0x Firmware version = 0x%x\n", uc_reg_value);
    /*uc_reg_value = ft5x0x_read_reg(FT5X0X_REG_FT5201ID, &ver);
    printk("[FST] FT55x0x FiT5201ID = 0x%x\n", uc_reg_value);
    uc_reg_value = ft5x0x_read_reg(FT5X0X_REG_ERR, &ver);
    printk("[FST] FT55x0x ERR = 0x%x\n", uc_reg_value);*/


//    fts_ctpm_fw_upgrade_with_i_file();

//wake the CTPM
//	__gpio_as_output(GPIO_FT5X0X_WAKE);		
//	__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
//	 msleep(100);
//	__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
//	msleep(100);
//	ft5x0x_set_reg(0x88, 0x05); //5, 6,7,8
//	ft5x0x_set_reg(0x80, 30);
//	msleep(50);
//    enable_irq(IRQ_EINT(6));

//    	enable_irq(ct_irq_num);

	device_init_wakeup(&client->dev, 1);
	enable_irq(client->irq);

	return 0;

exit_input_register_device_failed:
	input_free_device(input);
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	dev_err(&client->dev, "singlethread error\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
out:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	const struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tsdata->early_suspend);
#endif

	free_irq(client->irq, tsdata);
	input_unregister_device(tsdata->input_dev);

	if (gpio_is_valid(pdata->irq_pin))
		gpio_free(pdata->irq_pin);
	if (gpio_is_valid(pdata->wake_pin))
		gpio_free(pdata->wake_pin);

	i2c_set_clientdata(client, NULL);

	kfree(tsdata);
	cancel_work_sync(&tsdata->pen_event_work);
	destroy_workqueue(tsdata->ts_workqueue);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = FT5X0X_NAME,
	},
	.id_table	= ft5x0x_ts_id,
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
};

module_i2c_driver(ft5x0x_ts_driver);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
