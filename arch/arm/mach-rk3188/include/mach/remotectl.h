/*$_rbox_$_ modify _$hzb,20120522*/
/*$_rbox_$_ modify _$add this file for rk29 remotectl*/

#ifndef __RKXX_REMOTECTL_H__
#define __RKXX_REMOTECTL_H__
#include <linux/input.h>

/********************************************************************
**                            宏定义                                *
********************************************************************/
#define TIME_BIT0_MIN  625  /*Bit0  1.125ms*/
#define TIME_BIT0_MAX  1625

#define TIME_BIT1_MIN  1650  /*Bit1  2.25ms*/
#define TIME_BIT1_MAX  2650

#define TIME_PRE_MIN   13000   /*4500*/
#define TIME_PRE_MAX   14000   /*5500*/           /*PreLoad 4.5+0.56 = 5.06ms*/

#define TIME_RPT_MIN   98100   /*101000*/
#define TIME_RPT_MAX   98300   /*103000*/         /*Repeat  105-2.81=102.19ms*/  //110-9-2.25-0.56=98.19ms

#define TIME_SEQ_MIN   11200   /*2650*/
#define TIME_SEQ_MAX   11300   /*3000*/           /*sequence  2.25+0.56=2.81ms*/ //11.25ms


/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef enum _RMC_STATE
{
    RMC_IDLE,
    RMC_PRELOAD,
    RMC_USERCODE,
    RMC_GETDATA,
    RMC_SEQUENCE
}eRMC_STATE;


typedef enum _KEY_TYPE
{
    KEY_TYPE_PANEL = 0,
    KEY_TYPE_IR = 1,
    KEY_TYPE_BACK_ADC = 2,
    KEY_TYPE_CNT,
}eKEY_TYPE;

struct RKxx_remotectl_platform_data {
	//struct rkxx_remotectl_button *buttons;
	int nbuttons;
	int rep;
	int gpio;
	int active_low;
	int timer;
	int wakeup;
	void (*set_iomux)(void);
};

struct rkxx_remote_key_table{
    int scanCode;
	int keyCode;		
};

struct rkxx_remotectl_button {	
    int usercode;
    int nbuttons;
    struct rkxx_remote_key_table *key_table;
    struct list_head node;
};

struct rkxx_remotectls {
    unsigned char mode;                  // the remote controler 's mode .
                                         // [0]=0 normal mode                          | [0]=1 custom mode 
                                         // [1]=0 don't get an addrcode in custom mode | [1]=1 get an addrcode in custom mode
                                         // [2]=0 normal                               | [2]=1 New addrcode
    int num;                             // the node's number in irctls_list 
    struct list_head irctls_list;        // list of infrared remote controls which were supported.
    rwlock_t ir_rwlock;                  // spin lock
};

/*
  store the keys which infrared remote controller get.
*/
struct bonovo_rc_key {
    int addrCode;
    int scanCode;
};

struct bonovo_rc_buff {
    int head;
    int tail;
    struct bonovo_rc_key * key_table;
    spinlock_t lock;
};

struct bonovo_rc_wkey {
    int addrCode;
    eKEY_TYPE key_type;
    struct rkxx_remote_key_table key_val;
};

struct bonovo_advance_keys {	
    int usercode;
    int nbuttons;
    eKEY_TYPE ekey_type;
    int accuracy;
    struct list_head node;
    struct rkxx_remote_key_table *key_table;
};

#define BONOVO_BUFF_LEN   10  // the max number of the key_table in struct bonovo_rc_buff

#endif

