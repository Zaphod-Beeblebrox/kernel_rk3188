#ifndef __LCD_LG097X02__
#define __LCD_LG097X02__

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define LVDS_FORMAT      	LVDS_8BIT_1
#define OUT_FACE		OUT_D888_P666
#if (SCREEN_X == 800)
#define DCLK			35000000  //100000000
#else
#define DCLK			52000000
#endif
#define LCDC_ACLK       500000000 //500000000
/* Timing */
#define H_PW			40        //320
#define H_BP			80        //480
#define H_VD			SCREEN_X  //1024
#define H_FP			200        //260

#define V_PW			5        //10
#define V_BP			10        //6
#define V_VD			SCREEN_Y  //768
#define V_FP			20        //16

#define LCD_WIDTH   196// 142  // 202
#define LCD_HEIGHT  147 //106//  152
/* Other */
#define DCLK_POL		0         //1 // 
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#endif
