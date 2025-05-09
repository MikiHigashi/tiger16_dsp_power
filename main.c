// tiger16 主電源dsPIC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "crc16.h"


#define USE_LCD 0 /* 液晶ディスプレイを使う:1 使わない:0 */


// PWM DIV16
// 0.5ms  2180
// 1.5ms  6542
// 2.5ms 10904

typedef union tagHL16 {
    signed short SHL;
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned :7;
        unsigned S:1;
        unsigned :7;
        unsigned T:1;
    };
} HL16;


typedef union tagHL32 {
    unsigned long HL;
    struct {
        uint8_t L;
        uint16_t M;
        uint8_t H;
    };
} HL32;


uint16_t table_pwm[] = {
	10,
	11,
	12,
	13,
	15,
	16,
	18,
	19,
	21,
	24,
	26,
	29,
	31,
	35,
	38,
	42,
	46,
	51,
	56,
	61,
	67,
	74,
	81,
	90,
	98,
	108,
	119,
	131,
	144,
	159,
	174,
	192,
	211,
	267,
	323,
	379,
	435,
	491,
	546,
	602,
	658,
	714,
	770,
	826,
	882,
	938,
	994,
	1050,
	1106,
	1161,
	1217,
	1273,
	1329,
	1385,
	1441,
	1497,
	1553,
	1609,
	1665,
	1720,
	1776,
	1832,
	1888,
	1944,
	2000,
	2312,
	2625,
	2938,
	3250,
	3562,
	3875,
	4188,
	4500,
	4812,
	5125,
	5438,
	5750,
	6062,
	6375,
	6688,
	7000,
	7312,
	7625,
	7938,
	8250,
	8562,
	8875,
	9188,
	9500,
	9812,
	10125,
	10438,
	10750,
	11062,
	11375,
	11688,
	12000,
	12312,
	12625,
	12938,
	13250,
	13562,
	13875,
	14188,
	14500,
	14812,
	15125,
	15438,
	15750,
	16062,
	16375,
	16688,
	17000,
	17312,
	17625,
	17938,
	18250,
	18562,
	18875,
	19188,
	19500,
	19812,
	20125,
	20438,
	20750,
	21062,
	21375,
	21688,
	21688
};            


// 右キャタの動き量テーブル
// [y+4][4-x]
uint8_t table_right[] = {
// -4
		128,96,64,32,1,1,1,1,1,
// -3
		128,104,80,56,32,32,32,32,32,
// -2
		128,112,96,80,64,64,64,64,64,
// -1
		128,120,112,104,96,96,96,96,96,
// 0
		128,128,128,128,128,128,128,128,128,
// +1
		128,136,144,152,160,160,160,160,160,
// +2
		128,144,160,176,192,192,192,192,192,
// +3
		128,152,176,200,224,224,224,224,224,
// +4
		128,160,192,224,255,255,255,255,255
};

// 左キャタの動き量テーブル
uint8_t table_left[] = {
// -4
		1,1,1,1,1,32,64,96,128,
// -3
		32,32,32,32,32,56,80,104,128,
// -2
		64,64,64,64,64,80,96,112,128,
// -1
		96,96,96,96,96,104,112,120,128,
// 0
		128,128,128,128,128,128,128,128,128,
// +1
		160,160,160,160,160,152,144,136,128,
// +2
		192,192,192,192,192,176,160,144,128,
// +3
		224,224,224,224,224,200,176,152,128,
// +4
		255,255,255,255,255,224,192,160,128
};


uint16_t v0;   // スロットル変換値 固定小数点 0=最大後退 4=ニュートラル 8=最大前進
uint16_t v1;   // エルロン変換値 固定小数点 0=左最大 4=ニュートラル 8=右最大

uint8_t k0;    // 表オフセット値（参照４点中の左下）
uint8_t k1;    // 表オフセット値（参照４点中の右下）
uint8_t k2;    // 表オフセット値（参照４点中の左上）
uint8_t k3;    // 表オフセット値（参照４点中の右上）

// AD変換値 DMAセットされる温度
uint16_t temp = 0;


#define CNT_MODE1 10
#define CNT_MODE2 50

#define TIMEOUT 1000 /* 受信タイムアウト */
#define SPI_BYTES 8 /* SPI受信するデーターのバイト数 */
#define MAX_CNT_ERR 5 /* 連続エラーがこれだけ続くと強制停止 */
#define CNT_TRIG 10 /* トリガーON持続時間（30ミリ秒単位）*/

//#define SPI_BYTES2 8 /* SPI送信するデーターのバイト数 */
uint8_t data[SPI_BYTES-2]; // SPI受信格納先
uint8_t send[SPI_BYTES]; // SPI送信格納先
uint16_t cnt_err = 0; // ERROR 連続回数
//HL16 data_ok; // 正常に受信できた最後のデーター 0:強制停止

// サーボ値
signed long cann1 = 0; // 仰角
signed long turn1 = 0; // 旋回
signed short roll1 = 0; // 左右傾斜補正量
// 走行用モーター値
uint8_t right = 128;  // TMR1 を使う
uint8_t left = 128; // TMR2 を使う


uint8_t rsvt[32]; // 受信バッファー
#define RSVA_BYTES 24 
uint8_t rsv[RSVA_BYTES]; // 正常に受信できたデーターの転送先
char buf[32];
uint16_t ct = 0;    


// 周囲４点の値 k0v k1v k2v k3v より内挿計算
// 内分点は v0 v1 の小数部を使う　２５６分の v0[0] v1[0]
uint8_t cal_move(uint8_t k0v, uint8_t k1v, uint8_t k2v, uint8_t k3v) {
    uint16_t v0s = v0 & 0xff;

    // k0v と k2v の内分点の値を計算し k02v へ格納
    HL16 k02v, K0v16; // 上位バイトが整数部で下位バイトが小数部
    k02v.H = 0;
    if (k2v > k0v) {
        k02v.L = k2v - k0v;
        k02v.HL *= v0s;
        k02v.H += k0v;
    }
    else {
        k02v.L = k0v - k2v;
        k02v.HL *= v0s;
        K0v16.H = k0v;
        K0v16.L = 0;
        k02v.HL = K0v16.HL - k02v.HL;
    }
    
    // k1v と k3v の内分点の値を計算し cal3 へ格納
    HL16 k13v, k1v16; // 上位バイトが整数部で下位バイトが小数部
    k13v.H = 0;
    if (k3v > k1v) {
        k13v.L = k3v - k1v;
        k13v.HL *= v0s;
        k13v.H += k1v;
    }
    else {
        k13v.L = k1v - k3v;
        k13v.HL *= v0s;
        k1v16.H = k1v;
        k1v16.L = 0;
        k13v.HL = k1v16.HL - k13v.HL;
    }

    // k02v と k13v の内分点の値を計算
    uint16_t v1s = v1 & 0xff;
    uint16_t v1sa;
    HL16 ab, ans;
    if (k13v.HL < k02v.HL) {
        ab.HL = k02v.HL - k13v.HL;
        v1sa = v1s * ab.H;
        ab.HL = v1s * ab.L;
        ans.HL = k13v.HL + v1sa + ab.H;
    }
    else {
        ab.HL = k13v.HL - k02v.HL;
        v1sa = v1s * ab.H;
        ab.HL = v1s * ab.L;
        ans.HL = k13v.HL - v1sa - ab.H;
    }

    if (ans.S) { // 四捨五入
        ans.H ++;
    }
    return ans.H;
}


// SPI受信
void int_strb(void) {
    if (SPI_STRB_GetValue() == 0) return;

    uint16_t t;
    uint8_t idx, b, d, m, b2, d2, tmp[SPI_BYTES];
    uint8_t *dp = tmp;
    uint8_t *dp2 = send;
    for (idx=0; idx<SPI_BYTES; idx++) {
        d = 0;
        d2 = send[idx];
        m = 0x80;
        for (b=0; b<8; b++) {
            t = 0;
            while (SPI_CLOCK_GetValue() == 0) {
                if ((t++) > 1000) return;
            } // CLOCK立ち上がりをソフトループで待つ
            if (d2 & m) {
                SPI_DATA_SetHigh();
            }
            else {
                SPI_DATA_SetLow();
            }

            m >>= 1;
            d <<= 1;
            t = 0;
            while (SPI_CLOCK_GetValue() == 1) {
                if ((t++) > 1000) return;
            } // CLOCK立ち下がりをソフトループで待つ
            d |= SPI_IN_GetValue();
        }
        (*(dp++)) = d;
    }

    // CRC16
    HL16 crc;
    uint8_t const *p = (uint8_t const *)tmp;
    crc.HL = crc16(0, p, SPI_BYTES-2);
    if (crc.H != tmp[6]) return;
    if (crc.L != tmp[7]) return;
    for (idx=0; idx<(SPI_BYTES-2); idx++) {
        data[idx] = tmp[idx];
    }

    // 入力スロットル量を 0 to 8 に正規化 4=ニュートラル　整数部8bit+小数部8bit
    v0 = (uint16_t)(data[2]);
    if (v0 == 255) v0 = 256;
    v0 <<= 3;

    // 入力エルロン量を 0 to 8 に正規化 4=ニュートラル　整数部8bit+小数部8bit
    v1 = (uint16_t)(data[3]);
    if (v1 == 255) v1 = 256;
    v1 <<= 3;

    // 入力座標周囲４点の値をキャタ移動速度テーブルのどの位置から引いてくれば良いか計算し、
    // k0 左下 k1 右下 k2 左上 k3 右上　に格納する
    uint8_t w = (uint8_t)((v0 >> 8) * 9 + 8 - (v1 >> 8));
    k1 = k3 = w;
    if (v1 < 0x0800) {
        w--;
    }
    k0 = k2 = w;
    if (v0 < 0x0800) {
        k2 = k0 + 9;
        k3 = k1 + 9;
    }
    // 左キャタの移動速度を計算    
    left = cal_move(table_left[k0], table_left[k1], table_left[k2], table_left[k3]);
    // 右キャタの移動速度を計算
    right = cal_move(table_right[k0], table_right[k1], table_right[k2], table_right[k3]);

    //if (data[0] & 128) { // UPボタンが押されていれば、
    if (v0 == 0x0400) { // スロットルが中立
        uint16_t ww;
        // 超信地旋回
        if (v1 > 0x400) {
            v1 -= 0x400;
            ww = (v1 / 24);
            left = (uint8_t)(128 + ww);
        }
        else {
            v1 = 0x400 - v1;
            ww = (v1 / 24);
            left = (uint8_t)(128 - ww);
        }
        right = (uint8_t)((256 - left) & 0xff);
    }
    
    // タイマー割り込み発生値 26168 0.5ms=4361 1.5ms=13084 2.5ms=21807
    //TMR1 = 21788 - (uint16_t)68 * (uint16_t)left; 
    //TMR2 = 21788 - (uint16_t)68 * (uint16_t)right; 
    TMR1 = 18204 - (uint16_t)40 * (uint16_t)right; 
    TMR2 = 18204 - (uint16_t)40 * (uint16_t)left; 

    TMR1_Start();
    TMR2_Start();
    PWM4_SetHigh();
    PWM5_SetHigh();
//    ct++;
}
void int_timer1(void) {
    PWM4_SetLow();
    TMR1_Stop();
}
void int_timer2(void) {
    PWM5_SetLow();
    TMR2_Stop();
}

uint8_t mode = 0;
uint8_t cnt_push = 0;


void power_off(void)
{
    // 電源OFF
    PDC1 = 0;
    __delay_ms(10);
    DCDC_SetLow();
    PICOUT_SetLow();
    LED2_SetLow();
}


//data[0]
    // bit0  TRR-U
    // bit1  TRR-D
    // bit2  TRR-L
    // bit3  TRR-R
    // bit4  B（LCD周囲のボタン）
    // bit5  A（LCD周囲のボタン）
    // bit6  Down（LCD周囲のボタン）
    // bit7  Up（LCD周囲のボタン）
//data[1]
    // bit0  TRL-U
    // bit1  TRL-D
    // bit2  TRL-L
    // bit3  TRL-R
    // bit4  左トグルスイッチが↓なら1
    // bit5  トリガーが押されたら1
    // bit6  左トリガーが押されたら1
    // bit7  トリガー同時押しで1
//data[2] アクセルスロットル　右十字の上下　上で＋
//data[3] ステア　右十字の左右　右で＋
//data[4] 砲塔旋回　左十字の左右　左で＋
//data[5] 砲身上下　左十字の上下　上で＋

int main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    TMR1_Stop();
    TMR2_Stop();
    PICOUT_SetHigh();
    DCDC_SetHigh(); // DCコンバーター
    LED2_SetHigh(); // 電源LED

    PDC1 = 0;
    
    CN_SetInterruptHandler(int_strb);
    TMR1_SetInterruptHandler(int_timer1);
    TMR2_SetInterruptHandler(int_timer2);
    
    DMA_ChannelEnable(DMA_CHANNEL_1);
    DMA_PeripheralAddressSet(DMA_CHANNEL_1, (volatile unsigned int) &ADC1BUF0);
    DMA_StartAddressASet(DMA_CHANNEL_1, (uint16_t)(&temp));        
    LED1_SetHigh();
    
#if (USE_LCD)
    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);
#endif
    
        PDC1 = 0; //5; // 旋回用ステッピングモーター

    uint8_t light = 0;
    uint8_t in_light, in_light0 = 0;
    
    while (1) {
        WATCHDOG_TimerClear();

        signed short mv = (((signed long)temp * 5000) >> 12);
        signed char t = (signed char)((mv + 5) / 10 - 50);
        send[0] = (uint8_t)t; // 温度を送信
        
         // 前照灯ボッシュライト
        if (data[1] & 12) in_light=1; else in_light=0;
        if (in_light != in_light0) {
            in_light0 = in_light;
            if (in_light) {
                if (light) light=0; else light=1;
            }
        }
        if (light) {
            LIGHT_SetHigh();
        }
        else {
            LIGHT_SetLow();
        }
        // 機銃
        if (data[0] & 128) {
            MGUN_SetHigh();
        }
        else {
            MGUN_SetLow();
        }
        // クラクション
        if (data[0] & 64) {
            BELL_SetHigh();
        }
        else {
            BELL_SetLow();
        }
        
        //LCD_i2C_cmd(0x80);
        //sprintf(buf, "%4d%4d%4d", v1, v2, v3);
        //LCD_i2C_data(buf);
        //LCD_i2C_cmd(0x80);
        //sprintf(buf, "%6d", temp);
        //LCD_i2C_data(buf);

        if (mode == 0) {
            if (PICIN_GetValue()) { // ボタン離れている
                cnt_push ++;
                if (cnt_push >= CNT_MODE1) {
                    cnt_push = 0;            
                    mode = 1;
                }
            }
            else {
                cnt_push = 0;            
            }
        }
        else if (mode == 1) {
            if (PICIN_GetValue()) { // ボタン離れている
                cnt_push = 0;
            }
            else {
                cnt_push ++;
                if (cnt_push >= CNT_MODE2) {
                    mode = 2;
                }
            }
        }
        else {
            power_off();
        }
       
        __delay_ms(5);
    }
    return 1; 
}
        
//        LCD_i2C_cmd(0xC0);
//        sprintf(buf, "%4d%4d%4d%4d", rsv[18], rsv[19], rsv[20], rsv[21]);
//        LCD_i2C_data(buf);

/**
 End of File
*/

