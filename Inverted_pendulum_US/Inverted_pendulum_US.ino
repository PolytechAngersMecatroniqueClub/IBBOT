// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
//#include <Time.h>
#include "TimerOne.h"                    //Ajout Frk
#include "mcp_can.h"
#include "pid.h"

#define IDMOT 0x30

typedef union _data {
  float f;
  char  c[4];
} UData;

#define ID_IMU   0x90
#define ID_US_AV  0xC0                    //Ajout Frk
#define ID_US_AR  0xC1                    //Ajout Frk
#define INPUT_ID 0X77
#define ID_RESET 0x20
#define ID_MOVE 0x21

#define Default_Kang_p  0x0600 //0x0500 //0x03e8    //1 jeu trouvé P:0x3e8 I:0x01 D:0x100
#define Default_Kang_i  0x0001
#define Default_Kang_d  0x0100 //0x0150

#define Default_Kvit_p  0x0004 //0x03e8    //1 jeu trouvé P:0x3e8 I:0x01 D:0x100
#define Default_Kvit_i  0x0000
#define Default_Kvit_d  0x0004 //0x0100

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
int flag = 0;
UData udata;
Pid pid_ang(Default_Kang_p,Default_Kang_i,Default_Kang_d);
Pid pid_vit(Default_Kvit_p,Default_Kvit_i,Default_Kvit_d);

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

unsigned char len_rcv = 0;
unsigned char len_snd = 0;
unsigned char buf_rcv[8]; 
unsigned char buf_snd[8]; 

int16_t speed_right = 0x0000;
int16_t speed_left = 0x0000;
int32_t sum_speed = 0;

float pitch; 
float target_ang=0;
int16_t target_vit=0;
float ponderateur=7;//14.6;//3.65;

void prepare_buf_snd();
unsigned long time_new = 0;
unsigned long time_old_ang = 0;
unsigned long time_old_vit = 0;
unsigned long time_old_us = 0;

int init_flag = 0;
int init_max = 20;
char enable = 0x00;
float correction_vit=0; 

#define ID_MOTOR_BOARD 0x3A //ms

int16_t vitesse_codeur=0;

void loop()
{
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len_rcv, buf_rcv);    // read data,  len: data length, buf: data buf
        unsigned char canId = CAN.getCanId();
        if(canId == ID_RESET){
            init_flag = 0;
            pid_ang.reset();
            pid_vit.reset();
            enable = buf_rcv[0];
            if(enable == 0x00){
              speed_right = 0x0000;
              speed_left = 0x0000;
              prepare_buf_snd();
              CAN.sendMsgBuf(IDMOT, 0, 4, buf_snd);
            }
            
        }
        else if(canId == ID_IMU && enable){
           if(buf_rcv[0] == 1) {
             if(buf_rcv[1] != 90 && buf_rcv[2] != 125 && buf_rcv[3] != 230 && buf_rcv[4] != 242){
               for(int i=0; i<4; i++) {
                 udata.c[i] = buf_rcv[1+i];
               }
               if(init_flag < init_max){
                  target_ang += udata.f; 
                  init_flag++;
               }else if(init_flag == init_max){
                   target_ang = target_ang/init_max;
                   init_flag++;
                   target_ang = 0;
               }
             }
           }  
        }else if(canId == ID_MOTOR_BOARD){
            vitesse_codeur = buf_rcv[0] + buf_rcv[1]*255;
        }
        else if(canId == ID_US_AV && len_rcv==2){                       //Ajout Frk
            int16_t Dst_val = (buf_rcv[0]*256 + buf_rcv[1]);             //Ajout Frk
            if((Dst_val < 150) && (Dst_val > 10)) target_vit -= 0x03FF;               //Ajout Frk
             else if (target_vit < 0) target_vit =0;
             //Serial.print("Avant:"); Serial.println(Dst_val);
        }else if(canId == ID_US_AR && len_rcv==2){                       //Ajout Frk
            int16_t Dst_val = (buf_rcv[0]*256 + buf_rcv[1]);             //Ajout Frk
            if((Dst_val < 150) && (Dst_val > 10)) target_vit += 0x03FF;
            else if (target_vit > 0) target_vit = 0;
            //Serial.print("Arriere:"); Serial.println(Dst_val);
        }
    }
    if(init_flag > init_max && enable){
        time_new = millis();

        speed_left = speed_right =  -pid_ang.update((int16_t)(target_ang*1000), (int16_t)(udata.f*1000));

        if(time_new - time_old_ang > 30){
            time_old_ang = time_new;
            prepare_buf_snd();
            CAN.sendMsgBuf(IDMOT, 0, 4, buf_snd);
        }

        if(time_new - time_old_vit > 100){
            time_old_vit = time_new;
            correction_vit = pid_vit.update(target_vit, vitesse_codeur);
            target_ang += correction_vit/30.0;
            //Serial.println(target_ang);
        }
        if(time_new - time_old_us > 500){
            time_old_us = time_new;
            CAN.sendMsgBuf(ID_US_AV, 0, 1,0, buf_snd);                           //Ajout Frk
            CAN.sendMsgBuf(ID_US_AR, 0, 1,0, buf_snd);
        }
    }
}

void prepare_buf_snd(){
    // left
    buf_snd[0] = (speed_left >> 8) & 0x00FF;
    buf_snd[1] = speed_left & 0x00FF;
    // right
    buf_snd[2] = (speed_right >> 8) & 0x00FF;
    buf_snd[3] = speed_right & 0x00FF;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
