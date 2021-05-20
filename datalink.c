#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2500 //重传定时器的时限
#define ACK_TIMER 1000 //ACK搭载定时器的时限
#define MAX_SEQ 31
#define NR_BUFS ((MAX_SEQ+1)>>1) //窗口大小
#define inc(k) if (k < MAX_SEQ) k = k + 1; else k = 0

typedef unsigned char seq_nr;
typedef unsigned char frame_kind;
typedef enum { false, true } boolean;

//帧格式定义
typedef struct FRAME { 
    seq_nr kind;
    seq_nr ack;
    seq_nr seq;
    seq_nr data[PKT_LEN]; 
    unsigned int  padding;
} frame;

//参数定义
static boolean no_nak = true; //true表示该帧没有发过nak
static seq_nr nbuffered = 0; //发送方缓冲区存放了多少数据
static boolean phl_ready = false; //物理层是否准备好
static seq_nr ack_expected = 0, next_frame_to_send = 0; //发送窗口上下限
static seq_nr frame_expected = 0, too_far = NR_BUFS; //接收窗口上下限
static seq_nr out_buf[NR_BUFS][PKT_LEN];  //发送方缓冲区
static seq_nr in_buf[NR_BUFS][PKT_LEN]; //接收方缓冲区
static boolean arrived[NR_BUFS]; //接收方标志数组，用于记录哪些序号的帧收到了

//between函数，用于主要用于判断frame.seq是否在窗口中
static boolean between(seq_nr a, seq_nr b, seq_nr c)
{
    return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

//将帧送到物理层
static void put_frame(seq_nr *frame, int len)
{
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

//发送帧
static void send_data_frame(frame_kind fk, seq_nr frame_nr) 
{
    //给要发的帧的各种参数赋值
    frame s;
    s.kind = fk;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1); 
    s.seq = frame_nr;

    //发帧，这时要根据不同的帧类型来判断发多长的帧
    if (fk == FRAME_DATA) {
        memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN);
        dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *) s.data);
        put_frame((unsigned char *) &s, 3 + PKT_LEN);
        start_timer(frame_nr % NR_BUFS, DATA_TIMER); //开启数据帧帧序号为frame.seq帧的计时器
    } else if (fk == FRAME_NAK) {
        no_nak = false; //该帧发了nak所以no_nak置false
        dbg_frame("Send NAK %d\n", s.ack);
        put_frame((unsigned char *) &s, 2);
    } else if (fk == FRAME_ACK) {
        dbg_frame("Send ACK %d\n", s.ack);
        put_frame((unsigned char *) &s, 2);
    }
    stop_ack_timer(); //因为这时有帧发，所以捎带上ack，停止ack计时器
}

int main(int argc, char **argv)
{
    int event, arg;
    frame f;
    int len = 0;
    
    //初始化
    protocol_init(argc, argv); 
    lprintf("Designed by wjj, build: " __DATE__"  "__TIME__"\n");
    for (int i = 0; i < NR_BUFS; i++)  arrived[i] = false;
    enable_network_layer();
    
    //循环，事件监听及处理
    for (;;) {
        event = wait_for_event(&arg);

        switch (event) {
            case NETWORK_LAYER_READY: //网络层准备就绪
                dbg_event("Network layer ready:\n");
                nbuffered=nbuffered+1;
                get_packet(out_buf[next_frame_to_send % NR_BUFS]); //从网络层接收数据
                send_data_frame(FRAME_DATA, next_frame_to_send); //向物理层发送帧
                inc(next_frame_to_send);
                break;

            case PHYSICAL_LAYER_READY: //物理层准备就绪
                dbg_event("Physical layer ready:\n");
                phl_ready = true; //改变phl_ready
                break;

            case FRAME_RECEIVED: //收到帧
                dbg_event("Frame has received:\n");
                len = recv_frame((unsigned char *) &f, sizeof(f)); 
                //通过crc校验判断是否传输中出现误码
                if ((len != 6 && len != 263) || crc32((unsigned char *) &f, len) != 0) {
                    //有误码
                    dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                    if (no_nak) { //如果没发过nak，则发送nak
                        send_data_frame(FRAME_NAK, 0);
                    }
                    break;
                }
                //数据帧
                if (f.kind == FRAME_DATA) {
                    dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short *) f.data);
                    if (f.seq != frame_expected && no_nak) //帧序号不对应且没发过nak，则发送nak
                        send_data_frame(FRAME_NAK, 0); else start_ack_timer(ACK_TIMER); 
                    if (between(frame_expected, f.seq, too_far) &&(arrived[f.seq % NR_BUFS] == false)) { 
                        arrived[f.seq % NR_BUFS] = true; //该帧所处序号标记上有帧占用，说明缓冲区此处当前有帧占用
                        memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);
                        while (arrived[frame_expected % NR_BUFS]) { //将接收方缓冲区中的数据上传网络层
                            put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN); //缓冲区数据上传网络层
                            no_nak = true; //标识发了nak
                            arrived[frame_expected % NR_BUFS] = false; //因为已经上传，所以该缓冲区的数据清除，标记取消掉
                            inc(frame_expected);
                            inc(too_far);
                            start_ack_timer(ACK_TIMER); //开启一个个收到的帧对应的ack计时器，等待数据发送
                        }
                    }
                }
                //NAK帧
                if (f.kind == FRAME_NAK && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) {
                    //如果是NAK帧且对应的帧序号在发送方所在窗口内，则重发该帧
                    dbg_frame("Recv NAK %d\n", f.ack);
                    send_data_frame(FRAME_DATA, (f.ack + 1) % (MAX_SEQ + 1));
                }
                //ACK帧
                //发送窗口下沿一直移到f.ack处，累积确认
                while (between(ack_expected, f.ack, next_frame_to_send)) {
                    nbuffered--; //确认了发送成功了则nbuffered减小，如果nbuffered<发送窗口则可以接着enable_network_layer();
                    stop_timer(ack_expected % NR_BUFS);//对应帧的计时器关闭
                    inc(ack_expected);
                }
                break;

            case DATA_TIMEOUT: //帧定时器超时，重发该帧
                dbg_event("---------------------------- DATA %d timeout\n", arg);
                send_data_frame(FRAME_DATA, ack_expected);
                break;

            case ACK_TIMEOUT: //ACK定时器超时，不能捎带了，直接发ACK帧
                dbg_event("---------------------------- ACK %d timeout\n", arg);
                send_data_frame(FRAME_ACK, 0);
                break;
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
   }
}
