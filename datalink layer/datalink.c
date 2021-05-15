#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2000
#define ACK_TIMER 500
#define WT 7 //发送窗口的大小
#define W 8  //窗口的最大值，即帧序号可以是0到W-1
/*实现协议5，接收窗口大小为1*/

struct FRAME
{
    unsigned char kind;          //表明帧的类型是FRAME_DATA还是FRAME_ACK
    unsigned char ack;           //ack帧时，表示为ack的序号；data帧时，表示为可能捎带的ack
    unsigned char seq;           //seq是当前帧序号
    unsigned char data[PKT_LEN]; //当前帧的数据部分
    unsigned int padding;
};

static unsigned char frame_nr = 0, buffer[WT][PKT_LEN], nbuffered;
/*frame_nr是帧序号，或者说当前发送窗口的起始序号，
buffer是用于缓存当前要发的数据的，
nbuffered是当前发送窗口中已经发出的帧的个数,
WT是指数据链路层最大的缓存数量，也是发送窗口的大小*/
static unsigned char frame_expected = 0; //这是接受窗口的序号
static int phl_ready = 0;                //置为0时物理层忙
static int frame_buffer = 0;             //表示当前缓冲数据的buffer下标

static void put_frame(unsigned char *frame, int len) //将长为len的帧加上CRC校验码后发出
{
    *(unsigned int *)(frame + len) = crc32(frame, len); //在帧的末尾添加上由crc32()计算得出的校验码，对于数据帧，一定是添加在padding区
    send_frame(frame, len + 4);                         //CRC一定是4位，将frame帧原本len长+4位CRC全部交给物理层
    stop_ack_timer();                                   //将ack定时器停止
    phl_ready = 0;                                      //帧交给物理层后，将phl_ready置0，会禁止网络层继续向链路层发包
}

static void send_data_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_DATA;
    s.seq = (frame_nr + nbuffered) % W;
    s.ack = (frame_expected + W - 1) % W;
    /*表示发送的该数据帧时捎带的ack(接收窗口序号的上一位，仅当接收窗口与对方的发送窗口不一致时有用，
    因为此时表明我的接收窗口已经滑动，但对方没有收到ack)*/
    memcpy(s.data, buffer[frame_buffer], PKT_LEN); //将网络层中读来的数据buffer拷贝到帧的data中

    dbg_frame("Send DATA %d %d, ID %d WINDOWS %d\n", s.seq, s.ack, *(short *)s.data, nbuffered);

    put_frame((unsigned char *)&s, 3 + PKT_LEN);
    start_timer((frame_nr + nbuffered) % W, DATA_TIMER); //启动帧序号对应的定时器
}

static void send_ack_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_ACK;
    s.ack = (frame_expected + W - 1) % W; //发送ack前接收窗口已经滑动了。所以要发的ack序号应该是滑动窗口的前一个

    dbg_frame("Send ACK  %d\n", s.ack);

    put_frame((unsigned char *)&s, 2); //ack帧只有kind和ack两个数据，帧长为2
}

static void send_nak_frame(int nak)
{
    struct FRAME s;

    s.kind = FRAME_NAK;
    s.ack = nak;

    dbg_frame("Send NAK  %d\n", s.ack);

    put_frame((unsigned char *)&s, 2); //nak帧只有kind和ack两个数据，帧长为2
}

int main(int argc, char **argv) //启动程序时命令行参数个数argc和储存命令行的agrv
{
    int event, arg, N;
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);
    lprintf("Designed by bupt, build: " __DATE__ "  "__TIME__
            "\n");

    disable_network_layer();

    for (;;)
    {
        event = wait_for_event(&arg);

        switch (event)
        {
        case NETWORK_LAYER_READY:
            get_packet(buffer[frame_buffer]); //从网络层接受的data数据缓存于buffer[frame_buffer]中
            send_data_frame();                //在send_data_frame中成帧
            frame_buffer = (frame_buffer + 1) % WT;
            nbuffered++; //发送窗口中的已经发送的帧个数加1
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:                                     //如果物理层收到一帧
            len = recv_frame((unsigned char *)&f, sizeof f);     //从物理层取来这一帧放到f中，len是实际长度
            if (len < 5 || crc32((unsigned char *)&f, len) != 0) //对取的来帧进行CRC校验，校验位有4位，所以帧长小于5的肯定是错误数据
            {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                if (f.kind == FRAME_DATA && f.seq == frame_expected) //当错误帧为当前接受窗口的数据帧时发送nak
                    send_nak_frame(f.seq);                           //发送一个序号为出错帧的nak
                break;
            }
            if (f.kind == FRAME_NAK) //如果收到的是nak，也要进行gobackn
            {
                dbg_frame("Recv NAK  %d\n", f.ack);

                //因为nak就是接收方的接收窗口，所以相当于收到了nak序号之前ack，要进行滑动
                if ((frame_nr + nbuffered > W && frame_nr <= (f.ack + W - 1) % W && (f.ack + W - 1) % W < W) ||
                    (frame_nr + nbuffered > W && 0 <= (f.ack + W - 1) % W && (f.ack + W - 1) % W < (frame_nr + nbuffered) % W) ||
                    ((f.ack + W - 1) % W >= frame_nr && (f.ack + W - 1) % W < (frame_nr + nbuffered))) //表示全部的ack(这里指nak-1)落在接收窗口内的情况
                {
                    while (frame_nr != f.ack) //接收窗口向后滑动直到第一位是收到的ack的后一位
                    {
                        stop_timer(frame_nr);
                        frame_nr = (frame_nr + 1) % W;
                        nbuffered--;
                    }
                }

                frame_buffer = (frame_buffer + WT - nbuffered) % WT;
                N = nbuffered; //需要重发的帧个数是N = nbuffered, 即GoBackN
                nbuffered = 0;
                for (int i = 0; i < N; i++)
                {
                    send_data_frame(); //发送完帧后定时器会被重新设置
                    nbuffered++;
                    frame_buffer = (frame_buffer + 1) % WT;
                }

                break;
            }
            if (f.kind == FRAME_ACK) //帧中的kind表明身份是ack还是data
            {
                dbg_frame("Recv ACK  %d\n", f.ack);
            }
            if (f.kind == FRAME_DATA)
            {
                dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                if (f.seq == frame_expected)
                {                                              //如果收到的帧是我接收窗口的序号
                    put_packet(f.data, len - 7);               //这里的7指的是帧中处了data的大小为7字节，给网络层时要去掉这7字节
                    frame_expected = (frame_expected + 1) % W; //frame_expected是指接收窗口，接受正确的帧使窗口向后滑动一位
                    start_ack_timer(ACK_TIMER);                //启动ack定时器
                }
            }
            if ((frame_nr + nbuffered > W && frame_nr <= f.ack && f.ack < W) ||
                (frame_nr + nbuffered > W && 0 <= f.ack && f.ack < (frame_nr + nbuffered) % W) ||
                (f.ack >= frame_nr && f.ack < (frame_nr + nbuffered))) //表示全部的ack落在接收窗口内的情况
            {
                while (frame_nr != (f.ack + 1) % W) //接收窗口向后滑动直到第一位是收到的ack的后一位
                {
                    stop_timer(frame_nr);
                    frame_nr = (frame_nr + 1) % W;
                    nbuffered--;
                }
            }
            break;

        case DATA_TIMEOUT:                            //数据超时则重新发送data_frame
            dbg_event("---- DATA %d timeout\n", arg); //arg是超时的定时器的序号,一定是当前发送窗口的第一位的序号
            frame_buffer = (frame_buffer + WT - nbuffered) % WT;
            N = nbuffered; //需要重发的帧个数是N = nbuffered, 即GoBackN
            nbuffered = 0;
            for (int i = 0; i < N; i++)
            {
                send_data_frame();
                nbuffered++;
                frame_buffer = (frame_buffer + 1) % WT;
            }
            break;

            /*若是捎带应答应该还有ACK_TIMEOUT*/
        case ACK_TIMEOUT:
            send_ack_frame();
            break;
        }

        if (nbuffered < WT && phl_ready) //发送窗口大小为WT，所以只能同时发送WT个帧，然后等待，所以nbuffered大于等于WT时要禁用网络层
            enable_network_layer();
        else
            disable_network_layer();
    }
}
