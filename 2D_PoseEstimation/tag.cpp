#include "mbed.h"

Serial uwb(PA_9, PA_10, 115200);
Serial pc(USBTX, USBRX, 115200);
Serial ble(PC_10, PC_11, 115200);

char buf[256];
int buf_size = 0;

int main()
{
    uwb.printf("AT+anchor_tag=0\r\n");
    wait_ms(5000);
    uwb.printf("AT+RST\r\n");
    wait_ms(5000);
    uwb.printf("AT+interval=5\r\n");
    wait_ms(5000);
    uwb.printf("AT+switchdis=1\r\n");
    wait_ms(5000);
    while (true) {
        if (uwb.readable()) {
            char c = uwb.getc();
            if (c == ':'){
                c = '#';
            }
            if (c == 'm'){
                c = '#';
            }
            if (c == '#') {
                    if (buf_size > 0) { // 시작 문자(#)가 아닌 경우
                        buf[buf_size] = '\0'; // 버퍼의 마지막에 널 문자를 추가하여 문자열 종료
                        if (buf_size==4){
                            pc.printf("%s\n", buf); // 받은 문자열을 출력
                            ble.printf("%s\n", buf);
                        }
                        // 파이썬 루프백 코드 작성 (buf 배열을 파이썬으로 전송하는 방법은 제외)
                    }
                    buf_size = 0;
                } else {
                    buf[buf_size] = c;
                    buf_size++;
                }
        }
    }
}
