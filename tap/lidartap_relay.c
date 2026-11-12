// lidartap_relay - runs on the robot while the PC is listening:
//   ssh root@robot /mnt/data/lidartap_relay > capture.bin
// 1. Binds UDP 127.0.0.1:9876 and writes every datagram from lidartap.so to stdout unchanged
//    (datagrams carry their own length, so the PC can re-split the stream).
// 2. Every 100 ms adds a "port 8" datagram: the first 24 bytes of /run/shm/SharedMapInfo = the live SLAM pose
//    [u32 flags][i16 x_mm][i16 y_mm][f32 theta_rad] + the charger pose in the same layout. Read-only.
// Exits when stdout closes.
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static int write_all(const unsigned char *p, size_t n) {
    while (n) {
        ssize_t k = write(1, p, n);
        if (k <= 0) return -1;
        p += k;
        n -= (size_t)k;
    }
    return 0;
}

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // same clock lidartap.so stamps its datagrams with
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

int main(void) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_port = htons(9876);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (s < 0 || bind(s, (struct sockaddr *)&a, sizeof a) < 0) {
        perror("lidartap_relay: bind 127.0.0.1:9876");
        return 1;
    }
    unsigned char buf[2048];
    uint32_t last_pose = 0;
    for (;;) {
        fd_set r;
        FD_ZERO(&r);
        FD_SET(s, &r);
        struct timeval tv = {0, 50000};
        if (select(s + 1, &r, 0, 0, &tv) > 0) {
            ssize_t n = recv(s, buf, sizeof buf, 0);
            if (n > 0 && write_all(buf, (size_t)n) < 0) return 0; // ssh went away
        }
        uint32_t t = now_ms();
        if (t - last_pose >= 100) {
            last_pose = t;
            unsigned char p[10 + 24] = {'L', 'T', 8, 0, 24, 0};
            memcpy(p + 6, &t, 4);
            int fd = open("/run/shm/SharedMapInfo", O_RDONLY); // reopened each time in case it gets replaced
            if (fd >= 0) {
                ssize_t k = pread(fd, p + 10, 24, 0);
                close(fd);
                if (k == 24 && write_all(p, sizeof p) < 0) return 0;
            }
        }
    }
}
