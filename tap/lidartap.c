// lidartap.so - LD_PRELOAD shim for rr_loader on Roborock S5.
// Copies every byte rr_loader reads from / writes to the serial ports (lidar, MCU) to UDP 127.0.0.1:9876.
// It never changes the data, never blocks and never raises signals: if nobody listens, copies are dropped.
//
// Datagram: "LT" | port u8 (1=ttyS1, 2=ttyS2, 3=read_lds_mmap_buf, 4=/dev/input/event1 lds_input,
//   5=/dev/uart_lds, 6=/dev/uart_mcu, 7=MCU sensor package (get_package), 9=test match) | dir u8 (0=rx, 1=tx) | len u16 LE | t_ms u32 LE | payload
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define MAX_FD 1024
#define MAX_PAYLOAD 1400 // ponytail: bigger reads are split, never seen on a 115200-baud UART anyway

static unsigned char port_of[MAX_FD]; // fd -> tapped port id (0 = not tapped)
static int sock = -1;
static struct sockaddr_in dst;
static const char *test_match; // LIDARTAP_MATCH: extra path to tap (for safe testing with `cat`)

static ssize_t (*real_read)(int, void *, size_t);
static ssize_t (*real_write)(int, const void *, size_t);
static int (*real_open)(const char *, int, ...);
static int (*real_open64)(const char *, int, ...);
static int (*real_openat)(int, const char *, int, ...);
static int (*real_close)(int);

__attribute__((constructor)) static void init(void) {
    real_read = dlsym(RTLD_NEXT, "read");
    real_write = dlsym(RTLD_NEXT, "write");
    real_open = dlsym(RTLD_NEXT, "open");
    real_open64 = dlsym(RTLD_NEXT, "open64");
    real_openat = dlsym(RTLD_NEXT, "openat");
    real_close = dlsym(RTLD_NEXT, "close");
    test_match = getenv("LIDARTAP_MATCH");
    sock = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    dst.sin_family = AF_INET;
    dst.sin_port = htons(9876);
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
}

static unsigned char classify(const char *path) {
    if (!path) return 0;
    if (strcmp(path, "/dev/ttyS1") == 0) return 1;
    if (strcmp(path, "/dev/ttyS2") == 0) return 2;
    if (strcmp(path, "/dev/input/event1") == 0) return 4; // "lds_input": kernel notifications from the LDS driver
    if (strcmp(path, "/dev/uart_lds") == 0) return 5;
    if (strcmp(path, "/dev/uart_mcu") == 0) return 6;
    if (test_match && strcmp(path, test_match) == 0) return 9;
    return 0;
}

static void remember(int fd, const char *path) {
    if (fd >= 0 && fd < MAX_FD) port_of[fd] = classify(path);
}

static void send_copy(unsigned char port, unsigned char dir, const void *buf, ssize_t n) {
    if (n <= 0 || sock < 0) return;
    int saved = errno; // the caller must see exactly the errno of the real call
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint32_t t = (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
    for (ssize_t off = 0; off < n; off += MAX_PAYLOAD) {
        unsigned char pkt[10 + MAX_PAYLOAD];
        uint16_t len = (uint16_t)((n - off) > MAX_PAYLOAD ? MAX_PAYLOAD : (n - off));
        pkt[0] = 'L'; pkt[1] = 'T'; pkt[2] = port; pkt[3] = dir;
        memcpy(pkt + 4, &len, 2);
        memcpy(pkt + 6, &t, 4);
        memcpy(pkt + 10, (const unsigned char *)buf + off, len);
        sendto(sock, pkt, 10 + len, MSG_DONTWAIT | MSG_NOSIGNAL, (struct sockaddr *)&dst, sizeof dst);
    }
    errno = saved;
}

static void tap(int fd, unsigned char dir, const void *buf, ssize_t n) {
    if (fd >= 0 && fd < MAX_FD && port_of[fd]) send_copy(port_of[fd], dir, buf, n);
}

static mode_t mode_arg(int flags, va_list ap) {
    return (flags & (O_CREAT | __O_TMPFILE)) ? va_arg(ap, mode_t) : 0;
}

int open(const char *path, int flags, ...) {
    va_list ap; va_start(ap, flags); mode_t m = mode_arg(flags, ap); va_end(ap);
    int fd = real_open(path, flags, m);
    remember(fd, path);
    return fd;
}

int open64(const char *path, int flags, ...) {
    va_list ap; va_start(ap, flags); mode_t m = mode_arg(flags, ap); va_end(ap);
    int fd = (real_open64 ? real_open64 : real_open)(path, flags, m);
    remember(fd, path);
    return fd;
}

int openat(int dirfd, const char *path, int flags, ...) {
    va_list ap; va_start(ap, flags); mode_t m = mode_arg(flags, ap); va_end(ap);
    int fd = real_openat(dirfd, path, flags, m);
    remember(fd, path);
    return fd;
}

int close(int fd) {
    if (fd >= 0 && fd < MAX_FD) port_of[fd] = 0;
    return real_close(fd);
}

ssize_t read(int fd, void *buf, size_t count) {
    ssize_t n = real_read(fd, buf, count);
    tap(fd, 0, buf, n);
    return n;
}

ssize_t __read_chk(int fd, void *buf, size_t count, size_t buflen) {
    (void)buflen;
    return read(fd, buf, count);
}

// Lidar data does not go through read(): the kernel driver /dev/uart_lds exposes a ring buffer (mmap) and
// librr_plugin_laser.so pulls bytes with libuart_api's read_lds_mmap_buf(buf, len) -> bytes copied (<0 = error).
// Disassembly (libuart_api.so.4.0.0): it only sets a 23 ms poll timeout and tail-calls rua_lds_read_from_mmap.
static int (*real_read_lds)(unsigned char *, int);

int read_lds_mmap_buf(unsigned char *buf, int len) {
    if (!real_read_lds) real_read_lds = dlsym(RTLD_NEXT, "read_lds_mmap_buf");
    if (!real_read_lds) return -5; // same code libuart_api returns when the LDS isn't initialised
    int n = real_read_lds(buf, len);
    if (n > 0 && n <= len) send_copy(3, 0, buf, n);
    return n;
}

// MCU sensor packets: librr_plugin_chassis.so calls libuart_api's get_package(buf, 0x1000) -> payload length,
// then walks it with get_next() as TLV items [id u8][len u8][data]. IDs (from the chassis jump table):
// 7 odo+gyro, 8 battery, 33 bumper, 34 wheel drop, 35 cliff, 64 wall sensor, 80 fan, 81 wheels, 82 main brush,
// 83 side brush, 4 error. Copied as port 7.
static int (*real_get_package)(unsigned char *, int);

int get_package(unsigned char *buf, int size) {
    if (!real_get_package) real_get_package = dlsym(RTLD_NEXT, "get_package");
    if (!real_get_package) return -5;
    int n = real_get_package(buf, size);
    if (n > 0 && n <= size) send_copy(7, 0, buf, n);
    return n;
}

ssize_t write(int fd, const void *buf, size_t count) {
    ssize_t n = real_write(fd, buf, count);
    tap(fd, 1, buf, n);
    return n;
}
