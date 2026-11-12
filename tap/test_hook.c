// On-robot smoke test for the libuart_api hooks. Run twice - plain and with LD_PRELOAD=lidartap.so - the
// output must be identical: the hooks have to return exactly what the library returns (here: its error
// codes for "not initialised"), and must not crash.
#include <dlfcn.h>
#include <stdio.h>

int main(void) {
    if (!dlopen("/opt/rockrobo/cleaner/lib/libuart_api.so.4.0.0", RTLD_NOW | RTLD_GLOBAL)) {
        printf("dlopen failed: %s\n", dlerror());
        return 2;
    }
    static unsigned char buf[0x1000];
    int (*lds)(unsigned char *, int) = (int (*)(unsigned char *, int))dlsym(RTLD_DEFAULT, "read_lds_mmap_buf");
    int (*pkg)(unsigned char *, int) = (int (*)(unsigned char *, int))dlsym(RTLD_DEFAULT, "get_package");
    printf("read_lds_mmap_buf -> %d\n", lds(buf, 64));
    printf("get_package -> %d\n", pkg(buf, sizeof buf));
    return 0;
}
