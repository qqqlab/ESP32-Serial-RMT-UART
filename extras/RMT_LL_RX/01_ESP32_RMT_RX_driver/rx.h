#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void rx_setup();
int rx_receive(uint32_t* buf, int size);

#ifdef __cplusplus
}
#endif
