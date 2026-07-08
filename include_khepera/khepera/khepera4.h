#ifndef KHEPERA4_H
#define KHEPERA4_H

typedef struct knet_dev_s knet_dev_t;

// O compilador precisa saber que existe esse ponteiro global de hardware
knet_dev_t *kh4_get_device(const char *name);
extern knet_dev_t * dsPic;

#define KNET_BUS_ANY 0
#define kh4_RegSpeed 1

int kh4_init(int argc, char *argv[]);
knet_dev_t *knet_open(const char * name, int bus, void * extra1, void * extra2);
void kh4_SetMode(int mode, knet_dev_t * dev);
void kh4_ResetEncoders(knet_dev_t * dev);
void kh4_get_position(int * left, int * right, knet_dev_t * dev);
void kh4_set_speed(int left, int right, knet_dev_t * dev);
void kh4_proximity_ir(int * buffer, knet_dev_t * dev);

#endif // KHEPERA4_H