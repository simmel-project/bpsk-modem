#ifndef __MODULATE_H__
#define __MODULATE_H__

struct modulate_cfg {
    uint32_t rate;
    uint32_t carrier;
    uint32_t baud;
    float pll_incr;
    void (*write)(void *arg, void *data, unsigned int count);
    void *write_arg;
    float omega;
};

struct modulate_state {
    struct modulate_cfg cfg;
    float bit_pll;
    float baud_pll;
    int polarity;
};

void modulate_init(struct modulate_state *state, uint32_t carrier,
                   uint32_t rate, uint32_t baud,
                   void (*write)(void *arg, void *data, unsigned int count),
                   void *arg);
void modulate_string(struct modulate_state *state, const char *string);

#endif /* __MODULATE_H__ */
