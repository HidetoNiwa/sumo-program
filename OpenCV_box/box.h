#ifndef BOX_H
#define BOX_H

#include <unistd.h>

class box{
    public:
    box(void);

    float sum;
    float ave;
    unsigned char count;
    unsigned char flag;
    unsigned char motion;
};

#endif