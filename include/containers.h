#ifndef CONTAINERS_H
    #define CONTAINERS_H

    #include "constants.h"
    
    typedef struct {
        double x, y;
    } container_position_t;

    const container_position_t containers_position[CONTAINERS_NUM] = {{0, 0},
                                                                    {0.32, 0.04},
                                                                    {0.54, 0.15},
                                                                    {0.71, 0.25},
                                                                    {0.93, 0.42},
                                                                    {1.04, 0.62},
                                                                    {1.10, 0.91},
                                                                    {1.19, 1.16},
                                                                    {1.24, 1.42},
                                                                    {1.15, 1.74},
                                                                    {1.12, 1.96},
                                                                    {1.02, 2.14},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {},
                                                                    {}};
#endif