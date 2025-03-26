#ifndef CONTAINERS_H
    #define CONTAINERS_H

    #include "constants.h"
    
    typedef struct {
        double x, y;
    } container_position_t;

    const container_position_t containers_position[CONTAINERS_NUM+4] = {{0, 0},
                                                                        {0.26, 0.02},
                                                                        {0.52, 0.10},
                                                                        {0.72, 0.31},
                                                                        {0.92, 0.42},
                                                                        {1.04, 0.68},
                                                                        {1.12, 1.10},
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
                                                                        {},
                                                                        {},
                                                                        {},
                                                                        {},
                                                                        {},
                                                                        {}};
#endif