/*
 * Copyright © 2014 Kosma Moczek <kosma@cloudyourcar.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "minmea.h"

int main(void)
{
    char line[MINMEA_MAX_LENGTH];
    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        printf("%s", line);

        switch (minmea_sentence_id(line, false)) {

            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, line)) {
                    printf("$xxGGA: fix quality: %d\n", frame.fix_quality);
                }
                else {
                    printf("$xxGGA sentence is not parsed\n");
                }
            } break;

            case MINMEA_INVALID: {
                printf("$xxxxx sentence is not valid\n");
            } break;

            default: {
                printf("$xxxxx sentence is not parsed\n");
            } break;
        }
    }
    return 0;
}
