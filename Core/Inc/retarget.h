//
// Created by Felix on 6/20/2021.
//

#ifndef TEST_FREERTOS_RETARGET_H
#define TEST_FREERTOS_RETARGET_H
#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f4xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>

void RetargetInit(UART_HandleTypeDef *huart);

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif //#ifndef _RETARGET_H__
#endif //TEST_FREERTOS_RETARGET_H
