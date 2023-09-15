/*
 * Public.h
 *
 *  Created on: Sep 13, 2023
 *      Author: David7_Yuan
 */

/*
    PA0 P1    PB0 P9
    PA1 P2    PB1 P10
    PA2 P3    PB2 P11
    PA3 P4    PB3 P12
    PA4 P5    PB4 P13
    PA5 P6    PB5 P14
    PA6 P7    PB6 P15
    PA7 P8    PB7 P16
*/

#ifndef INC_PUBLIC_H_
#define INC_PUBLIC_H_

#define TASK1_BIT (1 << 0)
#define TASK2_BIT (1 << 1)
#define TASK3_BIT (1 << 2)
#define ALL_TASKS (TASK1_BIT | TASK2_BIT | TASK3_BIT)
#define SN_LENGTH 5
#define CMD_NAME_LENGTH 6
#define PIN_LENGTH 4
#define PIN_VALUE_LENGTH 1
#define BUFFER_LENGTH 32

typedef struct {
    GPIO_TypeDef* GPIO_Port;
    uint16_t GPIO_Pin;
} PinMap;

typedef enum {
    ERR_OK,            // No error
    ERR_INVALID_SN,    // Invalid sn
    ERR_INVALID_CMD,   // Invalid command
    ERR_INVALID_PIN,   // Invalid pin
    ERR_INVALID_VALUE, // Invalid value
    ERR_INVALID_FMT,   // Invalid format
    ERR_UNKNOWN        // Unknown error
} ErrorCode;

const uint8_t READ_CMD[] = "Read";
const uint8_t WRITE_CMD[] = "Write";
const uint8_t INVALID_SN[] = "Invalid_Sn";
const uint8_t INVALID_CMD[] = "Invalid_Cmd";
const uint8_t INVALID_FMT[] = "Invalid_Format";
const uint8_t INVALID_PIN[] = "Invalid_PinName";
const uint8_t INVALID_VALUE[] = "Invalid_Value";
const uint8_t UNKNOWN_ISSUE[] = "Unknown_Issue";

#endif /* INC_PUBLIC_H_ */
