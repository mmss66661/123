#ifndef HC05_GAMEPAD_H
#define HC05_GAMEPAD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HC05_GAMEPAD_FRAME_SIZE       20U
#define HC05_GAMEPAD_PROTOCOL_VERSION 0x01U
#define HC05_GAMEPAD_LINK_TIMEOUT_MS  100U

enum {
    HC05_INDICATOR_STOP = 0U,
    HC05_INDICATOR_RECORDING = 1U,
    HC05_INDICATOR_PLAYBACK = 2U,
    HC05_INDICATOR_FAULT_FEEDBACK = 3U,
    HC05_INDICATOR_FAULT_MOTOR = 4U,
    HC05_INDICATOR_FAULT_FLASH = 5U,
    HC05_INDICATOR_FAULT_DATA = 6U,
    HC05_INDICATOR_OFF = 7U
};

enum {
    HC05_BUTTON_A     = (1U << 0),
    HC05_BUTTON_B     = (1U << 1),
    HC05_BUTTON_X     = (1U << 2),
    HC05_BUTTON_Y     = (1U << 3),
    HC05_BUTTON_LB    = (1U << 4),
    HC05_BUTTON_RB    = (1U << 5),
    HC05_BUTTON_BACK  = (1U << 6),
    HC05_BUTTON_START = (1U << 7),
    HC05_BUTTON_LS    = (1U << 8),
    HC05_BUTTON_RS    = (1U << 9),
    HC05_BUTTON_GUIDE = (1U << 10),
    HC05_BUTTON_UP    = (1U << 11),
    HC05_BUTTON_DOWN  = (1U << 12),
    HC05_BUTTON_LEFT  = (1U << 13),
    HC05_BUTTON_RIGHT = (1U << 14)
};

typedef struct {
    uint8_t sequence;
    uint16_t buttons;
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t lt;
    uint8_t rt;
    int8_t hat_x;
    int8_t hat_y;
    volatile uint8_t operation_mode;  // 0=停止，1=录制，3=回放
    volatile uint32_t start_press_count;
    volatile uint32_t mode_press_count;

    volatile uint32_t last_update_tick;
    volatile uint32_t frame_count;
    volatile uint32_t dropped_frame_count;
    volatile uint32_t crc_error_count;
    volatile uint32_t format_error_count;
    volatile uint32_t uart_error_count;
} HC05_GamepadState;

extern volatile HC05_GamepadState hc05_gamepad;

void HC05_Gamepad_Init(void);
void HC05_Gamepad_FeedByte(uint8_t byte);
void HC05_Gamepad_NotifyUartError(void);
bool HC05_Gamepad_IsFresh(uint32_t now, uint32_t timeout_ms);
void HC05_Gamepad_UpdateIndicator(uint32_t now, uint32_t timeout_ms);
void HC05_Gamepad_SetRuntimeIndicator(uint8_t indicator);

#ifdef __cplusplus
}
#endif

#endif
