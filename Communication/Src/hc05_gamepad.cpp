#include "hc05_gamepad.h"

#include "dbus.h"
#include "main.h"

#include <cstring>

volatile HC05_GamepadState hc05_gamepad{};

namespace {

uint8_t frame_buffer[HC05_GAMEPAD_FRAME_SIZE]{};
uint8_t frame_used = 0U;
bool have_sequence = false;
uint8_t last_sequence = 0U;
uint8_t gripper_state = 3U;  // 与 main.c 的上电默认状态一致：夹爪打开。
volatile uint16_t last_buttons = 0U;
volatile uint8_t operation_mode = 0U;

void set_runtime_indicator(uint8_t indicator) {
    uint32_t set_pins = GPIO_PIN_11;  // 停止：绿灯
    uint32_t reset_pins = GPIO_PIN_10 | GPIO_PIN_12;
    if (indicator == HC05_INDICATOR_OFF) {
        set_pins = 0U;
        reset_pins = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    } else if (indicator == HC05_INDICATOR_RECORDING) {
        set_pins = GPIO_PIN_11 | GPIO_PIN_12;  // 录制：红 + 绿 = 黄
        reset_pins = GPIO_PIN_10;
    } else if (indicator == HC05_INDICATOR_PLAYBACK) {
        set_pins = GPIO_PIN_12;  // 回放：红灯
        reset_pins = GPIO_PIN_10 | GPIO_PIN_11;
    } else if (indicator == HC05_INDICATOR_FAULT_FEEDBACK) {
        set_pins = GPIO_PIN_10;  // 反馈/CAN 故障：蓝灯
        reset_pins = GPIO_PIN_11 | GPIO_PIN_12;
    } else if (indicator == HC05_INDICATOR_FAULT_MOTOR) {
        set_pins = GPIO_PIN_10 | GPIO_PIN_12;  // 电机报码：紫灯
        reset_pins = GPIO_PIN_11;
    } else if (indicator == HC05_INDICATOR_FAULT_FLASH) {
        set_pins = GPIO_PIN_10 | GPIO_PIN_11;  // Flash 故障：青灯
        reset_pins = GPIO_PIN_12;
    } else if (indicator == HC05_INDICATOR_FAULT_DATA) {
        set_pins = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;  // 数据/限位：白灯
        reset_pins = 0U;
    }
    GPIOH->BSRR = set_pins | (reset_pins << 16U);
}

void stop_operation(void) {
    operation_mode = 0U;
    hc05_gamepad.operation_mode = 0U;
    dbus.s2 = 0U;
    set_runtime_indicator(HC05_INDICATOR_STOP);
}

uint16_t read_u16_le(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint16_t crc16_modbus(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U
                      ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                      : static_cast<uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

int16_t axis_to_dbus(int16_t axis) {
    constexpr int32_t DBUS_RANGE = 660;
    int32_t value = axis;
    if (value > 32767) value = 32767;
    if (value < -32767) value = -32767;
    return static_cast<int16_t>((value * DBUS_RANGE) / 32767);
}

int16_t triggers_to_dbus(uint8_t left, uint8_t right) {
    const int32_t difference = static_cast<int32_t>(right) -
                               static_cast<int32_t>(left);
    return static_cast<int16_t>((difference * 660) / 255);
}

void update_dbus_compatibility(const HC05_GamepadState &state, uint32_t now) {
    // Android 的 Y 轴向下为正；DBUS 习惯上推为正，因此两个 Y 轴取反。
    dbus.ch[0] = axis_to_dbus(state.lx);
    dbus.ch[1] = axis_to_dbus(static_cast<int16_t>(-state.ly));
    dbus.ch[2] = axis_to_dbus(state.rx);
    dbus.ch[3] = axis_to_dbus(static_cast<int16_t>(-state.ry));
    dbus.ch[4] = triggers_to_dbus(state.lt, state.rt);

    const bool close_pressed = (state.buttons & HC05_BUTTON_A) != 0U;
    const bool open_pressed = (state.buttons & HC05_BUTTON_B) != 0U;
    if (close_pressed != open_pressed) {
        gripper_state = close_pressed ? 1U : 3U;
    }
    dbus.s1 = (close_pressed || open_pressed) ? gripper_state : 2U;

    // 只在按下沿切换工作状态，松开按钮后状态继续保持。
    const uint16_t pressed = static_cast<uint16_t>(state.buttons & ~last_buttons);
    last_buttons = state.buttons;
    if ((pressed & HC05_BUTTON_START) != 0U) {
        operation_mode = 0U;  // START 优先级最高，立即停止。
        ++hc05_gamepad.start_press_count;
    } else if ((pressed & HC05_BUTTON_RB) != 0U) {
        operation_mode = 1U;  // RB 单击开始录制。
        ++hc05_gamepad.mode_press_count;
    } else if ((pressed & HC05_BUTTON_LB) != 0U) {
        operation_mode = 3U;  // LB 单击开始回放。
        ++hc05_gamepad.mode_press_count;
    }
    dbus.s2 = operation_mode;
    hc05_gamepad.operation_mode = operation_mode;

    dbus.mouse.x = 0;
    dbus.mouse.y = 0;
    dbus.mouse.z = 0;
    dbus.mouse.l = close_pressed ? 1U : 0U;
    dbus.mouse.r = open_pressed ? 1U : 0U;
    dbus.key = state.buttons;

    // 最后发布时戳和帧计数，任务层不会把半更新的数据当成新帧。
    dbus.last_update_tick = now;
    ++dbus.frame_count;
}

bool decode_frame(const uint8_t frame[HC05_GAMEPAD_FRAME_SIZE]) {
    if (frame[0] != 0xAAU || frame[1] != 0x55U ||
        frame[2] != HC05_GAMEPAD_PROTOCOL_VERSION) {
        ++hc05_gamepad.format_error_count;
        return false;
    }

    const uint16_t expected_crc = read_u16_le(&frame[18]);
    if (crc16_modbus(&frame[2], 16U) != expected_crc) {
        ++hc05_gamepad.crc_error_count;
        return false;
    }

    HC05_GamepadState decoded{};
    decoded.sequence = frame[3];
    decoded.buttons = read_u16_le(&frame[4]);
    decoded.lx = static_cast<int16_t>(read_u16_le(&frame[6]));
    decoded.ly = static_cast<int16_t>(read_u16_le(&frame[8]));
    decoded.rx = static_cast<int16_t>(read_u16_le(&frame[10]));
    decoded.ry = static_cast<int16_t>(read_u16_le(&frame[12]));
    decoded.lt = frame[14];
    decoded.rt = frame[15];
    decoded.hat_x = static_cast<int8_t>(frame[16]);
    decoded.hat_y = static_cast<int8_t>(frame[17]);

    if ((decoded.buttons & 0x8000U) != 0U ||
        decoded.lx < -32767 || decoded.ly < -32767 ||
        decoded.rx < -32767 || decoded.ry < -32767 ||
        decoded.hat_x < -1 || decoded.hat_x > 1 ||
        decoded.hat_y < -1 || decoded.hat_y > 1) {
        ++hc05_gamepad.format_error_count;
        return false;
    }

    if (have_sequence) {
        const uint8_t delta = static_cast<uint8_t>(decoded.sequence - last_sequence);
        if (delta > 1U) {
            hc05_gamepad.dropped_frame_count += static_cast<uint8_t>(delta - 1U);
        }
    }
    last_sequence = decoded.sequence;
    have_sequence = true;

    const uint32_t now = HAL_GetTick();
    hc05_gamepad.sequence = decoded.sequence;
    hc05_gamepad.buttons = decoded.buttons;
    hc05_gamepad.lx = decoded.lx;
    hc05_gamepad.ly = decoded.ly;
    hc05_gamepad.rx = decoded.rx;
    hc05_gamepad.ry = decoded.ry;
    hc05_gamepad.lt = decoded.lt;
    hc05_gamepad.rt = decoded.rt;
    hc05_gamepad.hat_x = decoded.hat_x;
    hc05_gamepad.hat_y = decoded.hat_y;

    update_dbus_compatibility(decoded, now);

    hc05_gamepad.last_update_tick = now;
    ++hc05_gamepad.frame_count;
    return true;
}

void retain_possible_header(void) {
    // 坏帧中可能已经含有下一帧帧头；保留它及其后的数据以快速重同步。
    for (uint8_t i = 1U; i + 1U < HC05_GAMEPAD_FRAME_SIZE; ++i) {
        if (frame_buffer[i] == 0xAAU && frame_buffer[i + 1U] == 0x55U) {
            frame_used = static_cast<uint8_t>(HC05_GAMEPAD_FRAME_SIZE - i);
            std::memmove(frame_buffer, &frame_buffer[i], frame_used);
            return;
        }
    }

    if (frame_buffer[HC05_GAMEPAD_FRAME_SIZE - 1U] == 0xAAU) {
        frame_buffer[0] = 0xAAU;
        frame_used = 1U;
    } else {
        frame_used = 0U;
    }
}

}  // namespace

extern "C" {

void HC05_Gamepad_Init(void) {
    std::memset(frame_buffer, 0, sizeof(frame_buffer));
    frame_used = 0U;
    have_sequence = false;
    last_sequence = 0U;
    last_buttons = 0U;
    operation_mode = 0U;
    gripper_state = 3U;
    std::memset((void *)&hc05_gamepad, 0, sizeof(hc05_gamepad));
    stop_operation();
}

void HC05_Gamepad_FeedByte(uint8_t byte) {
    if (frame_used == 0U) {
        if (byte == 0xAAU) {
            frame_buffer[0] = byte;
            frame_used = 1U;
        }
        return;
    }

    if (frame_used == 1U) {
        if (byte == 0x55U) {
            frame_buffer[1] = byte;
            frame_used = 2U;
        } else if (byte != 0xAAU) {
            frame_used = 0U;
        }
        return;
    }

    frame_buffer[frame_used++] = byte;
    if (frame_used < HC05_GAMEPAD_FRAME_SIZE) return;

    if (decode_frame(frame_buffer)) {
        frame_used = 0U;
    } else {
        retain_possible_header();
    }
}

void HC05_Gamepad_NotifyUartError(void) {
    ++hc05_gamepad.uart_error_count;
}

bool HC05_Gamepad_IsFresh(uint32_t now, uint32_t timeout_ms) {
    return hc05_gamepad.frame_count != 0U &&
           (now - hc05_gamepad.last_update_tick) <= timeout_ms;
}

void HC05_Gamepad_UpdateIndicator(uint32_t now, uint32_t timeout_ms) {
    if (!HC05_Gamepad_IsFresh(now, timeout_ms)) {
        stop_operation();
    }
}

void HC05_Gamepad_SetRuntimeIndicator(uint8_t indicator) {
    set_runtime_indicator(indicator);
}

}  // extern "C"
