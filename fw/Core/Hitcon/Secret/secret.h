#ifndef HITCON_SECRET_SECRET_H
#define HITCON_SECRET_SECRET_H

#include <Logic/ButtonLogic.h>

namespace hitcon {
constexpr button_t COMBO_BUTTON[] = {
    BUTTON_UP,    BUTTON_UP,   BUTTON_DOWN,  BUTTON_DOWN,       BUTTON_LEFT,
    BUTTON_RIGHT, BUTTON_LEFT, BUTTON_RIGHT, BUTTON_BRIGHTNESS, BUTTON_MODE,
};
constexpr button_t COMBO_BUTTON_DINO[] = {
    BUTTON_DOWN, BUTTON_DOWN, BUTTON_RIGHT, BUTTON_RIGHT, BUTTON_OK};
constexpr size_t COMBO_BUTTON_LEN = sizeof(COMBO_BUTTON) / sizeof(button_t);
constexpr size_t COMBO_BUTTON_DINO_LEN =
    sizeof(COMBO_BUTTON_DINO) / sizeof(button_t);
extern int combo_button_ctr;

#define DATA(col) (col), 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08

constexpr uint8_t kGameAchievementData[] = {
    // First byte is the column
    // 0x00,
    // Next 8 bytes is the data to give to AcceptData().
    // 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    // Repeat...

    // clang-format off
    // Tetris
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x03, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x03, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    0x05, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 31
    0x05, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 32
    0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 33
    0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 34
    0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 35
    0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 37

    // Dino
    0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 31
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 32
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 33
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 34
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 35
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 37

    // Snake
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 10
    0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 10
    0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 12
    0x03, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 12
    0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 14
    0x05, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 32
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 38
    // clang-format on
};
constexpr size_t kGameAchievementDataSize =
    sizeof(kGameAchievementData) / sizeof(kGameAchievementData[0]);
static_assert(kGameAchievementDataSize % 9 == 0);
constexpr size_t kGameAchievementDataCount = kGameAchievementDataSize / 9;

constexpr uint8_t PARTITION_LEN = 4;
constexpr uint32_t PER_DATA_PERIOD = 5 * 60;
#if defined SPONSOR

constexpr uint8_t kStrayIRData[] = {
    // First byte is the column
    // 0x00,
    // Repeat...

    // clang-format off
    /* PARTITION 1 */
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 15
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18

    /* PARTITION 2 */
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21

    /* PARTITION 3 */
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24

    /* PARTITION 4 */
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 27
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 27
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 29
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 29
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    // clang-format on
};
constexpr size_t PER_IR_PARTITION_SIZE =
    sizeof(kStrayIRData) / sizeof(uint8_t) / PARTITION_LEN / 9;

constexpr uint8_t kStrayXBoardData[] = {
    // First byte is the column
    // 0x00,
    // Repeat...

    // clang-format off
    /* PARTITION 1 */
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 16
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 17
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 18
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 19

    /* PARTITION 2 */
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 20
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 21
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23

    /* PARTITION 3 */
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 22
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 23
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 24
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 25

    /* PARTITION 4 */
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 26
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 27
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 27
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 27
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 27
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 28
    0x0e, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 29
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 29
    0x0a, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    0x0b, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 30
    0x0c, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 31
    0x0d, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // 31
    // clang-format on
};
constexpr size_t PER_IR_PARTITION_SIZE =
    sizeof(kStrayIRData) / sizeof(uint8_t) / PARTITION_LEN / 9;
constexpr size_t PER_XBoard_PARTITION_SIZE =
    sizeof(kStrayXBoardData) / sizeof(uint8_t) / PARTITION_LEN / 9;

#elif defined SPEAKER

#elif defined HITCON_R1

#elif defined HITCON_R2

#elif defined HITCON_R3

#elif defined HITCON_R4

#endif

}  // namespace hitcon
#endif  // HITCON_SECRET_SECRET_H
