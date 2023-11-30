#include "display.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "display_defines.h"
#include "mspi_raydium.h"
#include "gpios.h"

static uint8_t display_buffer[188160];
static const unsigned char font57[] = {

    0x00, 0x00, 0x00, 0x00, 0x00, // 0x0
    0x3E, 0x5B, 0x4F, 0x5B, 0x3E, // 0x1
    0x3E, 0x6B, 0x4F, 0x6B, 0x3E, // 0x2
    0x1C, 0x3E, 0x7C, 0x3E, 0x1C, // 0x3
    0x18, 0x3C, 0x7E, 0x3C, 0x18, // 0x4
    0x1C, 0x57, 0x7D, 0x57, 0x1C, // 0x5
    0x1C, 0x5E, 0x7F, 0x5E, 0x1C, // 0x6
    0x00, 0x18, 0x3C, 0x18, 0x00, // 0x7
    0xFF, 0xE7, 0xC3, 0xE7, 0xFF, // 0x8
    0x00, 0x18, 0x24, 0x18, 0x00, // 0x9
    0xFF, 0xE7, 0xDB, 0xE7, 0xFF, // 0xA
    0x30, 0x48, 0x3A, 0x06, 0x0E, // 0xB
    0x26, 0x29, 0x79, 0x29, 0x26, // 0xC
    0x40, 0x7F, 0x05, 0x05, 0x07, // 0xD
    0x40, 0x7F, 0x05, 0x25, 0x3F, // 0xE
    0x5A, 0x3C, 0xE7, 0x3C, 0x5A, // 0xF
    0x7F, 0x3E, 0x1C, 0x1C, 0x08, // 0x10
    0x08, 0x1C, 0x1C, 0x3E, 0x7F, // 0x11
    0x14, 0x22, 0x7F, 0x22, 0x14, // 0x12
    0x5F, 0x5F, 0x00, 0x5F, 0x5F, // 0x13
    0x06, 0x09, 0x7F, 0x01, 0x7F, // 0x14
    0x00, 0x66, 0x89, 0x95, 0x6A, // 0x15
    0x60, 0x60, 0x60, 0x60, 0x60, // 0x16
    0x94, 0xA2, 0xFF, 0xA2, 0x94, // 0x17
    0x08, 0x04, 0x7E, 0x04, 0x08, // 0x18
    0x10, 0x20, 0x7E, 0x20, 0x10, // 0x19
    0x08, 0x08, 0x2A, 0x1C, 0x08, // 0x1A
    0x08, 0x1C, 0x2A, 0x08, 0x08, // 0x1B
    0x1E, 0x10, 0x10, 0x10, 0x10, // 0x1C
    0x0C, 0x1E, 0x0C, 0x1E, 0x0C, // 0x1D
    0x30, 0x38, 0x3E, 0x38, 0x30, // 0x1E
    0x06, 0x0E, 0x3E, 0x0E, 0x06, // 0x1F
    0x00, 0x00, 0x00, 0x00, 0x00, // 0x20
    0x00, 0x00, 0x5F, 0x00, 0x00, // 0x21
    0x00, 0x07, 0x00, 0x07, 0x00, // 0x22
    0x14, 0x7F, 0x14, 0x7F, 0x14, // 0x23
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // 0x24
    0x23, 0x13, 0x08, 0x64, 0x62, // 0x25
    0x36, 0x49, 0x56, 0x20, 0x50, // 0x26
    0x00, 0x08, 0x07, 0x03, 0x00, // 0x27
    0x00, 0x1C, 0x22, 0x41, 0x00, // 0x28
    0x00, 0x41, 0x22, 0x1C, 0x00, // 0x29
    0x2A, 0x1C, 0x7F, 0x1C, 0x2A, // 0x2A
    0x08, 0x08, 0x3E, 0x08, 0x08, // 0x2B
    0x00, 0x80, 0x70, 0x30, 0x00, // 0x2C
    0x08, 0x08, 0x08, 0x08, 0x08, // 0x2D
    0x00, 0x00, 0x60, 0x60, 0x00, // 0x2E
    0x20, 0x10, 0x08, 0x04, 0x02, // 0x2F
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0x30
    0x00, 0x42, 0x7F, 0x40, 0x00, // 0x31
    0x72, 0x49, 0x49, 0x49, 0x46, // 0x32
    0x21, 0x41, 0x49, 0x4D, 0x33, // 0x33
    0x18, 0x14, 0x12, 0x7F, 0x10, // 0x34
    0x27, 0x45, 0x45, 0x45, 0x39, // 0x35
    0x3C, 0x4A, 0x49, 0x49, 0x31, // 0x36
    0x41, 0x21, 0x11, 0x09, 0x07, // 0x37
    0x36, 0x49, 0x49, 0x49, 0x36, // 0x38
    0x46, 0x49, 0x49, 0x29, 0x1E, // 0x39
    0x00, 0x00, 0x14, 0x00, 0x00, // 0x3A
    0x00, 0x40, 0x34, 0x00, 0x00, // 0x3B
    0x00, 0x08, 0x14, 0x22, 0x41, // 0x3C
    0x14, 0x14, 0x14, 0x14, 0x14, // 0x3D
    0x00, 0x41, 0x22, 0x14, 0x08, // 0x3E
    0x02, 0x01, 0x59, 0x09, 0x06, // 0x3F
    0x3E, 0x41, 0x5D, 0x59, 0x4E, // 0x40
    0x7C, 0x12, 0x11, 0x12, 0x7C, // 0x41
    0x7F, 0x49, 0x49, 0x49, 0x36, // 0x42
    0x3E, 0x41, 0x41, 0x41, 0x22, // 0x43
    0x7F, 0x41, 0x41, 0x41, 0x3E, // 0x44
    0x7F, 0x49, 0x49, 0x49, 0x41, // 0x45
    0x7F, 0x09, 0x09, 0x09, 0x01, // 0x46
    0x3E, 0x41, 0x41, 0x51, 0x73, // 0x47
    0x7F, 0x08, 0x08, 0x08, 0x7F, // 0x48
    0x00, 0x41, 0x7F, 0x41, 0x00, // 0x49
    0x20, 0x40, 0x41, 0x3F, 0x01, // 0x4A
    0x7F, 0x08, 0x14, 0x22, 0x41, // 0x4B
    0x7F, 0x40, 0x40, 0x40, 0x40, // 0x4C
    0x7F, 0x02, 0x1C, 0x02, 0x7F, // 0x4D
    0x7F, 0x04, 0x08, 0x10, 0x7F, // 0x4E
    0x3E, 0x41, 0x41, 0x41, 0x3E, // 0x4F
    0x7F, 0x09, 0x09, 0x09, 0x06, // 0x50
    0x3E, 0x41, 0x51, 0x21, 0x5E, // 0x51
    0x7F, 0x09, 0x19, 0x29, 0x46, // 0x52
    0x26, 0x49, 0x49, 0x49, 0x32, // 0x53
    0x03, 0x01, 0x7F, 0x01, 0x03, // 0x54
    0x3F, 0x40, 0x40, 0x40, 0x3F, // 0x55
    0x1F, 0x20, 0x40, 0x20, 0x1F, // 0x56
    0x3F, 0x40, 0x38, 0x40, 0x3F, // 0x57
    0x63, 0x14, 0x08, 0x14, 0x63, // 0x58
    0x03, 0x04, 0x78, 0x04, 0x03, // 0x59
    0x61, 0x59, 0x49, 0x4D, 0x43, // 0x5A
    0x00, 0x7F, 0x41, 0x41, 0x41, // 0x5B
    0x02, 0x04, 0x08, 0x10, 0x20, // 0x5C
    0x00, 0x41, 0x41, 0x41, 0x7F, // 0x5D
    0x04, 0x02, 0x01, 0x02, 0x04, // 0x5E
    0x40, 0x40, 0x40, 0x40, 0x40, // 0x5F
    0x00, 0x03, 0x07, 0x08, 0x00, // 0x60
    0x20, 0x54, 0x54, 0x78, 0x40, // 0x61
    0x7F, 0x28, 0x44, 0x44, 0x38, // 0x62
    0x38, 0x44, 0x44, 0x44, 0x28, // 0x63
    0x38, 0x44, 0x44, 0x28, 0x7F, // 0x64
    0x38, 0x54, 0x54, 0x54, 0x18, // 0x65
    0x00, 0x08, 0x7E, 0x09, 0x02, // 0x66
    0x18, 0xA4, 0xA4, 0x9C, 0x78, // 0x67
    0x7F, 0x08, 0x04, 0x04, 0x78, // 0x68
    0x00, 0x44, 0x7D, 0x40, 0x00, // 0x69
    0x20, 0x40, 0x40, 0x3D, 0x00, // 0x6A
    0x7F, 0x10, 0x28, 0x44, 0x00, // 0x6B
    0x00, 0x41, 0x7F, 0x40, 0x00, // 0x6C
    0x7C, 0x04, 0x78, 0x04, 0x78, // 0x6D
    0x7C, 0x08, 0x04, 0x04, 0x78, // 0x6E
    0x38, 0x44, 0x44, 0x44, 0x38, // 0x6F
    0xFC, 0x18, 0x24, 0x24, 0x18, // 0x70
    0x18, 0x24, 0x24, 0x18, 0xFC, // 0x71
    0x7C, 0x08, 0x04, 0x04, 0x08, // 0x72
    0x48, 0x54, 0x54, 0x54, 0x24, // 0x73
    0x04, 0x04, 0x3F, 0x44, 0x24, // 0x74
    0x3C, 0x40, 0x40, 0x20, 0x7C, // 0x75
    0x1C, 0x20, 0x40, 0x20, 0x1C, // 0x76
    0x3C, 0x40, 0x30, 0x40, 0x3C, // 0x77
    0x44, 0x28, 0x10, 0x28, 0x44, // 0x78
    0x4C, 0x90, 0x90, 0x90, 0x7C, // 0x79
    0x44, 0x64, 0x54, 0x4C, 0x44, // 0x7A
    0x00, 0x08, 0x36, 0x41, 0x00, // 0x7B
    0x00, 0x00, 0x77, 0x00, 0x00, // 0x7C
    0x00, 0x41, 0x36, 0x08, 0x00, // 0x7D
    0x02, 0x01, 0x02, 0x04, 0x02, // 0x7E
    0x3C, 0x26, 0x23, 0x26, 0x3C, // 0x7F
    0x1E, 0xA1, 0xA1, 0x61, 0x12, // 0x80
    0x3A, 0x40, 0x40, 0x20, 0x7A, // 0x81
    0x38, 0x54, 0x54, 0x55, 0x59, // 0x82
    0x21, 0x55, 0x55, 0x79, 0x41, // 0x83
    0x21, 0x54, 0x54, 0x78, 0x41, // 0x84
    0x21, 0x55, 0x54, 0x78, 0x40, // 0x85
    0x20, 0x54, 0x55, 0x79, 0x40, // 0x86
    0x0C, 0x1E, 0x52, 0x72, 0x12, // 0x87
    0x39, 0x55, 0x55, 0x55, 0x59, // 0x88
    0x39, 0x54, 0x54, 0x54, 0x59, // 0x89
    0x39, 0x55, 0x54, 0x54, 0x58, // 0x8A
    0x00, 0x00, 0x45, 0x7C, 0x41, // 0x8B
    0x00, 0x02, 0x45, 0x7D, 0x42, // 0x8C
    0x00, 0x01, 0x45, 0x7C, 0x40, // 0x8D
    0xF0, 0x29, 0x24, 0x29, 0xF0, // 0x8E  Ä
    0xF0, 0x28, 0x25, 0x28, 0xF0, // 0x8F
    0x7C, 0x54, 0x55, 0x45, 0x00, // 0x90
    0x20, 0x54, 0x54, 0x7C, 0x54, // 0x91
    0x7C, 0x0A, 0x09, 0x7F, 0x49, // 0x92
    0x32, 0x49, 0x49, 0x49, 0x32, // 0x93
    0x32, 0x48, 0x48, 0x48, 0x32, // 0x94
    0x32, 0x4A, 0x48, 0x48, 0x30, // 0x95
    0x3A, 0x41, 0x41, 0x21, 0x7A, // 0x96
    0x3A, 0x42, 0x40, 0x20, 0x78, // 0x97
    0x7f, 0x25, 0x25, 0x25, 0x1a, // 0x00, 0x9D, 0xA0, 0xA0, 0x7D,//0x98++++++++++++++++++++++++ß
    0x39, 0x44, 0x44, 0x44, 0x39, // 0x99
    0x3D, 0x40, 0x40, 0x40, 0x3D, // 0x9A
    0x3C, 0x24, 0xFF, 0x24, 0x24, // 0x9B
    0x48, 0x7E, 0x49, 0x43, 0x66, // 0x9C
    0x2B, 0x2F, 0xFC, 0x2F, 0x2B, // 0x9D
    0xFF, 0x09, 0x29, 0xF6, 0x20, // 0x9E
    0xC0, 0x88, 0x7E, 0x09, 0x03, // 0x9F
    0x20, 0x54, 0x54, 0x79, 0x41, // 0xA0
    0x00, 0x00, 0x44, 0x7D, 0x41, // 0xA1
    0x30, 0x48, 0x48, 0x4A, 0x32, // 0xA2
    0x38, 0x40, 0x40, 0x22, 0x7A, // 0xA3
    0x00, 0x7A, 0x0A, 0x0A, 0x72, // 0xA4
    0x7D, 0x0D, 0x19, 0x31, 0x7D, // 0xA5
    0x26, 0x29, 0x29, 0x2F, 0x28, // 0xA6
    0x26, 0x29, 0x29, 0x29, 0x26, // 0xA7
    0x30, 0x48, 0x4D, 0x40, 0x20, // 0xA8
    0x38, 0x08, 0x08, 0x08, 0x08, // 0xA9
    0x08, 0x08, 0x08, 0x08, 0x38, // 0xAA
    0x2F, 0x10, 0xC8, 0xAC, 0xBA, // 0xAB
    0x2F, 0x10, 0x28, 0x34, 0xFA, // 0xAC
    0x00, 0x00, 0x7B, 0x00, 0x00, // 0xAD
    0x08, 0x14, 0x2A, 0x14, 0x22, // 0xAE
    0x22, 0x14, 0x2A, 0x14, 0x08, // 0xAF
    0xAA, 0x00, 0x55, 0x00, 0xAA, // 0xB0
    0xAA, 0x55, 0xAA, 0x55, 0xAA, // 0xB1
    0x00, 0x00, 0x00, 0xFF, 0x00, // 0xB2
    0x10, 0x10, 0x10, 0xFF, 0x00, // 0xB3
    0x14, 0x14, 0x14, 0xFF, 0x00, // 0xB4
    0x10, 0x10, 0xFF, 0x00, 0xFF, // 0xB5
    0x10, 0x10, 0xF0, 0x10, 0xF0, // 0xB6
    0x14, 0x14, 0x14, 0xFC, 0x00, // 0xB7
    0x14, 0x14, 0xF7, 0x00, 0xFF, // 0xB8
    0x00, 0x00, 0xFF, 0x00, 0xFF, // 0xB9
    0x14, 0x14, 0xF4, 0x04, 0xFC, // 0xBA
    0x14, 0x14, 0x17, 0x10, 0x1F, // 0xBB
    0x10, 0x10, 0x1F, 0x10, 0x1F, // 0xBC
    0x14, 0x14, 0x14, 0x1F, 0x00, // 0xBD
    0x10, 0x10, 0x10, 0xF0, 0x00, // 0xBE
    0x00, 0x00, 0x00, 0x1F, 0x10, // 0xBF
    0x10, 0x10, 0x10, 0x1F, 0x10, // 0xC0
    0x10, 0x10, 0x10, 0xF0, 0x10, // 0xC1
    0x00, 0x00, 0x00, 0xFF, 0x10, // 0xC2
    0x10, 0x10, 0x10, 0x10, 0x10, // 0xC3
    0x10, 0x10, 0x10, 0xFF, 0x10, // 0xC4
    0x00, 0x00, 0x00, 0xFF, 0x14, // 0xC5
    0x00, 0x00, 0xFF, 0x00, 0xFF, // 0xC6
    0x00, 0x00, 0x1F, 0x10, 0x17, // 0xC7
    0x00, 0x00, 0xFC, 0x04, 0xF4, // 0xC8
    0x14, 0x14, 0x17, 0x10, 0x17, // 0xC9
    0x14, 0x14, 0xF4, 0x04, 0xF4, // 0xCA
    0x00, 0x00, 0xFF, 0x00, 0xF7, // 0xCB
    0x14, 0x14, 0x14, 0x14, 0x14, // 0xCC
    0x14, 0x14, 0xF7, 0x00, 0xF7, // 0xCD
    0x14, 0x14, 0x14, 0x17, 0x14, // 0xCE
    0x10, 0x10, 0x1F, 0x10, 0x1F, // 0xCF
    0x14, 0x14, 0x14, 0xF4, 0x14, // 0xD0
    0x10, 0x10, 0xF0, 0x10, 0xF0, // 0xD1
    0x00, 0x00, 0x1F, 0x10, 0x1F, // 0xD2
    0x00, 0x00, 0x00, 0x1F, 0x14, // 0xD3
    0x00, 0x00, 0x00, 0xFC, 0x14, // 0xD4
    0x00, 0x00, 0xF0, 0x10, 0xF0, // 0xD5
    0x10, 0x10, 0xFF, 0x10, 0xFF, // 0xD6
    0x14, 0x14, 0x14, 0xFF, 0x14, // 0xD7
    0x10, 0x10, 0x10, 0x1F, 0x00, // 0xD8
    0x00, 0x00, 0x00, 0xF0, 0x10, // 0xD9
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xDA
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, // 0xDB
    0xFF, 0xFF, 0xFF, 0x00, 0x00, // 0xDC
    0x00, 0x00, 0x00, 0xFF, 0xFF, // 0xDD
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 0xDE
    0x38, 0x44, 0x44, 0x38, 0x44, // 0xDF
    0x7C, 0x2A, 0x2A, 0x3E, 0x14, // 0xE0
    0x7E, 0x02, 0x02, 0x06, 0x06, // 0xE1
    0x02, 0x7E, 0x02, 0x7E, 0x02, // 0xE2
    0x63, 0x55, 0x49, 0x41, 0x63, // 0xE3
    0x38, 0x44, 0x44, 0x3C, 0x04, // 0xE4
    0x40, 0x7E, 0x20, 0x1E, 0x20, // 0xE5
    0x06, 0x02, 0x7E, 0x02, 0x02, // 0xE6
    0x99, 0xA5, 0xE7, 0xA5, 0x99, // 0xE7
    0x1C, 0x2A, 0x49, 0x2A, 0x1C, // 0xE8
    0x4C, 0x72, 0x01, 0x72, 0x4C, // 0xE9
    0x30, 0x4A, 0x4D, 0x4D, 0x30, // 0xEA
    0x30, 0x48, 0x78, 0x48, 0x30, // 0xEB
    0xBC, 0x62, 0x5A, 0x46, 0x3D, // 0xEC
    0x3E, 0x49, 0x49, 0x49, 0x00, // 0xED
    0x7E, 0x01, 0x01, 0x01, 0x7E, // 0xEE
    0x2A, 0x2A, 0x2A, 0x2A, 0x2A, // 0xEF
    0x44, 0x44, 0x5F, 0x44, 0x44, // 0xF0
    0x40, 0x51, 0x4A, 0x44, 0x40, // 0xF1
    0x40, 0x44, 0x4A, 0x51, 0x40, // 0xF2
    0x00, 0x00, 0xFF, 0x01, 0x03, // 0xF3
    0xE0, 0x80, 0xFF, 0x00, 0x00, // 0xF4
    0x08, 0x08, 0x6B, 0x6B, 0x08, // 0xF5
    0x36, 0x12, 0x36, 0x24, 0x36, // 0xF6
    0x06, 0x0F, 0x09, 0x0F, 0x06, // 0xF7
    0x00, 0x00, 0x18, 0x18, 0x00, // 0xF8
    0x00, 0x00, 0x10, 0x10, 0x00, // 0xF9
    0x30, 0x40, 0xFF, 0x01, 0x01, // 0xFA
    0x00, 0x1F, 0x01, 0x01, 0x1E, // 0xFB
    0x00, 0x19, 0x1D, 0x17, 0x12, // 0xFC
    0x00, 0x3C, 0x3C, 0x3C, 0x3C, // 0xFD
    0x00, 0x00, 0x00, 0x00, 0x00  // 0xFE
};

bool last_uni_char = false;
unsigned char last_char;

void setColor(uint32_t x, uint32_t y, uint16_t color)
{

    int height = 490;
    int width = 192;
    if (y >= height)
        return;
    if (x >= width)
        return;
    int position_buffer = ((y * width) + x);
    display_buffer[position_buffer * 2] = color >> 8;
    display_buffer[(position_buffer * 2) + 1] = color & 0xff;
}

void displayRect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t color)
{
    for (int w_t = 0; w_t < w; w_t++)
    {
        for (int h_t = 0; h_t < h; h_t++)
        {
            setColor(x + w_t, y + h_t, color);
        }
    }
    // setAddrWindowDisplay(x, y, w, h);
    // displayColor(color);
}

bool drawChar(uint32_t x, uint32_t y, unsigned char c, uint16_t color, uint16_t bg, uint32_t size)
{
    if (c < 32)
        return false;
    if (c >= 127)
    {
        if (!last_uni_char)
        {
            last_char = c;
            last_uni_char = true;
            return false;
        }
        else
        {
            last_uni_char = false;
            if (last_char == 0xC3)
            {
                switch (c)
                {
                case 0x84: // Ä
                    c = 0x8E;
                    break;
                case 0xA4: // ä
                    c = 0x84;
                    break;
                case 0x96: // Ö
                    c = 0x99;
                    break;
                case 0xB6: // ö
                    c = 0x94;
                    break;
                case 0x9C: // Ü
                    c = 0x9A;
                    break;
                case 0xBC: // ü
                    c = 0x81;
                    break;
                case 0x9F: // ß
                    c = 0x98;
                    break;
                default:
                    return false;
                }
            }
            else if (last_char == 0xF0 && c == 0x9F)
                c = 0x02;
            else
                return false;
        }
    }
    for (int8_t i = 0; i < 5; i++)
    {
        uint8_t line = font57[c * 5 + i];
        for (int8_t j = 0; j < 8; j++, line >>= 1)
        {
            if (line & 1)
            {
                displayRect(x + i * size, y + j * size, size, size, color);
            }
            else if (bg != color)
            {
                displayRect(x + i * size, y + j * size, size, size, bg);
            }
        }
    }
    if (bg != color)
    {
        displayRect(x + 5 * size, y, size, 8 * size, bg);
    }
    return true;
}

size_t my_strlen(const char *str)
{
    size_t i;

    for (i = 0; str[i]; i++)
        ;
    return i;
}

void displayPrintln(uint32_t x, uint32_t y, char text[], uint16_t color, uint16_t bg, uint32_t size)
{
    int tempPosition = 0;
    for (unsigned int f = 0; f < my_strlen(text); f++)
    {
        if (x + (tempPosition * 6 * size) >= 192 - (6 * size))
        {
            x = -(tempPosition * 6 * size);
            y += (8 * size);
        }
        if (drawChar(x + (tempPosition * 6 * size), y, text[f], color, bg, size))
        {
            tempPosition++;
        }
    }
}

#define XFER_MAX_WAIT_MS 100

static bool bXferDone = false;

static void *g_MSPIDisplayHandle;
static void *g_DisplayHandle;

static uint32_t ui32MspiDisplayQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * 12];

static am_devices_mspi_rm69330_config_t QuadDisplayMSPICfg =
    {
        .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4,
        .eClockFreq = AM_HAL_MSPI_CLK_48MHZ,
        .pNBTxnBuf = ui32MspiDisplayQBuffer,
        .ui32NBTxnBufLength = sizeof(ui32MspiDisplayQBuffer) / sizeof(uint32_t),
};

am_devices_display_hw_config_t g_sDispCfg =
    {
        .eIC = DISP_IC_RM69330,
        .eInterface = DISP_IF_QSPI,
        .ui16TEpin = DISPLAY_TE_PIN,
        .ui16ResX = 192,
        .ui16ResY = 490,
        .bFlip = false,
        .ui32Module = DISPLAY_MSPI_INST,
        .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4,
        .eClockFreq = AM_HAL_MSPI_CLK_48MHZ,
        .bClockonD4 = false,
        .ui8DispMspiSelect = 1,
        .eTEType = DISP_TE_GPIO,
        .ui16Offset = 0};

am_hal_gpio_pincfg_t gpio_default =
    {
        .GP.cfg = 3};

am_hal_gpio_pincfg_t g_AM_GPIO_TE_43 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_43_GPIO,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_100K,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

//*****************************************************************************
//
// MSPI1_D0 (37) - MSPI1 data 0.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_D0 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_37_MSPI1_0,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

//*****************************************************************************
//
// MSPI1_D1 (38) - MSPI1 data 1.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_D1 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_38_MSPI1_1,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

//*****************************************************************************
//
// MSPI1_D2 (39) - MSPI1 data 2.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_D2 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_39_MSPI1_2,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

//*****************************************************************************
//
// MSPI1_D3 (40) - MSPI1 data 3.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_D3 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_40_MSPI1_3,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

//*****************************************************************************
//
// MSPI1_CE0 (86) - MSPI1 chip select for device 0
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_CE0 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_86_NCE86,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = AM_HAL_GPIO_NCE_MSPI1CEN0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};
//*****************************************************************************
//
// MSPI1_CE1 (52) - MSPI1 chip select for device 1
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_CE1 =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_52_NCE52,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = AM_HAL_GPIO_NCE_MSPI1CEN1,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

//*****************************************************************************
//
// MSPI1_SCK (45) - MSPI1 clock.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_GPIO_MSPI1_SCK =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_45_MSPI1_8,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

am_hal_gpio_pincfg_t g_AM_GPIO_RESET_11 =
    {
        .GP.cfg = 0x183};
am_hal_gpio_pincfg_t g_AM_GPIO_RESET_41 =
    {
        .GP.cfg = 0x183};

am_hal_gpio_pincfg_t g_AM_GPIO_RESET_11_d =
    {
        .GP.cfg = 0x2093};

am_hal_gpio_pincfg_t g_AM_GPIO_RESET_41_d =
    {
        .GP.cfg = 0x2093};

am_hal_gpio_pincfg_t g_AM_GPIO_RESET_42 =
    {
        .GP.cfg = 0xD03};

#define TE_GPIO_IDX GPIO_NUM2IDX(DISPLAY_TE_PIN)

typedef void (*am_devices_disp_handler_t)(void *);

typedef struct
{
    // the display resolution
    uint16_t ui16ResX;
    uint16_t ui16ResY;
    // Address of vertex
    uint16_t ui16MinX;
    uint16_t ui16MinY;
    // the color mode of application frame-buffer
    am_devices_disp_color_e eColorMode;
} am_devices_display_user_setting_t;

typedef struct
{
    // if frame transfer is pending for TE
    volatile bool bXferPending;
    // if frame trander in progress
    volatile bool bXferBusy;
    // frame transfer information
    uint16_t ui16XferResX;
    uint16_t ui16XferResY;
    uint32_t ui32XferAddress;
    // application callback when frame transfer completes
    am_devices_disp_handler_t fnXferDoneCb;
    void *pArgXferDone;
    // total stripe
    uint32_t total_stripe;

} am_devices_display_tranfer_t;

static am_devices_display_user_setting_t sDispUserSetting = {0};

static am_devices_display_tranfer_t sDispTransfer =
    {
        .bXferPending = false,
        .bXferBusy = false,
        .ui16XferResX = 0,
        .ui16XferResY = 0,
        .ui32XferAddress = 0,
        .fnXferDoneCb = NULL,
        .pArgXferDone = NULL,
};

static void display_transfer_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    sDispTransfer.bXferBusy = false;

    if (sDispTransfer.fnXferDoneCb)
    {
        if (sDispTransfer.total_stripe == 0)
        {
            sDispTransfer.fnXferDoneCb(sDispTransfer.pArgXferDone);
        }
        else
        {
            sDispTransfer.fnXferDoneCb(pCallbackCtxt);
        }
    }

    bXferDone = true;
}

static uint32_t am_devices_display_launch_transfer()
{
    uint32_t ui32Status = AM_DEVICES_DISPLAY_STATUS_SUCCESS;

    uint32_t parts = (sDispTransfer.total_stripe == 0) ? 1 : sDispTransfer.total_stripe;
    uint32_t pixel_size = (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565) ? 2 : 3;
    uint32_t part_size = sDispTransfer.ui16XferResX *
                         (sDispTransfer.ui16XferResY / parts) *
                         pixel_size;

    for (uint32_t i = 0; i < parts; i++)
    {
        uint8_t *ptr = (uint8_t *)sDispTransfer.ui32XferAddress + i * part_size;

        ui32Status = am_devices_mspi_rm69330_nonblocking_write_endian(
            g_DisplayHandle,
            ptr,
            part_size,
            0,
            0,
            (am_hal_mspi_callback_t)display_transfer_complete,
            (void *)i,
            (i != 0),
            sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565);

        if (ui32Status)
        {
            break;
        }
    }
    return ui32Status;
}

uint32_t am_devices_display_set_scanline_recommended_parameter(uint8_t TETimesPerFrame)
{
    return am_devices_mspi_rm69330_set_scanline_recommended_parameter(g_DisplayHandle, TETimesPerFrame);
}

static void am_devices_display_te_handler(void *pvUnused, uint32_t ui32Unused)
{
    //
    // Transfer the frame when TE interrupt arrives.
    //
    if (sDispTransfer.bXferPending)
    {
        am_devices_display_launch_transfer();
        sDispTransfer.bXferPending = false;
    }
}

void am_bsp_disp_pins_enable1(void)
{
    am_hal_gpio_pinconfig(86, g_AM_GPIO_MSPI1_CE0);
    am_hal_gpio_pinconfig(37, g_AM_GPIO_MSPI1_D0);
    am_hal_gpio_pinconfig(38, g_AM_GPIO_MSPI1_D1);
    am_hal_gpio_pinconfig(39, g_AM_GPIO_MSPI1_D2);
    am_hal_gpio_pinconfig(40, g_AM_GPIO_MSPI1_D3);
    am_hal_gpio_pinconfig(45, g_AM_GPIO_MSPI1_SCK);
    am_hal_gpio_pinconfig(43, g_AM_GPIO_TE_43);
}

void am_bsp_disp_pins_disable1(void)
{
    am_hal_gpio_pinconfig(86, gpio_default);
    am_hal_gpio_pinconfig(37, gpio_default);
    am_hal_gpio_pinconfig(38, gpio_default);
    am_hal_gpio_pinconfig(39, gpio_default);
    am_hal_gpio_pinconfig(40, gpio_default);
    am_hal_gpio_pinconfig(45, gpio_default);
    am_hal_gpio_pinconfig(43, gpio_default);
}

int32_t am_devices_display_init(uint16_t ui16ResX, uint16_t ui16ResY, am_devices_disp_color_e eColorMode, bool bEnableTE)
{

    //
    // store the user setting
    //
    if (ui16ResX < g_sDispCfg.ui16ResX)
    {
        sDispUserSetting.ui16ResX = ui16ResX;
    }
    else
    {
        sDispUserSetting.ui16ResX = g_sDispCfg.ui16ResX;
    }

    sDispUserSetting.ui16MinX = ((g_sDispCfg.ui16ResX - sDispUserSetting.ui16ResX) >> 2) << 1;

    if (ui16ResY < g_sDispCfg.ui16ResY)
    {
        sDispUserSetting.ui16ResY = ui16ResY;
    }
    else
    {
        sDispUserSetting.ui16ResY = g_sDispCfg.ui16ResY;
    }

    sDispUserSetting.ui16MinY = ((g_sDispCfg.ui16ResY - sDispUserSetting.ui16ResY) >> 2) << 1;

    sDispUserSetting.eColorMode = eColorMode;
    sDispTransfer.bXferPending = false;
    sDispTransfer.bXferBusy = false;

    //
    // check if the user would like to use TE
    //
    if (!bEnableTE)
    {
        g_sDispCfg.eTEType = DISP_TE_DISABLE;
    }

    //
    // Initialize the display specific GPIO signals.
    //
    am_bsp_disp_pins_enable1();

    uint32_t ui32Status = 0;
    uint8_t ui8Format = AM_DEVICES_MSPI_RM69330_COLOR_MODE_16BIT;

    if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB888)
    {
        ui8Format = AM_DEVICES_MSPI_RM69330_COLOR_MODE_24BIT;
    }
    else if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
    {
        ui8Format = AM_DEVICES_MSPI_RM69330_COLOR_MODE_16BIT;
    }
    //
    // modified default row, column and format parameters.
    //
    am_devices_rm69330_set_parameters(sDispUserSetting.ui16MinX + g_sDispCfg.ui16Offset,
                                      ui16ResX,
                                      sDispUserSetting.ui16MinY,
                                      ui16ResY,
                                      ui8Format);
    //
    // Initialize the MSPI Display
    //
    QuadDisplayMSPICfg.eClockFreq = g_sDispCfg.eClockFreq;
    QuadDisplayMSPICfg.eDeviceConfig = g_sDispCfg.eDeviceConfig;

    ui32Status = am_devices_mspi_rm69330_init(g_sDispCfg.ui32Module,
                                              &QuadDisplayMSPICfg,
                                              &g_DisplayHandle,
                                              &g_MSPIDisplayHandle);
    if (AM_DEVICES_MSPI_RM69330_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_DISPLAY_STATUS_PANEL_ERR;
    }
    NVIC_SetPriority(MSPI1_IRQn, 0x4);
    NVIC_EnableIRQ(MSPI1_IRQn);

    am_devices_mspi_rm69330_display_on(g_DisplayHandle);

    bXferDone = false;
    //
    // Setting default scanline
    //
    am_devices_display_set_scanline_recommended_parameter(1);
    //
    // Enable GPIO TE interrupt
    //
    if (g_sDispCfg.eTEType == DISP_TE_GPIO)
    {
        uint32_t IntNum = g_sDispCfg.ui16TEpin;
        am_hal_gpio_mask_t gpio_mask = AM_HAL_GPIO_MASK_DECLARE_ZERO;
        gpio_mask.U.Msk[GPIO_NUM2IDX(IntNum)] = GPIO_NUM2MSK(IntNum);
        am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, &gpio_mask);
        am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum, (am_hal_gpio_handler_t)am_devices_display_te_handler, NULL);
        am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, (void *)&IntNum);
        NVIC_SetPriority(GPIO0_203F_IRQn, 0x4);
        NVIC_EnableIRQ(GPIO0_203F_IRQn);
    }

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

int send_mode7()
{
    am_util_stdio_printf("Sending mode 7 display config now\r\n");
    uint8_t pData[20];

    pData[0] = 0x70;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 0xC9;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0, pData, 1u);
    pData[0] = 0x80;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 8u, pData, 1u);
    pData[0] = 0xC9;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 9u, pData, 1u);
    pData[0] = 0x80;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x11u, pData, 1u);
    pData[0] = 0x82;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 3;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 9u, pData, 1u);
    pData[0] = 0x1A;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x34u, pData, 1u);
    pData[0] = 0x1B;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x40u, pData, 1u);
    pData[0] = 1;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x17u, pData, 1u);
    pData[0] = 2;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x43u, pData, 1u);
    pData[0] = 0;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u))
        return 1;
    pData[0] = 23;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x80u, pData, 1u))
        return 1;
    pData[0] = 32;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x53u, pData, 1u))
        return 1;
    uint32_t v4 = 0xD7001800;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Au, (uint8_t *)&v4, 4u))
        return 1;
    uint32_t v3 = 0xE9010000;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Bu, (uint8_t *)&v3, 4u))
        return 1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x12u, 0, 0))
        return 1;
    uint32_t v2 = 0xD6001900;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x31u, (uint8_t *)&v2, 4u))
        return 1;
    uint32_t v1 = 0xE8010100;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x30u, (uint8_t *)&v1, 4u))
        return 1;
    pData[0] = 117;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x3Au, pData, 1u))
        return 1;
    pData[0] = 2;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x35u, pData, 1u))
        return 1;
    pData[0] = -1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x51u, pData, 1u))
        return 1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x11u, 0, 0))
        return 1;
    am_util_delay_ms(60);
    return am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x29u, 0, 0) != 0;
}

int send_mode4()
{
    am_util_stdio_printf("Sending mode 4 display config now\r\n");
    uint8_t pData[12];

    pData[0] = 0x20;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 0x5A;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xF4u, pData, 1u);
    pData[0] = 0x59;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xF5u, pData, 1u);
    pData[0] = 0xE0;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 0;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xBu, pData, 1u);
    pData[0] = 0x9B;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Du, pData, 1u);
    pData[0] = 0x1D;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x23u, pData, 1u);
    pData[0] = 0x81;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x24u, pData, 1u);
    pData[0] = 3;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x30u, pData, 1u);
    pData[0] = 0x40;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u))
        return 1;
    pData[0] = 1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x76u, pData, 1u))
        return 1;
    pData[0] = 0;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u))
        return 1;
    pData[0] = 0x80;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xC4u, pData, 1u))
        return 1;
    pData[0] = 0x55;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x3Au, pData, 1u))
        return 1;
    pData[0] = 0;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x35u, pData, 1u))
        return 1;
    pData[0] = 0x20;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x53u, pData, 1u))
        return 1;
    pData[0] = 0xFF;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x51u, pData, 1u))
        return 1;
    pData[0] = 0xFF;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x63u, pData, 1u))
        return 1;
    uint32_t v2 = 0xBF000000;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Au, (uint8_t *)&v2, 4u))
        return 1;
    uint32_t v1 = 0xE9010000;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Bu, (uint8_t *)&v1, 4u))
        return 1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x11u, 0, 0))
        return 1;
    am_util_delay_ms(60);
    return am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x29u, 0, 0) != 0;
}

int send_mode5()
{
    am_util_stdio_printf("Sending mode 5 display config now\r\n");
    uint8_t v4[12];

    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x11, 0, 0))
        return 1;
    am_util_delay_ms(5);
    uint32_t v3 = 0xBF000000;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2A, (uint8_t *)&v3, 4))
        return 1;
    uint32_t v2 = 0xE9010000;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2B, (uint8_t *)&v2, 4))
        return 1;
    uint16_t v1 = 0xE901;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x44, (uint8_t *)&v1, 2))
        return 1;
    v4[0] = 0;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x35, v4, 1))
        return 1;
    v4[0] = 0x53;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x4A, v4, 1))
        return 1;
    v4[0] = 0x17;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x80, v4, 1))
        return 1;
    v4[0] = 0x55;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x3A, v4, 1))
        return 1;
    v4[0] = 0x20;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x53, v4, 1))
        return 1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x29, 0, 0))
        return 1;
    am_util_delay_ms(20);
    return 0;
}

int send_mode6()
{
    am_util_stdio_printf("Sending mode 6 display config now\r\n");
    uint8_t pData[20];

    pData[0] = 0x20;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 0x5A;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xF4u, pData, 1u);
    pData[0] = 0x59;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xF5u, pData, 1u);
    pData[0] = 0x40;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 1;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x98u, pData, 1u);
    pData[0] = 0xE0;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u);
    pData[0] = 0x9E;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x23u, pData, 1u);
    pData[0] = 0x81;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x24u, pData, 1u);
    pData[0] = 3;
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x30u, pData, 1u);
    pData[0] = 0;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xFEu, pData, 1u))
        return 1;
    pData[0] = 0x80;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0xC4u, pData, 1u))
        return 1;
    pData[0] = 0x55;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x3Au, pData, 1u))
        return 1;
    pData[0] = 0;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x35u, pData, 1u))
        return 1;
    pData[0] = 0x20;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x53u, pData, 1u))
        return 1;
    pData[0] = 0xFF;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x51u, pData, 1u))
        return 1;
    pData[0] = 0xFF;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x63u, pData, 1u))
        return 1;
    uint32_t v2 = 0xD7001800;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Au, (uint8_t *)&v2, 4u))
        return 1;
    uint32_t v1 = 0xE9010000;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x2Bu, (uint8_t *)&v1, 4u))
        return 1;
    if (am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x11u, 0, 0))
        return 1;
    am_util_delay_ms(60);
    return am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x29u, 0, 0) != 0;
}

int getDisplayMode(uint32_t displayId)
{
    displayId = displayId & 0xffffff;
    if (displayId == 0x200216)
        return 1;
    if (displayId == 0x200116)
        return 7;
    if (((displayId & 0xffff) == 0x0216) && ((displayId >> 16) - 0x30) <= 1)
        return 2;
    if (displayId == 0x300416)
        return 6;
    if (displayId == 0x400216)
        return 5;
    if (displayId == 0x050A1D)
        return 3;
    if (displayId == 0x050A5A)
        return 4;
    return 0;
}

int dword_100077AC = 0;
void display_pins_irq_something_enable()
{
    uint32_t v0 = 0;
    am_hal_gpio_pinconfig(11, g_AM_GPIO_RESET_11);
    am_hal_gpio_pinconfig(41, g_AM_GPIO_RESET_41);
    v0 = am_hal_interrupt_master_disable();
    (*(volatile unsigned int *)0x40010224) = 2048;
    if (((*(volatile unsigned int *)0x40020000) & 0xFF00) == 0)
    {
        dword_100077AC = 0;
        while (((*(volatile unsigned int *)0x40010214) & 0x800) == 0)
        {
            (*(volatile unsigned int *)0x40010224) = 2048;
            dword_100077AC = 1;
        }
    }
    am_hal_interrupt_master_set(v0);
    am_util_delay_ms(5);
    v0 = am_hal_interrupt_master_disable();
    (*(volatile unsigned int *)0x40010228) = 512;
    if (((*(volatile unsigned int *)0x40020000) & 0xFF00) == 0)
    {
        dword_100077AC = 0;
        while (((*(volatile unsigned int *)0x40010218) & 0x200) == 0)
        {
            (*(volatile unsigned int *)0x40010228) = 512;
            dword_100077AC = 1;
        }
    }
    am_hal_interrupt_master_set(v0);
    am_util_delay_ms(15);
}
void display_pins_irq_something_disable()
{
    uint32_t v0 = 0;
    am_hal_gpio_pinconfig(11, g_AM_GPIO_RESET_11_d);
    am_hal_gpio_pinconfig(41, g_AM_GPIO_RESET_41_d);
    v0 = am_hal_interrupt_master_disable();
    (*(volatile unsigned int *)0x40010238) = 512;
    if (((*(volatile unsigned int *)0x40020000) & 0xFF00) == 0)
    {
        dword_100077AC = 0;
        while (((*(volatile unsigned int *)0x40010218) & 0x200) != 0)
        {
            (*(volatile unsigned int *)0x40010238) = 512;
            dword_100077AC = 1;
        }
    }
    am_hal_interrupt_master_set(v0);
    am_util_delay_ms(10);
    v0 = am_hal_interrupt_master_disable();
    (*(volatile unsigned int *)0x40010234) = 2048;
    if (((*(volatile unsigned int *)0x40020000) & 0xFF00) == 0)
    {
        dword_100077AC = 0;
        while (((*(volatile unsigned int *)0x40010214) & 0x800) != 0)
        {
            (*(volatile unsigned int *)0x40010234) = 2048;
            dword_100077AC = 1;
        }
    }
    am_hal_interrupt_master_set(v0);
    am_util_delay_ms(1);
}

void display_pins_enable(bool state)
{
    if (state)
    {
        am_hal_gpio_pinconfig(42, g_AM_GPIO_RESET_42);
        // display_pins_irq_something_enable();
        pinMode(11, 1);
        pinMode(41, 1);
        digitalWrite(11, 1);
        digitalWrite(41, 1);
        digitalWrite(42, 1);
        am_util_delay_ms(25);
        am_bsp_disp_pins_enable1();
        am_util_delay_ms(1);
    }
    else
    {
        am_util_delay_ms(100);
        digitalWrite(42, 0);
        am_util_delay_ms(1);
        am_bsp_disp_pins_disable1();
        am_util_delay_ms(1);
        // display_pins_irq_something_disable();
    }
}

void setBrightness(uint8_t brightness)
{
    uint8_t dd[1] = {brightness};
    am_devices_mspi_rm69330_command_write(g_DisplayHandle, 0x51u, dd, 1);
}

static uint32_t am_devices_display_transfer_frame_inter(void)
{
    // am_devices_display_prepare_transfer();
    sDispTransfer.bXferPending = true;
    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

uint32_t am_devices_display_transfer_frame(uint16_t ui16ResX, uint16_t ui16ResY, uint32_t ui32Address, am_devices_disp_handler_t fnXferDoneCb, void *pArgXferDone)
{
    if (sDispTransfer.bXferBusy)
    {
        return AM_DEVICES_DISPLAY_STATUS_TRY_AGAIN;
    }

    //
    // Record the transfer setting
    //
    sDispTransfer.bXferBusy = true;
    sDispTransfer.bXferPending = false;
    sDispTransfer.ui16XferResX = ui16ResX;
    sDispTransfer.ui16XferResY = ui16ResY;
    sDispTransfer.ui32XferAddress = ui32Address;
    sDispTransfer.fnXferDoneCb = fnXferDoneCb;
    sDispTransfer.pArgXferDone = pArgXferDone;
    sDispTransfer.total_stripe = 0;

    return am_devices_display_transfer_frame_inter();
}

uint16_t x_here, y_here;

void set_xy_here(uint16_t x_now, uint16_t y_now)
{
    x_here = x_now;
    y_here = y_now;
    displayPrintln(5, 300, (char *)"ATC1441", 0x1A1E, 0x0000, 4);
    char time_string[14];
    sprintf(time_string, "X %i  ", x_here);
    displayPrintln(5, 340, time_string, 0xFBA8, 0x0000, 4);
    sprintf(time_string, "Y %i  ", y_here);
    displayPrintln(5, 380, time_string, 0xFBA8, 0x0000, 4);
    am_devices_display_transfer_frame(490, 192, (uint32_t)&display_buffer, NULL, NULL);
}

uint16_t light_here;
uint16_t light_small_here;
void set_light_here(uint16_t light_value, uint16_t light_small)
{
    light_here = light_value;
    light_small_here = light_small;
    displayPrintln(5, 100, (char *)"ATC1441", 0x1A1E, 0x0000, 4);
    displayPrintln(5, 260, (char *)"Light", 0x1A1E, 0x0000, 4);
    displayPrintln(5, 300, (char *)"Sensor", 0x1A1E, 0x0000, 4);
    char time_string[14];
    sprintf(time_string, "%i   ", light_here);
    displayPrintln(5, 340, time_string, 0xFBA8, 0x0000, 4);
    sprintf(time_string, "%i   ", light_small_here);
    displayPrintln(5, 380, time_string, 0xFBA8, 0x0000, 4);
    am_devices_display_transfer_frame(490, 192, (uint32_t)&display_buffer, NULL, NULL);
}

void display_buffset(uint16_t data)
{
    for (int i = 0; i < sizeof(display_buffer) / 2; i++)
    {
        display_buffer[i * 2] = data >> 8;
        display_buffer[(i * 2) + 1] = data & 0xff;
    }
    am_devices_display_transfer_frame(490, 192, (uint32_t)&display_buffer, NULL, NULL);
}

void init_display()
{
    display_pins_enable(1);
    am_util_stdio_printf("Init display start\r\n");
    int returnVal = am_devices_display_init(192, 490, COLOR_FORMAT_RGB565, true);
    am_util_stdio_printf("Init display mid %i \r\n", returnVal);

    uint32_t data;
    returnVal = am_devices_mspi_rm69330_read_id(g_DisplayHandle, (uint32_t *)&data);
    am_util_stdio_printf("Init display done %i - %06X\r\n", returnVal, data & 0xffffff);
    if (returnVal)
    {
        am_util_stdio_printf("Init display failed!!!\r\n");
    }
    int displayMode = getDisplayMode(data);
    switch (displayMode)
    {
    case 1:
    case 7:
        send_mode7();
        break;
    case 2:
    case 4:
        send_mode4();
        break;
    case 5:
        send_mode5();
        break;
    case 6:
        send_mode6();
        break;
    default:
        break;
    }
    setBrightness(170);

    am_util_delay_ms(10);
    uint16_t X_start = 0;
    if (displayMode == 1 || displayMode == 3 || displayMode == 6 || displayMode == 7)
        X_start += 24;
    am_devices_mspi_rm69330_set_transfer_window(g_DisplayHandle, X_start, 192, 0, 490);
    am_util_delay_ms(10);

    display_buffset(0);
}

void end_display()
{
    am_util_stdio_printf("END Displays\r\n");
    display_pins_enable(0);
    am_util_delay_ms(150);
    am_devices_mspi_rm69330_term(g_DisplayHandle);
}

void am_mspi1_isr(void)
{
    uint32_t ui32Status;
    am_hal_mspi_interrupt_status_get(g_MSPIDisplayHandle, &ui32Status, false);
    am_hal_mspi_interrupt_clear(g_MSPIDisplayHandle, ui32Status);
    am_hal_mspi_interrupt_service(g_MSPIDisplayHandle, ui32Status);
}

/*void am_gpio0_001f_isr(void)
{
    uint32_t ui32IntStatus;
    am_hal_gpio_interrupt_irq_status_get(GPIO0_001F_IRQn, true, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_service(GPIO0_001F_IRQn, ui32IntStatus);
}
*/
void am_gpio0_203f_isr(void)
{
    uint32_t ui32IntStatus;
    am_hal_gpio_interrupt_irq_status_get(GPIO0_203F_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO0_203F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_service(GPIO0_203F_IRQn, ui32IntStatus);
}
