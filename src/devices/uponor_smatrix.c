/** @file
    Decoder for Uponor Smatrix floor heating thermostats (T-165 and compatible).

    Copyright (C) 2026 Thomas Passer <thomas@passer.ws>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Uponor Smatrix Base floor heating thermostat, model T-165.

- Modulation: FSK PCM
- Frequency: 868.3 MHz
- Symbol rate: 26 µs/bit (~38.4 kbps)
- Preamble: {32} 0xd5555555
- Sync word: {32} 0x69c8e9c8
- Payload: ~228 bits (short temperature packets)

Packet layout (bytes after sync word):

    Byte  0-1   : 0x8c 0x8c  frame type marker (short packet)
    Byte  2     : 0x80
    Bytes 3-8   : device serial number (6 bytes)
    Byte  11    : temperature bits 8-1 (MSBs of 9-bit value)
    Byte  12    : bit 7 = temperature bit 0 (LSB)
    Bytes 24-27 : checksum (proprietary algorithm, not validated)

Temperature encoding (9-bit integer):

    val9   = byte[11] * 2 + (byte[12] >> 7)
    temp_C = floor((val9 * 9 + 1684) / 16) / 10

Step size: 9/160 °C ≈ 0.05625 °C per LSB.
Offset:    1684/160 °C = 10.525 °C.

To capture raw packets for analysis:

    rtl_433 -f 868.3M -X 'n=Uponor,m=FSK_PCM,s=26,l=26,r=1000,preamble={32}0x69c8e9c8'
*/

#include "decoder.h"

static int uponor_smatrix_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const sync[]    = {0x69, 0xc8, 0xe9, 0xc8};
    int const sync_bits     = sizeof(sync) * 8;
    int const payload_bytes = 28;
    int const payload_bits  = payload_bytes * 8;

    int decoded = 0;

    for (int row = 0; row < bitbuffer->num_rows; row++) {
        unsigned const start_pos = bitbuffer_search(bitbuffer, row, 0, sync, sync_bits);
        if (start_pos == bitbuffer->bits_per_row[row]) {
            continue; // sync word not found
        }

        unsigned const payload_pos = start_pos + sync_bits;
        if (bitbuffer->bits_per_row[row] < payload_pos + payload_bits) {
            continue; // packet truncated
        }

        uint8_t msg[28];
        bitbuffer_extract_bytes(bitbuffer, row, payload_pos, msg, payload_bits);

        decoder_log_bitrow(decoder, 2, __func__, msg, payload_bits, "payload");

        // Short temperature packet starts with 0x8c 0x8c 0x80
        if (msg[0] != 0x8c || msg[1] != 0x8c || msg[2] != 0x80) {
            continue;
        }

        // 9-bit temperature: byte 11 holds bits 8-1, MSB of byte 12 is bit 0
        int const val9        = msg[11] * 2 + ((msg[12] >> 7) & 1);
        int const temp_tenths = (val9 * 9 + 1684) / 16; // floor division, units of 0.1 °C
        double const temp_c   = temp_tenths / 10.0;

        if (temp_c < -10.0 || temp_c > 60.0) {
            continue;
        }

        // Device serial: bytes 3-8 as a 12-char hex string
        char id_str[13];
        snprintf(id_str, sizeof(id_str), "%02x%02x%02x%02x%02x%02x",
                msg[3], msg[4], msg[5], msg[6], msg[7], msg[8]);

        /* clang-format off */
        data_t *data = data_make(
                "model",         "",            DATA_STRING, "Uponor-Smatrix-T165",
                "id",            "Device ID",   DATA_STRING, id_str,
                "temperature_C", "Temperature", DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_c,
                NULL);
        /* clang-format on */

        decoder_output_data(decoder, data);
        decoded++;
    }

    return decoded > 0 ? decoded : DECODE_ABORT_EARLY;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "temperature_C",
        NULL,
};

r_device const uponor_smatrix = {
        .name        = "Uponor Smatrix T-165 thermostat",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 26,
        .long_width  = 26,
        .reset_limit = 1000,
        .decode_fn   = &uponor_smatrix_decode,
        .fields      = output_fields,
};
