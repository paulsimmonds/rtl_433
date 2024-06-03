/* Oil tank monitor using Si4320 framed FSK protocol
 *
 * Tested devices:
 * Dunraven OilPal (which is supposed to be the same as a Proteus)
 * Apollo Smart Sonic
 *
 * Copyright © 2015 David Woodhouse
 * Copyright © 2016 Nick Randell
 * Copyright @ 2021 Paul Simmonds
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "decoder.h"

// Start of frame preamble is 01011000
static const unsigned char preamble_pattern = 0x58;
static const int preamble_length = 8;

// End of frame is 00xxxxxx or 11xxxxxx depending on final data bit
static const unsigned char postamble_pattern[2] = {0x00, 0xc0};

static int oil_proteus_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t *b;
    uint32_t unit_id;
    uint16_t depth = 0;
    uint16_t binding_countdown = 0;
    uint8_t flags;
    uint8_t maybetemp;
    double temperature;
    data_t *data;
    unsigned bitpos = 0;
    bitbuffer_t databits = {0};
    int events = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, &preamble_pattern, preamble_length)) + 136 <=
	   bitbuffer->bits_per_row[0])
    {

	// Skip the matched preamble bits to point to the data
	bitpos += preamble_length;

	bitpos = bitbuffer_manchester_decode(bitbuffer, 0, bitpos, &databits, 64);
	if (databits.bits_per_row[0] != 64)
	    continue;

	b = databits.bb[0];

	// Check for postamble, depending on last data bit
	if (bitbuffer_search(bitbuffer, 0, bitpos, &postamble_pattern[b[7] & 1], 2) != bitpos)
	    continue;

	if (b[7] != crc8le(b, 7, 0x31, 0))
	    continue;

	// The unit ID changes when you rebind by holding a magnet to the
	// sensor for long enough; it seems to be time-based.
	unit_id = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];

	// Not sure what these do
	// 0x80
	// 0x40: Leak/theft alarm
	// Bottom 6 bits not sure
	flags = b[4];

	// Think temperature may be in celcius
	maybetemp = b[5];
	temperature = (double)(maybetemp) / 4.0;
	if (flags & 1) {
	    // When binding, the countdown counts up from 0x51 to 0x5a
	    // (as long as you hold the magnet to it for long enough)
	    // before the device ID changes. The receiver unit needs
	    // to receive this *strongly* in order to change its
	    // allegiance.
	    binding_countdown = b[6];
	}
	
	// A depth reading of zero indicates no reading. Even with
	// the sensor flat down on a table, it still reads about 13.
	depth = b[6];
		
	data = data_make(
			 "model", "", DATA_STRING, "Oil-Proteus",
			 "id", "", DATA_FORMAT, "%06x", DATA_INT, unit_id,
			 "flags", "", DATA_FORMAT, "%02x", DATA_INT, flags,
			 "maybetemp", "", DATA_INT, maybetemp,
			 "temperature_C", "", DATA_DOUBLE, temperature,
			 "binding_countdown", "", DATA_INT, binding_countdown,
			 "depth", "", DATA_INT, depth,
			 NULL);
	decoder_output_data(decoder, data);
	events++;
    }
    return events;
}

static char *output_fields[] = {
    "model",
    "id",
    "flags",
    "maybetemp",
    "temperature_C",
    "binding_countdown",
    "depth",
    NULL};

r_device oil_proteus = {
    .name = "Dunraven OilPal / Proteus",
    .modulation = FSK_PULSE_PCM,
    .short_width = 500,
    .long_width = 500, // NRZ
    .reset_limit = 4000,
    .decode_fn = &oil_proteus_callback,
    .disabled = 0,
    .fields = output_fields,
};
