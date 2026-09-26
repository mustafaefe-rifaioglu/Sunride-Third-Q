#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

/*
In this project I have intended to save bits in several different ways.

My assumptions are:
- Acceleration is finite and stays between -100 and +100.
- Temperature is an unsigned raw code from 0 to 255.
- The first timestamp is 0 and each subsequent record is exactly
  100 ms later, with unsigned 16-bit timestamp rollover.
- The decoder receives all records in order from the beginning.
*/

_Static_assert(CHAR_BIT == 8, "Eight-bit bytes required");
_Static_assert(INT_MAX >= 65535, "int must hold the timestamp");

/*
32-bit layout:
Bit 31       : NO2
Bits 30-16   : Acceleration
Bits 15-8    : Barometer
Bits 7-0     : Temperature
*/

typedef struct
{
    uint16_t next_timestamp;
} CodecState;

typedef struct
{
    int timestamp_msec;
    int NO2_data;
    int Baro_data;
    int Temp_data;
    float accel;
} SensorData;

bool encode_data(
    CodecState *state,
    int timestamp_msec,
    int NO2_data,
    int Baro_data,
    int Temp_data,
    float accel,
    uint8_t out[4])
{
    if (state == NULL || out == NULL)
    {
        return false;
    }

    /* Check that the inputs meet the design assumptions. */
    if (timestamp_msec != (int)state->next_timestamp ||
        (NO2_data != 0 && NO2_data != 23) ||
        Baro_data < 0 || Baro_data > 255 ||
        Temp_data < 0 || Temp_data > 255 ||
        !isfinite(accel) ||
        accel < -100.0f || accel > 100.0f)
    {
        return false;
    }

    /*
    One of the main problems was how much data accel was taking:
    its original 32-bit floating-point representation takes 4 bytes.

    Because of that, I first convert it to a scaled integer.
    Multiplying by 100 and rounding retains steps of 0.01.

    Assuming acceleration stays between -100 and +100,
    the scaled integer stays between -10000 and +10000.
    This fits in a signed 15-bit field.

    The int variable itself is not compressed; the saving happens
    when its representation is packed into the output.
    */
    int accel_int = (int)lroundf(accel * 100.0f);

    /*
    Also, considering that NO2 data has only two possible states:
    "0 if no NO2, 23 if NO2 triggered",
    we can represent it using one bit instead of the five bits
    needed to store the numeric value 23 directly. So, I will store 1
    if NO2 is 23 and 0 if NO2 is 0. Making it an Boolean value, which is
     more efficient than storing the actual number.
    */

    uint32_t NO2_bit = (NO2_data == 23) ? 1u : 0u;

    /*
    To save timestamp data, I take it from the record sequence
    instead of storing it in the four bytes.

    This assumes, as it was said, recording begins at 0 ms and records are made
    exactly every 100 ms.

    Both encoder and decoder maintain their own counter.
    */
    uint32_t accel_bits =
        (uint32_t)accel_int & UINT32_C(0x7FFF);

    /* Position each field and combine them into 32 bits. */
    uint32_t packed =
        (NO2_bit << 31) |
        (accel_bits << 16) |
        ((uint32_t)Baro_data << 8) |
        (uint32_t)Temp_data;

    /* Split the word into four bytes, lowest byte first. */
    out[0] = (uint8_t)(packed & UINT32_C(0xFF));
    out[1] = (uint8_t)((packed >> 8) & UINT32_C(0xFF));
    out[2] = (uint8_t)((packed >> 16) & UINT32_C(0xFF));
    out[3] = (uint8_t)((packed >> 24) & UINT32_C(0xFF));

    /* Advance time only after successfully encoding a record. */
    state->next_timestamp =
        (uint16_t)((uint32_t)state->next_timestamp + 100u);

    return true;
}

bool decode_data(
    CodecState *state,
    const uint8_t in[4],
    SensorData *result)
{
    if (state == NULL || in == NULL || result == NULL)
    {
        return false;
    }

    // Combine the four stored bytes into a 32-bit word.
    uint32_t packed =
        (uint32_t)in[0] |
        ((uint32_t)in[1] << 8) |
        ((uint32_t)in[2] << 16) |
        ((uint32_t)in[3] << 24);

    // Extract the fields using shifts and masks.
    int Temp_data = (int)(packed & UINT32_C(0xFF));
    int Baro_data =
        (int)((packed >> 8) & UINT32_C(0xFF));

    // Restore the original NO2 labels: either 0 or 23.
    int NO2_data =
        ((packed >> 31) & 1u) ? 23 : 0;

    uint32_t accel_bits =
        (packed >> 16) & UINT32_C(0x7FFF);

    int accel_int = (int)accel_bits;

    // The highest bit of the 15-bit acceleration field is its sign.
    // Restore negative values from the two's-complement encoding.

    if ((accel_bits & UINT32_C(0x4000)) != 0u)
    {
        accel_int -= 32768;
    }

    if (accel_int < -10000 || accel_int > 10000)
    {
        return false;
    }

    result->timestamp_msec = (int)state->next_timestamp;
    result->NO2_data = NO2_data;
    result->Baro_data = Baro_data;
    result->Temp_data = Temp_data;

    result->accel = (float)accel_int / 100.0f;

    state->next_timestamp =
        (uint16_t)((uint32_t)state->next_timestamp + 100u);

    return true;
}

int main(void)
{

    int timestamp_msec = 0;
    int NO2_data = 23;
    int Baro_data = 150;
    int Temp_data = 80;
    float accel = -12.347f;

    CodecState encoder_state = {0};
    CodecState decoder_state = {0};

    uint8_t bytes[4];
    SensorData recovered;

    if (!encode_data(&encoder_state,
                     timestamp_msec, NO2_data,
                     Baro_data, Temp_data, accel, bytes))
    {
        printf("Encoding failed: invalid input or timing.\n");
        return 1;
    }

    printf("Four encoded bytes: ");
    for (unsigned i = 0; i < 4; ++i)
    {
        printf("%02X ", (unsigned)bytes[i]);
    }
    printf("\n");

    if (!decode_data(&decoder_state, bytes, &recovered))
    {
        printf("Decoding failed.\n");
        return 1;
    }

    printf("Timestamp: %d ms\n", recovered.timestamp_msec);
    printf("NO2: %d\n", recovered.NO2_data);
    printf("Barometer code: %d\n", recovered.Baro_data);
    printf("Temperature code: %d\n", recovered.Temp_data);
    printf("Acceleration: %.2f\n", recovered.accel);

    return 0;
}

// I want to sadly say that I am not quite proficient enough in C and because
// of that have used AI assistance to an degree to help me code the project itself.
// I did do the thinking and coming up with the logic and design of the project,
// but I did use AI to help me with some of the coding itself.