#pragma once

#include <cstdint>

//--------------------------------------
enum NmeaMsgId
{
    UBX_NMEA_GGA = 0x00, // Глобальное позиционирование (GGA)
    UBX_NMEA_GLL = 0x01, // Географическая широта/долгота (GLL)
    UBX_NMEA_GSA = 0x02, // Режим и геометрия определения (GSA)
    UBX_NMEA_GSV = 0x03, // Информация о спутниках (GSV)
    UBX_NMEA_RMC = 0x04, // Рекомендуемое минимальное навигационное сообщение (RMC)
    UBX_NMEA_VTG = 0x05, // Информация о скорости относительно земли (VTG)
    UBX_NMEA_ZDA = 0x08  // Временная информация (ZDA)
};
//--------------------------------------


//--------------------------------------
uint8_t ubx_msg_enable(uint8_t* buf, uint8_t id, uint8_t rate_devider)
{
    // UBX header: 0xB5 0x62
    buf[0] = 0xB5;
    buf[1] = 0x62;
    // Message Class и ID для CFG-MSG
    buf[2] = 0x06; // CFG
    buf[3] = 0x01; // CFG-MSG
    // Длина полезной нагрузки: 8 байт (little-endian)
    buf[4] = 0x08; // LSB
    buf[5] = 0x00; // MSB
    // Полезная нагрузка (8 байт):
    // Byte 0: msgClass для NMEA = 0xF0
    buf[6] = 0xF0;
    // Byte 1: msgID (переданный параметр id)
    buf[7] = id;
    // Byte 2: rateI2C = 0 (отключено)
    buf[8] = 0x00;
    // Byte 3: rateUART1 = rate (например, 4 для 4 Гц)
    buf[9] = rate_devider;
    // Byte 4: rateUSB = 0
    buf[10] = 0x00;
    // Byte 5: rateSPI = 0
    buf[11] = 0x00;
    // Byte 6-7: Reserved, всегда 0
    buf[12] = 0x00;
    buf[13] = 0x00;

    // Вычисляем контрольные суммы (суммируем байты с buf[2] по buf[13])
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 2; i < 14; i++)
    {
      ck_a = ck_a + buf[i];
      ck_b = ck_b + ck_a;
    }

    buf[14] = ck_a;
    buf[15] = ck_b;

    return 16;
}
//--------------------------------------




//--------------------------------------
uint8_t ubx_set_rate(uint8_t* buf, uint16_t rate)
{
    // UBX header: 0xB5 0x62
    buf[0] = 0xB5;
    buf[1] = 0x62;
    // Message Class и ID для CFG-MSG
    buf[2] = 0x06; // CFG
    buf[3] = 0x08; // CFG-RATE
    // Длина полезной нагрузки: 6 байт (little-endian)
    buf[4] = 0x06; // LSB
    buf[5] = 0x00; // MSB
    // MeasRate
    buf[6] = (uint8_t)(rate & 0xFF); // LSB
    buf[7] = (uint8_t)(rate >> 8); // MSB
    // NavRate = 1
    buf[8] = 0x01;
    buf[9] = 0x00;
    // time source UTC
    buf[10] = 0x00;
    buf[11] = 0x00;

    // Вычисляем контрольные суммы (суммируем байты с buf[2] по buf[11])
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 2; i < 12; i++)
    {
      ck_a = ck_a + buf[i];
      ck_b = ck_b + ck_a;
    }

    buf[12] = ck_a;
    buf[13] = ck_b;

    return 14;
}
//--------------------------------------
