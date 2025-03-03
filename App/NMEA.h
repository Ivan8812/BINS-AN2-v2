#pragma once

#include <cstdint>


class NMEA
{
public:
  // Определение состояний конечного автомата
  enum state_t
  {
      WAIT_FOR_START_BYTE,
      RECEIVING_BODY,           // Прием символов между '$' и '*'
      RECEIVING_CHECKSUM_HIGH,  // Прием первой шестнадцатеричной цифры контрольной суммы
      RECEIVING_CHECKSUM_LOW,   // Прием второй шестнадцатеричной цифры контрольной суммы
      WAIT_FOR_END,             // Ожидание завершающих символов (например, '\r' или '\n')
      NMEA_RECEIVED,
      BROKEN_NMEA_STRING
  };

  static constexpr uint32_t MAX_LENGTH = 91;

  NMEA() { reset(); }

    // Метод для побайтовой обработки входящего символа.
    // Возвращает текущее состояние автомата.
  state_t parse(char c)
  {
    switch (state)
    {
    case NMEA_RECEIVED:
    case BROKEN_NMEA_STRING:
    default:
      state = WAIT_FOR_START_BYTE;
      [[fallthrough]];

    case WAIT_FOR_START_BYTE:
      if (c == '$')
      {
        reset();  // Сброс предыдущего сообщения
        state = RECEIVING_BODY;
        buffer[pos++] = c;
        // Символ '$' не включается в вычисление контрольной суммы
      }
      break;

    case RECEIVING_BODY:
      if (pos >= MAX_LENGTH)
      {
        state = BROKEN_NMEA_STRING;
        pos = 0;
        break;
      }
      if (c == '*')
      {
        // Символ '*' сигнализирует о переходе к приему контрольной суммы.
        buffer[pos++] = c;
        state = RECEIVING_CHECKSUM_HIGH;
      }
      else
      {
        // Обновляем контрольную сумму (XOR всех символов между '$' и '*')
        running_checksum ^= static_cast<uint8_t>(c);
        buffer[pos++] = c;
      }
      break;

    case RECEIVING_CHECKSUM_HIGH:
      if (pos >= MAX_LENGTH)
      {
        state = BROKEN_NMEA_STRING;
        pos = 0;
        break;
      }
      {
        int digit = hex_digit(c);
        if (digit < 0)
        {
          state = BROKEN_NMEA_STRING;
          pos = 0;
          break;
        }

        // Сохраняем первую цифру, сдвигая её в старший полубайт
        provided_checksum = static_cast<uint8_t>(digit << 4);
        buffer[pos++] = c;
        state = RECEIVING_CHECKSUM_LOW;
      }
      break;

    case RECEIVING_CHECKSUM_LOW:
      if (pos >= MAX_LENGTH)
      {
        state = BROKEN_NMEA_STRING;
        pos = 0;
        break;
      }
      {
        int digit = hex_digit(c);
        if (digit < 0)
        {
          state = BROKEN_NMEA_STRING;
          pos = 0;
          break;
        }

        // Сохраняем вторую цифру, объединяя с предыдущей
        provided_checksum |= static_cast<uint8_t>(digit);
        buffer[pos++] = c;
        state = WAIT_FOR_END;
      }
      break;

    case WAIT_FOR_END:
      if (pos >= MAX_LENGTH)
      {
        state = BROKEN_NMEA_STRING;
        pos = 0;
        break;
      }
      if (c == '\r')
      {
        buffer[pos++] = c;
        // Если получен '\r', остаемся в состоянии ожидания до '\n'
      }
      else if (c == '\n')
      {
        buffer[pos++] = c;
        buffer[pos++] = '\0';
        // По получении символа '\n' завершаем прием и проводим валидацию
        state = (running_checksum == provided_checksum) ? NMEA_RECEIVED : BROKEN_NMEA_STRING;
      }
      else
      {
        // Любой другой символ в завершающей части считается ошибочным.
        state = BROKEN_NMEA_STRING;
        pos = 0;
      }
      break;
    }
    return state;
  }

  // Возвращает принятое сообщение в виде C-строки
  uint32_t get_message(char* msg) const
  {
    if((state != NMEA_RECEIVED) || (msg == nullptr))
      return 0;

    for(uint32_t i=0; i<pos; i++)
      msg[i] = buffer[i];
    return pos;
  }


  // Сброс автомата в исходное состояние
  void reset()
  {
    state = WAIT_FOR_START_BYTE;
    pos = 0;
    running_checksum = 0;
    provided_checksum = 0;
    buffer[0] = '\0';
  }

  state_t get_state() const { return state; }

private:
  // Преобразование шестнадцатеричного символа в число.
  static int hex_digit(char ch)
  {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
  }

  state_t state;           // Текущее состояние автомата
  uint32_t pos;           // Текущая позиция в буфере
  char buffer[MAX_LENGTH + 1]; // Фиксированный буфер (+1 для завершающего нуля)

  // Переменные для вычисления контрольной суммы "на лету"
  uint8_t running_checksum;   // Вычисляемая XOR контрольная сумма (символы между '$' и '*')
  uint8_t provided_checksum;  // Принятая контрольная сумма (сформированная из двух шестнадцатеричных цифр)
};

