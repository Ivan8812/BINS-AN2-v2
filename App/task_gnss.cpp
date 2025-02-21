#include <app.h>
#include <ubx_cfg_msg.h>
#include <NMEA.h>
#include <optional>
#include <cstring>

using namespace std;

//--------------------------------------
typedef struct
{
  std::optional<float> timestamp;
  std::optional<int32_t> lat;
  std::optional<int32_t> lon;
  std::optional<int32_t> alt;
  std::optional<uint8_t> quality;
} position_t;
//--------------------------------------
typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} date_time_t;
//--------------------------------------


//--------------------------------------
static bool parse_gga(char *sentence, position_t &pos);
static bool parse_zda(char *sentence, date_time_t &dt);
static int tokenize(char *str, char *tokens[], int maxTokens);
//--------------------------------------


//--------------------------------------
char nmea_string[NMEA::MAX_NMEA_LENGTH+1];
//--------------------------------------


//------------------------------------------------------------------------------
void task_func_gnss(void *arg)
{
  HAL_GPIO_WritePin(RST_GNSS_GPIO_Port, RST_GNSS_Pin, GPIO_PIN_SET);
  vTaskDelay(500);

  enum{BUFSIZE = 16};
  uint8_t out_buf[BUFSIZE];

  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_set_rate(out_buf, 200), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GGA, 1), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GLL, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GSA, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GSV, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_RMC, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_VTG, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_ZDA, 5), HAL_MAX_DELAY);

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    taskENTER_CRITICAL();
    auto len = strlen(nmea_string);
    if(len > NMEA::MAX_NMEA_LENGTH)
    {
      taskEXIT_CRITICAL();
      continue;
    }
    char current_str[len];
    memcpy(current_str, nmea_string, len);
    taskEXIT_CRITICAL();

    if(strncmp(&current_str[3], "GGA", 3) == 0)
    {
      position_t pos{};
      if(parse_gga(current_str, pos))
      {
        can_msg_position_t msg = {.state={.raw = 0x0}};
        msg.state.bins_ok = bins_status.bins_ok;
        msg.state.valid = (pos.quality && (*pos.quality != CAN_POS_QUAL_GNSS_INVALID)) ? 1 : 0;
        msg.quality = pos.quality.value_or(CAN_POS_QUAL_GNSS_INVALID);

        if(pos.lat)
        {
          static uint8_t cntr = 0;
          msg.cntr = cntr++;
          msg.value = *pos.lat;
          if(bins_status.bins_standalone)
            can_send_dat(CAN_MSG_BINS2_LATITUDE, &msg, sizeof(msg));
          else
            can_send_dat(CAN_MSG_BINS1_LATITUDE, &msg, sizeof(msg));
        }

        if(pos.lon)
        {
          static uint8_t cntr = 0;
          msg.cntr = cntr++;
          msg.value = *pos.lon;
          if(bins_status.bins_standalone)
            can_send_dat(CAN_MSG_BINS2_LONGITUDE, &msg, sizeof(msg));
          else
            can_send_dat(CAN_MSG_BINS1_LONGITUDE, &msg, sizeof(msg));
        }

        if(pos.alt)
        {
          static uint8_t cntr = 0;
          msg.cntr = cntr++;
          msg.value = *pos.alt;
          if(bins_status.bins_standalone)
            can_send_dat(CAN_MSG_BINS2_GNSS_HEIGHT, &msg, sizeof(msg));
          else
            can_send_dat(CAN_MSG_BINS1_GNSS_HEIGHT, &msg, sizeof(msg));
        }

#if RS422_LOG_OUT
        opack_gnss_t pack;
        pack.systime = systime;
        pack.timestamp = pos.timestamp.value_or(0.0f);
        pack.alt = pos.alt.value_or(0.0f);
        pack.lat = pos.lat.value_or(0.0f);
        pack.lon = pos.lon.value_or(0.0f);
        pack.quality = pos.quality.value_or(0x0);
        packer.send(PID_GNSS, &pack, sizeof(pack));
#endif

      }
    }
    else if(strncmp(&current_str[3], "ZDA", 3) == 0)
    {
      date_time_t time{};
      if(parse_zda(current_str, time))
      {
        can_msg_time_t msg = {.state={.raw = 0x0}};
        msg.state.bins_ok = bins_status.bins_ok;
        msg.state.valid = true;
        msg.year = time.year;
        msg.month = time.month;
        msg.day = time.day;
        msg.hour = time.hour;
        msg.min = time.min;
        msg.sec = time.sec;
        if(bins_status.bins_standalone)
          can_send_dat(CAN_MSG_BINS2_TIME_DATE, &msg, sizeof(msg));
        else
          can_send_dat(CAN_MSG_BINS1_TIME_DATE, &msg, sizeof(msg));
      }
    }
  }
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
bool parse_gga(char *sentence, position_t &pos)
{
  // Пропускаем символ '$'
  char *start = sentence;
  if (*start == '$')
      start++;

  // Ищем символ '*' – конец полезной части сообщения
  const char *star = strchr(start, '*');
  if (!star)
    return false;

  // обрезаем часть строки с контрольной суммой
  start[star-start] = '\0';

  // Разбиваем строку на токены, учитывая пустые поля
  char *tokens[16] = {0};
  int tokenCount = tokenize(start, tokens, 16);
  // Можно добавить проверку минимального количества токенов, если необходимо.
  if (tokenCount < 13)
    return false;

  // обработка времени (токен 1) в формате hhmmss.ss
  if(tokens[1] && tokens[1][0])
    pos.timestamp = atof(tokens[1]);
  else
    pos.timestamp = nullopt;

  // Обработка широты (токен 2) в формате ddmm.mmmm
  if(tokens[2] && tokens[2][0])
  {
    double rawLat = atof(tokens[2]);
    double deg = static_cast<int>(rawLat / 100);
    double minutes = rawLat - deg*100;
    double lat_deg = deg + minutes/60.0;
    // Если направление "S", делаем отрицательным
    if (tokens[3] && tokens[3][0] == 'S')
      lat_deg = -lat_deg;

    pos.lat = static_cast<int32_t>(lat_deg*1e7 + ((lat_deg >= 0) ? 0.5 : -0.5));
  }
  else
    pos.lat = nullopt;


  // Обработка долготы (токен 4) в формате dddmm.mmmm
  if(tokens[4] && tokens[4][0])
  {
    double rawLon = atof(tokens[4]);
    double deg = static_cast<int>(rawLon / 100);
    double minutes = rawLon - deg*100;
    double lon_deg = deg + minutes/60.0;
    if (tokens[5] && tokens[5][0] == 'W')
      lon_deg = -lon_deg;

    pos.lon = static_cast<int32_t>(lon_deg*1e7 + ((lon_deg >= 0) ? 0.5 : -0.5));
  }
  else
    pos.lon = nullopt;


  // Качество фиксации (токен 6)
  if(tokens[6] && tokens[6][0])
    pos.quality = atoi(tokens[6]);
  else
    pos.quality = nullopt;

  // Высота (токен 9) в метрах; преобразуем в int32_t с масштабированием 1e-3
  if(tokens[9] && tokens[9][0])
  {
    double alt = atof(tokens[9]);
    pos.alt = static_cast<int32_t>(alt*1000.0 + ((alt >= 0) ? 0.5 : -0.5));
  }
  else
    pos.alt = nullopt;

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool parse_zda(char *sentence, date_time_t &dt)
{
  // Пропускаем символ '$', если он присутствует.
  char *start = sentence;
  if (*start == '$')
    start++;

  // Находим символ '*' – конец полезной части сообщения.
  const char *star = strchr(start, '*');
  if (!star)
    return false;

  // обрезаем часть строки с контрольной суммой
  start[star-start] = '\0';

  // Токенизируем строку с учётом пустых полей.
  char *tokens[8] = {0};
  int tokenCount = tokenize(start, tokens, 8);
  if (tokenCount < 5)
    return false; // Минимально должны присутствовать: тип, время, день, месяц, год.

  // проверяем наличие данных в сообщении
  if (!(tokens[1] && (strlen(tokens[1]) >= 6)) ||
      !(tokens[2] && tokens[2][0]) ||
      !(tokens[3] && tokens[3][0]) ||
      !(tokens[4] && tokens[4][0]))
    return false;

  // Извлекаем часы, минуты и секунды (целая часть секунд).
  dt.hour = static_cast<uint8_t>((tokens[1][0] - '0')*10 + (tokens[1][1] - '0'));
  dt.min  = static_cast<uint8_t>((tokens[1][2] - '0')*10 + (tokens[1][3] - '0'));
  dt.sec  = static_cast<uint8_t>((tokens[1][4] - '0')*10 + (tokens[1][5] - '0'));

  // Парсим день, месяц и год.
  dt.day   = static_cast<uint8_t>(atoi(tokens[2]));
  dt.month = static_cast<uint8_t>(atoi(tokens[3]));
  dt.year  = static_cast<uint16_t>(atoi(tokens[4]));

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Функция токенизации, которая корректно обрабатывает пустые токены.
// Разбивает строку str, заменяя каждый символ ',' на '\0' и записывая указатели
// на начало каждого токена в массив tokens.
// Функция возвращает количество полученных токенов.
int tokenize(char *str, char *tokens[], int maxTokens)
{
  int count = 0;
  tokens[count++] = str; // Первый токен начинается с начала строки.
  for (char *p = str; *p && count < maxTokens; p++)
  {
    if (*p == ',')
    {
      *p = '\0';           // Завершаем предыдущий токен.
      tokens[count++] = p + 1; // Следующий токен начинается сразу после запятой.
    }
  }
  return count;
}
//------------------------------------------------------------------------------
