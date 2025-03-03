#include <app.h>
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
  std::optional<uint8_t> sats;
  std::optional<uint8_t> hdop;
} position_t;
//--------------------------------------
typedef struct
{
  std::optional<float> timestamp;
  std::optional<int32_t> heading;
  std::optional<int32_t> pitch;
  std::optional<int32_t> roll;
  std::optional<uint8_t> quality;
} attitude_t;
//--------------------------------------
typedef struct
{
  std::optional<int32_t> ground_speed;
  std::optional<uint8_t> quality;
} speed_t;
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
static int tokenize(char *str, char *tokens[], int maxTokens);
static bool parse_gga(char *sentence, position_t &pos);
static bool parse_hpr(char *sentence, attitude_t &att);
static bool parse_zda(char *sentence, date_time_t &dt);
static bool parse_vtg(char *sentence, speed_t &spd);
static void process_position(const position_t &pos);
static void process_attitude(const attitude_t &att);
static void process_datetime(const date_time_t &dt);
static void process_speed(const speed_t &spd);
//--------------------------------------


//--------------------------------------
char nmea_string[NMEA::MAX_LENGTH+1];
//--------------------------------------


//------------------------------------------------------------------------------
void task_func_gnss(void *arg)
{
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    char current_str[NMEA::MAX_LENGTH+1];

    // на всякий случай выставляю признак конца строки
    current_str[NMEA::MAX_LENGTH] = '\0';
    taskENTER_CRITICAL();
    // строка короткая поэтому быстрее скопировать весь массив, чем искать длину строки и потом копировать
    memcpy(current_str, nmea_string, NMEA::MAX_LENGTH);
    taskEXIT_CRITICAL();

    if(strncmp(&current_str[3], "GGA", 3) == 0)
    {
      position_t pos{};
      if(parse_gga(current_str, pos))
      {
        process_position(pos);
#if SERIAL_LOG_OUT
        opack_gnss_pos_t pack;
        pack.systime = systime;
        pack.timestamp = pos.timestamp.value_or(0.0f);
        pack.alt = pos.alt.value_or(0.0f);
        pack.lat = pos.lat.value_or(0.0f);
        pack.lon = pos.lon.value_or(0.0f);
        pack.quality = pos.quality.value_or(0x0);
        packer.send(PID_GNSS_POS, &pack, sizeof(pack));
#endif
      }
    }
    else if(strncmp(&current_str[3], "HPR", 3) == 0)
    {
      attitude_t att{};
      if(parse_hpr(current_str, att))
      {
        process_attitude(att);
#if SERIAL_LOG_OUT
        opack_gnss_att_t pack;
        pack.systime = systime;
        pack.timestamp = att.timestamp.value_or(0.0f);
        pack.heading = att.heading.value_or(0.0f);
        pack.pitch = att.pitch.value_or(0.0f);
        pack.roll = att.roll.value_or(0.0f);
        pack.quality = att.quality.value_or(0x0);
        packer.send(PID_GNSS_ATT, &pack, sizeof(pack));
#endif
      }
    }
    else if(strncmp(&current_str[3], "VTG", 3) == 0)
    {
      speed_t spd{};
      if(parse_vtg(current_str, spd))
      {
        process_speed(spd);
#if SERIAL_LOG_OUT
        opack_gnss_speed_t pack;
        pack.systime = systime;
        pack.ground_speed = spd.ground_speed.value_or(0.0f);
        pack.quality = spd.quality.value_or(0x0);
        packer.send(PID_GNSS_SPEED, &pack, sizeof(pack));
#endif
      }
    }
    else if(strncmp(&current_str[3], "ZDA", 3) == 0)
    {
      date_time_t time{};
      if(parse_zda(current_str, time))
        process_datetime(time);
    }
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
static void process_position(const position_t &pos)
{
  can_gnss_val_t msg = {.state={.raw = 0x0}};
  msg.state.bins_ok = bins_status.bins_ok;
  msg.state.valid = (pos.quality && (*pos.quality != CAN_GNSS_QUAL_INVALID)) ? 1 : 0;
  msg.quality = pos.quality.value_or(CAN_GNSS_QUAL_INVALID);

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

  if(pos.hdop && pos.sats)
  {
    static uint32_t prev_sec = 0;
    if(pos.timestamp && ((uint32_t)*pos.timestamp - prev_sec))
    {
      static uint8_t cntr = 0;
      can_gnss_dop_t dop;
      dop.state = msg.state;
      dop.cntr = cntr++;
      dop.hdop = *pos.hdop;
      dop.sats = *pos.sats;
      if(bins_status.bins_standalone)
        can_send_dat(CAN_MSG_BINS2_GNSS_DOP, &dop, sizeof(dop));
      else
        can_send_dat(CAN_MSG_BINS1_GNSS_DOP, &dop, sizeof(dop));
    }
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
static void process_attitude(const attitude_t &att)
{
  can_gnss_val_t msg = {.state={.raw = 0x0}};
  msg.state.bins_ok = bins_status.bins_ok;
  msg.state.valid = (att.quality && (*att.quality != CAN_GNSS_QUAL_INVALID)) ? 1 : 0;
  msg.quality = att.quality.value_or(CAN_GNSS_QUAL_INVALID);

  if(att.heading)
  {
    static uint8_t cntr = 0;
    msg.cntr = cntr++;
    msg.value = *att.heading;
    if(bins_status.bins_standalone)
      can_send_dat(CAN_MSG_BINS2_TRUE_HEADING, &msg, sizeof(msg));
    else
      can_send_dat(CAN_MSG_BINS1_TRUE_HEADING, &msg, sizeof(msg));
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
static void process_speed(const speed_t &spd)
{
  can_gnss_val_t msg = {.state={.raw = 0x0}};
  msg.state.bins_ok = bins_status.bins_ok;
  msg.state.valid = (spd.quality && (*spd.quality != CAN_GNSS_QUAL_INVALID)) ? 1 : 0;
  msg.quality = spd.quality.value_or(CAN_GNSS_QUAL_INVALID);

  if(spd.ground_speed)
  {
    static uint8_t cntr = 0;
    msg.cntr = cntr++;
    msg.value = *spd.ground_speed;
    if(bins_status.bins_standalone)
      can_send_dat(CAN_MSG_BINS2_GROUND_SPEED, &msg, sizeof(msg));
    else
      can_send_dat(CAN_MSG_BINS1_GROUND_SPEED, &msg, sizeof(msg));
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
static void process_datetime(const date_time_t &dt)
{
  can_msg_time_t msg = {.state={.raw = 0x0}};
  msg.state.bins_ok = bins_status.bins_ok;
  msg.state.valid = true;
  msg.year = dt.year;
  msg.month = dt.month;
  msg.day = dt.day;
  msg.hour = dt.hour;
  msg.min = dt.min;
  msg.sec = dt.sec;
  if(bins_status.bins_standalone)
    can_send_dat(CAN_MSG_BINS2_TIME_DATE, &msg, sizeof(msg));
  else
    can_send_dat(CAN_MSG_BINS1_TIME_DATE, &msg, sizeof(msg));
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
    double raw_lat = atof(tokens[2]);
    double deg = static_cast<int>(raw_lat/100);
    double minutes = raw_lat - deg*100;
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
    double raw_lon = atof(tokens[4]);
    double deg = static_cast<int>(raw_lon/100);
    double minutes = raw_lon - deg*100;
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


  // количество спутников (токен 7)
  if(tokens[7] && tokens[7][0])
  {
    int satn = atoi(tokens[7]);
    pos.sats = ((satn > 0) && (satn < 256)) ? satn : 0;
  }
  else
    pos.sats = nullopt;

  // hdop (токен 8)
  if(tokens[8] && tokens[8][0])
  {
    double hdop = atof(tokens[8])*10.0;
    if(hdop < 0.0)
      pos.hdop = nullopt;
    else
      pos.hdop = (hdop < 255.0) ? static_cast<uint8_t>(hdop + ((hdop >= 0) ? 0.5 : -0.5)) : 255;
  }
  else
    pos.hdop = nullopt;

  // Высота (токен 9) в метрах; преобразуем в int32_t с масштабированием 1e-3
  if(tokens[9] && tokens[9][0])
  {
    double alt = atof(tokens[9])*1000.0;
    pos.alt = static_cast<int32_t>(alt + ((alt >= 0) ? 0.5 : -0.5));
  }
  else
    pos.alt = nullopt;

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool parse_hpr(char *sentence, attitude_t &att)
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
  char *tokens[10] = {0};
  int tokenCount = tokenize(start, tokens, 10);
  // Можно добавить проверку минимального количества токенов, если необходимо.
  if (tokenCount < 6)
    return false;

  // обработка времени (токен 1) в формате hhmmss.ss
  if(tokens[1] && tokens[1][0])
    att.timestamp = atof(tokens[1]);
  else
    att.timestamp = nullopt;

  // обработка курса (токен 2) в формате hh.hh
  if(tokens[2] && tokens[2][0])
  {
    double heading = atof(tokens[2]);
    if(heading > 180.0)
      heading -= 360.0;
    att.heading = static_cast<int32_t>(heading*1e7 + ((heading >= 0) ? 0.5 : -0.5));
  }
  else
    att.heading = nullopt;

  // обработка тангажа (токен 3) в формате pp.pp
  if(tokens[3] && tokens[3][0])
  {
    double pitch = atof(tokens[3]);
    att.pitch = static_cast<int32_t>(pitch*1e7 + ((pitch >= 0) ? 0.5 : -0.5));
  }
  else
    att.pitch = nullopt;

  // обработка крена (токен 4) в формате rr.rr
  if(tokens[4] && tokens[4][0])
  {
    double roll = atof(tokens[4]);
    att.roll = static_cast<int32_t>(roll*1e7 + ((roll >= 0) ? 0.5 : -0.5));
  }
  else
    att.roll = nullopt;

  // Качество фиксации (токен 5)
  if(tokens[5] && tokens[5][0])
    att.quality = atoi(tokens[5]);
  else
    att.quality = nullopt;

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool parse_vtg(char *sentence, speed_t &spd)
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
  char *tokens[10] = {0};
  int tokenCount = tokenize(start, tokens, 10);
  // Можно добавить проверку минимального количества токенов, если необходимо.
  if (tokenCount < 10)
    return false;

  // обработка скорости (токен 7)
  if(tokens[7] && tokens[7][0])
  {
    double speed = atof(tokens[7])*1000.0/3.6;
    spd.ground_speed = static_cast<int32_t>(speed + ((speed >= 0) ? 0.5 : -0.5));
  }
  else
    spd.ground_speed = nullopt;

  // Качество фиксации (токен 9)
  if(tokens[9] && tokens[9][0])
  {
    switch(tokens[9][0])
    {
      case 'A': spd.quality = CAN_GNSS_QUAL_SINGLE; break;
      case 'D': spd.quality = CAN_GNSS_QUAL_DIFF; break;
      case 'E': spd.quality = CAN_GNSS_QUAL_DEAD_RECKONING; break;
      case 'M': spd.quality = CAN_GNSS_QUAL_MANUAL; break;
      case 'N': spd.quality = CAN_GNSS_QUAL_INVALID; break;
      case 'P': spd.quality = CAN_GNSS_QUAL_SINGLE; break;
      case 'S': spd.quality = CAN_GNSS_QUAL_SIMULATOR; break;
      default: spd.quality = CAN_GNSS_QUAL_INVALID; break;
    }
  }
  else
    spd.quality = nullopt;

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
