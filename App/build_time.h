#pragma once

#include <cstdint>
#include <string_view>
#include <iostream>

// Преобразование символа в цифру (без проверки, что символ – цифра)
constexpr int char_to_int(char c)
{
    return c - '0';
}

// Извлечение числа из подстроки s начиная с позиции start, длиной count символов.
// Игнорируются пробелы (например, при однозначном дне).
constexpr int parse_number(std::string_view s, size_t start, size_t count)
{
    int result = 0;
    for (size_t i = start; i < start + count; ++i) {
        char c = s[i];
        if (c == ' ') continue; // пропускаем пробелы
        result = result * 10 + char_to_int(c);
    }
    return result;
}

// Преобразование трёхбуквенного названия месяца в номер месяца.
constexpr int month_to_int(std::string_view month)
{
    return (month == "Jan") ? 1 :
           (month == "Feb") ? 2 :
           (month == "Mar") ? 3 :
           (month == "Apr") ? 4 :
           (month == "May") ? 5 :
           (month == "Jun") ? 6 :
           (month == "Jul") ? 7 :
           (month == "Aug") ? 8 :
           (month == "Sep") ? 9 :
           (month == "Oct") ? 10 :
           (month == "Nov") ? 11 :
           (month == "Dec") ? 12 : 0;
}

// Определение, является ли год високосным.
constexpr bool is_leap(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

// Количество дней в заданном месяце указанного года.
constexpr int days_in_month(int year, int month)
{
    return (month == 2) ? (is_leap(year) ? 29 : 28) :
           (month == 4 || month == 6 || month == 9 || month == 11) ? 30 : 31;
}

// Вычисление количества дней с 1 января 1970 до заданной даты (год, месяц, день).
constexpr int days_since_epoch(int year, int month, int day)
{
    int days = 0;
    for (int y = 1970; y < year; ++y)
        days += is_leap(y) ? 366 : 365;
    for (int m = 1; m < month; ++m)
        days += days_in_month(year, m);
    days += (day - 1);
    return days;
}

// Функция для преобразования __DATE__ (формат "Mmm dd yyyy") и __TIME__ ("hh:mm:ss")
// в Unix time (количество секунд с 1 января 1970).
constexpr std::uint32_t compile_time_to_unix(std::string_view date, std::string_view time)
{
    // Извлекаем месяц, день и год:
    // - date.substr(0, 3) содержит название месяца (например, "Jan")
    // - date.substr(4, 2) содержит день (с возможным ведущим пробелом)
    // - date.substr(7, 4) содержит год
    int month = month_to_int(date.substr(0, 3));
    int day = parse_number(date, 4, 2);
    int year = parse_number(date, 7, 4);

    // Извлекаем часы, минуты и секунды:
    int hour = parse_number(time, 0, 2);
    int minute = parse_number(time, 3, 2);
    int second = parse_number(time, 6, 2);

    int total_days = days_since_epoch(year, month, day);
    return static_cast<std::uint32_t>(total_days * 86400 + hour * 3600 + minute * 60 + second);
}

constexpr uint32_t BuildTime = compile_time_to_unix(__DATE__, __TIME__);

