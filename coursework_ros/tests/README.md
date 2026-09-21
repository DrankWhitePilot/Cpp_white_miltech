# Перевірка журналу телеметрії

Ці тести використовують синтетичні дані; вони не запускають контролер,
UART, GPIO або симуляцію польоту.

З кореня репозиторію в Linux:

```bash
test_dir=$(mktemp -d /tmp/hw11-telemetry-test.XXXXXX)
g++ -std=c++20 -Wall -Wextra -Werror -Ihomework_11/include \
  coursework_ros/tests/telemetry_log_test.cpp -o "$test_dir/telemetry_log_test"
"$test_dir/telemetry_log_test" "$test_dir/synthetic_telemetry.csv"
HW11_LOG_FIXTURE="$test_dir/synthetic_telemetry.csv" \
  node coursework_ros/tests/telemetry_parser.test.js
```

Остання команда потребує Node.js. Якщо він доступний тільки у Windows,
скопіювати синтетичний CSV і передати його шлях через змінну
`HW11_LOG_FIXTURE` перед запуском того самого JavaScript-тесту.
Без змінної міжмовний тест позначається як пропущений; решта перевірок
читача виконується. Читач для тесту береться безпосередньо з робочої HTML-панелі.
