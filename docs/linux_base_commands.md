# Linux based commands

## Основные комманды терминала Linux

Эти команды предоставляют возможности для работы с системой ROS, управления nodes, топиками, сервисами и параметрами, а также позволяют визуализировать данные и взаимодействовать с графическими инструментами.

`cd <directory>` - изменить текущую рабочую директорию.

Аргументы:
*  directory: путь к директории, в которую нужно перейти.

`ls [options] [files]` - просмотр содержимого директории.

Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -l, -a, -la).
* files (необязательно): файлы или директории, содержимое которых нужно отобразить.

`mkdir [options] <directory>` - создание новой директории

Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -p для создания вложенных директорий).
* directory: имя новой директории.

`touch <file>` - создание нового файла.

Аргументы:
* file: имя нового файла.


`rm [options] <file>` - удаление файла или директории.
Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -r для удаления директории).
* file: имя файла или директории для удаления.

`mv [options] <source> <destination>` - перемещение или переименование файла/директории.
Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -f для принудительного перемещения).
* source: исходный файл или директория.
* destination: новое имя файла/директории или путь к директории, куда нужно переместить исходный файл/директорию.

`cp [options] <source> <destination>` - копирование файла/директории.
Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -r для копирования директории).
* source: исходный файл или директория.
* destination: путь и имя файла/директории, куда нужно скопировать исходный файл/директорию.

`cat [options] <file>` - вывод содержимого файла на экран.
Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -n для вывода номеров строк).
* file: имя файла, содержимое которого нужно отобразить.

`nano <file>` - простой консольный текстовый редактор для редактирования файлов.'
Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -B для создания резервной копии файла перед сохранением).
* file (необязательно): имя файла для редактирования. Если файл не существует, он будет создан при сохранении изменений.

`grep [options] <pattern> <files>` - поиск определенного текста в файлах.
Аргументы:
* options (необязательно): флаги, изменяющие поведение команды (например, -i для игнорирования регистра символов).
* pattern: строка или регулярное выражение для поиска.
* files: файлы, в

`source <file>` - выполнение команд из файла в текущем окружении.
Аргументы:
* file: имя файла, содержащего команды для выполнения в текущем окружении.