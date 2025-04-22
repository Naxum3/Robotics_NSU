```mermaid
---
title: Диаграмма компонентов - Покупка 3D модели
---
componentDiagram
    %% Основные компоненты системы
    component "Покупатель" as Buyer {
        [Интерфейс пользователя]
    }

    component "Продавец" as Seller {
        [Интерфейс продавца]
    }

    component "Платформа" as Platform {
        component "Каталог моделей" as Catalog {
            [Поиск]
            [Фильтры]
            [Просмотр]
        }

        component "Корзина" as Cart {
            [Добавление]
            [Удаление]
            [Изменение количества]
        }

        component "Оплата" as Payment {
            [Выбор способа]
            [Обработка платежа]
            [Возвраты]
        }

        component "Лицензии" as Licensing {
            [Типы лицензий]
            [Генерация ключа]
        }

        component "Доставка" as Delivery {
            [Скачивание]
            [API интеграции]
        }
    }

    component "Платежный шлюз" as Gateway {
        [Банковские карты]
        [Электронные кошельки]
        [Криптовалюты]
    }

    component "База данных" as DB {
        [Модели]
        [Пользователи]
        [Транзакции]
    }

    %% Связи между компонентами
    Buyer -- Catalog : Поиск/Просмотр
    Buyer -- Cart : Управление
    Buyer -- Payment : Оплата
    Buyer -- Delivery : Получение файла

    Seller -- Catalog : Загрузка/Обновление
    Seller -- DB : Управление моделями

    Catalog -- DB : Запрос данных
    Cart -- Payment : Передача заказа
    Payment -- Gateway : Проведение платежа
    Payment -- Licensing : Активация
    Licensing -- Delivery : Разрешение доступа
    Delivery -- DB : Логирование

    Platform -- DB : Хранение/Получение
```
