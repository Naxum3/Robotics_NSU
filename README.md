graph TD
    %% Основные компоненты системы
    subgraph Покупатель
        A[Интерфейс пользователя]
    end

    subgraph Продавец
        B[Интерфейс продавца]
    end

    subgraph Платформа
        subgraph Каталог моделей
            C1[Поиск]
            C2[Фильтры]
            C3[Просмотр]
        end

        subgraph Корзина
            D1[Добавление]
            D2[Удаление]
        end

        subgraph Оплата
            E1[Выбор способа]
            E2[Обработка платежа]
        end

        subgraph Лицензии
            F1[Типы лицензий]
        end

        subgraph Доставка
            G1[Скачивание]
        end
    end

    subgraph Внешние системы
        H[Платежный шлюз]
        I[(База данных)]
    end

    %% Связи
    A -->|Поиск/Просмотр| C1
    A -->|Управление| D1
    A -->|Оплата| E1
    A -->|Получение файла| G1

    B -->|Загрузка моделей| C3
    B --> I

    C1 --> I
    D1 -->|Передача заказа| E1
    E1 -->|Проведение платежа| H
    E1 -->|Активация| F1
    F1 -->|Разрешение доступа| G1
    G1 --> I
