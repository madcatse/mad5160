Добро пожаловать в моqй проект по созданию реплики материнской платы IBM 5160, или как её называют - PC XT. 

**Предыстория**

Как-то вечером, глядя видеоролик Adrain Digital Basement о ремонте оригинальной IBM CGA видеокарты, обратил внимание что в ролике он демонстрировал схему всей видеокарты, которую кто-то выполнил в KiCAD. И упомянул, что схема иначально взята из оригинальной документации самой IBM. У меня сразу возникла мысль - а могу ли я сделать реплику такой видеокарты, раз уже проверенная схема есть в наличии?
Быстрый поиск показал что да, такая схема доступна, а также что кто-то уже сделал реплику CGA видеокарты (проект CGA Redux). Глядя на разведенную карточку, в голову проникла мысль: если есть схема видеокарты от IBM, то наверное есть схема и от материнской платы? И да - IBM 5160 Technical Reference от Апреля 1983 года действительно содержит схему на 11 листах всей материнской платы. Ну а если есть схема - то насколько сложным будет взять её и развести? Сделать реплику. Так появился данный проект.

![изображение](https://github.com/user-attachments/assets/90822081-0028-469e-8dd8-7aae1b48e75e)

**Начало работ и поиск подводных камней**

Вооружившись документацией из Technical Reference я принялся за работу. Нашел и скачал всю доступную документацию на 5160, разобрал, и проанализировал. Нашел фотографии материнской платы (спасибо retroweb), с которых бралась визуальная информация по компонентам. 

![изображение](https://github.com/user-attachments/assets/cfabadd7-c356-40de-b222-d7cdb6a497d4)

Потом выписал список всех компонентов на плате, с которыми предстояло работать. В этот список попали:

*центральный процессор и микросхемы, которые составляли его обвязку (чип сет, понимаете?)

*микросхемы памяти, которыми набиралась оперативная память (от 16 килобайт до 256 килобайт)

*микросхемы логики, по большей части представленные 74LS и 74S сериями

*микросхемы ПЗУ, по счастью две обычные 27С256 EPROM

*двойной периферийный драйвер SN75477P

*PAL-чип U44

*мимикрирующие под микросхемы резисторные сборки в DIP-корпусах (RN1-RN5)

*две временные задержки TD1 и TD2

*сравнительно небольшое разнообразие резисторов и конденсаторов, а также кварцевый резонатор

*стандартные 62-пиновые слоты, DIN5-коннектор под клавиатуру и Molex коннектор питания.

**С чем не было проблем**

Меньше всего проблем принес центральный процессор и его обвязка: 
Центральный процессор 8088 (совесткий аналог КР1810ВМ88)
Математический сопроцессор 8087 (чудовищно редкий совесткий аналог КР1810ВМ88)
Программируемый DMA контроллер 8237A (совесткий аналог КР1810ВТ37А)
Программируемый таймер 8253 (совесткий аналог КР580ВИ53Д)
Программируемый интерфейс периферийных устройств 8255A (совесткий аналог КР580ВВ55А)
Программируемый контроллер прерываний 8259A (совесткий аналог КР1810ВН59А)
Генератор частоты 8284A (совесткий аналог КР1810ГФ84А)
Контроллер шины M8288 (совесткий аналог КР1810ВГ88)
Все эти микросхемы сравнительно легкодоступны и их приобретение и проверка не составила никакого труда.

Микросхемы памяти также представляют собой совершенно стандартные 4164/41256 и также не вызывают проблем с их приобретением и проверкой.

То же самое касается и микросхем логики - 74 серия и её советские клоны достаточно распространены, легко приобретаются и проверяются. Неприятность в общем разнообразии микросхем. Для сборки Вам потребуются: 
7407.

74LS00,
74LS04,
74LS04,
74LS10,
74LS20,
74LS27,
74LS32,
74LS138,
74LS158,
74LS175,
74LS243,
74LS244,
74LS245,
74LS322,
74LS373,
74LS670.

74S00,
74S08,
74S74,
74S280.

Часть совершенно свободно находится на aliexpress, часть есть там же, но поддельная, часть найти будет сложно, особенно S-чипы. Но это реализуемо.

Двойной периферийный драйвер _SN75477P_ не относится к 74ой серии, но также легко доступен.

Пассивные компоненты, кварцевый резонатор, подстроеный конденсатор, слоты ISA и коннектор DIN5 для подключения клавиатуры также совершенно стандартные. 

**Проблемы**

Все это касалось комплектующих, с которыми никаких принципиальных проблем не было. К сожалению, это не все компоненты на плате. Есть часть, с которыми проблемы были, и которые пришлось решать.

**PAL 82S129N (U44)**

Злосчастная U44, на месте которой располагается PAL-чип 82S129N, программируемая логическая микросхема декодера памяти. 

![изображение](https://github.com/user-attachments/assets/c02593ad-41a8-4b3a-90f4-200988af408c)

Обширный поиск показал отсутствие легкодоступного дампа содержимого данной микросхемы. Однако с давных времен (декабрь 1989 года!) существует мод, 1megXT, который позволяет поставить на старые 5160-материснкие платы до 1 мегабайта памяти, и заключается в замене вышеуказанной 82S129N на GAL16V8 с хорошо задокументированной зашивкой, плюс установке одной дополнительной микросхемы. Установка такого мода мало того что решает проблему с отсутствием зашивки, так ещё и позволяет учетвертить доступный объем оперативной памяти! Хоть я и хотел поиграться с PC с 16кб памяти (минимально возмомжный объем), но как вышло так вышло. 
Для реализации этого мода Вам потребуется одна микросхема GAL16V8 и зашивка для неё. Зашивка с уравнениями присутствует в каталоге firmware\GAL16V8 Там же есть ссылка на оригинальный тред на vcfed. Микросхема GAL16V8 длиннее, чем оригинальная 82S129N, поэтому установить её напрямую не получится. Установите панель на место U44, после чего посадите GAL16V8 так чтобы в панель вошли первый и последний вины, а "хвост" остался висеть в воздухе. Поднимите пины, которые не попали в сокет, и поставьте перемычку между 8 и 10 пинами (это соединит пин земли на GAL16 и пин земли сокета). 

![изображение](https://github.com/user-attachments/assets/4c8a9c62-f149-4736-84bc-9fe170a814ce)

Обратите внимание, что в моем случае я использовал **не свою замену резистороной сборки**, а **оригинальную резисторную сборку**. Так как плата была тестовая, то резисторая сборка ставилась в панель, и она будет мешать моду, не позволяя поставить его по высоте. Надо либо паять плату сборки под основание платы, без панели, либо поднимать микросхему мода несколькими панельками. Если делать без панельки под резисторную сборку, места должно хватить.
После этого мод готов к работе, останется установить джампер на E2 и на блоке SW2 третьим и четвертым переключателем задать необходимый объем оперативной памяти (подробнее см. документацию на мод в файле \firmware\1megXT.txt)

**Резисторные сборки**

IBM установил на плате пять резисторных сборок в DIP-16 корпусах. Это:

RN1, RN5 - 15 резисторов по 4.7 кОм

![изображение](https://github.com/user-attachments/assets/e8d23337-4815-40cb-8dac-a297d92129fa)

RN3, RN4 - 8 резисторов по 30 Ом

![изображение](https://github.com/user-attachments/assets/bc655acc-cf03-406f-8ae0-dc06f44cbf1f)

RN2 - 15 резисторов по 8.2 кОм

![изображение](https://github.com/user-attachments/assets/a4fd3e20-902d-4323-98f7-c4d5a5fbbec3)

Частично такие сборки можно купить, но цены на них очень и очень негуманные. Некоторые (8 по 30) имеют неходовой номинал и представляют собой проблему. Чтобы не заморачиваться с поиском такой экзотики я создал проект "RNDip16", располагающийся в папке \supporting_prj\RNDip16
Проект представляет собой две небольшие платы, по габаритам схожими с футпринтом микросхемы в корпусе DIP-16, на которые можно установить 8 либо 15 SMD-резисторов любого номинала, в корпусе 0805. 

![изображение](https://github.com/user-attachments/assets/e80664aa-76ec-460a-a007-5cee896d1f75)

После пайки резисторов следует запаять пины ножек и замена резисторной сборки готова. 

![изображение](https://github.com/user-attachments/assets/4aaab928-570c-44aa-899a-44ff76aaa635)

Установка такой замены допускается в сокет, либо напрямую запайкой в плату. Gerber-файлы лежат в папке \supporting_prj\RNDip16\gerbers

**Линии временной задержки TD1 и TD2**

Довольно экзотические компоненты установлены под обозначениями TD1 и TD2. Данные компоненты обеспечивают контролируемую задержку сигнала и являются критичными для корректной работы компьютера. По счастью само IBM в документации от 1986ого года выбросил из схемы TD2, заменив её обычным резистором, убрав резисторы R4 и R5. Но вот TD1 осталась, как и проблемы с ней. Часто на её месте стоят компоненты от BELFUSE 0447-0100-98, либо SPRAGUE 61Z14A100. Это линия задержки на 5 выходов, каждый из которых добавляет 20нс к времени выхода сигнала. На ebay такие компоненты можно купить, однако у нас в настоящий момент это исключено. 

![изображение](https://github.com/user-attachments/assets/6878d08c-caec-4499-8d10-95a942bb1d73)

Поиск показал что существуют современные вариации таких задержек - DS1100Z-100 в корпусе NSOIC-8. Я создал проект time_delay, расположенный по пути \supporting_prj\time_delay ,который представляет собой крошечную плату с габаритами исходной временной задержки, на которую необходимо напаять DS1100Z-100 и пины, после чего данный компонент на 100% заменяет исходную временную задержку. 

![изображение](https://github.com/user-attachments/assets/1811b318-883d-4258-a52c-3a4f1056ee5b)

![изображение](https://github.com/user-attachments/assets/7c6a79d9-6e5d-4dce-9286-2f3f800af844)

Gerber-файлы лежат в папке \supporting_prj\time_delay\Gerbers

**Коннектор питания P8/P9**

Старый-добрый AT-коннектор питания (что, получается, неверно, так как у нас XT и этот тип коннектора должен тогда называться XT-коннектор). 

![изображение](https://github.com/user-attachments/assets/79c6b0cd-b527-43f1-a14b-e2b205653f7d)

Его можно взять с донорской материнской платы, но я решил поставить современный вариант. Molex 26604060, имеет такие же габариты, но пины квадратной, а не плоской формы. 

![изображение](https://github.com/user-attachments/assets/aa931aae-e5c0-4b87-b571-96693d76142f)

![изображение](https://github.com/user-attachments/assets/68dc11a1-4919-4df5-a6f1-b7300a6994e1)


Однако типовой AT-блок питания подключается без проблем, и нет нужды искать донора.

**BIOS**

Микросхемы биоса используются совершенно стандартные, 27C256. В одной находится сам биос, в другой - ром с IBM BASIC. Все оригинальные BIOS есть здесь: https://www.minuszerodegrees.net/bios/bios.htm
Также можно использовать различные сторонние BIOS, в частности биос от TurboXT, доступный по пути firmware\turboxt\turboxt.bin

**Ошибки в схеме**

Первая ревизия платы (версия 0.1) содержала в себе ошибку, связанную с опечаткой в оригинальной IBM документации. Для сравнения, фрагмент схемы 1983 года:

![изображение](https://github.com/user-attachments/assets/0bfd545a-5d26-473a-a5d1-ec1b94df72ea)

Тот же участок в схеме от 1986 года:

![изображение](https://github.com/user-attachments/assets/e7e3428a-767b-48de-98ff-cdbbb0e183bc)

Из-за этого линия проверки четности на ISA-слоте была подтянута на +5 постоянно через резистор, и система ВСЕГДА при старте определяла ошибку четности. Это единственный фикс, который потребовалось сделать в схеме после того как был изготовлен, собран и испытан первый образец.

**Печатная плата**

Печатную плату я разводил используя в качестве ориентира скан платы, любезно предоставленный пользователем jafir с форума vcfed, а также остальные референсные фото. Так как оригинальная плата четырехслойная, я не имел на руках оригинальной платы, и тем более не мог сделать послойную декомпозицию платы с целью анализа проводников на внутренних слоях платы, трассировка всех проводников выполнена с нуля. Сигнальные слои - F.Cu и B.Cu. Слой In1.Cu - +5В, слой In2.Cu - земля. Линии +12В, -12В и -5В проведены по сигнальным слоям. Контур платы, координаты посадочных отверстий, некоторые посадочные места компонентов (трехвыводные конденсаторы, пленочный конденсатор, подстроечный конденсатор) взяты из открытого проекта-реплики IBM PC 5150 (PC-Retro, MTM Scientific)

**Изготовление и сборка**

Четырехслойная печатная плата без каких либо особенностей. Изготовление возможно силами китайских производителей. Gerbar-файлы для изготовления находятся по пути \gerbers

![изображение](https://github.com/user-attachments/assets/7a1b5ae1-7e3b-4019-92b0-b3445d71e51a)

**Благодарности**

Выражаю огромное спасибо за идею, помощь в проектировании и отладке:
Adrinan Black с канала Adrian Drigital Basement за идею, а также диагностический ром для памяти, который помог локализовать проблему с Parity

jafir с форума vcfed за качественный скан материнской платы

modem7 с форума vcfed за помощь в локализации проблемы и идеи по исправлению.

**Контакты**

Связаться со мной можно в телеграме, на моем небольшом канале:
https://t.me/madcatse