/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Struktóra buforu kołowego czyli zmienne które będą używane do manipulowania buforem kołowym.
/*
 * Ta struktura implementuje logikę bufora kołowego FIFO (FIFO - First-In, First-Out) ta struktura została
 * zaprojektowana dla bufora przechowującego dane odebrane i które mają zostać wysłane. Co mówi o tym typ struktóry
 * jaki będzie przechowywany.
 *
 * Gdzie ta struktóra jest użyta:
 * 	- rx_circular_buffer -> Bufor do którego przerwanie UART HAL_UART_RxCpltCallback zapisuje każdy przychodzący bajt.
 * 	Pętla główna (`main`) odczytuje z niego dane w celu parsowania ramek.
 *
 * 	- tx_circular_buffer -> Główny bufor do wysyłania danych do PC. Funkcje logiczne umieszczają w nim
 * 	odpowiedzi tekstowe. Mechanizm transmisji ProcessTxBuffer pobiera z niego dane, aby utworzyć i wysłać
 * 	kompletne ramki zgodne z zasadami mojego protokołu komunikacyjnego.
 *
 * 	- Pozostałe pufory kołowe (decoded_data_circ_buff, encoded_data_circ_buff czy history_circular_buffer)
 * 	Są używane jako tymczasowe magazyny podczas procesów kodowania/dekodowania czy przechowywania danych archiwalnych.
 */
typedef struct {
	uint8_t * const dataArray; 	// wskaźnik na tablicę danych
	/*
	 * Wskazuje indeks w `dataArray`, pod którym zostanie umieszczony następny bajt.
	 * Jego pozycja względem `tail` określa, ile danych znajduje się w buforze kołowym.
	 */
    int head;
    /*
     * Wskazuje indeks najstarszego, nieprzeczytanego bajtu w buforze.
     * Gdy `tail` dogania `head`, bufor staje się pusty.
     */
    int tail;
    /*
     * Przechowuje całkowitą pojemność tablicy `dataArray`. Jest kluczowy dla operacji modulo (`%`), która
     * implementuje "zawijanie się" indeksów `head` i `tail`.
     */
    int size;
} CircularBuffer_t;

// Tabela stanów do odbioru kompletnej ramki
// Są to wypisane stany informujące o tym jakie działąnia teraz są podejmowane przez maszynę stanów
typedef enum {
    STATE_WAIT_START,        // Oczekiwanie znaku rozpoczęcia ramki ':'
    STATE_READ_FRAME_CONTENT // Odczyt całej zawartości ramki (zakodowany blok + CRC), ale po odebraniu jego końca.
} ParserState;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Rozmiar buforów kołowych
#define BUF_SIZE_RX 1000	// Rozmiar bufora odbiorczego
#define BUF_SIZE_TX 1000	// Rozmiar bufora transmisyjnego
#define HISTORY_BUFFER_SIZE 1000 // Rozmiar bufora na dane archiwalne
#define SIZE_OF_DANE 99	// Maksymalny rozmiar danych w ramce
#define CRC_OF_SIZE 4		// Rozmiar pola CRC

// Bufor na całą zawartość ramki (zakodowany blok + CRC)
#define FRAME_CONTENT_MAX_SIZE ((SIZE_OF_DANE * 4 / 3) + 4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// -------------------------W dokumentacji------------------góra
// Definicja struktury przechowującej pojedynczy pomiar
typedef struct {

	// Typem danych musi być float ponieważ czujnik pobiera wartości z dużą dokładnością po przecinku.
    float temperature;
    float humidity;
} Measurement_t;

// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra

// Struktóra buforu kołowego dla danych archiwalnych
typedef struct {
	Measurement_t * const dataArray; 	// wskaźnik na tablicę danych
    int head;					// indeks miejsca, gdzie zapisujemy nowy znak
    int tail;					// indeks miejsca, skąd odczytujemy znak
    int size;					// rozmiar bufora
} CircularBuffer_arch;

// Zmienne do obsługi asynchronicznego wysyłania historii
typedef struct {
    uint16_t start_index;    // Od którego indeksu zaczęliśmy
    uint16_t count_total;    // Ile łącznie mamy pobrać
} ArchiveState;

ArchiveState archive_state;

// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra
// Zmienne do zarządzania cyklicznym pobieraniem danych
volatile uint32_t logging_interval_ms = 0; // Zmienna przechowująca interwał w milisekundach

// Flaga informująca o tym czy włączono cykliczne pobieranie.
// Jest to ważne ponieważ mamy dwa tryby odbierania danych. PIerwszy to tryb ręcznego pobierania aktualnej
// wartości pobiaru z czujnika a drugi to cykliczne zapisywanie do interwału.
volatile uint8_t logging_enabled = 0;

// Zmienna przechowująca ilość wystąpień tim10
// Ta zmienna jest niezbędna ponieważ potrzebujemy zmiennej która będzie obliczała ile milisekunt upłyneło.
// Ta zmienna iteruje się o jeden co każdą jedną milisekundę i dzięki temu zyskuje możliwość pobierania danych w
// określonym czasem cyklu danych.
volatile size_t TIM10Counter = 0;

// Flaga do odróżnienia pomiaru cyklicznego od ręcznego.
// Ta flaga reprezentuje włączenie pomiaru ręcznego.
uint8_t is_manual_measurement = 0;

// Definicja stanów dla maszyny stanów obsługującej czujnik AHT15
typedef enum {
    AHT15_STATE_IDLE,                 // Stan bezczynności, brak operacji
    AHT15_STATE_INIT_SEND_CMD,        // Wysłanie komendy inicjalizacyjnej
    AHT15_STATE_INIT_DONE,            // Inicjalizacja zakończona, wyślij odpowiedź
    AHT15_STATE_MEASURE_SEND_CMD,     // Wysłanie komendy rozpoczęcia pomiaru
    AHT15_STATE_MEASURE_READ_DATA,    // Odczyt danych pomiarowych
    AHT15_STATE_PROCESS_DATA,         // Przetwarzanie odczytanych danych pomiarowych
    AHT15_STATE_ERROR                 // Stan błędu komunikacji
} AHT15_State;

// Aktualny stan czujnika
// Zmienna do której będzie zapisywany aktualny stan czujnika. Jest to nam potrzebne bo dzięki tej zmiennej możemy
// Dynamicznie włączać potrzebny nam stan w różnej części programu a także określać kolejność wykonywanych działań.
AHT15_State aht15_current_state = AHT15_STATE_IDLE;

// Flaga informująca, czy operacja I2C jest w toku. Ta flaga jest potrzebna ponieważ dzięki temu zyskujemy pewność że
// Operacje nie będą się wykonywały wtedy kiedy jest wykonywane jakieś działanie co tym samym by spowodowało nadpisywanie
// i błędy w przetwarzanych danych.
volatile uint8_t i2c_busy_flag = 0;

// Flaga błędu komunikacji I2C
// Flaga informująca czy wystąpił błąd podczas transmisji danych. Dzięki tej fladze możemy wiedzieć czy mamy obsłużyć
// błąd czy też nie.
volatile uint8_t i2c_error_flag = 0;

// Zmienna na kod błędu wysłany przez funkcję.
volatile uint32_t i2c_error_code = HAL_I2C_ERROR_NONE;

// ZMienna przechowująca znak przeznaczony dla adresata
char sender = '\0';

// Bufor na dane odczytane z czujnika
// Dzięki temu buforowi zyskujemy miejsce w które może zapisać czujnik jak chcemy odebrać od niego nasze dane.
uint8_t sensor_data_buffer[6];

// Adres 7-bitowy czujnika AHT15 to 0x38. W HAL używamy adresu przesuniętego.
uint8_t AHT15_ADDRESS = 0x38 << 1;

// -------------------------W dokumentacji------------------duł

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// -------------------------W dokumentacji------------------góra
uint8_t buffer_RX[BUF_SIZE_RX];				// Bufor wejściowy (odbierane dane)
Measurement_t history_buffer[HISTORY_BUFFER_SIZE]; // Deklaracja bufora na dane archiwalne
uint8_t buffer_TX[BUF_SIZE_TX];				// Bufor wyjściowy (dane do wysłania)
uint8_t decoded_data_buffer[SIZE_OF_DANE];	// Zdekodowane dane użytkownika
uint8_t encoded_data_buffer[FRAME_CONTENT_MAX_SIZE];	// Zakodowane dane użytkownika. Odpowiednio zwiększony buffer dla zakodowanych danych.

// Bufor przechowujący zawartość ramki. W pętli głównej.
uint8_t frame_content_buffer[FRAME_CONTENT_MAX_SIZE];

// To jest bufor kołowy stworzony do odróżnienia bufora kołowego przechowującego dane do wysłania od tego któy
// ma przechowywać tylko te dane które zawierają się w ramce.
uint8_t tx_frame_buffer_data[1000];

// Inicjalizacja struktur buforów kołowych
CircularBuffer_t rx_circular_buffer = { buffer_RX, 0, 0, BUF_SIZE_RX };
CircularBuffer_arch history_circular_buffer = { history_buffer, 0, 0, HISTORY_BUFFER_SIZE };
CircularBuffer_t tx_circular_buffer = { buffer_TX, 0, 0, BUF_SIZE_TX };
CircularBuffer_t decoded_data_circ_buff = {decoded_data_buffer, 0, 0, SIZE_OF_DANE};
CircularBuffer_t encoded_data_circ_buff = {encoded_data_buffer, 0, 0, FRAME_CONTENT_MAX_SIZE};

// -------------------------W dokumentacji------------------duł

// Aktualny stan parsowania ramki, początkowo ustawiony na oczekiwanie startu czyli naszego początku ramki.
ParserState current_state = STATE_WAIT_START;

// Zmienna zliczająca długość (w bajtach) wnętrza ramki żeby wiedzieć kiedy jest etap obliczania sumy kontrolnej.
// Ta zmienna jest wykorzystywana w pętli głównej. Dzięki tej zmiennej możemy w łatwy sposób i dzięki określoenj
// struktórze ramki określić gdzie dokłądnie znajduje się jakie pole w ramce.
int frame_content_index = 0;

// Jest to zmienna przechowująca aktualny znak jaki został odebrany z uart. Stworzenie takiej osobnej zmiennej było konieczne
// ze względu na to że ta zmienna jest korzystana w przerwaniu i gdybyśmy stworzyli globalną zmienną obsługującą w
// całym kodzie odbieranie znaku to przerwanie by powodowało że w tej zmiennej występowały by powturzenia albo nie po
// - rządane informacje.
uint8_t uart_received_char = '0';

// Zmienna określająca czy funkcja służąca za wysyłanie danych z bufora nadawczego zakończyła nadawanie znaków.
// Dzienki tej jesteśmy w stanie ustawić by dane wysyłane przez usart wysyłały się po kolei a nie wtedy kiedy
// program ma dostęp do usart co powodowało by później kołopot z nakładaniem się danych.
uint8_t tx_in_progress = 0;

// Zmienna przechowująca wszystkie obsługiwane komendy
uint8_t commends[] = {'!','#','$','%','&','^','*','@'};

// Flaga wskazująca czy tajmer jest aktywny.
uint8_t frame_timeout_flag = 0;

// Flaga któa jest ustawiana na 1 dopiero wtedy kiedy bufor odbiorczy jest przepełniony. Dzięki tej fladze jesteśmy w
// stanie podjądź odpowiednie działania dla przepełnienia bufora.
uint8_t rx_overflow_flag = 0;

int CALC_B64_LEN(int n) {
	return ((((n) + 2) / 3) * 4);
}

// Pobieranie ilości zajętego miejsca dla bufora na dane archiwalne
int ArchBufferOccupied(CircularBuffer_arch *buffer) {
    if (buffer->head >= buffer->tail) {
    	// Proste określenie ilości wypełnienia buforu gdy nie jest zawinięty.
        return buffer->head - buffer->tail;
    } else {

    	// Obliczanie zajętości bufora kołowego w momęcie kiedy jest faktycznie zawinięty.
    	// Podany wzór działa w taki sposób że najpierw obliczamy ile jest danych między ogonem a końcem bufora.
    	// Dzięki temu wiemy ile jest danych między ostanią daną a ogonem potem dodajemy do tego głowę i
    	// dzięki temu dodajemy do ilości ogona a końcem bufora ilość danych z głowy bufora.
        return (buffer->size - buffer->tail + buffer->head);
    }
}

// -------------------------W dokumentacji------------------góra
// Funkcja do zapisywania danych archiwalnych do bufora
int ArchPutData(CircularBuffer_arch *buffer, float temperature, float humanity)
{
	Measurement_t data;

	data.temperature = temperature;
	data.humidity = humanity;

	// Przypisywanie do zmiennej następny indeks head.
	// Wykonujemy obliczenia z modulo żeby przez rozmiar bufora
	// by zasymulować działanie "zawijania". Modulo z użyciem rozmiaru działa jak swojego rodzaju zwijanie ponieważ
	// nie dopuszcza by indeks wyszedł poza daną granicę.
	int next_head = (buffer->head + 1) % buffer->size;

	// Sprawdzanie czy bufor będzie pełny i jeśli warunek będzie prawdziwy
	// to ogon przejdzie na kolejne miejsce zapominając o aktualnej wartości.
	if (next_head == buffer->tail) {
		buffer->tail = (buffer->tail + 1) % buffer->size;
	}

    // Wpisywanie wartość do bufora
    buffer->dataArray[buffer->head] = data;

    // Przesuwa głowe na następne wolne miejsce, nadpisując starą wartość jeśli
    // Warunek o zapełnieniu bufora będzie prawidłowy.
    buffer->head = next_head;

    return 0;
}
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra
// Funkcja odczytująca dane z bufora
int ArchGetData(CircularBuffer_arch *buffer, Measurement_t *data) {

    // Sprawdzanie czy bufor jest pusty
    if (buffer->head == buffer->tail) {
    	return -1;
    }

    // Odczytywanie wartość z bufora
	data -> temperature = buffer -> dataArray[buffer -> tail].temperature;
	data -> humidity = buffer -> dataArray[buffer -> tail].humidity;

	// Przesówa wskaźnik na ostatni aktualny element w buforze.
    buffer->tail = (buffer->tail + 1) % buffer->size;

    return 0;
}
// -------------------------W dokumentacji------------------duł

// Odczytuje dane z bufora archiwalnego o podanym indeksie
// bez modyfikowania wskaźników bufora.
int ArchGetDataAtIndex(CircularBuffer_arch *buffer, int index, Measurement_t *data) {
    int occupied = ArchBufferOccupied(buffer);

    // Sprawdzenie, czy żądany indeks mieści się w rozmiarze bufora a także czy żądana ilość do pobrania znajduje się
    // w podanym indeksie.
    if (index < 0 || index >= occupied) {
        return -1; // Indeks poza zakresem
    }

    // Obliczenie fizycznego indeksu w tablicy
    // tail wskazuje ostatnio zajęte miejsce, więc pierwszy element jest pod tail + 1
    // Podane działanie (buffer->tail + 1 + index) oblicza docelowy indeks tak jakby nasza tablica była nieskończenie długa.
    // To jest działanie % buffer->size które zapewnia nam że dasz indeks będzie się zawijał. To działanie jest konieczne
    // Przez pętle która operuje na kolejnych liczbyach występujących po sobie. Bo pętla wykonuje się w podany sposób,
    // że najpierw wykonujemy pierwszy indeks który przesyłamy i do niego jest dodawana liczba całkowita z pętli,
    // Następnie jest dodawana kolejna liczba z indeksu zanjdującego się w pętli. To działanie działa na podanej zasadzie,
    // że jak liczba jest równa rozmiarowi to reszta wynosi 0 czyli jest "zawijaja" na startowy indeks a jak jest o kolejną
    // liczbę większa to jest "zawijana" na indeks 1. A gdy liczba jest mniejsza niż rozmiar to liczba pozostaje taka jaka była.
    int physical_index = (buffer->tail + index) % buffer->size;

    // Kopiowanie danych
    *data = buffer->dataArray[physical_index];

    return 0;
}

// -------------------------W dokumentacji------------------góra
/*
 * Zapis pojedynczego znaku do bufora kołowego.
 *
 * Opis parametrów:
 * buffer to Wskaźnik na strukturę bufora.
 * data to Bajt do zapisania.
 *
 * Zwracana wartość:
 * int 0 przy powodzeniu, -1 gdy bufor jest pełny.
*/
int CircularBufferPutChar(CircularBuffer_t *buffer, uint8_t data)
{
	// Przypisywanie do zmiennej zwykłej ale nie w zmiennej w buforze tylko w zmiennej następny indeks głowy
	int next_head = (buffer->head + 1) % buffer->size;

	// Sprawdzanie czy jest miejsce w buforze. Porównuję tutaj następną głowę a nie aktualną głowę z ogonem by odróżnić
	// sprawdzanie pełnego i pustego bufora bo pusty bufor to taki który AKTUUALNA GŁOWA równa się aktualnemu ogonowi
	// a pełny gdy NASTĘPNA głowa równa się aktualnemu ogonowi.
	if ( next_head == buffer->tail ) {
		return -1;
	}

    // Wpisywanie wartość do bufora
    buffer->dataArray[buffer->head] = data;

    // Zapisywanie nowy indeks head
    buffer->head = next_head;

    return 0;

}
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra
/*
 * Odczyt pojedynczego znaku z bufora kołowego.
 *
 * Parametry:
 * buffer to Wskaźnik na strukturę bufora.
 * data to Wskaźnik na zmienną, do której zostanie zapisany odczytany bajt.
 *
 * return:
 * int 0 przy powodzeniu, -1 gdy bufor jest pusty.
*/
int CircularBufferGetChar(CircularBuffer_t *buffer, uint8_t *data) {

    // Sprawdzanie czy bufor jest pusty
    if (buffer->head == buffer->tail) {
    	return -1;
    }

    // Odczytywanie wartość z bufora
    *data = buffer->dataArray[buffer->tail];

    // Przesówanie wskaźnika ogona na kolejne miejsce albo jego też zawijanie.
    buffer->tail = (buffer->tail + 1) % buffer->size;

    return 0;
}
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra
/*
 * Sprawdza, czy w buforze kołowym znajdują się dane do odczytania.
 *
 * Parametry:
 * buffer - Wskaźnik na strukturę bufora, która ma zostać sprawdzona.
 *
 * Zwracana wartość:
 * 0, jeśli w buforze znajdują się dane.
 * -1, jeśli bufor jest pusty.
*/
int CircularBufferHasData(CircularBuffer_t *buffer) {

    if (buffer->head == buffer->tail) {
    	return -1;
    }

    return 0;
}
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra

// -------------------------W dokumentacji------------------góra
/**
*	Oblicza liczbę zajętych pozycji w buforze kołowym.
*	Taka sama analogia co w funkcji ArchBufferOccupied.
*
*	return:
*	buffer Wskaźnik na strukturę bufora.
*
*	return:
*	int Liczba zajętych miejsc w buforze.
*/
int CircularBufferOccupied(CircularBuffer_t *buffer) {
	if(buffer -> head >= buffer -> tail){
		return buffer -> head - buffer -> tail;
	} else {
		return (buffer -> size - buffer -> tail + buffer -> head);
	}
}
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra

// Dekodowanie z kodu base64
/*
 * Podana funkcja zapisuje zdekodowane dane do bufora przeznaczonego na zdekodowane dane.
 *
 * Zwraca:
 * -1 -> Jeśli wystąpił błąd
 * Wartość większą bądź równą zero -> Podana liczba ma odzwierciedlać ilość zakodowanych znaków co będzie sugerowało
 * że działanie przebiegło poprawnie.
 */
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra

// Kodowanie na base 64
/*
 * Analiza Parametrów Funkcji
Funkcja przyjmuje dwa argumenty:
1.	uint8_t* data: To jest wskaźnik na dane wejściowe.
2.	int data_sz: To jest rozmiar danych w bajtach. Mówi funkcji, ile
 kolejnych bajtów, zaczynając od adresu data, ma przetworzyć.

Zasada Działania: Konwertowanie 8-bitowych do 6-bitowych
•	Dane wejściowe to seria bajtów.
•	Dane wyjściowe to seria 6 bitów.
Ponieważ 24 jest najmniejszą wspólną wielokrotnością 8 i 6, algorytm operuje na blokach 24-bitowych:
•	Bierze 3 „pudełka” 8-bitowe (3 * 8 = 24 bity).
•	Przepakowuje ich zawartość do 4 pudełek 6-bitowych (4 * 6 = 24 bity).
•	Każde 6-bitowe pudełko ma wartość od 0 do 63, której odpowiada jeden znak z tabeli base64_table.
 *
 */
void Base64Encode(uint8_t* data, int data_sz)
{
	// Tabela ze znakami zawartymi w base64
	uint8_t base64_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'};


	// •	Pętla iteruje po danych wejściowych, ale w każdej iteracji przeskakuje o 3 bajty (i += 3).
	for (int i = 0; i < data_sz; i += 3)
	{

		// Wyłuskiwanie pierwszych szejściu bitów i dopasowywanie ich do odpowiedniej litery.
		/*
		 * •	data[i]: Pobiera pierwszy bajt z bieżącego, 3-bajtowego bloku.
•	& 0xFC: Maska 0xFC to binarnie 11111100. Ta operacja izoluje 6 najstarszych bitów z bajtu.
•	>> 2: Wynik (np. xxxxxx00) jest przesuwany o 2 bity w prawo, aby stał się poprawną 6-bitową
 wartością (00xxxxxx), czyli liczbą od 0 do 63.
•	base64_table[...]: Używamy tej wartości jako indeksu, aby znaleźć odpowiedni znak w tabeli.
•	CircularBufferPutChar(...): Wstawiamy pierwszy zakodowany znak do bufora wyjściowego.
		 *
		 */
		CircularBufferPutChar(&encoded_data_circ_buff, base64_table[(data[i] & 0xFC) >> 2]);

		/*
		 * Wyrażenie data_sz - i oblicza, ile bajtów zostało do przetworzenia w ostatnim bloku.
		 *  Może to być 3 (przypadek default), 2 lub 1.

Przypadek default: Pełny blok (3 bajty wejściowe -> 4 znaki wyjściowe)

To jest standardowy scenariusz, gdy mamy co najmniej 3 bajty do przetworzenia.

Wizualizacja:
  BAJT 1 (data[i])      BAJT 2 (data[i+1])      BAJT 3 (data[i+2])                                                                                             ----------------------------------------------------------------+                                                                                           |7 6 5 4 3 2 1 0|       |7 6 5 4 3 2 1 0|       |7 6 5 4 3 2 1 0|                                                                                                       ----------------------------------------------------------------+                                                                                                                                           \__ZNAK 1__/\_ZNAK 2____________/\____ZNAK 3_______/\___ZNAK 4_/
•	Drugi znak: base64_table[((data[i] & 0x03) << 4) | ((data[i + 1] & 0xF0) >> 4)]
o	data[i] & 0x03: Bierze 2 najmłodsze bity z BAJTU 1.
o	<< 4: Przesuwa je w lewo, robiąc „miejsce”.
o	data[i + 1] & 0xF0: Bierze 4 najstarsze bity z BAJTU 2.
o	>> 4: Przesuwa je w prawo.
o	|: Łączy je w 6-bitową wartość.
•	Trzeci znak: base64_table[((data[i + 1] & 0x0F) << 2) | ((data[i + 2] & 0xC0) >> 6)]
o	data[i + 1] & 0x0F: Bierze 4 najmłodsze bity z BAJTU 2.
o	<< 2: Przesuwa je w lewo.
o	data[i + 2] & 0xC0: Bierze 2 najstarsze bity z BAJTU 3.
o	>> 6: Przesuwa je w prawo.
o	|: Łączy je.
•	Czwarty znak: base64_table[((data[i + 2] & 0x3F))]
o	data[i + 2] & 0x3F: Bierze 6 najmłodszych bitów z BAJTU 3.

Przypadek case 2: Dwa bajty na końcu (2 bajty wejściowe -> 3 znaki + 1 znak dopełnienia)
Gdy na końcu danych zostały tylko 2 bajty.
•	Drugi znak: Taki sam jak w default.
•	Trzeci znak: base64_table[((data[i + 1] & 0x0F) << 2)]
o	data[i + 1] & 0x0F: Bierze 4 najmłodsze bity z BAJTU 2.
o	<< 2: Przesuwa je w lewo. Dwa najmłodsze bity powstałej 6-bitowej wartości są zerami.
•	Dopełnienie: CircularBufferPutChar(&encoded_data_circ_buff, '=');
o	Dodajemy jeden znak =, aby wynikowy ciąg miał długość podzielną przez 4.

Przypadek case 1: Jeden bajt na końcu (1 bajt wejściowy -> 2 znaki + 2 znaki dopełnienia)
Gdy na końcu danych został tylko 1 bajt.
•	Drugi znak: base64_table[((data[i] & 0x03) << 4)]
o	data[i] & 0x03: Bierze 2 najmłodsze bity z BAJTU 1.
o	<< 4: Przesuwa je w lewo. Cztery najmłodsze bity są zerami.
•	Dopełnienie: CircularBufferPutChar(&encoded_data_circ_buff, '=');
CircularBufferPutChar(&encoded_data_circ_buff, '=');
o	Dodajemy dwa znaki =, aby wynikowy ciąg miał długość podzielną przez 4.
		 *
		 */
		switch (data_sz - i)
		{

		// Przypadek w którym został w tablicy tylko 1 znak.
		case 1:

			// Wyłuskiwanie ostatnich dwóch bitów i wypełnianie pustych
			// wartości tak by zakodowany tekst był wielokrotnością 4.
			CircularBufferPutChar(&encoded_data_circ_buff, base64_table[((data[i] & 0x03) << 4)]);
			CircularBufferPutChar(&encoded_data_circ_buff, '=');
			CircularBufferPutChar(&encoded_data_circ_buff, '=');
			break;

		// Przypadek w którym zostały w tablicy tylko 2 znaki.
		case 2:

			// Wyłuskiwanie szejściu bitów ze znaków.
			CircularBufferPutChar(&encoded_data_circ_buff, base64_table[((data[i] & 0x03) << 4) | ((data[i + 1] & 0xF0) >> 4)]);

			// Wyłuskiwanie ostatnich dwóch znaków i wypełnianie znakiem =
			CircularBufferPutChar(&encoded_data_circ_buff, base64_table[((data[i + 1] & 0x0F) << 2)]);
			CircularBufferPutChar(&encoded_data_circ_buff, '=');
			break;

		default:

			// Wyłuskiwanie kolejnych 6 znaków.
			CircularBufferPutChar(&encoded_data_circ_buff, base64_table[((data[i] & 0x03) << 4) | ((data[i + 1] & 0xF0) >> 4)]);
			CircularBufferPutChar(&encoded_data_circ_buff, base64_table[((data[i + 1] & 0x0F) << 2) | ((data[i + 2] & 0xC0) >> 6)]);
			CircularBufferPutChar(&encoded_data_circ_buff, base64_table[((data[i + 2] & 0x3F))]);
			break;
		}
	}
}

// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra

uint16_t ComputeCRC16(uint8_t* puchMsg, unsigned short usDataLen) {
    static const uint8_t auchCRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40
    };

    static const uint8_t auchCRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
        0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
        0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
        0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
        0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
        0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
        0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
        0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
        0x40
    };

    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    unsigned uIndex;

    while (usDataLen--) {
        uIndex = uchCRCHi ^ *puchMsg++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }

    return (uchCRCHi << 8 | uchCRCLo);
}

// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra
// Ustawianie domyślnych wartości w zmiennych. Ta funkcja jest wykonywana w momęcie kiedy wystąpi jakieś przepęłninie
// albo błąt i potrzebne jest zresetowanie wszsytkich stanów by jak najszybciej zacząć przyjmować nowe ramki.
void ClearAndResetState() {
    current_state = STATE_WAIT_START;
    frame_content_index = 0; // Resetujemy indeks bufora tymczasowego ramki

    HAL_TIM_Base_Stop_IT(&htim11);
    __HAL_TIM_SET_COUNTER(&htim11, 0);

    // Czyścimy bufory używane do przetwarzania danych, aby były gotowe na nową ramkę
    uint8_t temp_char;

    int counter = 0;
    while (CircularBufferGetChar(&decoded_data_circ_buff, &temp_char) == 0 && counter++ < SIZE_OF_DANE + 1);

    counter = 0;
    while (CircularBufferGetChar(&encoded_data_circ_buff, &temp_char) == 0 && counter++ < FRAME_CONTENT_MAX_SIZE + 1);
}

// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra
// Zamiana wartości hex na kodowanie w ASCII
void HexToAscii(uint16_t hex_value, uint8_t* output) {
    const char hex_digits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

    // Wyciągnięcie każdej cyfry hexadecymalnej i zamiana na odpowiedni znak
    output[0] = hex_digits[(hex_value >> 12) & 0xF]; // Najstarsze 4 bity
    output[1] = hex_digits[(hex_value >> 8) & 0xF];  // Kolejne 4 bity
    output[2] = hex_digits[(hex_value >> 4) & 0xF];  // Kolejne 4 bity
    output[3] = hex_digits[hex_value & 0xF];         // Najmłodsze 4 bity
}
// -------------------------W dokumentacji------------------duł

// -------------------------W dokumentacji------------------góra

// Funkcja sprawdzająca czy podany znak należy do znaków komend
int CheckCommandChar(uint8_t chr) {
	for (int var = 0; var < sizeof(commends)/sizeof(commends[0]); ++var) {
		if(chr == commends[var]) {
			return 1;
		}
	}

	return 0;
}

// Inicjuje transmisję danych z bufora TX.
void ProcessTxBuffer() {

	__disable_irq();

    // Wysyłaj kolejne dane tylko jeśli dane są w buforze nadawczym
    if (tx_in_progress == 0 && CircularBufferHasData(&tx_circular_buffer) == 0) {

    	uint8_t byte;
    	// Pobieranie danych z bufora nadawczego i ich wysyłąnie
        if (CircularBufferGetChar(&tx_circular_buffer, &byte) == 0) {

        	tx_in_progress = 1;

        	__enable_irq();

        	// Rozpoczęcie transmisji.
            HAL_UART_Transmit_IT(&huart2, &byte, 1);
        } else {
        	__enable_irq();
        }
    } else {
    	__enable_irq();
    }
}

// -------------------------W dokumentacji------------------duł

/**
 * Sprawdza, czy podany znak jest cyfrą heksadecymalną (0-9, a-f, A-F).
 * c Znak do sprawdzenia.
 * 1 jeśli znak jest cyfrą heksadecymalną, 0 w przeciwnym razie.
 */
int my_isxdigit(int c) {
	// Chodzi o to by znaleść odpowiedni przedział do którego należy przesłana liczba.
    if ((c >= '0' && c <= '9') ||
        (c >= 'a' && c <= 'f') ||
        (c >= 'A' && c <= 'F')) {
        return 1;
    }
    return 0;
}

/**
 * Sprawdza, czy podany znak jest alfanumeryczny (litera lub cyfra).
 * c Znak do sprawdzenia.
 * 1 jeśli znak jest alfanumeryczny, 0 w przeciwnym razie.
 */
int my_isalnum(int c) {
    if ((c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9')) {
        return 1;
    }
    return 0;
}

// -------------------------- TESTY ------------------- start
// Ta funkcja będzie budować ramkę i wkładać ją do GŁÓWNEGO bufora TX
// Parametrami tej funkcji jest:
/*
 * cargo -> które ma za cal przechowywać zawartość wartości która ma zostać przesłana
 * cargo_len -> zmienna przechowująca długość zmiennej cargo która ma zostać wysłana
 */
int QueueFrameForSending(const char* cargo, uint8_t cargo_len) {
    if (cargo_len == 0 || cargo_len > SIZE_OF_DANE) {
        // Obsługa błędu - pusty lub za długi ładunek (przekraczający 99 bajtów)
        return -1;
    }

    // Obliczamy dokładny rozmiar ramki, która powstanie
    uint8_t raw_total_len = 4 + cargo_len;

    // Długość po zakodowaniu Base64
    int b64_len = CALC_B64_LEN(raw_total_len);

    // Całkowita długość ramki w buforze TX:
    // ':' (1) + Base64 (b64_len) + CRC (4) + ';' (1)
    int total_frame_size = 1 + b64_len + CRC_OF_SIZE + 1;

    // Sprawdzamy czy mamy tyle miejsca w buforze TX
    int free_space = BUF_SIZE_TX - CircularBufferOccupied(&tx_circular_buffer) - 1;

    // Sprawdzanie czy w buforze nadawczym jest miejsce.
    if (free_space < total_frame_size) {
        return -1;
    }

    // Zbuduj blok do zakodowania (w lokalnej tablicy)
    uint8_t decoded_block[4 + SIZE_OF_DANE];
    decoded_block[0] = '?'; // Nadawca MC
    decoded_block[1] = sender; // Odbiorca PC
    decoded_block[2] = 'G'; // Komenda (odpowiedź ogólna)
    decoded_block[3] = cargo_len;

    /*
     * Kopiowanie danych do jednego bufora liniowego.
     * Jest to zrobione po to bypóźniej te dane muc zakodować.
     */
    memcpy(&decoded_block[4], cargo, cargo_len);

    // Zmienna przehowująca długość danych nie zakodowanych, jest ona nam potrzebna bo funkcja kodująca dane wysyła
    // je do funkcji kodującej dane.
    uint8_t decoded_len = 4 + cargo_len;

    // Obliczanie CRC
    uint16_t computed_crc = ComputeCRC16(decoded_block, decoded_len);
    uint8_t computed_crc_ascii[CRC_OF_SIZE];
    HexToAscii(computed_crc, computed_crc_ascii);

    // Kodowanie zawartości do wysyłania.
    Base64Encode(decoded_block, decoded_len);

    // Wkładanie całej ramki do GŁÓWNEGO bufora tx_circular_buffer
    CircularBufferPutChar(&tx_circular_buffer, ':');

    // Wkładane ZAKODOWANYCH danych do bufora kołowego
    uint8_t b64_char;
    while(CircularBufferGetChar(&encoded_data_circ_buff, &b64_char) == 0) {
        CircularBufferPutChar(&tx_circular_buffer, b64_char);
    }

    // Wkładanie obliczonego CRC16 - MODBUS
    for(int i = 0; i < CRC_OF_SIZE; i++) {
        CircularBufferPutChar(&tx_circular_buffer, computed_crc_ascii[i]);
    }

    // Dodawanie końca ramki
    CircularBufferPutChar(&tx_circular_buffer, ';');

    return 0;
}
// ---------------------------- Testy --------------------- Koniec

// -------------------------W dokumentacji------------------góra
// Inicjuje maszynę stanów w odpowiedzi na otrzymaną komendę.
/*
 * Jest wywoływana po pomyślnym zwalidowaniu całej ramki komunikacyjnej. Jej zadaniem jest
 *  zinterpretowanie komendy otrzymanej od użytkownika (PC) i zainicjowanie odpowiedniej akcji.
 *
 *  Funkcja jest zaimplementowana jako instrukcja switch-case. Każdy case odpowiada jednemu
 *  poleceniu zdefiniowanemu w protokole.
 */
int Base64Decode(uint8_t* data, int data_sz) {

	// Zmienna przechowująca ilość zdekodowanych danych.
	int decoded_total_len = 0;

	/*
	 * Podany fragment Sprawdza, czy długość danych wejściowych jest wielokrotnością 4.
	 * Ponieważ algorytm Base64 zawsze operuje na blokach po 4 znaki, nawet jeśli oryginalne
	 *  dane nie miały długości podzielnej przez 4 (wtedy stosuje się znaki dopełnienia =). Jeśli
	 *  długość jest inna, oznacza to, że ramka jest uszkodzona.
	 */
	// Walidacja długości bloku wejściowego
    if (data_sz % 4 != 0) {
        return -1; // Błąd: nieprawidłowa długość bloku Base64
    }

    for (int var = 0; var < data_sz - 3; ++var) {
		if(data[var] == '='){

            // Wysyłanie do pc informacji z mikrokontrolera.
            char header1[] = "Odpowiedz: Znak padding = występuje w środku kodowania base64";

            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

            // Uruchamianie wysyłania ramki.
            ProcessTxBuffer();

			return -1;
		}
	}

    if((data[data_sz - 2] == '=') && (data[data_sz - 1] != '=')){

        // Wysyłanie do pc informacji z mikrokontrolera.
        char header1[] = "Odpowiedz: Znak padding = występuje ma błędne miejsce gdzieś na koncu.";

        QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

        // Uruchamianie wysyłania ramki.
        ProcessTxBuffer();

    	return -1;
    }

	// Deklaracja tablicy ze wszystkimi znakami dla base64
	uint8_t base64_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'};

	// Inijalizacja tablicy przechowującej przypisane wartości decymalne w kodowaniu ASCII do karzdego znaku w base64.
	/*
	 * Tworzy odwrotną tabelę przeglądową. Zamiast dla każdego znaku z danych wejściowych przeszukiwać
	 *  całą base64_table w poszukiwaniu jego wartości (np. szukać 'A' by dowiedzieć się, że to wartość 0
	 *  w kodowaniu Base64), tworzymy dużą tablicę, gdzie indeksem jest kod ASCII a wartościom jest wartość
	 *  w kodzie Base64. Po wykonaniu pętli, reverse_table['A'] będzie miało wartość 0, reverse_table['B']
	 *  będzie miało wartość 1 itd.
	 */
	uint8_t reverse_table[256];

	for (int i = 0; i < 64; i++) {
		/*
		 * Przykład:
		 *
		 * 1.	base64_table[i]
		 * W naszym przykładzie tablica to base64_table[26].
		 * Patrząc na definicję tej tablicy, 27. elementem (indeks 26) jest znak 'a'.
		 * Więc wyrażenie sprowadza się do: reverse_table[ (unsigned int)'a' ] = 26;
		 *
		 * 2.	(unsigned int)'a'
		 * W tabeli ASCII znak 'a' ma wartość liczbową 97.
		 * Rzutowanie (unsigned int) formalnie konwertuje typ char na typ liczbowy,
		 * który może być użyty jako indeks tablicy.
		 * Nasza linia kodu staje się więc: reverse_table[97] = 26;
		 *
		 * 3.	reverse_table[97] = 26;
		 * "Do tablicy reverse_table, pod indeks 97, wpisz wartość 26".
		 * Stworzyliśmy w ten sposób bezpośrednie mapowanie: znak 'a' -> kod ASCII 97 -> wartość Base64 26.
		 */
		reverse_table[(unsigned int)base64_table[i]] = i;
	}

	for (int i = 0; i < data_sz; i += 4) {

		// Zmienna przechowująca aktualnie przerabiany blok danych.
		uint32_t block_value = 0;

		// Zmienna przechowująca ilość danch do przetworzenia. Ta zmienna jest konieczna ponieważ moje kodowanie base64
		// uwzględnia padding a dzięki temu mogę pominąć znaki paddnigowe w sowich obliczeniach.
		int valid_bytes = 3;

		// Odkodowuje podane liczby
		/*
		 * Rekonstrukcja ciągu binarnego: przykład:
		 * Pętla przetwarza dane w blokach po 4 znaki (32 bity). Przetwarzamy podany ciąg bitowy po 4 znaki z
		 *  tego powodu że skoro ja bazuje na kodowaniu ASCII a jedno kodowanie składa się z 8 bitów to a jeden
		 *  znak z kodowania Base64 składa się z 6 bitów to wspólną wielokrotnością będzie właśnie 24. A przez to
		 *   że nie ma typu 24-bitowego to więc dlatego przechowuje to w typie 32-bitowym i dlatego też przetwarzam
		 *    kod co 4 bajty.
		 *
		 *    Podana zmienna będzie przechowywała nasze znaki.
		 *
		 * reverse_table[data[i]] - Pobiera 6-bitową wartość pierwszego znaku Base64
		 * << 18 - Przesuwa wartość na pozycje 18-23 w 32-bitowej liczbie
		 * reverse_table[data[i+1]] - 6-bitowa wartość drugiego znaku\
		 * << 12 - Przesunięcie na pozycje 12-17 drógiego znaku
		 * | => Suma logiczna łączy obie wartości, tworząc częściowy 24-bitowy blok
		 *
		 * Przykład:
		 * Załóżmy, że dekodujemy fragment "TW" z dłuższego ciągu "TWF4".
		 * •	Pierwszy znak data[i] to 'T'.
		 * •	Drugi znak data[i+1] to 'W'.
		 * Zgodnie z tabelą Base64:
		 * •	Wartość dla 'T' to 19, co w zapisie 6-bitowym daje 010011.
		 * •	Wartość dla 'W' to 22, co w zapisie 6-bitowym daje 010110.
		 * Nasza zmienna block_value to 32-bitowy kontener, ale interesują nas tylko 24 bity
		 * , które posłużą do zrekonstruowania danych.
		 *
		 * Krok 1: Lewa strona operacji (reverse_table[data[i]] << 18)
		 * 1.	Pobranie wartości: reverse_table['T'] zwraca liczbę 19.
		 * 2.	Przesunięcie bitowe (<< 18): Bierzemy 6 bitów reprezentujących 19 i przesuwamy je o 18 pozycji w lewo.
		 *
		 * Wizualizacja:
    // 24 bity w block_value (początkowo same zera)
Bit:   23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
      +---------------------------------------------------------------------------------------------------+
Val:  | 0  0  0    0  0    0   0   0    0  0   0   0    0  0   0   0   0  0    0   0  0    0   0   0 |
      +---------------------------------------------------------------------------------------------------+

// Bity wartości 19 (010011)
       (0  1  0  0  1  1)

        << SHIFT 18 <<

// Wynik operacji:
Bit:   23  22 21  20   19  18  17  16  15  14  13 12 11 10 09 08 07 06 05 04 03 02 01 00
      +---------------------------------------------------------------------------------------------------+
Val:  | 0  1  0   0    1   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0 |
      +---------------------------------------------------------------------------------------------------+

		 * Krok 2: Prawa strona operacji (reverse_table[data[i+1]] << 12)

1.	Pobranie wartości: reverse_table['W'] zwraca liczbę 22.
2.	Przesunięcie bitowe (<< 12): Bierzemy 6 bitów reprezentujących 22 i przesuwamy je o 12 pozycji w lewo.

Wizualizacja:

// 24 bity (początkowo same zera)
Bit:   23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
+--------------------------------------------------------------------------------------------------------+
Val: | 0  0    0   0   0   0   0   0   0   0   0   0   0   0  0    0   0   0   0   0   0   0   0   0 |
+--------------------------------------------------------------------------------------------------------+
// Bity wartości 22 (010110)
(0 1 0 1 1 0)
    << SHIFT 12 <<

// Wynik operacji:
Bit:    23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
+---------------------------------------------------------------------------------------------------------+
Val: |  0    0  0   0    0   0   0  1   0    1   1   0  0    0  0   0   0  0    0    0  0   0   0   0 |
+---------------------------------------------------------------------------------------------------------+
		 * Krok 3: Końcowa operacja | (Bitowe OR)

Teraz bierzemy wyniki z Kroku 1 i Kroku 2 i łączymy je za pomocą operacji OR. OR działa tak, że jeśli na danej pozycji w którejkolwiek z liczb jest 1, to w wyniku też będzie 1.

Wizualizacja:

// Wynik z Kroku 1:
| 0 1 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 |
| (OR)
// Wynik z Kroku 2:
| 0 0 0 0 0 0 0 1 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 |
=
// Ostateczny wynik zapisany do block_value:
Bit:   23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
+--------------------------------------------------------------------------------------------------------+
Val: | 0   1   0   0   1   1   0   1   0   1   1   0   0   0   0   0   0   0   0   0   0   0   0   0 |
+--------------------------------------------------------------------------------------------------------+
        \------ BAJT 1 -------------/   \------ BAJT 2 ------------/

Po tej jednej linijce kodu, `block_value` zawiera już pierwsze 12 bitów zrekonstruowanych danych,
idealnie ułożone na swoich miejscach. Następne operacje w kodzie wypełnią pozostałe, puste miejsca
(lub pozostawią je puste, jeśli napotkają znak `=`).
		 *
		 */
		block_value = (reverse_table[data[i]] << 18) | (reverse_table[data[i + 1]] << 12);

		/*
		 *
Ta część kodu odpowiada na pytanie: "Czy trzeci znak jest normalną daną, czy informacją o końcu kodowania danych?"
•	Scenariusz A: if (data[i + 2] != '=') jest prawdą.
	Co to oznacza? Trzeci znak jest normalnym znakiem z alfabetu Base64 (np. 'F' w naszym
	przykładzie "TWF4"). Oznacza to, że mamy więcej danych do odkodowania.

	reverse_table[data[i + 2]]: Pobieramy 6-bitową wartość tego znaku (dla 'F' będzie to 5).
	<< 6: Przesuwamy te 6 bitów w lewo o 6 pozycji, aby umieścić je zaraz za bitami ze znaku 'W'.
	block_value |= ...: Używamy operacji OR, aby "wkleić" te 6 bitów na swoje miejsce w block_value,
	 nie naruszając bitów, które już tam są.
o	Stan valid_bytes: Pozostaje 3, ponieważ na razie wszystko wskazuje na to, że odkodujemy pełne 3 bajty.
•	Scenariusz B: else jest wykonywane.
o	Co to oznacza? Trzeci znak to =. Jest to sygnał, że oryginalne dane kończyły się w tym miejscu.
 Konkretnie, oznacza to, że ostatni 4-znakowy blok Base64 koduje tylko jeden oryginalny bajt. (np. dla "TQ==")
o	valid_bytes = 2;: Ustawiamy valid_bytes na 2, ale w tym przypadku valid_bytes po całej operacji
(wliczając obsługę czwartego znaku) będzie równe 1. W tym momencie kodu ustawienie na 2 jest krokiem pośrednim,
 który zostanie skorygowany w następnym bloku if.
o	Stan block_value: Nic więcej nie jest do niego dodawane. Pozostaje z 12 bitami z pierwszych dwóch znaków.
		 *
		 */
		if (data[i + 2] != '=') {
			block_value |= (reverse_table[data[i + 2]] << 6);
		} else {
			valid_bytes = 2;
		}

		/*
		 * Ta część odpowiada na pytanie: "Czy czwarty znak jest normalną daną, czy dopełnieniem?" i
		 *  finalizuje liczbę bajtów do odkodowania.
•	Scenariusz A: if (data[i + 3] != '=') jest prawdą.
o	Co to oznacza?: Czwarty znak jest normalnym znakiem (np. `'u'` w "TWF4"). Oznacza to, że mamy do
 czynienia z pełnym, 3-bajtowym blokiem danych.
o	reverse_table[data[i + 3]]: Pobieramy 6-bitową wartość tego znaku.
o	block_value |= : Wklejamy te 6 bitów na najniższe pozycje w block_value. Nie ma potrzeby przesuwania ( << 0 ).
o	Stan valid_bytes: Pozostaje 3.
•	Scenariusz B: else jest wykonywane.
o	Co to oznacza?: Czwarty znak to ‘=’. To jest ostateczne potwierdzenie, że strumień danych się kończy.
o	if(valid_bytes == 3): Kiedy ten warunek jest prawdziwy? Gdy trzeci znak był normalną daną, ale
 czwarty jest dopełnieniem (np. dla "TWE="). Oznacza to, że 4 znaki Base64 kodują dwa oryginalne bajty.
o	valid_bytes = 2: Ustawiamy ostateczną, poprawną wartość valid_bytes na 2.
o	else (czyli valid_bytes jest już równe 2 z poprzedniego bloku if): Kiedy ten warunek jest
prawdziwy? Gdy trzeci znak był już dopełnieniem (`=`), a czwarty również jest dopełnieniem (np. dla "TQ==").
 Co to oznacza? Oznacza to, że 4 znaki Base64 kodują tylko jeden oryginalny bajt.
o	valid_bytes = 1: Ustawiamy ostateczną, poprawną wartość valid_bytes na 1.
		 *
		 */

		if (data[i + 3] != '=') {
			block_value |= reverse_table[data[i + 3]];
		} else {
			if(valid_bytes == 3){
				valid_bytes = 2;
			} else {
				valid_bytes = 1;
			}
		}

		/*
		 * Po wykonaniu tego fragmentu kodu mamy dwie kluczowe informacje:

1.	block_value: Zawiera zrekonstruowane bity (od 8 do 24, w zależności od dopełnienia).
2.	valid_bytes: Precyzyjnie informuje, ile pełnych, bajtów mamy "wykroić" z block_value.

Kontynuacja przykładu:
Co mamy na wejściu do tego fragmentu?
Po wykonaniu poprzednich kroków, mamy dwie kluczowe zmienne:
1.	block_value: 32-bitowa zmienna, której 24 najmłodsze bity zawierają zrekonstruowane dane.
2.	valid_bytes: Liczba (1, 2 lub 3) mówiąca nam, ile z tych 24 bitów to faktyczne dane, a
 ile to "śmieci" wynikające z dopełnienia.

Przykład wizualny dla block_value po zdekodowaniu "TWF4" (gdzie valid_bytes = 3):

Bit:     23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
      +------------------------------------------------------------------------------------------------------+
Val:     0   1   0   0   1   1   0   1 | 0   1    1  0   0   0   0   1 | 0  1    1   1   1   0   0   0   |
      +------------------------------------------------------------------------------------------------------+
          \------- BAJT 1 ------------/   \------- BAJT 2 -----------/  \------ BAJT 3 -------------/
               (wartość 84)           (wartość 118)           (wartość 117)
             odpowiada 'T' i 'W'     odpowiada 'W' i 'F'     odpowiada 'F' i '4'
		 *
		 */

        // Dodawanie tylko poprawnych bajtów na podstawie paddingu
		// Sprawdzanie przepełnienia bufora docelowego
		/*
		 * •	if(valid_bytes >= 1): Ten warunek jest zawsze prawdziwy dla
		 * poprawnego bloku Base64. Sprawdza on, czy mamy do odzyskania co najmniej jeden bajt.
		 *  Jeśli blok nie byłby pusty, valid_bytes będzie miało wartość 1, 2 lub 3.
•	(block_value >> 16) (Przesunięcie bitowe w prawo):
o	Bierzemy naszą 24-bitową liczbę i przesuwamy wszystkie bity o 16 pozycji w prawo.
o	Efekt: Bity, które były na pozycjach 23-16 (czyli nasz BAJT 1), lądują teraz na najniższych pozycjach (7-0).
Wszystkie inne bity "wypadają" z prawej strony lub zostają na wyższych pozycjach.
	// Przed: |...| 01010100 | 01110110 | 01110101 |
	// Po >> 16: |...| 00000000 | 00000000 | 01010100 |
•	& 0xFF (Maska bitowa):
o	0xFF w systemie binarnym to 11111111.
o	Operacja & (bitowe AND) z tą maską zeruje wszystkie bity poza ostatnimi ośmioma.
o	Efekt: Izolujemy czysty, 8-bitowy BAJT 1 (wartość 84).
•	CircularBufferPutChar(...): Wywołujemy funkcję, aby zapisać odzyskany bajt (84) do bufora
 kołowego decoded_data_circ_buff.
•	if(... != 0): Sprawdzamy, czy zapis się powiódł. Jeśli CircularBufferPutChar zwróci błąd
(co oznacza, że bufor wyjściowy jest pełny).
•	return -1: natychmiast przerywamy całą funkcję Base64Decode, sygnalizując błąd.
		 *
		 */
		if(valid_bytes >= 1) {
			if(CircularBufferPutChar(&decoded_data_circ_buff, (block_value >> 16) & 0xFF) != 0){
				return -1;
			}

			decoded_total_len++;
		}

		/*
		 *
•	if(valid_bytes >= 2): Ten blok wykona się tylko wtedy, gdy z 4-znakowego
 bloku Base64 mamy odzyskać 2 lub 3 bajty (czyli np. dla "TWF4" lub "TWE="). Dla "TQ=="
  ten warunek będzie fałszywy i blok zostanie pominięty.
•	(block_value >> 8): Przesuwamy bity o 8 pozycji w prawo.
o	Efekt: Bity, które były na pozycjach 15-8 (czyli nasz BAJT 2), lądują na pozycjach 7-0.
	// Przed: |...| 01010100 | 01110110 | 01110101 |
	// Po >> 8: |...| 00000000 | 01010100 | 01110110 |
•	& 0xFF: Ponownie izolujemy 8 najmłodszych bitów, otrzymując czysty 2 BAJT.
•	Pozostałe operacje (zapis do bufora i obsługa błędu) są analogiczne.
		 *
		 */

		if(valid_bytes >= 2) {
			if(CircularBufferPutChar(&decoded_data_circ_buff, (block_value >> 8) & 0xFF) != 0){
				return -1;
			}

			decoded_total_len++;
		}

		/*
		 * •	if(valid_bytes >= 3): Ten blok wykona się tylko wtedy, gdy mamy do
		 * czynienia z pełnym blokiem, z którego odzyskujemy 3 bajty (np. tylko dla "TWF4").
•	block_value & 0xFF: Nie ma potrzeby przesuwania bitów (>> 0). BAJT 3 jest już na najniższych 8 pozycjach (7-0).
o	Efekt: Maska & 0xFF po prostu izoluje te 8 bitów, dając nam czysty BAJT 3.
•	Zapisujemy go do bufora.
		 *
		 */

		if(valid_bytes >= 3) {
			if(CircularBufferPutChar(&decoded_data_circ_buff, block_value & 0xFF) != 0){
				return -1;
			}

			decoded_total_len++;
		}
	}

	return decoded_total_len;
}

void SimulateSensorResponse(uint8_t command, CircularBuffer_t* decoded_data_buffer)
{

    switch(command)
    {
        case '!': // Inicjalizacja czujnika

            aht15_current_state = AHT15_STATE_INIT_SEND_CMD; // Inicjalizacja czujnika.
            break;

        case '$': // Włączanie pomiaru danych przez interwał

            // Wysyłanie do pc informacji z mikrokontrolera.
            char header2[] = "Odpowiedz: Włączenie pomiaru cyklicznego.";

            QueueFrameForSending(header2, (sizeof(header2)/sizeof(header2[0]))); // Rozpocznij wysyłanie nagłówka

            is_manual_measurement = 0; // Wyłączenie manualnego pobierania bieżącej wartości jeśli była włączona.
            logging_enabled = 1;       // Traktujemy to jako włączenie cyklicznego logowania

            // Warunek sprawdzajcy czy maszyna stanów jest aktualnie w stanie nie zajętym, by nie kolidować
            // z aktualnie wykonywanymi pomiarami.
            if(aht15_current_state == AHT15_STATE_IDLE) {
                 aht15_current_state = AHT15_STATE_MEASURE_SEND_CMD;
            }

            // Uruchamianie wysyłania ramki.
            ProcessTxBuffer();

            break;

        case '&': // Ustawianie interwału

            // Wysyłanie do pc informacji z mikrokontrolera.
        	char header3[] = "Odpowiedz: Ustawianie interwalu.";

            QueueFrameForSending(header3, (sizeof(header3)/sizeof(header3[0]))); // Rozpocznij wysyłanie nagłówka

            // Zmienna tymczasowa do pobierania wartości z bufora kołowego.
            uint8_t  data = 0;

            // Tymczasowa zmienna do "składania liczby mili sekund do interwału".
            uint32_t interval_ms = 0;

            for (int var = 0; var < 5; ++var) {
            	if (CircularBufferGetChar(&decoded_data_circ_buff, &data) == 0){
                	interval_ms <<= var == 0 ? 0 : 8;
                	interval_ms |= data;
            	}
			}

            logging_interval_ms = interval_ms;

            // Uruchamianie wysyłania ramki.
            ProcessTxBuffer();

            break;

        case '%': // Komenda kończąca wykonywanie interwału pobierania danych z czujnika.

            // Wysyłanie do pc informacji z mikrokontrolera.
        	char header4[] = "Odpowiedz: Zakończenie pobierania co interwał";

            QueueFrameForSending(header4, (sizeof(header4)/sizeof(header4[0]))); // Rozpocznij wysyłanie nagłówka

            // Uruchamianie wysyłania ramki.
            ProcessTxBuffer();

            logging_enabled = 0;
            break;

        case '^':
        	// Rzeby nie nadpisywać danych podczas pobierania wartości
        	HAL_TIM_Base_Stop_IT(&htim10);

        	// Zmienna przechowująca ilość danych jakie mają zostać wysłane do pc
        	uint8_t size_to_sent = 0;

        	// Zmienna do której będzie zapisany indeks od którego ma zostać pobrane dane archiwalne z tablicy bufora
        	// kołowego.
            uint16_t index_start = 0;

            // Przedstawia ilość danych jakie mają zostać pobrane z tablicy bufora kołowego.
            uint16_t count_to_get = 0;

            // Zmienna tymczasowa do pobierania wartości z bufora kołowego.
            data = 0;

            for (int var = 0; var < 5; ++var) {
            	if (CircularBufferGetChar(&decoded_data_circ_buff, &data) == 0){
            		if(var < 2){
            			index_start <<= var == 0 ? 0 : 8;
            			index_start |= data;
            		} else {
            			count_to_get <<= var == 0 ? 0 : 8;
            			count_to_get |= data;
            		}
            	}
			}

            // Pobieranie ilości zajętych wartości z danego buforu. Ta zmienna przyda się nam do walidacji
            // tego czy użtkownik podał prawidłowy zakres danych jakich chce pobrać.
            uint16_t occupied = ArchBufferOccupied(&history_circular_buffer);

            // Walidacja zakresu, czy pobierana ilość danych nie jest mniejsza od 0 i sprawdzanie czy taka
            // ilość danych może zostać pobrana. Jeśli warunek jest prawdzywy to jest wysyłana wiado
            // w celu przekazania błędu w który jest zapisany zakres w jakim chce pobrać dane a także jaka jest ogólna
            // możliwa ilość danych do pobrania.
            if (index_start < 0 || count_to_get <= 0 || (index_start + count_to_get) > occupied) {

            	// Zmienna do której będą zapisywane dane które mają zostać wysłane.
            	char header6[150];

                size_to_sent = printf("Blad: Zadany zakres (%d, %d) jest nieprawidlowy. Dostepnych pomiarow: %d.", index_start, count_to_get, occupied);

                snprintf(header6, size_to_sent, "Blad: Zadany zakres (%d, %d) jest nieprawidlowy. Dostepnych pomiarow: %d.", index_start, count_to_get, occupied);

                QueueFrameForSending(header6, size_to_sent);

                // Uruchamianie wysyłania ramki.
                ProcessTxBuffer();

                break;
            }

            // Struktura dla mojej maszyny stanów
            archive_state.count_total = count_to_get;
            archive_state.start_index = index_start;
            break;

        case '*': // Podgląda danych bierzących z czujnika
        	// Ustawienie flagi na 1 by umożliwić odczyt aktualnej danej z czujnika.
        	is_manual_measurement = 1;
        	aht15_current_state = AHT15_STATE_MEASURE_SEND_CMD;

            break;

        case '@':

        	char header8[100];

            size_to_sent = snprintf(header8, sizeof(header8), "Ustawiony interwal jest na podana liczbe %" PRIu32 " ms", logging_interval_ms);

            QueueFrameForSending(header8, size_to_sent);

            // Uruchamianie wysyłania ramki.
            ProcessTxBuffer();

        	break;

        default:
            return;
    }
}
// -------------------------W dokumentacji------------------duł

// Konwertuje małą literę z kodowania ASCI na wielką literę z kodowania ASCII.
uint8_t my_toupper(uint8_t c) {
    // Sprawdza, czy znak 'c' znajduje się w zakresie małych liter ('a' do 'z')
    if (c >= 'a' && c <= 'z') {
        // Jeśli tak, odejmij 32, aby uzyskać jego wielki odpowiednik
        return c - 32;
    }
    // Jeśli nie, zwróć oryginalny znak
    return c;
}

// Maszyna stanów do czujnika AHT15.
void AHT15_Process()
{
    // Jeśli flaga błędu jest ustawiona by przejść do stanu błędu
    if (i2c_error_flag) {
    	// Ustawienie flagi błędu na 0 by jak gdyby pojawił się nowy błąd
    	// to żeby mógł zostać obsłużony dopiero jak wystąpi.
        i2c_error_flag = 0;
        i2c_busy_flag = 0; // Ustawienie że czujnik jest nie aktywny.
        aht15_current_state = AHT15_STATE_ERROR; // Ustawienie aktualnego stanu jako stan błędu.
    }

    // Jeśli operacja I2C jest w toku, nic nie rób - czekaj na przerwanie
    if (i2c_busy_flag) {
        return;
    }

    // Tworzenie tablicy w której będą przechowywyane odpowiedzi.
    char response_text[100];

    // Zerowanie całej tablicy by nie znajdywały się tam "śmieci"Test.
    for (int var = 0; var < (sizeof(response_text)/sizeof(response_text[0])); ++var) {
    	response_text[var] = '\0';
	}

    switch (aht15_current_state)
    {
        case AHT15_STATE_IDLE: // Stan bezczynności dla czujnika.
            // Nic do zrobienia, czekamy na nowe polecenie
            break;

        case AHT15_STATE_INIT_SEND_CMD: // Sekcja inicjalizacji czujnika
        {
        	i2c_busy_flag = 1;
            static uint8_t init_command[] = { 0xE1, 0x08, 0x00 };
            if (HAL_I2C_Master_Transmit_IT(&hi2c1, AHT15_ADDRESS, init_command, sizeof(init_command)) != HAL_OK) {
                i2c_busy_flag = 0;
                aht15_current_state = AHT15_STATE_ERROR;

            	snprintf(response_text, sizeof(response_text), "Odpowiedz: Czujnik AHT15 NIE poprawnie zainicjalizowany.");
            } else {
                aht15_current_state = AHT15_STATE_INIT_DONE;
            }
            break;
        }

        case AHT15_STATE_INIT_DONE: // Sekcja wykonująca się gdy czujnik został poprawnie zainicjalizowany.
        {
            // Ten stan jest wykonywany, gdy transmisja I2C się zakończy
        	snprintf(response_text, sizeof(response_text), "Odpowiedz: Czujnik AHT15 poprawnie zainicjalizowany.");
            aht15_current_state = AHT15_STATE_IDLE; // Wracanie do stanu bezczynności.
            break;
        }

        case AHT15_STATE_MEASURE_SEND_CMD: // Sekcja wysyłania żądania wykonywania pomiaru temperatury i wilgotności.
        {
        	if(logging_enabled == 1 || is_manual_measurement == 1){
            	i2c_busy_flag = 1;
                static uint8_t trigger_command[] = { 0xAC, 0x33, 0x00 };
                if (HAL_I2C_Master_Transmit_IT(&hi2c1, AHT15_ADDRESS, trigger_command, sizeof(trigger_command)) != HAL_OK) {
                    i2c_busy_flag = 0;
                    aht15_current_state = AHT15_STATE_ERROR;
                } else {

                	aht15_current_state = AHT15_STATE_IDLE;
                }
        	}else{
        		aht15_current_state = AHT15_STATE_IDLE;
        	}
            break;
        }

        case AHT15_STATE_MEASURE_READ_DATA: // Stan odczytywania wykonanego pomiaru z czujnika.
        {
        	// Sprawdzanie czy czujnika ma za zadanie wykonanie pomiaru manualnego czy cyklicznego.
        	if(logging_enabled == 1 || is_manual_measurement == 1){
                i2c_busy_flag = 1;

                // Odczytywanie podanego pomiaryu i zapisania go do sensor_data_buffer
                if (HAL_I2C_Master_Receive_IT(&hi2c1, AHT15_ADDRESS, sensor_data_buffer, 6) != HAL_OK) {
                    i2c_busy_flag = 0;
                    aht15_current_state = AHT15_STATE_ERROR;
                } else {
                    aht15_current_state = AHT15_STATE_PROCESS_DATA;
                }
        	}else{
        		aht15_current_state = AHT15_STATE_IDLE;
        	}
            break;
        }

        // Przetwarzanie odczytanych danych po ich pobraniu z czujnika i zapisanie ich do bufora archiwalnego.
        case AHT15_STATE_PROCESS_DATA:
        {
            if (logging_enabled == 1 || is_manual_measurement == 1) {
                if ((sensor_data_buffer[0] & 0x80) != 0) {
                     if (is_manual_measurement == 1) {
                    	 snprintf(response_text, sizeof(response_text), "Blad: Czujnik AHT15 jest wciaz zajety.");
                     }
                     // Jeśli błąd wystąpił przy pomiarze cyklicznym, po prostu go ignorujemy.
                     // Bo jak jeden pomiar się nie wykona z powodu zajętości to nic nie szkodzi w pomiarze cyklicznym.
                } else {
                     /*
                      * Wyłuskiwanie pomiarów z bajtów zgodnie z schematem zawartym w dokumentacji.
                      */
                	uint32_t raw_humidity = ((uint32_t)(sensor_data_buffer[1] << 12)) | ((uint32_t)(sensor_data_buffer[2] << 4)) | ((uint32_t)(sensor_data_buffer[3] >> 4));
                     uint32_t raw_temperature = ((uint32_t)((sensor_data_buffer[3] & 0x0F) << 16)) | ((uint32_t)(sensor_data_buffer[4] << 8)) | ((uint32_t)sensor_data_buffer[5]);

                     /*
                      * Konwersja na tym zmiennoprzecinkowy.
                      */
                     float humidity = ((float)raw_humidity / 1048576.0) * 100.0;
                     float temperature = ((float)raw_temperature / 1048576.0) * 200.0 - 50.0;

                     if (is_manual_measurement == 1) {
                         // Jeśli to pomiar na żądanie, wyślij wynik przez UART
                         snprintf(response_text, sizeof(response_text), "Odpowiedz: T=%.2f C, H=%.2f %%", temperature, humidity);
                     } else {
                         // W przeciwnym wypadku (pomiar cykliczny), zapisz do historii
                         ArchPutData(&history_circular_buffer, temperature, humidity);
                     }
                }

                // Wyzeruj flagę pomiaru ręcznego po jego obsłużeniu
                if (is_manual_measurement == 1) {
                    is_manual_measurement = 0;
                }
            }
            aht15_current_state = AHT15_STATE_IDLE;
            break;
        }

        case AHT15_STATE_ERROR:

        	snprintf(response_text, sizeof(response_text), "Blad I2C: Blad, kod: %lu", i2c_error_code);

            i2c_error_code = HAL_I2C_ERROR_NONE; // Wyzeruj kod błędu po obsłużeniu
            aht15_current_state = AHT15_STATE_IDLE; // Zresetuj stan po błędzie

            break;
    }

    // Wysyłanie danych które zostały zapisane w zmiennej
    if (response_text[0] != '\0') {
        QueueFrameForSending(response_text, sizeof(response_text));

        // Uruchamianie wysyłania ramki.
        ProcessTxBuffer();
    }
}

void ProcessArchiveTransmission() {
	if(archive_state.count_total > 0) {

		Measurement_t tempData;
		uint8_t size_to_sent;
		char cargo_buffer[SIZE_OF_DANE];
		int cargo_len = 0;

		/*
		 * Ta zmienna będzie nam służyła do tego by sprawdzać czy w zmiennej cargo_buffer
		 * aby na pewno niczego nie ma i by tym samym nie tracić danych. Działa to na zasadzie
		 * że jak jakieś wartości, nie zmieszczą się aktualnie w buforze to dzięki tej wartości mogę "cofnąć"
		 * dane które aktualnie miały być wysłane by były przerobione jeszcze raz i tak do skutku aż bufor będzie
		 * miał miejsce. Dzięki temu rozwiązaniu to ta funkcja nie blokuje wykonywania kodu aż do opróżnienia buforu
		 * nadawczego tylko umożliwia nawet wpisywanie wartości nawet gdy ten bufor jest aktualnie w użyciu.
		 */
		int number_of_current_request = 0;

        while (archive_state.count_total != 0) {
            if (ArchGetDataAtIndex(&history_circular_buffer, archive_state.start_index, &tempData) == 0) {

            	char temp_line_buffer[50];

            	// Sformułowanie jednej linii do bufora tymczasowego by następnie go wysłać.
            	// Używam funkcji snprintf by przeformatować liczbę na znaki i przekomiować do mojej zmiennej.
            	size_to_sent = snprintf(temp_line_buffer, sizeof(temp_line_buffer),"Pomiar %d: T=%.2f; H=%.2f",
            			archive_state.start_index, tempData.temperature, tempData.humidity);

                // Sprawdź, czy dodanie nowej linii przekroczy maksymalny rozmiar danych dla JEDNEJ ramki.
                if (cargo_len + size_to_sent > SIZE_OF_DANE) {
                	// Warunek sprawdzajądy czy zmieści się ładunek do bufora kołowego.
                	if(QueueFrameForSending(cargo_buffer, cargo_len) != 0){
                        // Uruchamianie wysyłania ramki.
                        ProcessTxBuffer();

                        if(number_of_current_request != 0) {
                        	/*
                        	 * Odpowiednio zmieniejszanie i powiększanie zmiennych by
                        	 * powturzyły jeszcze raz odczyty które nie załapały się na wysłanie.
                        	 */
                        	archive_state.start_index -= number_of_current_request;
                        	archive_state.count_total += number_of_current_request;
                        }

                		return;
                	}

                    // Zresetuj bufor cargo, aby zacząć zbierać dane do następnej ramki.
                    cargo_len = 0;

                	// Zerowanie całego bufora przechowującego całą ramkę by nie znajdywały się tam śmieci
                    for (int var = 0; var < (sizeof(cargo_buffer)/sizeof(cargo_buffer[0])); ++var) {
                    	cargo_buffer[var] = '\0';
                	}

                    number_of_current_request = 0;
                }

                // Kopiowanie nową linię do bufora cargo_buffer.
                /*
                 * Podana konstrukcja cargo_buffer + cargo_len wysyłam tutaj adres w pamięci
                 * przesunięty o wartość cargo_len. Jest to zrobione w tym celu aby kopiować dane do wolnych pól
                 * a nie żeby te dane nadpisywać.
                 */
                memcpy(cargo_buffer + cargo_len, temp_line_buffer, size_to_sent);
                number_of_current_request++;

                cargo_len += size_to_sent;
                archive_state.start_index++;
                archive_state.count_total--;
            }
        }

        // Po wyjściu z pętli w cargo_buffer mogą być resztki danych.
        if (cargo_len > 0) {
        	// Próbujemy wysłać teraz dane.
            if (QueueFrameForSending(cargo_buffer, cargo_len) != 0) {

                // Jeśli nie ma miejsca na ostatnią ramkę to wysyłamy resztki.
                ProcessTxBuffer();

                // Cofamy liczniki, żeby spróbować wysłać tę końcówkę w następnym obiegu
                archive_state.start_index -= number_of_current_request;
                archive_state.count_total += number_of_current_request;
                return;
            }
        }

        // Zawsze na końcu upewniamy się, że transmisja UART działa by tym samym wysłać dane które zostały zapisane
        // ale ich jeszcze nie wysłano
        ProcessTxBuffer();
	}

    // Warunek sprawdzający czy aby na pewno pobrano wszystkie odpowiednie pomiary.
    if (archive_state.count_total == 0) {
         // Warunek sprawdzajćy czy jest włączone pobieranie pomiarów cykliczne by włączyć pobieranie pomiarów
         if (logging_enabled) {
             HAL_TIM_Base_Start_IT(&htim10);
         }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &uart_received_char, 1);
  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
   * Architektura pętli:
   * 1. MAszyna stanów dla czujnika AHT15_Process(); logika obsługi czujnika jest wykonywana w karzdej iteracji ale
   * dzięki maszynie stanów zawartej w tej funkcji to podana logika wywoła się tylko wtedy kiedy zostanie zmieniony
   * stan czujnika (np. na skutek komendy od użytkownika lub upływu czasu).
   *
   * 2. Przetwarzanie odebrachych danych
   * Przetwarzanie odebranych danych ze UART, pętla aktywnie opróżnia bufor odbiorczy przetwarzając karzdy bajt z
   * osobna.
   *
   * 3. Maszyna stanów do przetwarzania odebranych danych według naszego protokołu komunikacyjnego
   * Jest po to by złożyć kompletną ramkę z pojedyńczych bajtów.
   *
   * 4. Obsługa błędów i przekroczenia czasów
   * Podstawową obsługą błędów w pętli jest to że obsługuje podane flagi (rx_overflow_flag, frame_timeout_flag).
   * Dzięki tym flagą mój kod jest odporny na błąd przepełnienia bufora odbiorczego a także na błąd polegający na
   * wiecznej bezczynności z powodu oczekiwania na kompletną ramkę anych.
   *
   * 5. Zasada działania
   * 	1. Uruchom maszynę stanów czujnika (wykona akcję, jeśli został zmieniony stan czujnika).
   * 	2. Sprawdź, czy wystąpiło przepełnienie bufora odbiorczego. Jeśli tak, zresetuj wszystko, by można by było
   * 	odebrać kolejne dane.
   * 	3. Przetwarzanie wszystkich bajtów z bufora odbiorczego:
   * 		a. Szukaj znaku początku ramki (':').
   * 		b. Po znalezieniu ':', przejdź w stan odczytu i uruchom timer niecierpliwości.
   * 		c. Zbieraj znaki aż do znaku końca (';') i zapisuj je w zmiennej frame_content_buffer.
   * 		d. Po odebraniu ';', zatrzymaj timer i rozpocznij walidację całej ramki
   * 		etapami walidacji jest:
   * 			I. Sprawdzenie czy nie wdarł się szum(losowe dane) w naszym kodowaniu base64, poprzez sprawdzenie czy
   * 			wszystkie znaki należą do zakresu znaków z base64.
   * 			II. Walidacja znaków w CRC. Sprawdzanie czy znaki zawarte w kodowaniu crc są poprawne. Czyli to jest
   * 			sprawdzenie czy zakres znaków w crc16 - modbus jest poprawny.
   * 			III. Dekodowanie znaków z base64 i przy tym sprawdzenie czy znaki nie powodują żadnego błędu podczas
   * 			dekodowania.
   * 			IV. Sprawdzenie czy znak nadawcy i odbiorcy jest poprawny.
   * 			V. Sprawdzanie czy wysłana komenda jest tą która może zostać obsłużona, czyli należy do obsługiwanych
   * 			koment.
   * 			VI. Sprawdzenie czy nie pomylono się z odpowiednimi znakami (czyli znaki nie przedstawiają cyfr)
   * 			z polu określającym długość pola dane.
   * 			VII. Srawdzanie czy długość pola długość odpowiada napewno polu dane
   * 			VIII sprawdzenie czy crc 16 - modbus pasuje do zdekodowanych danych z ramki.
   * 	4. Sprawdzenie czy wystąpił stan niecierpliwości to resetuje stan by odbierać nowe dane.
   *
   */
  while (1)
  {
	  /*
	   * Konieczne jest sprawdzanie stanu czujnika na samym początku ponieważ dzięki temu czujnik może wykonywać bez
	   * większych opóźnień ani bez czekania na jakieś dane.
	   */

	  AHT15_Process();
	  ProcessArchiveTransmission();

	  // Reakcja na przepełnienie bufora RX
	  if (rx_overflow_flag) {
		  rx_overflow_flag = 0; // Wyzeruj flagę
		  ClearAndResetState(); // Zresetowanie stanu, aby móc odbierać nowe, poprawne ramki
		  }

	  // Zmienna do której będą zapisywane kolejne znaki zbufora kołowego która będzie zapisywana do naszego bufora
	  // liniwego przechowującego aktualnie przerabianą ramkę. Jest to konieczne by to był bufor liniowy ponieważ
	  // dzięki temu mamy uporządkowaną kolejność w indeksach. Na przykład dzięki temu założeniu możemy być pewni że
	  // w kratce zero bufora frame_content_buffer będzie nadawca a potem odbiorca itd.
	  uint8_t ch;

	  /*
	   * To jest główna pętla przetwarzająca dane wejściowe.
	   */
	  while(CircularBufferGetChar(&rx_circular_buffer, &ch) == 0) {

	  		// Ignorujemy wszystkie znaki przed ':'
		  /*
		   * Podany warunek za za cel zresetowanie odbieranego indeksu w ramce do odbierania naszych danych.
		   * W tym warunku jest ustawiany timer by funkcja nie oczekiwała w nieskończoność na nowe dane.
		   * Ustawia również stan odpowiedzialny za odbieranie zawartości.
		   */
	  		  if(ch == ':') {
	  			  frame_content_index = 0;

	  			  // Ustawianie flagi na stan do odczytywania.
                current_state = STATE_READ_FRAME_CONTENT;

                // Uruchamianie timera.
                // Urzuchamienie timera ma zapobiec przypadkowi kiedy przyjdzie tylko jeden znak począdku a
                // reszta może nie przyjść.
                // To ma zapobiec oczekiwaniu na wartości które i tak nie przyjdą.
                frame_timeout_flag = 0;
                __HAL_TIM_SET_COUNTER(&htim11, 0); // Wyzeruj licznik
                HAL_TIM_Base_Start_IT(&htim11);     // Start timera w trybie przerwań

                continue;
	  		  }

		  // Przełącznik do przełączania się między flagami
		  switch(current_state) {
		  	  case STATE_WAIT_START:
		  		  break;

		  	  case STATE_READ_FRAME_CONTENT:
		  		  if(ch == ';') {
		  			  HAL_TIM_Base_Stop_IT(&htim11); // zatrzymaj i wyzeruj timer, bo ramka przyszła
		  			  frame_timeout_flag = 0;

		  	      	  // Sprawdzenie minimalnej wielkości ramki (zakodowany minimalny nagłówek + CRC)
		  			  if (frame_content_index < (5 + CRC_OF_SIZE)) {
		  			      // Ramka jest za krótka, aby zawierać poprawny nagłówek i CRC. Błąd.
		  			      ClearAndResetState();

		  	            // Wysyłanie do pc informacji z mikrokontrolera.
		  	            char header1[] = "Odpowiedz: Podany warunek nie spełnia minimalnej wielkości ramki";

		  	            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

		  	            // Uruchamianie wysyłania ramki.
		  	            ProcessTxBuffer();

		  			      break;
		  			  }

		  			  // Dzielimy odebraną zawartość na ZAKODOWANY BLOK i SUMĘ KONTROLNĄ
		  			  int encoded_block_len = frame_content_index - CRC_OF_SIZE;

		  			  // Zapisywanie pierwszego elementu zakodowanego bloku do zmiennej.
		  			  // Tworzymy nowy wskaźnik dla pierwszego bloku dla lepszej organizacji.
		  			  uint8_t* encoded_block = &frame_content_buffer[0];

		  			  // Zapisywanie elementu pierwszego elementu zaczynającego sume kontrolną
		  			  uint8_t* received_crc_ascii = &frame_content_buffer[encoded_block_len];

		  			  // Zmienna stworzona po to by nie powielać wywoływania funkcji ClearAndResetState
		  			  // a także wiążących się z tym także warunków dodatkowego wyjąścia z pętli. Dodatkowo jeszcze
		  			  // służy do tego by nie wykonywać operacji na i tak już błędnych ramkach.
		  			  // 1 - Oznacza true, 0 oznacza false
		  			  int valid_chars = 1;

	  				  // Sprawdzanie czy zakodowana długość zgadza się z odpowiadającą tej długości.
	  				  if (encoded_block_len % 4 != 0) {
	  					  valid_chars = 0; // Błąd: nieprawidłowa długość bloku Base64

	  		            // Wysyłanie do pc informacji z mikrokontrolera.
	  		            char header1[] = "Odpowiedz: Zakodowana wartość nie jest wielokrotnością 4.";

	  		            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

	  		            // Uruchamianie wysyłania ramki.
	  		            ProcessTxBuffer();
	  				  }

		  			  // Warunek obsługujący wyjście z pętli jeśli wystąpił błąd
		  			  if(valid_chars == 0) {
		  				  ClearAndResetState();
		  				  break;
		  			  }

		  			  // Walidacja znaków w bloku Base64.
		  			  // Jest to warunek sprawdzający czy w kodowaniu base64 nie znalazły się jakieś zniekształcone
		  			  // które nie pasują do kodowania Base64.
		  			  for (int i = 0; i < encoded_block_len; i++) {
		  				  if (my_isalnum(encoded_block[i]) == 0 &&
		  						  encoded_block[i] != '+' &&
								  encoded_block[i] != '/' &&
								  encoded_block[i] != '=') {
		  					  // Nieprawidłowy znak w danych Base64

		  		            // Wysyłanie do pc informacji z mikrokontrolera.
		  		            char header1[] = "Odpowiedz: Źle zakodowano dane w base64.";

		  		            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

		  		            // Uruchamianie wysyłania ramki.
		  		            ProcessTxBuffer();


		  					  valid_chars = 0;

		  					  break;
		  					  }
		  			  }

		  			  // Warunek obsługujący wyjście z pętli jeśli wystąpił błąd
		  			  if(valid_chars == 0) {
		  				  ClearAndResetState();
		  				  break;
		  			  }

		  			  if(valid_chars == 1){
			  			  // Walidacja znaków w CRC. Sprawdzanie czy znaki zawarte w kodowaniu crc są poprawne.
			  			  for (int i = 0; i < CRC_OF_SIZE; i++) {
			  				  if (my_isxdigit(received_crc_ascii[i]) == 0) {
			  					  // Nieprawidłowy znak w CRC (nie jest to cyfra z zakresu hex)

			  					  valid_chars = 0;

			  		            // Wysyłanie do pc informacji z mikrokontrolera.
			  		            char header1[] = "Odpowiedz: Nie poprawne znaki w CRC";

			  		            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

			  		            // Uruchamianie wysyłania ramki.
			  		            ProcessTxBuffer();


								  break;
			  				  }
			  			  }
		  			  }

		  			  int decoded_total_len = 0;

		  			  // Warunek który się włącza jeśli dekodowanie nie przebiegło poprawnie.
		  			  if(valid_chars == 1){
		  				  decoded_total_len = Base64Decode(encoded_block, encoded_block_len);
				  			if (decoded_total_len <= 0) {
				  				// Błąd podczas dekodowania Base64
				  				valid_chars = 0;

				  	            // Wysyłanie do pc informacji z mikrokontrolera.
				  	            char header1[] = "Odpowiedz: Nie można poprawnie zdekodować danych z base64";

				  	            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

				  	            // Uruchamianie wysyłania ramki.
				  	            ProcessTxBuffer();

				  			}
		  			  }

		  			  // Warunek obsługujący wyjście z pętli jeśli wystąpił błąd
		  			  if(valid_chars == 0) {
		  				  ClearAndResetState();
		  				  break;
		  			  }

                    // bufor na zdekodowane dane wynosi SIZE_OF_DANE + 5 ponieważ ta 5 reprezentuje nadawcę,
                    // Odbiorcę, komendę i dwa znaki dla długości pola dane. Poniższa zmienna posłuży nam do
		  			// obliczenia crc16. Jest to ninieczne bo moja funkcja jest przystosowana do pracy z wysłaną
		  			// tablicą danych a nie z buforem kołowym.
                    uint8_t decoded_data_linear_buffer[4 + SIZE_OF_DANE];

                    // Inicjujemy wartości potrzebne do analizowania naszgo nagłówka i dla lepszej orgranizacji w kodzie.
                    uint8_t sender_2 = 0;
                    CircularBufferGetChar(&decoded_data_circ_buff, &sender_2);
                    decoded_data_linear_buffer[0] = sender_2;
                    sender = sender_2;

					uint8_t receiver = 0;
					CircularBufferGetChar(&decoded_data_circ_buff, &receiver);
					decoded_data_linear_buffer[1] = receiver;

                    uint8_t command = 0;
                    CircularBufferGetChar(&decoded_data_circ_buff, &command);
                    decoded_data_linear_buffer[2] = command;

                    uint8_t data_length = 0;
                    CircularBufferGetChar(&decoded_data_circ_buff, &data_length);
                    decoded_data_linear_buffer[3] = data_length;

                    for (int var = 4; var < decoded_total_len; ++var) {
						if(CircularBufferGetChar(&decoded_data_circ_buff, &ch) == 0){
							decoded_data_linear_buffer[var] = ch;
						}
					}

					// Warunek sprawdzający czy czy komenda należy do zbioru komend i czy znaki długości
					// są aby napewno cyframi.
					if(valid_chars == 1){
	                    if ((CheckCommandChar(command) == 0)) {

	                    	// Błąd - komenda lub długość zawiera nie poprawne znaki.
	                    	valid_chars = 0;

	                        // Wysyłanie do pc informacji z mikrokontrolera.
	                        char header1[] = "Odpowiedz: Nie poprawna komenda";

	                        QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

	                        // Uruchamianie wysyłania ramki.
	                        ProcessTxBuffer();


	                        break;
	                    }
					}

                    // Długość ładunku to długość całości minus 5 bajtów nagłówka.
                    // Podany warunek jest po to by sprawdzić czy w polu dane znajduje się długość
                    // danych zdefiniowana w polu długość.
                    if(valid_chars == 1){
                        if (data_length != (decoded_total_len - 4)) {
                            // Błąd - zadeklarowana długość nie zgadza się z rzeczywistą długością pola dane.
                        	valid_chars = 0;

                            // Wysyłanie do pc informacji z mikrokontrolera.
                            char header1[] = "Odpowiedz: Pole długość danych nie zgadza się z polem dane.";

                            QueueFrameForSending(header1, (sizeof(header1)/sizeof(header1[0]))); // Rozpocznij wysyłanie nagłówka

                            // Uruchamianie wysyłania ramki.
                            ProcessTxBuffer();


                            break;
                        }
                    }

		  			  // Warunek obsługujący wyjście z pętli jeśli wystąpił błąd
		  			  if(valid_chars == 0) {
		  				  ClearAndResetState();
		  				  break;
		  			  }

                    // Zamiana małych znaków crc zawartych w ramce na duże znaki.
                    for (int var = 0; var < sizeof(received_crc_ascii); ++var) {
                    	received_crc_ascii[var] = my_toupper(received_crc_ascii[var]);
					}

		  			  // Obliczamy CRC na odebranym zakodowanym bloku.
		  			  uint16_t computed_crc = ComputeCRC16(decoded_data_linear_buffer, decoded_total_len);
		  			  uint8_t computed_crc_ascii[CRC_OF_SIZE];
		  			  HexToAscii(computed_crc, computed_crc_ascii);

		  			  // Porównujemy CRC
		  			  if (memcmp(received_crc_ascii, computed_crc_ascii, CRC_OF_SIZE) == 0) {

	                          // CRC poprawne, dane już zwalidowane - można je przetworzyć
	                          // Ramka jest w 100% poprawna w tym miejscu.

							// Przywracamy dane do bufora kołowego, aby reszta logiki mogła działać.
							for(int i = 4; i < decoded_total_len; i++) {
								CircularBufferPutChar(&decoded_data_circ_buff, decoded_data_linear_buffer[i]);
							}

	                          // Wywołujemy funkcję symulującą odpowiedź czujnika
	                          SimulateSensorResponse(command, &decoded_data_circ_buff);

	                      } else {
	                    	    // Błąd CRC!
	                    	    QueueFrameForSending("Blad: Nieprawidlowa suma kontrolna CRC.", sizeof("Blad: Nieprawidlowa suma kontrolna CRC."));

	                            // Uruchamianie wysyłania ramki.
	                            ProcessTxBuffer();
	                      }

	                      // Niezależnie od wyniku, resetujemy stan i czekamy na nową ramkę.
	                      ClearAndResetState();

	                  }else {
	                	  /*
	                	   * Cel działania podanego kodu:
	                	   * 	1.Każdy znak między ':' a ';' jest dodawany do bufora liniowego `frame_content_buffer`.
	                	   * 	2. Sprawdzenie `frame_content_index` chroni przed przepełnieniem tego bufora,
	                	   * 	co mogłoby uszkodzić inne zmienne w pamięci.
	                	   * 	3. Reset licznika timera (`__HAL_TIM_SET_COUNTER`) po każdym znaku
	                	   * 	działa jak odświeżenie "timeru" - przerwanie wystąpi tylko wtedy,
	                	   * 	gdy przerwa między kolejnymi znakami będzie zbyt długa.
	                	   */
	                      if ((frame_content_index + 1) < FRAME_CONTENT_MAX_SIZE) {
	                          frame_content_buffer[frame_content_index++] = ch;
	                          __HAL_TIM_SET_COUNTER(&htim11, 0);
	                      } else {
	                          // Przepełnienie bufora - ramka jest za długa, reset.
	                          ClearAndResetState();
	                      }
	                  }
	                  break;
	          }
	      }

	  // Warunek pilnujący żeby zbyt dugi czas oczekiwania nie zawieszał nam porgramu.
	  if (frame_timeout_flag) {
		  HAL_TIM_Base_Stop_IT(&htim11);
	      frame_timeout_flag = 0;
	      if (current_state == STATE_READ_FRAME_CONTENT) {
	          // Timeout! Ramka nie została zakończona na czas.
	          ClearAndResetState();
	      }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 41999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 2603;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 64934;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart2){
        // Warunek sprawdzający czy udało się zapisać do bufora kołowegi i ustawienie odpowiedniej flagi,
		// Dającej znać rze dane są tracone przez przepełnienie bufora.
        if (CircularBufferPutChar(&rx_circular_buffer, uart_received_char) != 0) {
            // Jeśli nie, ustaw flagę przepełnienia. Dane są tracone.
            rx_overflow_flag = 1;
        }
        HAL_UART_Receive_IT(&huart2, &uart_received_char, 1);
	}
}

// Funkcja wywołująca się gdy znak został poprawnie przesłąny. Ta funkcja przesyłą wszystkie dane z bufora kołowego
// Przeznaczonego tylko dla jednej ramki. A potem jak wyśle całą ramkę to wywołuje funkcję BuildAndStartFrame która
// tworzy kolejną nową ramkę do wysłania. I tak do momętu aż wszystkie dane z głównego bufora kołowego transmisyjengo
// zostanie wysłana.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        uint8_t data_to_send;
        // Pobieraj dane z GŁÓWNEGO bufora TX
        if (CircularBufferGetChar(&tx_circular_buffer, &data_to_send) == 0) {
            // Jeśli są dane, kontynuuj wysyłanie
            HAL_UART_Transmit_IT(&huart2, &data_to_send, 1);
        } else {
            // Jeśli bufor jest pusty, zakończ transmisję i ustaw flagę że nie są wysyłanie żadne wartości
            tx_in_progress = 0;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// Warunek zliczający wystąpienie ilości milisekund
	if(TIM10 == htim->Instance){
		if(TIM10Counter < INT64_MAX){
			TIM10Counter++;
		}

		if(TIM10Counter >= logging_interval_ms && logging_interval_ms != 0){
			TIM10Counter = 0;
			aht15_current_state = AHT15_STATE_MEASURE_SEND_CMD;
		}
	}

	// Sprawdź, czy przerwanie pochodzi od TIM11 (timer timeoutu ramki UART)
	else if (htim->Instance == TIM11)
	{
		// Ustaw flagę timeoutu, która zostanie obsłużona w pętli głównej
		frame_timeout_flag = 1;
		// Zatrzymaj timer, aby nie generował kolejnych przerwań bez potrzeby
		HAL_TIM_Base_Stop_IT(&htim11);
	}

	// Sprawdź, czy przerwanie pochodzi od TIM3 (timer oczekiwania na wykonanie pomiaru)
	else if (htim->Instance == TIM3)
	{
		aht15_current_state = AHT15_STATE_MEASURE_READ_DATA;

		// Zatrzymaj timer, aby nie generował kolejnych przerwań bez potrzeby
		HAL_TIM_Base_Stop_IT(&htim3);
	}
}

// -------------------------W dokumentacji------------------góra

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_busy_flag = 0;
        i2c_error_flag = 0;

        if (logging_enabled || is_manual_measurement) {
             __HAL_TIM_SET_COUNTER(&htim3, 0);
             HAL_TIM_Base_Start_IT(&htim3);
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_busy_flag = 0;
        i2c_error_flag = 0;
    }
}

// Callback wywoływany w przypadku błędu I2C.
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
    	i2c_error_code = hi2c->ErrorCode; // Zapisywanie konkretnego stanu błędu.
        i2c_error_flag = 1; // Ustawiamy flagę błędu
        i2c_busy_flag = 0;  // Zwalniamy flagę, aby maszyna stanów mogła obsłużyć błąd
    }
}

// -------------------------W dokumentacji------------------duł

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
