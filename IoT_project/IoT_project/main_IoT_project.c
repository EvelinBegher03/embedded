#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/grlib/grlib.h>
#include "HAL_OPT3001.h"
#include "HAL_TMP006.h"
#include "HAL_I2C.h"
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define CENTER     8192
#define THRESHOLD  2000
#define PIN_LENGTH 1
#define MAX_LENGTH 6
#define LIGHT_THRESHOLD 10
#define BUZZER_PORT GPIO_PORT_P2
#define BUZZER_PIN  GPIO_PIN7

#define S2_PORT GPIO_PORT_P3
#define S2_PIN  GPIO_PIN5
#define JOY_SW_PORT GPIO_PORT_P4
#define JOY_SW_PIN  GPIO_PIN1   // J1.5 -> P4.1 (joystick press) (penso per rilevare se qualcuno scuote)

//memoria per il salvataggio dello stato della porta nella flash
#define DOOR_NVM_ADDR   ((uint32_t)0x0003F000)  // ultima pagina 4KB (256KB flash)
#define DOOR_NVM_MAGIC  ((uint32_t)0xD00A5A7E)

typedef enum
{
    DIRECTION_NEUTRAL,
    DIRECTION_UP,
    DIRECTION_DOWN,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} JoystickDirection;

//stato della porta
typedef enum
{
    DOOR_CLOSED = 0, DOOR_OPEN = 1
} DoorState;
static DoorState doorState = DOOR_CLOSED;


typedef struct {
    uint32_t magic;
    uint32_t state;   // 0 = closed, 1 = open
    uint32_t inv;     // ~state (check semplice)
} DoorNvmRecord;

//timer
static volatile uint32_t g_ms = 0;
static void initSysTickMs(void);
void SysTick_Handler(void);
static inline int32_t timePassed(uint32_t now, uint32_t deadline);
static uint32_t welcomeUntilMs = 0;
static uint8_t p51Prev = 1;
static uint32_t p51DebounceUntil = 0;
static uint8_t p51PressedEvent = 0;

typedef enum
{
    BUZ_IDLE = 0, BUZ_ERR, BUZ_SHAKE
} BuzMode;

static volatile BuzMode buzMode = BUZ_IDLE;
static uint32_t buzNextToggleMs = 0;
static uint32_t buzEndPhaseMs = 0;
static uint8_t buzPhase = 0;

//task timer
static void buzzerTask(void);
static void p51Task(void);
static uint8_t p51ConsumePressedEvent(void);

typedef enum
{
    SERVO_IDLE = 0, SERVO_OPEN_RUN, SERVO_OPEN_BRAKE, SERVO_CLOSE_RUN
} ServoPhase;
static volatile ServoPhase servoPhase = SERVO_IDLE;
static uint32_t servoDeadlineMs = 0;

int backGroundColor = GRAPHICS_COLOR_BLACK;
int textColor = GRAPHICS_COLOR_WHITE;

typedef enum
{
    STATE_IDLE, STATE_FEEDBACK, STATE_BLOCKED, STATE_WELCOME
} AppState;

bool prevIsDay = true;
// PIN corretto
static const char correctPIN[PIN_LENGTH + 1] = "1";

Graphics_Context g_sContext;
int selectedRow = 0;
int selectedCol = 0;

uint16_t xValue, yValue;
JoystickDirection prevDirection = DIRECTION_NEUTRAL;

char enteredPIN[MAX_LENGTH + 1];
int pinIndex = 0;
int attemptCount = 0;

AppState currentState = STATE_IDLE;
int lastFeedbackShown = 0; // 0=nessuno, 1=CORRETTO, 2=ERRATO, 3=LUNGHEZZA ERRATA, 4=TROPPI CARATTERI

// Coordinate del tastierino
int keyX[3] = { 32, 64, 96 };
int keyY[4] = { 40, 65, 90, 115 };

char keys[4][3] = { { '1', '2', '3' }, { '4', '5', '6' }, { '7', '8', '9' }, {
        'X', '0', 'E' } };

// Font disponibili
extern const tFont g_sFontFixed6x8;   // Font piccolo
extern const tFont g_sFontCmss20b;    // Font grande

void brightnessSensor(uint16_t lightValue);
void initSystem(void);
static void initBuzzer(void);
void initDisplay(void);
void initADC(void);
void initButton(void);
JoystickDirection getJoystickDirection(uint16_t x, uint16_t y);
void readJoystick(void);
void drawInitialScreen(void);
void drawKeypad(void);
void drawKeyAt(int row, int col, bool selected);
void updateSelectedKey(int oldRow, int oldCol, int newRow, int newCol);
void showPinLabel(void);
void showPin(void);
void showFeedback(const char *msg);
void clearFeedback(void);
static void beepError(void);
static void beepShake(void);
void handleSelectedChar(char c);
void checkPIN(void);
static void onPinCorrect(void);
static void checkTamper(uint16_t ax, uint16_t ay, uint16_t az);
void resetPIN(void);
void returnToInitial(void);
bool inputAvailable(void);
void processInputDuringFeedback(void);

//apertura e chiusura porta
static void servoTask(void);
static void s2Task(void);
static void servoOpenFull(void);
static void servoOpenNudge(void);
static void servoCloseFull(void);
static void loadDoorStateFromFlash(void);
static void saveDoorStateToFlash(DoorState st);

//controllo temperatura
static void showWelcomeTemp(void);

int main(void)
{
    initSystem();
    loadDoorStateFromFlash();
    initBuzzer();
    initADC();
    initDisplay();
    initButton();

    // Configura il clock a 3 MHz
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //init timer
    initSysTickMs();

    // Pin P2.4 per PWM
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
    GPIO_PIN4,
                                                GPIO_PRIMARY_MODULE_FUNCTION);

    // PWM a 50 Hz
    Timer_A_PWMConfig pwmConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,
                                    TIMER_A_CLOCKSOURCE_DIVIDER_3, 20000, // 20 ms
                                    TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                    TIMER_A_OUTPUTMODE_RESET_SET,
                                    1500    // 1.5 ms => fermo
            };

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    drawInitialScreen(); // Disegna tutto all'inizio

    while (1)

    {
        buzzerTask();
        p51Task();
        //chiusura porta
        s2Task();
        servoTask();

        ADC14_toggleConversionTrigger();
        while (ADC14_isBusy())
            ;

        xValue = ADC14_getResult(ADC_MEM0);
        yValue = ADC14_getResult(ADC_MEM1);
        uint16_t accX = ADC14_getResult(ADC_MEM2);
        uint16_t accY = ADC14_getResult(ADC_MEM3);
        uint16_t accZ = ADC14_getResult(ADC_MEM4);
        checkTamper(accX, accY, accZ);

        uint16_t lightValue = OPT3001_getLux();
        ;

        // Ora chiama brightnessSensor con il valore letto
        brightnessSensor(lightValue);

        if (currentState == STATE_IDLE)
        {

            readJoystick();
            JoystickDirection currentDir = getJoystickDirection(xValue, yValue);
            if (currentDir != DIRECTION_NEUTRAL
                    && prevDirection == DIRECTION_NEUTRAL)
            {

                int oldRow = selectedRow;
                int oldCol = selectedCol;

                switch (currentDir)
                {
                case DIRECTION_UP:
                    if (selectedRow > 0)
                        selectedRow--;
                    break;
                case DIRECTION_DOWN:
                    if (selectedRow < 3)
                        selectedRow++;
                    break;
                case DIRECTION_LEFT:
                    if (selectedCol > 0)
                        selectedCol--;
                    break;
                case DIRECTION_RIGHT:
                    if (selectedCol < 2)
                        selectedCol++;
                    break;
                default:
                    break;
                }

                updateSelectedKey(oldRow, oldCol, selectedRow, selectedCol);
            }
            prevDirection = currentDir;

            if (p51ConsumePressedEvent())
            {
                char selectedChar = keys[selectedRow][selectedCol];
                handleSelectedChar(selectedChar);
            }

        }
        else if (currentState == STATE_FEEDBACK)
        {

            // aspetta input
            processInputDuringFeedback();

        }
        else if (currentState == STATE_BLOCKED)
        {

            // bloccato
        }

        if (currentState == STATE_WELCOME)
        {
            if ((int32_t) (g_ms - welcomeUntilMs) >= 0)
            {
                returnToInitial();
                currentState = STATE_IDLE;
            }
            // mentre sei in welcome, non processare tastierino
            // ma lascia andare servoTask() e s2Task()
        }

    }
}
//fa l init del sistema in poche parole lo toglie dalla low power mode che non ho ancora capito che cazzo è
void initSystem(void)
{
    WDT_A_holdTimer();
}

static void initBuzzer(void)
{
    GPIO_setAsOutputPin(BUZZER_PORT, BUZZER_PIN);
    GPIO_setOutputLowOnPin(BUZZER_PORT, BUZZER_PIN);
}

//inizializza il display dai driver e setta orientamento e altre cazzate
void initDisplay(void)
{
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                         &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, textColor);
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8); // font piccolo
    Graphics_clearDisplay(&g_sContext);
}
//sto schifo è l inizializzazione del jopystick
void initADC(void)
{

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0,
    GPIO_TERTIARY_MODULE_FUNCTION); // X
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4,
    GPIO_TERTIARY_MODULE_FUNCTION); // Y

    // Accelerometro (ADC A14, A13, A11)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1,
    GPIO_TERTIARY_MODULE_FUNCTION); // A14
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0,
    GPIO_TERTIARY_MODULE_FUNCTION); // A13
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN2,
    GPIO_TERTIARY_MODULE_FUNCTION); // A11

    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM4, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A15,
                                    false);

    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A9,
                                    false);
    ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A14,
                                    false); // accX
    ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A13,
                                    false); // accY
    ADC14_configureConversionMemory(ADC_MEM4, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A11,
                                    false); // accZ

    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();

    // Configura pin sensore luce
    Init_I2C_GPIO();
    I2C_init();

    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();

    //sensore di temperatura
    TMP006_init();
}

//init del buttone
void initButton(void)
{
    P5->DIR &= ~BIT1;
    P5->REN |= BIT1;
    P5->OUT |= BIT1;
}
//questa funzione serve a leggere dai due pin la posizione del joystic
void readJoystick(void)
{
    ADC14_toggleConversionTrigger();
    while (ADC14_isBusy())
        ;
    xValue = ADC14_getResult(ADC_MEM0);
    yValue = ADC14_getResult(ADC_MEM1);
}
//init timer
static void initSysTickMs(void)
{
    // con DCO=3MHz, SysTick a 1ms => 3000 tick
    SysTick->LOAD = (3000 - 1);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
    SysTick_CTRL_TICKINT_Msk |
    SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    g_ms++;
}

static inline int32_t timePassed(uint32_t now, uint32_t deadline)
{
    return (int32_t) (now - deadline) >= 0;
}

// gestione dei suoni di errore
static void buzzerTask(void)
{
    if (buzMode == BUZ_IDLE)
        return;

    // toggle a cadenza (regola qui il tono)
    if (timePassed(g_ms, buzNextToggleMs))
    {
        GPIO_toggleOutputOnPin(BUZZER_PORT, BUZZER_PIN);

        // frequenze: 1ms -> 500Hz toggle, 0.5ms non possibile con ms tick, quindi scegli 1-2ms
        buzNextToggleMs = g_ms + 1;
    }

    // fine fase?
    if (!timePassed(g_ms, buzEndPhaseMs))
        return;

    // spegni pin tra fasi
    GPIO_setOutputLowOnPin(BUZZER_PORT, BUZZER_PIN);

    if (buzMode == BUZ_ERR)
    {
        // Sequenza: beep corto, pausa, beep corto
        if (buzPhase == 0)
        {
            buzPhase = 1;
            buzEndPhaseMs = g_ms + 200;
            return;
        } // pausa 200ms
        if (buzPhase == 1)
        {
            buzPhase = 2;
            buzEndPhaseMs = g_ms + 120;
            buzNextToggleMs = g_ms;
            return;
        } // secondo beep
        buzMode = BUZ_IDLE;
        return;
    }

    if (buzMode == BUZ_SHAKE)
    {
        // Sequenza semplice: lungo, pausa, corto, ripeti 2 volte
        // Per restare breve: 2 cicli
        static uint8_t reps = 0;
        if (buzPhase == 0)
        {
            buzPhase = 1;
            buzEndPhaseMs = g_ms + 200;
            return;
        }      // pausa
        if (buzPhase == 1)
        {
            buzPhase = 2;
            buzEndPhaseMs = g_ms + 150;
            buzNextToggleMs = g_ms;
            return;
        } // beep corto
        reps++;
        if (reps < 2)
        {
            buzPhase = 0;
            buzEndPhaseMs = g_ms + 500;
            buzNextToggleMs = g_ms;
            return;
        }
        reps = 0;
        buzMode = BUZ_IDLE;
        return;
    }
}

static void p51Task(void)
{
    uint8_t now = ((P5->IN & BIT1 ) ? 1 : 0); // 1 = non premuto (pull-up), 0 = premuto

    if (!timePassed(g_ms, p51DebounceUntil))
    {
        p51Prev = now;
        return;
    }

    // fronte 1 -> 0 = pressione
    if (p51Prev == 1 && now == 0)
    {
        p51PressedEvent = 1;
        p51DebounceUntil = g_ms + 200; // 200ms (equivale a 2*100ms che avevi)
    }

    p51Prev = now;
}

static uint8_t p51ConsumePressedEvent(void)
{
    if (p51PressedEvent)
    {
        p51PressedEvent = 0;
        return 1;
    }
    return 0;
}

// praticamente questa toglie la zona morta e legge semplicemente da che parte è stato mosso, perchè prima proprio leggeva le coordinate come fosse un mouse tipo
JoystickDirection getJoystickDirection(uint16_t x, uint16_t y)
{
    if (y > CENTER + THRESHOLD)
        return DIRECTION_UP;
    if (y < CENTER - THRESHOLD)
        return DIRECTION_DOWN;
    if (x > CENTER + THRESHOLD)
        return DIRECTION_RIGHT;
    if (x < CENTER - THRESHOLD)
        return DIRECTION_LEFT;
    return DIRECTION_NEUTRAL;
}
//disegna schermo iniziale
void drawInitialScreen(void)
{

    Graphics_clearDisplay(&g_sContext);

    clearFeedback();

    showPinLabel();
    resetPIN();

    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    drawKeypad();
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    drawKeyAt(selectedRow, selectedCol, true);
}
// Disegna il tastierino
void drawKeypad(void)
{
    int r, c;
    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 3; c++)
        {
            drawKeyAt(r, c, false);
        }
    }
}
//colora di rosso il numero in cui ci troviamo
void drawKeyAt(int row, int col, bool selected)
{
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    if (selected)
    {
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    }
    else
    {
        Graphics_setForegroundColor(&g_sContext, textColor);
    }
    char keyStr[2] = { keys[row][col], '\0' };
    Graphics_drawStringCentered(&g_sContext, (int8_t*) keyStr,
    AUTO_STRING_LENGTH,
                                keyX[col], keyY[row],
                                OPAQUE_TEXT);
    Graphics_setForegroundColor(&g_sContext, textColor);
}
//aggiorna il numero in cui ci troviamo
void updateSelectedKey(int oldRow, int oldCol, int newRow, int newCol)
{
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    drawKeyAt(oldRow, oldCol, false);
    drawKeyAt(newRow, newCol, true);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
}
//Mostra l area del pin prima cancellandola e poi riscrivendo (sarà un bel casino se vogliamo usare anche il sensore di lum
void showPinLabel(void)
{
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, backGroundColor);
    Graphics_fillRectangle(&g_sContext,
                           &(Graphics_Rectangle ) { 0, 0, 127, 25 });
    Graphics_setForegroundColor(&g_sContext, textColor);
    Graphics_drawString(&g_sContext, (int8_t*) "PIN:", AUTO_STRING_LENGTH, 20,
                        10, OPAQUE_TEXT);
}
// pezzo dove viene effettivamente inserito il pin
void showPin(void)
{
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, backGroundColor);
    Graphics_fillRectangle(&g_sContext,
                           &(Graphics_Rectangle ) { 60, 0, 127, 25 });
    Graphics_setForegroundColor(&g_sContext, textColor);

    if (pinIndex > 0)
    {
        char buf[20];
        snprintf(buf, sizeof(buf), "%.*s", pinIndex, enteredPIN);
        Graphics_drawString(&g_sContext, (int8_t*) buf, AUTO_STRING_LENGTH, 60,
                            10, OPAQUE_TEXT);
    }
}
// mostra area di dove viene mostrato il feedback
void showFeedback(const char *msg)
{
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, backGroundColor);
    Graphics_fillRectangle(&g_sContext,
                           &(Graphics_Rectangle ) { 0, 0, 127, 25 });
    Graphics_setForegroundColor(&g_sContext, textColor);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) msg, AUTO_STRING_LENGTH,
                                64, 10, OPAQUE_TEXT);
}
//pulisce l area feedback
void clearFeedback(void)
{
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, backGroundColor);
    Graphics_fillRectangle(&g_sContext,
                           &(Graphics_Rectangle ) { 0, 0, 127, 5 });
    Graphics_setForegroundColor(&g_sContext, textColor);
}

static void beepError(void)
{
    buzMode = BUZ_ERR;
    buzPhase = 0;
    buzNextToggleMs = g_ms;          // inizia subito
    buzEndPhaseMs = g_ms + 120; // 120ms di beep (simile a 120 toggle con delay ~1k cycles)
}

static void beepShake(void)
{
    buzMode = BUZ_SHAKE;
    buzPhase = 0;
    buzNextToggleMs = g_ms;
    buzEndPhaseMs = g_ms + 300;      // beep lungo
}

//qui praticamente prende il carattere corrispondente a dove ci troviamo con il cursore e lo aggiunge al pin se è un numero da 0 a 9 mentre se è x canxella dal vet e se è e fa l inserimento
void handleSelectedChar(char c)
{
    if (currentState != STATE_IDLE)
        return;
    clearFeedback();

    if (c >= '0' && c <= '9')
    {
        if (pinIndex < MAX_LENGTH)
        {
            enteredPIN[pinIndex++] = c;
            showPin();
        }
        else
        {
            showFeedback("TROPPI CARATTERI");
            beepError();
            lastFeedbackShown = 4;
            currentState = STATE_FEEDBACK;
        }
    }
    else if (c == 'X')
    {
        if (pinIndex > 0)
        {
            pinIndex--;
            showPin();
        }
    }
    else if (c == 'E')
    {
        if (pinIndex == PIN_LENGTH)
        {
            enteredPIN[PIN_LENGTH] = '\0';
            checkPIN();
        }
        else
        {
            showFeedback("LUNGHEZZA ERRATA");
            beepError();
            lastFeedbackShown = 3;
            currentState = STATE_FEEDBACK;
        }
    }
}
// funzione che controlla se il pin è giusto
void checkPIN(void)
{
    bool correct = (strcmp(enteredPIN, correctPIN) == 0);
    if (correct)
    {
        showWelcomeTemp();
        currentState = STATE_WELCOME;
        welcomeUntilMs = g_ms + 5000;   // 5 secondi
        onPinCorrect();                 // avvia servo non bloccante
        return;
    }
    else
    {
        attemptCount++;
        if (attemptCount >= 3)
        {
            showFeedback("BLOCCATO");
            beepError();
            currentState = STATE_BLOCKED;
        }
        else
        {
            showFeedback("ERRATO");
            beepError();
            lastFeedbackShown = 2;
            currentState = STATE_FEEDBACK;
        }
    }
}

// se il pin è corretto
static void onPinCorrect(void)
{

    if (doorState == DOOR_CLOSED)
    {
        servoOpenFull();
        doorState = DOOR_OPEN;
        saveDoorStateToFlash(doorState);
    }
    else
    {
        servoOpenNudge();
    }
}

static void checkTamper(uint16_t ax, uint16_t ay, uint16_t az)
{
    if (doorState)
        return;
    static uint8_t shakeCooldown = 0;
    static uint16_t accX_prev = 0, accY_prev = 0, accZ_prev = 0;
    static uint8_t accInit = 0;
    int32_t dx, dy, dz;

    // inizializzazione (prima lettura)
    if (!accInit)
    {
        accX_prev = ax;
        accY_prev = ay;
        accZ_prev = az;
        accInit = 1;
        return;
    }

    // cooldown per evitare suono continuo
    if (shakeCooldown)
    {
        shakeCooldown--;
        accX_prev = ax;
        accY_prev = ay;
        accZ_prev = az;
        return;
    }

    // variazione rapida = manomissione
    dx = (int32_t) ax - accX_prev;
    if (dx < 0)
        dx = -dx;
    dy = (int32_t) ay - accY_prev;
    if (dy < 0)
        dy = -dy;
    dz = (int32_t) az - accZ_prev;
    if (dz < 0)
        dz = -dz;

    accX_prev = ax;
    accY_prev = ay;
    accZ_prev = az;

    // soglia: tarala una volta e basta
    // 800–1200 → apertura porta OK
    // >1500 → strattonata / colpo
    if (dx > 1500 || dy > 1500 || dz > 1500)
    {
        beepShake();        // allarme manomissione
        shakeCooldown = 100;
    }
}

// cambia lo schermo da bianco a nero e viceversa
void brightnessSensor(uint16_t lightValue)
{
    // Determina se è giorno o notte
    bool isDay = (lightValue >= LIGHT_THRESHOLD);

    // Se lo stato è cambiato rispetto al ciclo precedente, aggiorna i colori e ridisegna
    if (isDay != prevIsDay)
    {
        if (!isDay)
        {
            // notte -> schermo nero
            textColor = GRAPHICS_COLOR_WHITE;
            backGroundColor = GRAPHICS_COLOR_BLACK;
        }
        else
        {
            // giorno -> schermo bianco
            textColor = GRAPHICS_COLOR_BLACK;
            backGroundColor = GRAPHICS_COLOR_WHITE;
        }

        // Aggiorna i colori del contesto
        Graphics_setForegroundColor(&g_sContext, textColor);
        Graphics_setBackgroundColor(&g_sContext, backGroundColor);

        // Ridisegna schermo solo al cambio di condizione
        if (currentState == STATE_IDLE)
        {
            drawInitialScreen();
        }

        // Aggiorna lo stato precedente
        prevIsDay = isDay;
    }

    // Se non è cambiato nulla, non fare niente: nessun refresh dello schermo
}

// resetta il pin ogni volta che viene premuta la e
void resetPIN(void)
{
    pinIndex = 0;
    memset(enteredPIN, 0, sizeof(enteredPIN));
    showPin();
}
//resetta lo schermo come allinizio
void returnToInitial(void)
{
    drawInitialScreen();
    lastFeedbackShown = 0;
    currentState = STATE_IDLE;
}

//qui avevo problemi con la selezione dei numeri perchè era troppo sensibile e quindi ho messo che si può fare uno spostamento alla volta non tipo tenendo piegato il L3
bool inputAvailable(void)
{
    if (p51PressedEvent)
        return true; // Joystick: evento solo su transizione NEUTRAL -> direzione
    static JoystickDirection prevDirFB = DIRECTION_NEUTRAL;

    readJoystick();
    JoystickDirection currentDir = getJoystickDirection(xValue, yValue);

    // Edge: valido solo quando esco dal neutro
    if (prevDirFB == DIRECTION_NEUTRAL && currentDir != DIRECTION_NEUTRAL)
    {
        prevDirFB = currentDir;   // latch finché non torni neutro
        return true;
    }

    // reset latch quando torna neutro
    if (currentDir == DIRECTION_NEUTRAL)
    {
        prevDirFB = DIRECTION_NEUTRAL;
    }

    // Pulsante P5.1: non bloccante (evento generato da p51Task())
    if (p51PressedEvent)
        return true;

    return false;
}
//check di quale feedback stampare
void processInputDuringFeedback(void)
{
    if (currentState == STATE_FEEDBACK && inputAvailable())
    {
        if (lastFeedbackShown == 1)
        { // CORRETTO
            attemptCount = 0;
            returnToInitial();
        }
        else if (lastFeedbackShown == 2 || lastFeedbackShown == 3
                || lastFeedbackShown == 4)
        {
            if (currentState != STATE_BLOCKED)
            {
                returnToInitial();
            }
        }
        //bloccato
    }
}

//task del servo
static void servoTask(void)
{
    if (servoPhase == SERVO_IDLE)
        return;

    // scadenza raggiunta?
    if ((int32_t) (g_ms - servoDeadlineMs) < 0)
        return;

    switch (servoPhase)
    {
    case SERVO_OPEN_RUN:
        // fase brake come nel tuo codice: 1900 per 1000ms
        Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                1900);
        servoPhase = SERVO_OPEN_BRAKE;
        servoDeadlineMs = g_ms + 1000;
        break;

    case SERVO_OPEN_BRAKE:
        // torna neutro
        Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                1500);
        servoPhase = SERVO_IDLE;
        break;

    case SERVO_CLOSE_RUN:
        // fine chiusura -> neutro
        Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                1500);
        servoPhase = SERVO_IDLE;
        doorState = DOOR_CLOSED;     // chiusura completata
        saveDoorStateToFlash(doorState);
        break;

    default:
        servoPhase = SERVO_IDLE;
        break;
    }
}

static void s2Task(void)
{
    static uint8_t s2Prev = 1;
    static uint32_t s2DebounceUntil = 0;
    uint8_t s2Now = GPIO_getInputPinValue(S2_PORT, S2_PIN); // 1 = non premuto, 0 = premuto

    // debounce temporale
    if ((int32_t) (g_ms - s2DebounceUntil) < 0)
    {
        s2Prev = s2Now;
        return;
    }

    // rilevo fronte 1->0 (pressione)
    if (s2Prev == 1 && s2Now == 0)
    {
        s2DebounceUntil = g_ms + 200; // 200ms debounce

        if (doorState == DOOR_OPEN && servoPhase == SERVO_IDLE)
        {
            servoCloseFull();
            // doorState diventa CLOSED quando il servo finisce (in servoTask)
        }
    }

    s2Prev = s2Now;
}

//apertura porta totale
static void servoOpenFull(void)
{
    // avvio apertura “lunga”
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            1100);
    servoPhase = SERVO_OPEN_RUN;
    servoDeadlineMs = g_ms + 6667;   // 20,000,000 cycles @3MHz ≈ 6667ms
}

//apertura porta parziale
static void servoOpenNudge(void)
{
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            1100);
    servoPhase = SERVO_OPEN_RUN;
    servoDeadlineMs = g_ms + 3333;   // 10,000,000 cycles @3MHz ≈ 3333ms
}

//chiusura porta
static void servoCloseFull(void)
{
    // S2 premuto = LOW
    if (GPIO_getInputPinValue(S2_PORT, S2_PIN) == 0)
    {
        if (doorState == DOOR_OPEN)
        {
            Timer_A_setCompareValue(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                    1900);
            servoPhase = SERVO_CLOSE_RUN;
            servoDeadlineMs = g_ms + 3167;   // 9,500,000 cycles @3MHz ≈ 3167ms
            doorState = DOOR_CLOSED;
            saveDoorStateToFlash(doorState);
        }
    }
}

//load dalla flash dello stato della porta
static void loadDoorStateFromFlash(void)
{
    const DoorNvmRecord *rec = (const DoorNvmRecord *)DOOR_NVM_ADDR;

    if (rec->magic == DOOR_NVM_MAGIC && rec->inv == ~rec->state)
    {
        doorState = (rec->state ? DOOR_OPEN : DOOR_CLOSED);
    }
    else
    {
        doorState = DOOR_CLOSED; // default se flash vuota/sporca
    }
}

//salvataggio nella flash dello stato della porta
static void saveDoorStateToFlash(DoorState st)
{
    DoorNvmRecord rec;
    rec.magic = DOOR_NVM_MAGIC;
    rec.state = (st == DOOR_OPEN) ? 1u : 0u;
    rec.inv   = ~rec.state;

    // Erase pagina e poi scrivi (pochi byte)
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); // vedi nota sotto
    FlashCtl_eraseSector(DOOR_NVM_ADDR);
    FlashCtl_programMemory((void *)&rec, (void *)DOOR_NVM_ADDR, sizeof(rec));
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31);
}

//benvenuto con temperatura
static void showWelcomeTemp(void)
{
    // Leggi temperatura “ambient/die” dal TMP006 in °C
    int raw = TMP006_readAmbientTemperature();
    raw >>= 2;
    float tC = (float) raw * 0.03125f;

    // 1 decimale senza %f
    int t10 = (int) (tC * 10.0f);
    int whole = t10 / 10;
    int frac = t10 % 10;
    if (frac < 0)
        frac = -frac;

    char tempStr[24];
    // formato: "23.4 C" (se vuoi il simbolo ° dimmelo e lo metto)
    snprintf(tempStr, sizeof(tempStr), "%d.%d C", whole, frac);

    // --- SCHERMO INTERO ---
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, textColor);
    Graphics_clearDisplay(&g_sContext);

    // Titolo grande
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) "BENVENUTO",
    AUTO_STRING_LENGTH,
                                64, 28, OPAQUE_TEXT);

    // Linea
    Graphics_drawLineH(&g_sContext, 16, 112, 48);

    // Label piccola
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) "Temperatura interna:",
    AUTO_STRING_LENGTH,
                                64, 68, OPAQUE_TEXT);

    // Temp grande
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) tempStr,
    AUTO_STRING_LENGTH,
                                64, 98, OPAQUE_TEXT);

    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);

    welcomeUntilMs = g_ms + 5000;   // 5 secondi
    currentState = STATE_WELCOME;
}
