#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/grlib/grlib.h>
#include "HAL_OPT3001.h"
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <string.h>


#define CENTER     8192
#define THRESHOLD  2000
#define PIN_LENGTH 4
#define MAX_LENGTH 6
#define LIGHT_THRESHOLD 50
typedef enum
{
    DIRECTION_NEUTRAL,
    DIRECTION_UP,
    DIRECTION_DOWN,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} JoystickDirection;


int backGroundColor=GRAPHICS_COLOR_BLACK;
int textColor=GRAPHICS_COLOR_WHITE;

typedef enum
{
    STATE_IDLE,
    STATE_FEEDBACK,
    STATE_BLOCKED
} AppState;

bool prevIsDay = true;
// PIN corretto
static const char correctPIN[PIN_LENGTH + 1] = "1234";

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
        'x', '0', 'E' } };

// Font disponibili
extern const tFont g_sFontFixed6x8;   // Font piccolo
extern const tFont g_sFontCmss20b;    // Font grande


void brightnessSensor(uint16_t lightValue);
void initSystem(void);
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
void handleSelectedChar(char c);
void checkPIN(void);
void resetPIN(void);
void returnToInitial(void);
bool inputAvailable(void);
void processInputDuringFeedback(void);

int main(void)
{
    initSystem();
    initADC();
    initDisplay();



    initButton();


    // Configura il clock a 3 MHz
        CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
        CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

        // Pin P2.4 per PWM
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
                                                    GPIO_PIN4,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);

        // PWM a 50 Hz
        Timer_A_PWMConfig pwmConfig =
        {
            TIMER_A_CLOCKSOURCE_SMCLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_3,
            20000,  // 20 ms
            TIMER_A_CAPTURECOMPARE_REGISTER_1,
            TIMER_A_OUTPUTMODE_RESET_SET,
            1500    // 1.5 ms => fermo
        };

        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    drawInitialScreen(); // Disegna tutto all'inizio


    while (1)

    {
        ADC14_toggleConversionTrigger();
           while (ADC14_isBusy());

           xValue = ADC14_getResult(ADC_MEM0);
           yValue = ADC14_getResult(ADC_MEM1);
           uint16_t lightValue = OPT3001_getLux();;

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

            if ((P5->IN & BIT1 ) == 0)
            {

                __delay_cycles(300000);
                while ((P5->IN & BIT1 ) == 0)
                    ;
                __delay_cycles(300000);

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

        __delay_cycles(20000);


    }
}
//fa l init del sistema in poche parole lo toglie dalla low power mode che non ho ancora capito che cazzo è
void initSystem(void)
{
    WDT_A_holdTimer();
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
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION); // X
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION); // Y

        ADC14_enableModule();
        ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);
        ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
        ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);
        ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9, false);
        ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
        ADC14_enableConversion();



    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();

    // Configura pin sensore luce
        Init_I2C_GPIO();
            I2C_init();

            /* Initialize OPT3001 digital ambient light sensor */
            OPT3001_init();
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
                                AUTO_STRING_LENGTH, keyX[col], keyY[row],
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
            lastFeedbackShown = 4;
            currentState = STATE_FEEDBACK;
        }
    }
    else if (c == 'x')
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
        showFeedback("CORRETTO");
        lastFeedbackShown = 1;
        currentState = STATE_FEEDBACK;
        // Imposta una velocità in avanti (es. 1.7 ms)
        // e attendi il tempo necessario a compiere ~360°
        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 1900);
        // Esempio: 1 secondo di rotazione
        __delay_cycles(9500000);

        // Ferma il servo (1.5 ms)
        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 1500);

    }
    else
    {
        attemptCount++;
        if (attemptCount >= 3)
        {
            showFeedback("BLOCCATO");
            currentState = STATE_BLOCKED;
        }
        else
        {
            showFeedback("ERRATO");
            lastFeedbackShown = 2;
            currentState = STATE_FEEDBACK;
        }
    }
}
//legge il valore di luminosità del sensore
uint16_t readLightSensor(void) {

    uint16_t result = OPT3001_getLux();
    return result;
}


void brightnessSensor(uint16_t lightValue){
    // Determina se è giorno o notte
    bool isDay = (lightValue >= LIGHT_THRESHOLD);

    // Se lo stato è cambiato rispetto al ciclo precedente, aggiorna i colori e ridisegna
    if (isDay != prevIsDay) {
        if (!isDay) {
            // Notte
            textColor = GRAPHICS_COLOR_WHITE;
            backGroundColor = GRAPHICS_COLOR_BLACK;
        } else {
            // Giorno
            textColor = GRAPHICS_COLOR_BLACK;
            backGroundColor = GRAPHICS_COLOR_WHITE;
        }

        // Aggiorna i colori del contesto
        Graphics_setForegroundColor(&g_sContext, textColor);
        Graphics_setBackgroundColor(&g_sContext, backGroundColor);

        // Ridisegna schermo solo al cambio di condizione
        drawInitialScreen();

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
    readJoystick();
    JoystickDirection currentDir = getJoystickDirection(xValue, yValue);
    if (currentDir != DIRECTION_NEUTRAL)
    {
        // Joystick mosso
        return true;
    }
    // Controllo pulsante
    if ((P5->IN & BIT1 ) == 0)
    {

        __delay_cycles(300000);
        while ((P5->IN & BIT1 ) == 0)
            ;
        __delay_cycles(300000);
        return true;
    }

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
