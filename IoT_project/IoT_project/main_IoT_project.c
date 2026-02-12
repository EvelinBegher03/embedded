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

// Hardware pin mapping
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
#define JOY_SW_PIN  GPIO_PIN1

// Flash memory record used to persist the door state
#define DOOR_NVM_ADDR   ((uint32_t)0x0003F000)
#define DOOR_NVM_MAGIC  ((uint32_t)0xD00A5A7E)

typedef enum
{
    DIRECTION_NEUTRAL,
    DIRECTION_UP,
    DIRECTION_DOWN,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} JoystickDirection;

// Door state
typedef enum
{
    DOOR_CLOSED = 0, DOOR_OPEN = 1
} DoorState;
static DoorState doorState = DOOR_CLOSED;


typedef struct {
    uint32_t magic;
    uint32_t state;   // 0 = closed, 1 = open
    uint32_t inv;
} DoorNvmRecord;

// Time base and helper utilities
static volatile uint32_t g_ms = 0;
static void initSysTickMs(void);
void SysTick_Handler(void);
static inline int32_t timePassed(uint32_t now, uint32_t deadline);
static uint32_t welcomeUntilMs = 0;
static uint8_t p51Prev = 1;
static uint32_t p51DebounceUntil = 0;
static uint8_t p51PressedEvent = 0;

// Buzzer state machine
typedef enum
{
    BUZ_IDLE = 0, BUZ_ERR, BUZ_SHAKE
} BuzMode;

static volatile BuzMode buzMode = BUZ_IDLE;
static uint32_t buzNextToggleMs = 0;
static uint32_t buzEndPhaseMs = 0;
static uint8_t buzPhase = 0;

// Task timer for buzzer button
static void buzzerTask(void);
static void p51Task(void);
static uint8_t p51ConsumePressedEvent(void);

// Servo state machine
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
// Correct PIN
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
int lastFeedbackShown = 0; // 0=none, 1=CORRECT, 2=INCORRECT, 3=WRONG LENGTH, 4=TOO MANY CHARS

// Keypad coordinates for drawing
int keyX[3] = { 32, 64, 96 };
int keyY[4] = { 40, 65, 90, 115 };

char keys[4][3] = { { '1', '2', '3' }, { '4', '5', '6' }, { '7', '8', '9' }, {
        'X', '0', 'E' } };

// Font available for display
extern const tFont g_sFontFixed6x8;   // Small font
extern const tFont g_sFontCmss20b;    // Large font

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

// Door open/close control
static void servoTask(void);
static void s2Task(void);
static void servoOpenFull(void);
static void servoOpenNudge(void);
static void servoCloseFull(void);
static void loadDoorStateFromFlash(void);
static void saveDoorStateToFlash(DoorState st);

// Temperature display
static void showWelcomeTemp(void);

int main(void)
{
    // System and peripheral initialization
    initSystem();
    loadDoorStateFromFlash();
    initBuzzer();
    initADC();
    initDisplay();
    initButton();

    // Set up system clock to 3 MHz
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Initialize timer for 1 ms time base
    initSysTickMs();

    // Configure P2.4 for PWM output to servo
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
    GPIO_PIN4,
                                                GPIO_PRIMARY_MODULE_FUNCTION);

    // PWM at 50 Hz for servo
    Timer_A_PWMConfig pwmConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,
                                    TIMER_A_CLOCKSOURCE_DIVIDER_3, 20000, // 20 ms
                                    TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                    TIMER_A_OUTPUTMODE_RESET_SET,
                                    1500    // 1.5 ms = neutral position
            };

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    drawInitialScreen(); // Draw initial UI

    // Main control loop
    while (1)
    {
        buzzerTask();
        p51Task();
        s2Task();       // Door close button
        servoTask();    // Servo state machine

        // Sample ADC channels for joystick and accelerometer
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

        brightnessSensor(lightValue);

        if (currentState == STATE_IDLE)
        {

            // Handle keypad navigation and selection via joystick
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

            // Wait for user input
            processInputDuringFeedback();

        }
        else if (currentState == STATE_BLOCKED)
        {
            // Blocked
        }

        if (currentState == STATE_WELCOME)
        {
            //Welcome state
            if ((int32_t) (g_ms - welcomeUntilMs) >= 0)
            {
                returnToInitial();
                currentState = STATE_IDLE;
            }
            // While in welcome, keypad input is ignored but servoTask() and S2Task() run
        }

    }
}
// Initialize system: disables watchdog timer and exits low power mode
void initSystem(void)
{
    WDT_A_holdTimer();
}

// Initialize the buzzet GPIO as output and set it low
static void initBuzzer(void)
{
    GPIO_setAsOutputPin(BUZZER_PORT, BUZZER_PIN);
    GPIO_setOutputLowOnPin(BUZZER_PORT, BUZZER_PIN);
}

// Initialize the LCD display
void initDisplay(void)
{
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                         &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, textColor);
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
}
// Initialize ADC channels for joystick, accelerometer and I2C sensors
void initADC(void)
{

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0,
    GPIO_TERTIARY_MODULE_FUNCTION); // X
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4,
    GPIO_TERTIARY_MODULE_FUNCTION); // Y

    // Accelerometer (ADC A14, A13, A11)
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

    // Initialize light sensor PIN
    Init_I2C_GPIO();
    I2C_init();

    // Initialize OPT3001 digital ambient light sensor
    OPT3001_init();

    // Initialize temperature sensor
    TMP006_init();
}

// Initialize button
void initButton(void)
{
    P5->DIR &= ~BIT1;
    P5->REN |= BIT1;
    P5->OUT |= BIT1;
}
// Read joystick X and Y values and store in global variables
void readJoystick(void)
{
    ADC14_toggleConversionTrigger();
    while (ADC14_isBusy())
        ;
    xValue = ADC14_getResult(ADC_MEM0);
    yValue = ADC14_getResult(ADC_MEM1);
}
// Initialize timer
static void initSysTickMs(void)
{
    // with DCO=3MHz, SysTick at 1ms => 3000 tick
    SysTick->LOAD = (3000 - 1);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
    SysTick_CTRL_TICKINT_Msk |
    SysTick_CTRL_ENABLE_Msk;
}

// Interrupt handler
void SysTick_Handler(void)
{
    g_ms++;
}

static inline int32_t timePassed(uint32_t now, uint32_t deadline)
{
    return (int32_t) (now - deadline) >= 0;
}

// Buzzer state machine for error feedback
static void buzzerTask(void)
{
    if (buzMode == BUZ_IDLE)
        return;

    // toggle cadence
    if (timePassed(g_ms, buzNextToggleMs))
    {
        GPIO_toggleOutputOnPin(BUZZER_PORT, BUZZER_PIN);

        // frequences: 1ms -> 500Hz toggle
        buzNextToggleMs = g_ms + 1;
    }

    if (!timePassed(g_ms, buzEndPhaseMs))
        return;

    // Turn off pin between phases
    GPIO_setOutputLowOnPin(BUZZER_PORT, BUZZER_PIN);

    if (buzMode == BUZ_ERR)
    {
        // Sequence: short beep, pause, short beep
        if (buzPhase == 0)
        {
            buzPhase = 1;
            buzEndPhaseMs = g_ms + 200;
            return;
        } // pause 200ms
        if (buzPhase == 1)
        {
            buzPhase = 2;
            buzEndPhaseMs = g_ms + 120;
            buzNextToggleMs = g_ms;
            return;
        } // second beep
        buzMode = BUZ_IDLE;
        return;
    }

    if (buzMode == BUZ_SHAKE)
    {
        // Sequence: long, pause, short, repeat 2 times
        static uint8_t reps = 0;
        if (buzPhase == 0)
        {
            buzPhase = 1;
            buzEndPhaseMs = g_ms + 200;
            return;
        }      // pause
        if (buzPhase == 1)
        {
            buzPhase = 2;
            buzEndPhaseMs = g_ms + 150;
            buzNextToggleMs = g_ms;
            return;
        } // short beep
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

// Button P5.1 polling and debouncing task
static void p51Task(void)
{
    uint8_t now = ((P5->IN & BIT1 ) ? 1 : 0); // 1 = non premuto (pull-up), 0 = premuto

    if (!timePassed(g_ms, p51DebounceUntil))
    {
        p51Prev = now;
        return;
    }

    if (p51Prev == 1 && now == 0)
    {
        p51PressedEvent = 1;
        p51DebounceUntil = g_ms + 200;
    }

    p51Prev = now;
}

// Consumes and returns the button pressed event
static uint8_t p51ConsumePressedEvent(void)
{
    if (p51PressedEvent)
    {
        p51PressedEvent = 0;
        return 1;
    }
    return 0;
}

// Return the joystick direction based on X/Y ADC readings
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
// Draws the initial screen
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
// Draws the keypad
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
// Draw the keypad key selected
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
// Updated the keypad selection
void updateSelectedKey(int oldRow, int oldCol, int newRow, int newCol)
{
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    drawKeyAt(oldRow, oldCol, false);
    drawKeyAt(newRow, newCol, true);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
}
// Shows the "PIN:" label on the display, cleaning its area
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
// Displays the currently entered PIN digits
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
// Shows a feedback message at the top of the display
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
// Clears the feedback area
void clearFeedback(void)
{
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, backGroundColor);
    Graphics_fillRectangle(&g_sContext,
                           &(Graphics_Rectangle ) { 0, 0, 127, 5 });
    Graphics_setForegroundColor(&g_sContext, textColor);
}
// Start an error beep sequence on the buzzer
static void beepError(void)
{
    buzMode = BUZ_ERR;
    buzPhase = 0;
    buzNextToggleMs = g_ms;
    buzEndPhaseMs = g_ms + 120;
}
// Starts a shake beep sequence on the buzzer
static void beepShake(void)
{
    buzMode = BUZ_SHAKE;
    buzPhase = 0;
    buzNextToggleMs = g_ms;
    buzEndPhaseMs = g_ms + 300;
}

// Handles the character selected on the keypad
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
// Checks if the PIN is correct
void checkPIN(void)
{
    bool correct = (strcmp(enteredPIN, correctPIN) == 0);
    if (correct)
    {
        showWelcomeTemp();
        currentState = STATE_WELCOME;
        welcomeUntilMs = g_ms + 5000;
        onPinCorrect();
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

// Called when PIN is correct
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

// Checks for shake events using accelerometer readings
static void checkTamper(uint16_t ax, uint16_t ay, uint16_t az)
{
    if (doorState)
        return;
    static uint8_t shakeCooldown = 0;
    static uint16_t accX_prev = 0, accY_prev = 0, accZ_prev = 0;
    static uint8_t accInit = 0;
    int32_t dx, dy, dz;

    if (!accInit)
    {
        accX_prev = ax;
        accY_prev = ay;
        accZ_prev = az;
        accInit = 1;
        return;
    }

    if (shakeCooldown)
    {
        shakeCooldown--;
        accX_prev = ax;
        accY_prev = ay;
        accZ_prev = az;
        return;
    }

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

    if (dx > 1500 || dy > 1500 || dz > 1500)
    {
        beepShake();
        shakeCooldown = 100;
    }
}

// Handles ambient light theme switching
void brightnessSensor(uint16_t lightValue)
{
    
    bool isDay = (lightValue >= LIGHT_THRESHOLD);

    if (isDay != prevIsDay)
    {
        if (!isDay)
        {
            textColor = GRAPHICS_COLOR_WHITE;
            backGroundColor = GRAPHICS_COLOR_BLACK;
        }
        else
        {
            textColor = GRAPHICS_COLOR_BLACK;
            backGroundColor = GRAPHICS_COLOR_WHITE;
        }

        // Update context colors
        Graphics_setForegroundColor(&g_sContext, textColor);
        Graphics_setBackgroundColor(&g_sContext, backGroundColor);

        if (currentState == STATE_IDLE)
        {
            drawInitialScreen();
        }

        // Update the state
        prevIsDay = isDay;
    }
}

// Resets the entered PIN buffer and updates display
void resetPIN(void)
{
    pinIndex = 0;
    memset(enteredPIN, 0, sizeof(enteredPIN));
    showPin();
}

// Returns the UI to the initial state
void returnToInitial(void)
{
    drawInitialScreen();
    lastFeedbackShown = 0;
    currentState = STATE_IDLE;
}

// Returns true if keypad or button input is available
bool inputAvailable(void)
{
    if (p51PressedEvent)
        return true;
    static JoystickDirection prevDirFB = DIRECTION_NEUTRAL;

    readJoystick();
    JoystickDirection currentDir = getJoystickDirection(xValue, yValue);

    if (prevDirFB == DIRECTION_NEUTRAL && currentDir != DIRECTION_NEUTRAL)
    {
        prevDirFB = currentDir;
        return true;
    }

    if (currentDir == DIRECTION_NEUTRAL)
    {
        prevDirFB = DIRECTION_NEUTRAL;
    }

    if (p51PressedEvent)
        return true;

    return false;
}

// Handles user input during feedback state
void processInputDuringFeedback(void)
{
    if (currentState == STATE_FEEDBACK && inputAvailable())
    {
        if (lastFeedbackShown == 1)
        { // CORRECT
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
    }
}

// Handles servo position transitions
static void servoTask(void)
{
    if (servoPhase == SERVO_IDLE)
        return;

    if ((int32_t) (g_ms - servoDeadlineMs) < 0)
        return;

    switch (servoPhase)
    {
    case SERVO_OPEN_RUN:
        Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                1900);
        servoPhase = SERVO_OPEN_BRAKE;
        servoDeadlineMs = g_ms + 1000;
        break;

    case SERVO_OPEN_BRAKE:
        Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                1500);
        servoPhase = SERVO_IDLE;
        break;

    case SERVO_CLOSE_RUN:
        Timer_A_setCompareValue(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                1500);
        servoPhase = SERVO_IDLE;
        doorState = DOOR_CLOSED;
        saveDoorStateToFlash(doorState);
        break;

    default:
        servoPhase = SERVO_IDLE;
        break;
    }
}

// Handles debouncing and starts door close if pressed
static void s2Task(void)
{
    static uint8_t s2Prev = 1;
    static uint32_t s2DebounceUntil = 0;
    uint8_t s2Now = GPIO_getInputPinValue(S2_PORT, S2_PIN);

    if ((int32_t) (g_ms - s2DebounceUntil) < 0)
    {
        s2Prev = s2Now;
        return;
    }

    if (s2Prev == 1 && s2Now == 0)
    {
        s2DebounceUntil = g_ms + 200;

        if (doorState == DOOR_OPEN && servoPhase == SERVO_IDLE)
        {
            servoCloseFull();
        }
    }

    s2Prev = s2Now;
}

// Starts full door open sequence on the servo
static void servoOpenFull(void)
{
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            1100);
    servoPhase = SERVO_OPEN_RUN;
    servoDeadlineMs = g_ms + 6667;   // 20,000,000 cycles @3MHz ≈ 6667ms
}

// Starts partial door open sequence on the servo
static void servoOpenNudge(void)
{
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            1100);
    servoPhase = SERVO_OPEN_RUN;
    servoDeadlineMs = g_ms + 3333;   // 10,000,000 cycles @3MHz ≈ 3333ms
}

// Starts full door close sequence on the servo
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

// Loads door state from flash memory
static void loadDoorStateFromFlash(void)
{
    const DoorNvmRecord *rec = (const DoorNvmRecord *)DOOR_NVM_ADDR;

    if (rec->magic == DOOR_NVM_MAGIC && rec->inv == ~rec->state)
    {
        doorState = (rec->state ? DOOR_OPEN : DOOR_CLOSED);
    }
    else
    {
        doorState = DOOR_CLOSED;
    }
}

// Saves door state to flash memory
static void saveDoorStateToFlash(DoorState st)
{
    DoorNvmRecord rec;
    rec.magic = DOOR_NVM_MAGIC;
    rec.state = (st == DOOR_OPEN) ? 1u : 0u;
    rec.inv   = ~rec.state;

    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31);
    FlashCtl_eraseSector(DOOR_NVM_ADDR);
    FlashCtl_programMemory((void *)&rec, (void *)DOOR_NVM_ADDR, sizeof(rec));
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31);
}

// Displays welcome screen with temperature reading
static void showWelcomeTemp(void)
{
    int raw = TMP006_readAmbientTemperature();
    raw >>= 2;
    float tC = (float) raw * 0.03125f;

    int t10 = (int) (tC * 10.0f);
    int whole = t10 / 10;
    int frac = t10 % 10;
    if (frac < 0)
        frac = -frac;

    char tempStr[24];
    snprintf(tempStr, sizeof(tempStr), "%d.%d C", whole, frac);

    // Full screen
    Graphics_setBackgroundColor(&g_sContext, backGroundColor);
    Graphics_setForegroundColor(&g_sContext, textColor);
    Graphics_clearDisplay(&g_sContext);

    // Big title
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) "BENVENUTO",
    AUTO_STRING_LENGTH,
                                64, 28, OPAQUE_TEXT);

    // Line
    Graphics_drawLineH(&g_sContext, 16, 112, 48);

    // Small label
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) "Temperatura interna:",
    AUTO_STRING_LENGTH,
                                64, 68, OPAQUE_TEXT);

    // Big temp
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    Graphics_drawStringCentered(&g_sContext, (int8_t*) tempStr,
    AUTO_STRING_LENGTH,
                                64, 98, OPAQUE_TEXT);

    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);

    welcomeUntilMs = g_ms + 5000;
    currentState = STATE_WELCOME;
}
